"""Levantarse (fase 4): el robot empieza tumbado y debe ponerse de pie y quedarse quieto.

Parte de la tarea de marcha (`danny_flat_env_cfg`) sin reloj de marcha, sin orden de altura, sin empujones y sin terminación por caída.
El inicio es el de la búsqueda sin realimentación (`analysis/search_get_up.py`): boca abajo con el tronco girado 1,5 rad, o boca arriba
con -1,5 rad, articulaciones en la pose cero y guiñada aleatoria. La fracción boca abajo se elige con DANNY_PRONE_FRACTION (1 por defecto).
"""

from __future__ import annotations

import math
import os

import mujoco
import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.utils.lab_api.math import quat_from_euler_xyz, sample_uniform

from danny_training import crouch, env_cfgs, mdp as danny_mdp
from danny_training.robot import DANNY_XML

PRONE_FRACTION = float(os.environ.get("DANNY_PRONE_FRACTION", "1.0"))
START_PITCH_RAD = 1.5  # boca abajo +1,5; boca arriba -1,5 (igual que la búsqueda).
START_HEIGHT_M = {"prone": 0.045, "supine": 0.03}
START_JOINT_NOISE_RAD = 0.05
PELVIS_SITE = "pelvis_bottom"
# Ayuda vertical del entrenamiento (corrida 2): fuerza hacia arriba en la pelvis, como fracción del peso, que se mantiene y se quita. Solo
# con DANNY_GETUP_ASSIST=1; las medidas y los vídeos la dejan apagada. Pasos de entorno por iteración de PPO: 24.
ASSIST = os.environ.get("DANNY_GETUP_ASSIST", "0") == "1"
ASSIST_FRACTION = 0.8
ASSIST_HOLD_ITERATIONS = 500
ASSIST_END_ITERATIONS = 2500
STEPS_PER_ITERATION = 24
ROBOT_MASS_KG = 1.30  # PROJECT_HANDOFF.md, etapa 3.
SOFT_LIMIT_FACTOR = 0.9  # el mismo de `robot.py`.
# Umbrales de la definición de «se levanta» (PROJECT_HANDOFF.md, Fase 4).
STOOD_PELVIS_M = 0.20
STOOD_TILT_RAD = math.radians(15.0)
RISEN_PELVIS_M = 0.10
# Comprobación de avance (5 de noviembre): pelvis a 0,15 m o más con el tronco a 45° o menos. La de 0,10 m sola la pasaba el 99 % tumbado o boca abajo.
UP_PELVIS_M = 0.15
UP_TILT_RAD = math.radians(45.0)


def reset_lying(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, prone_fraction: float, joint_noise_rad: float) -> None:
  """Coloca el robot tumbado (boca abajo con probabilidad `prone_fraction`, si no boca arriba), con guiñada aleatoria y articulaciones en cero más ruido."""
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
  asset: Entity = env.scene["robot"]
  n = len(env_ids)
  prone = torch.rand(n, device=env.device) < prone_fraction
  pitch = torch.where(prone, START_PITCH_RAD, -START_PITCH_RAD)
  yaw = sample_uniform(-math.pi, math.pi, (n,), env.device)
  quat = quat_from_euler_xyz(torch.zeros_like(yaw), pitch, yaw)
  z = torch.where(prone, START_HEIGHT_M["prone"], START_HEIGHT_M["supine"])
  pos = torch.zeros(n, 3, device=env.device)
  pos[:, 2] = z
  pos += env.scene.env_origins[env_ids]
  asset.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
  asset.write_root_link_velocity_to_sim(torch.zeros(n, 6, device=env.device), env_ids=env_ids)

  limits = asset.data.soft_joint_pos_limits[env_ids]
  joint_pos = sample_uniform(-joint_noise_rad, joint_noise_rad, (n, limits.shape[1]), env.device)
  joint_pos = joint_pos.clamp(limits[..., 0], limits[..., 1])
  asset.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos), env_ids=env_ids)


def assist_lift(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, asset_cfg: SceneEntityCfg) -> None:
  """Empuja la pelvis hacia arriba con una fracción del peso: constante hasta ASSIST_HOLD_ITERATIONS y lineal hasta 0 en ASSIST_END_ITERATIONS."""
  iteration = env.common_step_counter / STEPS_PER_ITERATION
  fade = min(1.0, max(0.0, (ASSIST_END_ITERATIONS - iteration) / (ASSIST_END_ITERATIONS - ASSIST_HOLD_ITERATIONS)))
  forces = torch.zeros(env.num_envs, len(asset_cfg.body_ids), 3, device=env.device)
  forces[..., 2] = ASSIST_FRACTION * fade * ROBOT_MASS_KG * 9.81
  env.scene[asset_cfg.name].write_external_wrench_to_sim(forces, torch.zeros_like(forces), body_ids=asset_cfg.body_ids)
  env.extras["log"]["Metrics/getup_assist_fraction"] = torch.tensor(ASSIST_FRACTION * fade, device=env.device)


def soft_limit_clip() -> dict[str, tuple[float, float]]:
  """Rango de cada articulación (límite blando) en el que se acota el objetivo de posición pedido por la política."""
  model = mujoco.MjModel.from_xml_path(str(DANNY_XML))
  clip = {}
  for joint in range(model.njnt):
    if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE:
      low, high = model.jnt_range[joint]
      mid, half = (low + high) / 2, (high - low) / 2 * SOFT_LIMIT_FACTOR
      clip[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)] = (float(mid - half), float(mid + half))
  return clip


def _pelvis_z(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
  asset: Entity = env.scene[asset_cfg.name]
  return asset.data.site_pos_w[:, asset_cfg.site_ids[0], 2]


def pelvis_height_progress(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
  """Altura de la pelvis respecto a la de pie, de 0 a 1 (lineal, para que haya pendiente desde el suelo), por (1 + cos del tronco) / 2.

  Sin el factor de tronco, la corrida 1 se quedó del revés sobre la cabeza (pelvis a 0,152 m, tronco a 147°) cobrando casi todo.
  """
  cos_tilt = -env.scene["robot"].data.projected_gravity_b[:, 2]
  return (_pelvis_z(env, asset_cfg) / crouch.STAND_PELVIS_Z_M).clamp(0.0, 1.0) * (1.0 + cos_tilt) / 2.0


def feet_down_when_up(env: ManagerBasedRlEnv, sensor_name: str, asset_cfg: SceneEntityCfg, min_height: float = 0.15) -> torch.Tensor:
  """Fracción de pies apoyados (0, 0,5 o 1), solo con la pelvis sobre `min_height`."""
  found = env.scene[sensor_name].data.found.reshape(env.num_envs, -1)[:, :2] > 0
  return found.float().mean(dim=1) * (_pelvis_z(env, asset_cfg) > min_height).float()


def stand_still(env: ManagerBasedRlEnv, std: float, asset_cfg: SceneEntityCfg, min_height: float = 0.20) -> torch.Tensor:
  """Premia estar quieto (velocidad lineal y angular del tronco) una vez la pelvis pasa de `min_height`."""
  robot: Entity = env.scene["robot"]
  speed2 = robot.data.root_com_lin_vel_b.square().mean(dim=1) + 0.1 * robot.data.root_link_ang_vel_b.square().mean(dim=1)
  return (_pelvis_z(env, asset_cfg) > min_height).float() * torch.exp(-speed2 / std**2)


def pose_when_up(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg, min_height: float = 0.0) -> torch.Tensor:
  """Error cuadrático medio respecto a la pose de pie por defecto, solo con la pelvis sobre `min_height` (0: siempre).

  Con umbral 0,15 m y suma con peso -2 la política dejó de levantarse (corrida 3, ver PROJECT_HANDOFF.md: se quedó a 0,141 m, bajo el umbral).
  Corrida 4: sin umbral, para que no haya escalón en la recompensa; media con peso -1,5.
  """
  robot: Entity = env.scene["robot"]
  error = (robot.data.joint_pos - robot.data.default_joint_pos).square().mean(dim=1)
  return error * (_pelvis_z(env, asset_cfg) > min_height).float()


def servo_torque_excess_when_up(env: ManagerBasedRlEnv, threshold: float, asset_cfg: SceneEntityCfg, pelvis_cfg: SceneEntityCfg) -> torch.Tensor:
  """Como `servo_torque_excess` en todas las articulaciones, pero solo con la pelvis sobre 0,20 m: levantarse puede exigir picos, quedarse de pie no."""
  return danny_mdp.servo_torque_excess(env, threshold, asset_cfg) * (_pelvis_z(env, pelvis_cfg) > STOOD_PELVIS_M).float()


class getup_progress:
  """Recompensa sin efecto (peso casi nulo) que registra cuánto se ha levantado cada entorno.

  Guarda la altura máxima de la pelvis de cada entorno desde su último reinicio y registra, para TensorBoard, la fracción de entornos que
  alguna vez pasaron de 0,10 m (`Metrics/getup_risen_fraction`) y la que está ahora de pie según el criterio de altura e inclinación
  (`Metrics/getup_stood_fraction`) y la que supera la comprobación de avance (`Metrics/getup_up_fraction`: pelvis 0,15 m, tronco 45°).
  """

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    self.max_z = torch.zeros(env.num_envs, device=env.device)

  def __call__(self, env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    z = _pelvis_z(env, asset_cfg)
    self.max_z = torch.maximum(self.max_z, z)
    tilt = torch.acos((-env.scene["robot"].data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
    env.extras["log"]["Metrics/getup_risen_fraction"] = (self.max_z >= RISEN_PELVIS_M).float().mean()
    env.extras["log"]["Metrics/getup_stood_fraction"] = ((z >= STOOD_PELVIS_M) & (tilt <= STOOD_TILT_RAD)).float().mean()
    env.extras["log"]["Metrics/getup_up_fraction"] = ((z >= UP_PELVIS_M) & (tilt <= UP_TILT_RAD)).float().mean()
    return torch.zeros(env.num_envs, device=env.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    self.max_z[env_ids if env_ids is not None else slice(None)] = 0.0


def danny_getup_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tarea de levantarse. Parte de la de marcha y quita lo que no se usa."""
  cfg = env_cfgs.danny_flat_env_cfg(play=play)
  cfg.episode_length_s = 20.0 if not play else int(1e9)

  # Sin reloj de marcha ni orden de altura: la política los ve en la fase 3, aquí no existen.
  cfg.commands.pop("gait")
  cfg.commands.pop("height")
  for group in ("actor", "critic"):
    cfg.observations[group].terms.pop("gait_clock")
    cfg.observations[group].terms.pop("height")
  cfg.commands["twist"].rel_standing_envs = 1.0  # orden de velocidad siempre cero.

  cfg.events.pop("push_robot", None)
  cfg.events.pop("reset_base")
  cfg.events.pop("reset_robot_joints")
  cfg.events["reset_lying"] = EventTermCfg(
    func=reset_lying,
    mode="reset",
    params={"prone_fraction": PRONE_FRACTION, "joint_noise_rad": START_JOINT_NOISE_RAD},
  )

  # Sin terminación por caída: estar tumbado es el inicio.
  cfg.terminations.pop("fell_over", None)

  # Objetivo de posición acotado al rango blando: en la corrida 1 la política pidió -308° en una cadera de -94°.
  cfg.actions["joint_pos"].clip = soft_limit_clip()

  pelvis = SceneEntityCfg("robot", site_names=(PELVIS_SITE,))
  trunk = SceneEntityCfg("robot", body_names=(env_cfgs.TRUNK_BODY,))
  if ASSIST:
    cfg.events["assist_lift"] = EventTermCfg(func=assist_lift, mode="step", params={"asset_cfg": SceneEntityCfg("robot", body_names=(env_cfgs.TRUNK_BODY,))})

  cfg.rewards = {
    "pelvis_height": RewardTermCfg(func=pelvis_height_progress, weight=4.0, params={"asset_cfg": pelvis}),
    "upright": RewardTermCfg(func=velocity_mdp.upright, weight=2.0, params={"std": math.sqrt(0.2), "asset_cfg": trunk}),
    "feet_down_when_up": RewardTermCfg(
      func=feet_down_when_up, weight=1.0, params={"sensor_name": "feet_ground_contact", "asset_cfg": pelvis}
    ),
    "stand_still": RewardTermCfg(func=stand_still, weight=2.0, params={"std": 0.3, "asset_cfg": pelvis}),
    "pose_when_up": RewardTermCfg(func=pose_when_up, weight=-1.5, params={"asset_cfg": pelvis}),
    "dof_pos_limits": RewardTermCfg(func=velocity_mdp.joint_pos_limits, weight=-1.0),
    "action_rate_l2": RewardTermCfg(func=velocity_mdp.action_rate_l2, weight=-0.1),
    "link_pair_contacts": RewardTermCfg(func=danny_mdp.link_pair_contacts, weight=-1.0),
    "servo_torque_excess_when_up": RewardTermCfg(
      func=servo_torque_excess_when_up,
      weight=-317.0,
      params={"threshold": 0.8, "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)), "pelvis_cfg": pelvis},
    ),
    "getup_progress": RewardTermCfg(func=getup_progress, weight=1e-9, params={"asset_cfg": pelvis}),
  }
  return cfg
