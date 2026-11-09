"""Sentarse (fase 5): el robot empieza de pie y debe bajar al suelo, sentarse y quedarse quieto, sin caer ni golpear.

Parte de la tarea de levantarse (`getup`) sin reloj de marcha ni orden de altura, con el inicio de pie de la tarea de marcha. «Sentado» es: pelvis a
`SEATED_PELVIS_M` o menos del suelo, tronco a `SEATED_TILT_RAD` o menos, sin tocar el suelo con tronco, cuello ni cabeza, y quieto. Las manos, los
brazos, las piernas y los pies pueden apoyarse: una prueba pasiva (`scripts/check_sit_pose.py`) muestra que sentado con los brazos colgando el robot
se cae hacia atrás, así que la política tiene que apoyarse con los brazos o con los pies.
"""

from __future__ import annotations

import math

import mujoco
import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp as velocity_mdp

from danny_training import crouch, env_cfgs, getup, mdp as danny_mdp

SEATED_PELVIS_M = 0.07
SEATED_TILT_RAD = math.radians(35.0)
SEATED_SPEED_M_S = 0.05
SEAT_Z_M = 0.05  # altura de la pelvis sentado con que se normaliza el avance (estimación, ver PROJECT_HANDOFF.md).
MAX_DESCENT_SPEED_M_S = 0.15  # el criterio es 0,3 m/s de 0,218 a 0,16 m; 0,15 sale de bajar 0,15 m en 1 s (decisión del dueño, 9 nov, tras model_7700)
MAX_TRUNK_RATE_RAD_S = 2.0  # el criterio es 3 rad/s en cualquier eje
HAND_BODIES = ("left_forearm_1", "right_forearm_1")
HANDS_REWARD_PELVIS_M = 0.13
FALL_TILT_RAD = math.radians(80.0)  # decisión del dueño, 9 nov: el contacto de las manos pide el tronco a 65-70° (mjlab corta a 70°)
FORBIDDEN_BODIES = ("torso_1", "neck_link_1", "head_1")


def sit_progress(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
  """Cuánto ha bajado la pelvis, de 0 (de pie) a 1 (sentado), por (1 + cos del tronco) / 2 para que tumbarse hacia un lado no cobre."""
  cos_tilt = -env.scene["robot"].data.projected_gravity_b[:, 2]
  drop = 1.0 - ((getup._pelvis_z(env, asset_cfg) - SEAT_Z_M) / (crouch.STAND_PELVIS_Z_M - SEAT_Z_M)).clamp(0.0, 1.0)
  return drop * (1.0 + cos_tilt).clamp(min=0.0) / 2.0


def seated_still(env: ManagerBasedRlEnv, std: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
  """exp(-velocidad² / std²) por `sit_progress`: quieto solo cobra cuando ya está abajo."""
  speed = env.scene["robot"].data.root_com_lin_vel_b.norm(dim=1)
  return torch.exp(-speed.square() / std**2) * sit_progress(env, asset_cfg)


def descent_speed_excess(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Cuadrado de lo que la pelvis baja más deprisa que `MAX_DESCENT_SPEED_M_S`: castiga dejarse caer."""
  vz = env.scene["robot"].data.root_com_lin_vel_w[:, 2]
  return (-vz - MAX_DESCENT_SPEED_M_S).clamp(min=0.0).square()


def trunk_rate_excess(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Suma, por eje de la pelvis, del cuadrado de lo que el giro supera `MAX_TRUNK_RATE_RAD_S`."""
  rate = env.scene["robot"].data.root_link_ang_vel_b.abs()
  return (rate - MAX_TRUNK_RATE_RAD_S).clamp(min=0.0).square().sum(dim=1)


def _floor_contacts(env: ManagerBasedRlEnv, bodies: tuple[str, ...], key: str) -> torch.Tensor:
  """Número de contactos con el suelo de los cuerpos `bodies` (`key` identifica la caché)."""
  cache = env.extras.setdefault(key, {})
  model = env.sim.mj_model
  if "geoms" not in cache:
    ids = [model.body(f"robot/{name}").id for name in bodies]
    cache["geoms"] = torch.tensor([i for i in range(model.ngeom) if model.geom_bodyid[i] in ids], device=env.device)
    cache["floor"] = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=env.device)
  data = env.sim.data
  geom = data.contact.geom.long()
  active = (torch.arange(geom.shape[0], device=env.device) < data.nacon[0]) & (data.contact.dist < 0.0)
  first, second = torch.isin(geom[:, 0], cache["floor"]), torch.isin(geom[:, 1], cache["floor"])
  other = torch.where(first, geom[:, 1], geom[:, 0])
  hit = active & (first | second) & torch.isin(other, cache["geoms"])
  return torch.zeros(env.num_envs, device=env.device).index_add_(0, data.contact.worldid[hit].long(), torch.ones(int(hit.sum()), device=env.device))


def forbidden_floor_contacts(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Número de contactos con el suelo de tronco, cuello y cabeza."""
  return _floor_contacts(env, FORBIDDEN_BODIES, "_forbidden_floor")


def hands_on_floor_when_low(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
  """1 si algún antebrazo toca el suelo y la pelvis está a `HANDS_REWARD_PELVIS_M` o menos; 0 en otro caso.

  Con la pelvis más arriba las manos no llegan al suelo detrás (cálculo con el modelo), así que el tope impide agacharse sobre las manos desde arriba.
  """
  touching = _floor_contacts(env, HAND_BODIES, "_hand_floor") > 0
  return (touching & (getup._pelvis_z(env, asset_cfg) <= HANDS_REWARD_PELVIS_M)).float()


class sit_metrics:
  """Recompensa sin efecto que registra, para TensorBoard, la fracción de robots sentados (`Metrics/sit_seated_fraction`) y la que ha bajado de 0,15 m."""

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    pass

  def __call__(self, env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    z = getup._pelvis_z(env, asset_cfg)
    data = env.scene["robot"].data
    tilt = torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
    speed = data.root_com_lin_vel_b.norm(dim=1)
    env.extras["log"]["Metrics/sit_seated_fraction"] = ((z <= SEATED_PELVIS_M) & (tilt <= SEATED_TILT_RAD) & (speed <= SEATED_SPEED_M_S)).float().mean()
    env.extras["log"]["Metrics/sit_low_fraction"] = (z <= 0.15).float().mean()
    return torch.zeros(env.num_envs, device=env.device)


def danny_sit_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tarea de sentarse. Parte de la de marcha: sin reloj, sin orden de altura, sin empujones, con el inicio de pie."""
  cfg = env_cfgs.danny_flat_env_cfg(play=play)
  cfg.episode_length_s = 10.0 if not play else int(1e9)
  cfg.commands.pop("gait")
  cfg.commands.pop("height")
  for group in ("actor", "critic"):
    cfg.observations[group].terms.pop("gait_clock")
    cfg.observations[group].terms.pop("height")
  cfg.commands["twist"].rel_standing_envs = 1.0
  cfg.events.pop("push_robot", None)
  cfg.terminations["fell_over"].params["limit_angle"] = FALL_TILT_RAD
  cfg.actions["joint_pos"].clip = getup.soft_limit_clip()

  pelvis = SceneEntityCfg("robot", site_names=(getup.PELVIS_SITE,))
  cfg.rewards = {
    "sit_progress": RewardTermCfg(func=sit_progress, weight=4.0, params={"asset_cfg": pelvis}),
    "seated_still": RewardTermCfg(func=seated_still, weight=2.0, params={"std": 0.3, "asset_cfg": pelvis}),
    "descent_speed_excess": RewardTermCfg(func=descent_speed_excess, weight=-100.0),
    "trunk_rate_excess": RewardTermCfg(func=trunk_rate_excess, weight=-1.0),
    "hands_on_floor_when_low": RewardTermCfg(func=hands_on_floor_when_low, weight=1.0, params={"asset_cfg": pelvis}),
    "forbidden_floor_contacts": RewardTermCfg(func=forbidden_floor_contacts, weight=-1.0),
    "action_out_of_range": RewardTermCfg(func=getup.action_out_of_range_l1, weight=-0.035),
    "dof_pos_limits": RewardTermCfg(func=velocity_mdp.joint_pos_limits, weight=-1.0),
    "action_rate_l2": RewardTermCfg(func=velocity_mdp.action_rate_l2, weight=-0.1),
    "link_pair_contacts": RewardTermCfg(func=danny_mdp.link_pair_contacts, weight=-1.0),
    "servo_torque_excess": RewardTermCfg(
      func=danny_mdp.servo_torque_excess,
      weight=-317.0,
      params={"threshold": 0.8, "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",))},
    ),
    "sit_metrics": RewardTermCfg(func=sit_metrics, weight=1e-9, params={"asset_cfg": pelvis}),
  }
  return cfg
