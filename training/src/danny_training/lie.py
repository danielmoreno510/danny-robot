"""Tumbarse (fase 5): el robot empieza sentado en el suelo y debe quedar boca arriba, quieto, bajando de forma controlada.

Parte de la tarea de sentarse (`sit`): misma observación (78 y 93), sin órdenes y sin terminación por caída, porque tumbarse es inclinar el tronco.
«Boca arriba» es que el eje x del tronco apunte hacia arriba (`projected_gravity_b[:, 0]` a -0,9 o menos) con la inclinación lateral pequeña.
Inicio (`DANNY_LIE_START`): `sit_end` (por defecto) sortea uno de los estados reales con que acaba la política de sentarse, guardados por
`scripts/export_sit_end_states.py` en `training/data/sit_end_states.pt` (con brazos, cintura y cuello donde los deja esa política); `clean` es la pose
sentada limpia con las que se entrenaron las corridas 1 y 2 (cadera de cabeceo -91°, rodilla 7°, tobillo -32°, el resto en la pose de pie, más ruido).
En los dos el robot se apoya sin penetrar el suelo. Se castiga el giro de la pelvis en cualquier eje por encima de `PELVIS_RATE_TRAIN_LIMIT_RAD_S`, la
velocidad del tronco por encima de `TRUNK_SPEED_TRAIN_LIMIT_M_S` y que cuello o cabeza toquen el suelo; los límites de entrenamiento están por debajo de
los del criterio (3 rad/s y 0,8 m/s) para dejar margen.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import mujoco
import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.utils.lab_api.math import quat_from_euler_xyz, quat_mul, sample_uniform

from danny_training import crouch, getup, mdp as danny_mdp, sit

SEATED_JOINTS_RAD = {"hip_pitch": math.radians(-91.0), "knee": math.radians(7.0), "ankle_pitch": math.radians(-32.0)}
SEATED_PELVIS_M = 0.056
START_DROP_MARGIN_M = 0.01
START_CLEARANCE_M = 0.001  # separación entre el punto más bajo y el suelo al empezar: apoyado, sin penetrar.
SETTLE_STEPS = 10  # pasos quietos (articulaciones sujetas en su pose) antes de contar, solo en las medidas del inicio `clean`.
START_JOINT_NOISE_RAD = 0.05
SIT_END_JOINT_NOISE_RAD = 0.02
SIT_END_EXACT_FRACTION = 0.5  # fracción de los reinicios `sit_end` sin ruido ni margen: el estado exacto con que acaba sentarse, que es el que ve la cadena
SIT_END_STATES = Path(__file__).resolve().parents[2] / "data" / "sit_end_states.pt"
LIE_START = os.environ.get("DANNY_LIE_START", "sit_end")
assert LIE_START in ("sit_end", "clean"), f"DANNY_LIE_START debe ser sit_end o clean, no {LIE_START}"
LYING_GRAVITY_X = -0.9
LYING_SIDE_MAX = 0.3
LYING_SPEED_M_S = 0.05
LYING_ANGULAR_RAD_S = 0.1
MAX_PITCH_RATE_RAD_S = 2.0
PELVIS_RATE_TRAIN_LIMIT_RAD_S = 2.0
PELVIS_RATE_EXCESS_CLAMP_RAD_S = 5.0
TRUNK_SPEED_TRAIN_LIMIT_M_S = 0.5
TRUNK_SPEED_EXCESS_CLAMP_M_S = 1.0
TRUNK_BODY = "torso_1"
HEAD_BODIES = ("neck_link_1", "head_1")


def lowest_floor_distance(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Distancia mínima de los contactos con el suelo de cada robot (negativa si penetra); 1 si no hay ningún contacto cerca."""
  cache = env.extras.setdefault("_lowest_floor_distance", {})
  model = env.sim.mj_model
  if "floor" not in cache:
    cache["floor"] = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=env.device)
  data = env.sim.data
  geom = data.contact.geom.long()
  active = (torch.arange(geom.shape[0], device=env.device) < data.nacon[0]) & torch.isin(geom, cache["floor"]).any(dim=1)
  world = data.contact.worldid.long()
  distance = torch.ones(env.num_envs, device=env.device)
  distance.scatter_reduce_(0, world[active], data.contact.dist[active], reduce="amin")
  return distance


def _ground_and_write(env: ManagerBasedRlEnv, env_ids: torch.Tensor, pos: torch.Tensor, quat: torch.Tensor, joint_pos: torch.Tensor) -> None:
  """Escribe la pose y las articulaciones (en reposo) y sube o baja la pelvis hasta que el punto más bajo quede a `START_CLEARANCE_M` del suelo."""
  asset: Entity = env.scene["robot"]
  n = len(env_ids)
  asset.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
  asset.write_root_link_velocity_to_sim(torch.zeros(n, 6, device=env.device), env_ids=env_ids)
  limits = asset.data.soft_joint_pos_limits[env_ids]
  joint_pos = joint_pos.clamp(limits[..., 0], limits[..., 1])
  asset.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos), env_ids=env_ids)
  env.sim.forward()
  distance = lowest_floor_distance(env)[env_ids]
  shift = torch.where(distance < 0.5, START_CLEARANCE_M - distance, torch.zeros_like(distance))
  pos[:, 2] += shift
  asset.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
  env.sim.forward()


def reset_seated(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, joint_noise_rad: float) -> None:
  """Inicio `clean`: sentado con las piernas estiradas, guiñada aleatoria y ruido en las articulaciones."""
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
  asset: Entity = env.scene["robot"]
  n = len(env_ids)
  yaw = sample_uniform(-math.pi, math.pi, (n,), env.device)
  quat = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
  default = asset.data.default_root_state[env_ids]
  pos = default[:, :3].clone()
  pos[:, 2] += SEATED_PELVIS_M - crouch.STAND_PELVIS_Z_M + START_DROP_MARGIN_M
  pos += env.scene.env_origins[env_ids]
  joint_pos = asset.data.default_joint_pos[env_ids].clone()
  for j, name in enumerate(asset.joint_names):
    for key, angle in SEATED_JOINTS_RAD.items():
      if name.endswith(key):
        joint_pos[:, j] = angle
  joint_pos += sample_uniform(-joint_noise_rad, joint_noise_rad, joint_pos.shape, env.device)
  _ground_and_write(env, env_ids, pos, quat, joint_pos)


def reset_from_sit_end(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, joint_noise_rad: float) -> None:
  """Inicio `sit_end`: uno de los estados reales con que termina la política de sentarse (articulaciones y orientación sin guiñada), guiñada aleatoria.

  Una fracción `SIT_END_EXACT_FRACTION` de los reinicios usa el estado exacto, sin ruido en las articulaciones y sin el margen de caída de 1 cm (en la cadena el robot
  llega ya quieto y asentado); el resto lleva `joint_noise_rad` y el margen, como en la corrida 3.
  """
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
  cache = env.extras.setdefault("_sit_end_states", {})
  if "joint_pos" not in cache:
    saved = torch.load(SIT_END_STATES)
    asset_names = list(env.scene["robot"].joint_names)
    assert saved["joint_names"] == asset_names, "el orden de articulaciones de sit_end_states.pt no coincide con el robot"
    cache.update({k: saved[k].to(env.device) for k in ("joint_pos", "root_z", "tilt_quat_wxyz")})
  asset: Entity = env.scene["robot"]
  n = len(env_ids)
  pick = torch.randint(0, cache["joint_pos"].shape[0], (n,), device=env.device)
  yaw = sample_uniform(-math.pi, math.pi, (n,), env.device)
  yaw_q = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
  quat = quat_mul(yaw_q, cache["tilt_quat_wxyz"][pick])
  pos = asset.data.default_root_state[env_ids][:, :3].clone()
  exact = torch.rand(n, device=env.device) < SIT_END_EXACT_FRACTION
  pos[:, 2] = cache["root_z"][pick] + START_DROP_MARGIN_M * (~exact).float()
  pos[:, :2] += env.scene.env_origins[env_ids][:, :2]
  noise = sample_uniform(-joint_noise_rad, joint_noise_rad, (n, cache["joint_pos"].shape[1]), env.device) * (~exact).float()[:, None]
  joint_pos = cache["joint_pos"][pick] + noise
  _ground_and_write(env, env_ids, pos, quat, joint_pos)


def lie_progress(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Cuánto ha girado el tronco hacia atrás, de 0 (recto) a 1 (boca arriba), menos la inclinación lateral para que no valga tumbarse de lado."""
  gravity = env.scene["robot"].data.projected_gravity_b
  back = (-gravity[:, 0]).clamp(0.0, 1.0)
  side = (gravity[:, 1].abs() - LYING_SIDE_MAX).clamp(min=0.0)
  return (back - side).clamp(min=0.0)


def lying_still(env: ManagerBasedRlEnv, std: float) -> torch.Tensor:
  """exp(-(velocidad lineal² + angular²) / std²) por `lie_progress`: quieto solo cobra cuando ya está tumbado."""
  data = env.scene["robot"].data
  motion = data.root_com_lin_vel_b.square().sum(dim=1) + data.root_link_ang_vel_b.square().sum(dim=1)
  return torch.exp(-motion / std**2) * lie_progress(env)


def pelvis_rate_excess(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Suma, en los tres ejes de la pelvis, del cuadrado de lo que el giro pasa de `PELVIS_RATE_TRAIN_LIMIT_RAD_S` (recortado): castiga girar deprisa en cualquier eje."""
  rate = env.scene["robot"].data.root_link_ang_vel_b.abs()
  return (rate - PELVIS_RATE_TRAIN_LIMIT_RAD_S).clamp(0.0, PELVIS_RATE_EXCESS_CLAMP_RAD_S).square().sum(dim=1)


def trunk_speed_excess(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Cuadrado de lo que la velocidad del tronco pasa de `TRUNK_SPEED_TRAIN_LIMIT_M_S` (recortado): castiga el golpe del tronco contra el suelo."""
  robot = env.scene["robot"]
  cache = env.extras.setdefault("_trunk_id", {})
  if "id" not in cache:
    cache["id"] = robot.find_bodies([TRUNK_BODY])[0][0]
  speed = robot.data.body_com_lin_vel_w[:, cache["id"]].norm(dim=1)
  return (speed - TRUNK_SPEED_TRAIN_LIMIT_M_S).clamp(0.0, TRUNK_SPEED_EXCESS_CLAMP_M_S).square()


def head_floor_contacts(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Número de contactos con el suelo de cuello y cabeza."""
  cache = env.extras.setdefault("_head_floor", {})
  model = env.sim.mj_model
  if "geoms" not in cache:
    bodies = [model.body(f"robot/{name}").id for name in HEAD_BODIES]
    cache["geoms"] = torch.tensor([i for i in range(model.ngeom) if model.geom_bodyid[i] in bodies], device=env.device)
    cache["floor"] = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=env.device)
  data = env.sim.data
  geom = data.contact.geom.long()
  active = (torch.arange(geom.shape[0], device=env.device) < data.nacon[0]) & (data.contact.dist < 0.0)
  first, second = torch.isin(geom[:, 0], cache["floor"]), torch.isin(geom[:, 1], cache["floor"])
  other = torch.where(first, geom[:, 1], geom[:, 0])
  hit = active & (first | second) & torch.isin(other, cache["geoms"])
  return torch.zeros(env.num_envs, device=env.device).index_add_(0, data.contact.worldid[hit].long(), torch.ones(int(hit.sum()), device=env.device))


class lie_metrics:
  """Recompensa sin efecto que registra, para TensorBoard: `Metrics/lie_lying_fraction` (boca arriba y quieto), `Metrics/lie_back_fraction` (tronco a más de 45° hacia atrás),
  `Metrics/lie_head_touch_fraction` (cuello o cabeza en el suelo) y `Metrics/lie_pelvis_rate_max` (mayor giro de la pelvis en un eje, media de robots)."""

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    pass

  def __call__(self, env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    data = env.scene["robot"].data
    gravity = data.projected_gravity_b
    speed = data.root_com_lin_vel_b.norm(dim=1)
    angular = data.root_link_ang_vel_b.norm(dim=1)
    lying = (gravity[:, 0] <= LYING_GRAVITY_X) & (gravity[:, 1].abs() <= LYING_SIDE_MAX) & (speed <= LYING_SPEED_M_S) & (angular <= LYING_ANGULAR_RAD_S)
    env.extras["log"]["Metrics/lie_lying_fraction"] = lying.float().mean()
    env.extras["log"]["Metrics/lie_back_fraction"] = (gravity[:, 0] <= -0.7).float().mean()
    env.extras["log"]["Metrics/lie_head_touch_fraction"] = (head_floor_contacts(env) > 0).float().mean()
    env.extras["log"]["Metrics/lie_pelvis_rate_max"] = data.root_link_ang_vel_b.abs().max(dim=1).values.mean()
    return torch.zeros(env.num_envs, device=env.device)


def danny_lie_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tarea de tumbarse. Parte de la de sentarse: sin reloj, sin orden de altura, sin empujones, empieza sentado y no termina por inclinarse."""
  cfg = sit.danny_sit_env_cfg(play=play)
  cfg.episode_length_s = 10.0 if not play else int(1e9)
  cfg.terminations.pop("fell_over", None)
  cfg.events.pop("reset_base")
  cfg.events.pop("reset_robot_joints")
  if LIE_START == "sit_end":
    cfg.events["reset_seated"] = EventTermCfg(func=reset_from_sit_end, mode="reset", params={"joint_noise_rad": SIT_END_JOINT_NOISE_RAD})
  else:
    cfg.events["reset_seated"] = EventTermCfg(func=reset_seated, mode="reset", params={"joint_noise_rad": START_JOINT_NOISE_RAD})

  pelvis = SceneEntityCfg("robot", site_names=(getup.PELVIS_SITE,))
  cfg.rewards = {
    "lie_progress": RewardTermCfg(func=lie_progress, weight=4.0),
    "lying_still": RewardTermCfg(func=lying_still, weight=2.0, params={"std": 0.3}),
    "pelvis_rate_excess": RewardTermCfg(func=pelvis_rate_excess, weight=-8.0),
    "trunk_speed_excess": RewardTermCfg(func=trunk_speed_excess, weight=-20.0),
    "head_floor_contacts": RewardTermCfg(func=head_floor_contacts, weight=-2.0),
    "action_out_of_range": RewardTermCfg(func=getup.action_out_of_range_l1, weight=-0.035),
    "dof_pos_limits": RewardTermCfg(func=velocity_mdp.joint_pos_limits, weight=-1.0),
    "action_rate_l2": RewardTermCfg(func=velocity_mdp.action_rate_l2, weight=-0.1),
    "link_pair_contacts": RewardTermCfg(func=danny_mdp.link_pair_contacts, weight=-1.0),
    "servo_torque_excess": RewardTermCfg(
      func=danny_mdp.servo_torque_excess,
      weight=-317.0,
      params={"threshold": 0.8, "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",))},
    ),
    "lie_metrics": RewardTermCfg(func=lie_metrics, weight=1e-9, params={"asset_cfg": pelvis}),
  }
  return cfg
