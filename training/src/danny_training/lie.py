"""Tumbarse (fase 5): el robot empieza sentado en el suelo con las piernas estiradas y debe quedar boca arriba, quieto, sin dejarse caer de golpe.

Parte de la tarea de sentarse (`sit`): misma observación (78 y 93), sin órdenes y sin terminación por caída, porque tumbarse es inclinar el tronco.
El inicio es la pose final de la política de sentarse (`results/training_sentarse_pose_final.json`): cadera de cabeceo -91°, rodilla 7° y tobillo -32° en
las dos piernas, y el resto de articulaciones en la pose de pie, más ruido. «Boca arriba» es que el eje x del tronco apunte hacia arriba
(`projected_gravity_b[:, 0]` a -0,9 o menos) con la inclinación lateral pequeña. El ángulo de cabeceo del tronco se castiga si gira muy deprisa.
"""

from __future__ import annotations

import math

import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.utils.lab_api.math import quat_from_euler_xyz, sample_uniform

from danny_training import crouch, env_cfgs, getup, mdp as danny_mdp, sit

SEATED_JOINTS_RAD = {"hip_pitch": math.radians(-91.0), "knee": math.radians(7.0), "ankle_pitch": math.radians(-32.0)}
SEATED_PELVIS_M = 0.056
START_DROP_MARGIN_M = 0.01
START_JOINT_NOISE_RAD = 0.05
LYING_GRAVITY_X = -0.9
LYING_SIDE_MAX = 0.3
LYING_SPEED_M_S = 0.05
LYING_ANGULAR_RAD_S = 0.1
MAX_PITCH_RATE_RAD_S = 2.0


def reset_seated(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None, joint_noise_rad: float) -> None:
  """Coloca el robot sentado con las piernas estiradas (pelvis a `SEATED_PELVIS_M` más un margen), guiñada aleatoria y ruido en las articulaciones."""
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
  asset.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
  asset.write_root_link_velocity_to_sim(torch.zeros(n, 6, device=env.device), env_ids=env_ids)

  joint_pos = asset.data.default_joint_pos[env_ids].clone()
  for j, name in enumerate(asset.joint_names):
    for key, angle in SEATED_JOINTS_RAD.items():
      if name.endswith(key):
        joint_pos[:, j] = angle
  joint_pos += sample_uniform(-joint_noise_rad, joint_noise_rad, joint_pos.shape, env.device)
  limits = asset.data.soft_joint_pos_limits[env_ids]
  joint_pos = joint_pos.clamp(limits[..., 0], limits[..., 1])
  asset.write_joint_state_to_sim(joint_pos, torch.zeros_like(joint_pos), env_ids=env_ids)


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


def pitch_rate_excess(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Cuadrado de lo que el tronco gira en cabeceo más deprisa que `MAX_PITCH_RATE_RAD_S`: castiga dejarse caer de espaldas."""
  rate = env.scene["robot"].data.root_link_ang_vel_b[:, 1].abs()
  return (rate - MAX_PITCH_RATE_RAD_S).clamp(min=0.0).square()


class lie_metrics:
  """Recompensa sin efecto que registra `Metrics/lie_lying_fraction` (boca arriba, pelvis baja y quieto) y `Metrics/lie_back_fraction` (tronco a más de 45° hacia atrás)."""

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
    return torch.zeros(env.num_envs, device=env.device)


def danny_lie_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tarea de tumbarse. Parte de la de sentarse: sin reloj, sin orden de altura, sin empujones, empieza sentado y no termina por inclinarse."""
  cfg = sit.danny_sit_env_cfg(play=play)
  cfg.episode_length_s = 10.0 if not play else int(1e9)
  cfg.terminations.pop("fell_over", None)
  cfg.events.pop("reset_base")
  cfg.events.pop("reset_robot_joints")
  cfg.events["reset_seated"] = EventTermCfg(func=reset_seated, mode="reset", params={"joint_noise_rad": START_JOINT_NOISE_RAD})

  pelvis = SceneEntityCfg("robot", site_names=(getup.PELVIS_SITE,))
  cfg.rewards = {
    "lie_progress": RewardTermCfg(func=lie_progress, weight=4.0),
    "lying_still": RewardTermCfg(func=lying_still, weight=2.0, params={"std": 0.3}),
    "pitch_rate_excess": RewardTermCfg(func=pitch_rate_excess, weight=-200.0),
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
