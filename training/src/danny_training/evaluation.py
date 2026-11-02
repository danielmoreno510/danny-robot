"""Carga un punto de control de marcha en un entorno de evaluación sin interfaz. Lo usan los scripts de medida."""

from __future__ import annotations

from dataclasses import asdict, replace
from pathlib import Path

import torch

from mjlab.envs import ManagerBasedRlEnvCfg, ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

TASK = "Mjlab-Velocity-Flat-Danny"
RESULTS = Path(__file__).resolve().parents[3] / "results"


def eval_env_cfg(num_envs: int, seconds: float) -> ManagerBasedRlEnvCfg:
  """Configuración de entrenamiento sin empujones: se mide la marcha, no la recuperación."""
  env_cfg = load_env_cfg(TASK)
  env_cfg.scene.num_envs = num_envs
  env_cfg.episode_length_s = seconds
  env_cfg.events.pop("push_robot", None)
  return env_cfg


def fix_command(env_cfg: ManagerBasedRlEnvCfg, forward: float, turn: float = 0.0) -> None:
  """Fija la orden en `forward` m/s hacia delante y `turn` rad/s de giro (positivo a la izquierda), sin lateral, sin entornos parados y sin entornos de solo avance."""
  twist = env_cfg.commands["twist"]
  twist.ranges = replace(twist.ranges, lin_vel_x=(forward, forward), lin_vel_y=(0.0, 0.0), ang_vel_z=(turn, turn), heading=None)
  twist.heading_command = False
  twist.rel_standing_envs = 0.0
  twist.rel_forward_envs = 0.0  # mjlab fuerza el 20 % de los entornos a (0,3; 0; 0), sin giro.


def load_policy(env_cfg: ManagerBasedRlEnvCfg, checkpoint: Path):
  """Crea el entorno y devuelve (entorno envuelto, política determinista)."""
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  agent_cfg = load_rl_cfg(TASK)
  env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg=env_cfg, device=device), clip_actions=agent_cfg.clip_actions)
  runner_cls = load_runner_cls(TASK) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=device)
  runner.load(str(checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
  return env, runner.get_inference_policy(device=device)
