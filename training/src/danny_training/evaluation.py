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

# Par de bloqueo del servo con el límite de PWM de 0,97, medido por tensión (N·m): 0,739 a 6 V y 0,912 a 7,4 V. Crece con la tensión.
STALL_TORQUE_6V_NM = 0.739
STALL_TORQUE_7P4V_NM = 0.912


def stall_torque_nm(vin: float) -> float:
  """Par de bloqueo a `vin` voltios, por la recta de los dos puntos medidos (a 8,4 V es una extrapolación)."""
  return STALL_TORQUE_6V_NM + (vin - 6.0) * (STALL_TORQUE_7P4V_NM - STALL_TORQUE_6V_NM) / (7.4 - 6.0)


def eval_env_cfg(num_envs: int, seconds: float, vin: float | None = None, pushes: bool = False) -> ManagerBasedRlEnvCfg:
  """Configuración de entrenamiento sin empujones: se mide la marcha, no la recuperación.

  Con `vin`, todos los servos trabajan a esa tensión fija (V), sin caída de tensión por la carga. Sin ella, la tensión se
  sortea entre 6 y 8,4 V como en el entrenamiento. Con `pushes`, se mantienen los empujones del entrenamiento.
  """
  env_cfg = load_env_cfg(TASK)
  env_cfg.scene.num_envs = num_envs
  env_cfg.episode_length_s = seconds
  if not pushes:
    env_cfg.events.pop("push_robot", None)
  if vin is not None:
    actuator = env_cfg.scene.entities["robot"].articulation.actuators[0]
    actuator.vin_range = (vin, vin)
    actuator.vin_drop_gain_range = (0.0, 0.0)
  return env_cfg


def fix_command(env_cfg: ManagerBasedRlEnvCfg, forward: float, turn: float = 0.0, lateral: float = 0.0) -> None:
  """Fija la orden en `forward` m/s hacia delante, `lateral` m/s hacia la izquierda y `turn` rad/s de giro (positivo a la izquierda).

  Sin entornos parados ni entornos de solo avance. Con las tres a 0, todos los entornos reciben orden cero.
  """
  twist = env_cfg.commands["twist"]
  twist.ranges = replace(twist.ranges, lin_vel_x=(forward, forward), lin_vel_y=(lateral, lateral), ang_vel_z=(turn, turn), heading=None)
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
