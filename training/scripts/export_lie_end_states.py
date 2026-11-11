"""Guarda los estados con que termina la cadena sentarse + tumbarse, para medir levantarse desde ahí (fase 5).

Corre en el entorno de sentarse la política de sentarse (`--sit-seconds` s) y, sin reiniciar, la de tumbarse (`--lie-seconds` s), con la aleatorización completa de
la etapa 3 y sin empujones. De los robots que acaban tumbados y quietos (mismos umbrales que `lie.py`: eje x del tronco hacia arriba, inclinación lateral, velocidades)
guarda posición de las articulaciones, altura de la raíz y orientación sin la guiñada en `--output`. `measure_getup.py --lie-end-states` los usa como inicio.

Uso (desde training/):
    uv run python scripts/export_lie_end_states.py --sit-checkpoint <sentarse>.pt --lie-checkpoint <tumbarse>.pt --output <estados>.pt [--vin 6.6]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from mjlab.utils.lab_api.math import quat_inv, quat_mul, yaw_quat

from danny_training import lie
from danny_training.evaluation import eval_env_cfg, load_policy, load_policy_on

SIT_TASK, LIE_TASK = "Mjlab-SitDown-Danny", "Mjlab-LieDown-Danny"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--sit-checkpoint", required=True, type=Path)
  parser.add_argument("--lie-checkpoint", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--sit-seconds", type=float, default=10.0)
  parser.add_argument("--lie-seconds", type=float, default=10.0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.sit_seconds + args.lie_seconds + 2.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}
  env, sit_policy = load_policy(cfg, args.sit_checkpoint, task=SIT_TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(int(args.sit_seconds / u.step_dt)):
      obs, _, _, _ = env.step(sit_policy(obs))
    lie_policy = load_policy_on(env, args.lie_checkpoint, LIE_TASK)
    for _ in range(int(args.lie_seconds / u.step_dt)):
      obs, _, _, _ = env.step(lie_policy(obs))
  data = robot.data
  gravity = data.projected_gravity_b
  lying = (
    (gravity[:, 0] <= lie.LYING_GRAVITY_X)
    & (gravity[:, 1].abs() <= lie.LYING_SIDE_MAX)
    & (data.root_com_lin_vel_b.norm(dim=1) <= lie.LYING_SPEED_M_S)
    & (data.root_link_ang_vel_b.norm(dim=1) <= lie.LYING_ANGULAR_RAD_S)
  )
  quat = data.root_link_quat_w
  tilt_quat = quat_mul(quat_inv(yaw_quat(quat)), quat)
  out = {
    "joint_names": list(robot.joint_names),
    "joint_pos": data.joint_pos[lying].cpu(),
    "root_z": data.root_link_pos_w[lying, 2].cpu(),
    "tilt_quat_wxyz": tilt_quat[lying].cpu(),
    "sit_checkpoint": str(args.sit_checkpoint),
    "lie_checkpoint": str(args.lie_checkpoint),
  }
  args.output.parent.mkdir(parents=True, exist_ok=True)
  torch.save(out, args.output)
  print(f"{int(lying.sum())} de {args.num_envs} robots tumbados y quietos; escrito {args.output}")


if __name__ == "__main__":
  main()
