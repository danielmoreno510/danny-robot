"""Guarda los estados con que termina la política de sentarse, para que tumbarse empiece de la pose real y no de una pose limpia (fase 5).

Corre la política de sentarse con muchos robots (aleatorización completa de la etapa 3, tensión sorteada, sin empujones) durante `--seconds` s y, de los
que acaban sentados y quietos (pelvis a 0,07 m o menos, tronco a 35° o menos, velocidad de 0,05 m/s o menos), guarda en `training/data/sit_end_states.pt`:
posición de las articulaciones, altura de la raíz y orientación sin la guiñada (la guiñada se sortea al reiniciar). `lie.py` los usa como inicio.

Uso (desde training/):
    uv run python scripts/export_sit_end_states.py --checkpoint /d/Claude/puntos_de_control/fase5_sentarse_suave_model_3549.pt
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from mjlab.utils.lab_api.math import quat_inv, quat_mul, yaw_quat

from danny_training.evaluation import eval_env_cfg, load_policy
from danny_training import getup, lie, sit

SIT_TASK = "Mjlab-SitDown-Danny"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--seconds", type=float, default=10.0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, None, task=SIT_TASK)
  cfg.terminations = {}
  env, policy = load_policy(cfg, args.checkpoint, task=SIT_TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  pelvis_id = robot.find_sites([getup.PELVIS_SITE])[0][0]
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(int(args.seconds / u.step_dt)):
      obs, _, _, _ = env.step(policy(obs))
  data = robot.data
  z = data.site_pos_w[:, pelvis_id, 2]
  tilt = torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
  speed = data.root_com_lin_vel_b.norm(dim=1)
  seated = (z <= sit.SEATED_PELVIS_M) & (tilt <= sit.SEATED_TILT_RAD) & (speed <= sit.SEATED_SPEED_M_S)
  quat = data.root_link_quat_w
  tilt_quat = quat_mul(quat_inv(yaw_quat(quat)), quat)
  out = {
    "joint_names": list(robot.joint_names),
    "joint_pos": data.joint_pos[seated].cpu(),
    "root_z": data.root_link_pos_w[seated, 2].cpu(),
    "tilt_quat_wxyz": tilt_quat[seated].cpu(),
    "source_checkpoint": str(args.checkpoint),
  }
  lie.SIT_END_STATES.parent.mkdir(parents=True, exist_ok=True)
  torch.save(out, lie.SIT_END_STATES)
  print(f"{int(seated.sum())} de {args.num_envs} robots sentados y quietos; escrito {lie.SIT_END_STATES}")
  print("articulaciones (mediana, rad):", dict(zip(out["joint_names"], [round(float(x), 2) for x in out["joint_pos"].median(dim=0).values])))


if __name__ == "__main__":
  main()
