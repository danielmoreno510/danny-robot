"""Mide cuánto sale la acción cruda de una política del rango de cada articulación, antes del recorte, sin entrenar.

Corre `--num-envs` robots `--seconds` s a 6,6 V y, con escala 1 (objetivo = defecto + acción), compara el objetivo sin recorte con el rango de
`getup.soft_limit_clip()` (límite del MJCF por 0,9). Por articulación guarda, sobre los últimos `--last-seconds` s: fracción del tiempo fuera
de rango, exceso medio y máximo en rad (L1: lo que sale del rango) y acción cruda media. Sirve para la tarea de levantarse (la política puede
quedar saturada contra el recorte: corridas 1 a 6) y para las de marcha y agacharse de las fases 2 y 3. En el robot real, un servo apretado
contra un tope se calienta.

Uso (desde training/):
    uv run python scripts/measure_getup_raw_actions.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase4_pose_sin_umbral
    uv run python scripts/measure_getup_raw_actions.py --checkpoint ../puntos_de_control/... --name fase2 --task Mjlab-Velocity-Flat-Danny
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, TASK, eval_env_cfg, load_policy
from danny_training import getup


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--task", default="Mjlab-GetUp-Danny")
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--num-envs", type=int, default=128)
  parser.add_argument("--seconds", type=float, default=20.0)
  parser.add_argument("--last-seconds", type=float, default=10.0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=args.task)
  env, policy = load_policy(cfg, args.checkpoint, task=args.task)
  u = env.unwrapped
  robot = u.scene["robot"]
  names = list(robot.joint_names)
  clip = getup.soft_limit_clip()
  low = torch.tensor([clip[n][0] for n in names], device=u.device)
  high = torch.tensor([clip[n][1] for n in names], device=u.device)
  default = robot.data.default_joint_pos
  n = 0
  steps, last = int(args.seconds / u.step_dt), int(args.last_seconds / u.step_dt)
  raw_sum, excess_sum, outside, excess_max = (torch.zeros(args.num_envs, len(names), device=u.device) for _ in range(4))
  total_excess = []
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      action = policy(obs)
      if step >= steps - last:
        target = robot.data.default_joint_pos + action
        excess = (target - high).clamp(min=0.0) + (low - target).clamp(min=0.0)
        raw_sum += action
        excess_sum += excess
        outside += (excess > 0).float()
        excess_max = torch.maximum(excess_max, excess)
        total_excess.append(excess.sum(dim=1).mean())
        n += 1
      obs, _, _, _ = env.step(action)
  per_joint = {}
  for j, name in enumerate(names):
    per_joint[name] = {
      "range_deg": [round(math.degrees(float(low[j])), 1), round(math.degrees(float(high[j])), 1)],
      "raw_action_rad_mean": round(float((raw_sum[:, j] / n).mean()), 3),
      "fraction_of_time_outside_range": round(float((outside[:, j] / n).mean()), 3),
      "excess_rad_mean": round(float((excess_sum[:, j] / n).mean()), 3),
      "excess_deg_max": round(math.degrees(float(excess_max[:, j].max())), 1),
    }
  out = {
    "checkpoint": str(args.checkpoint),
    "task": args.task,
    "vin_v": args.vin,
    "num_envs": args.num_envs,
    "window_s": [args.seconds - args.last_seconds, args.seconds],
    "total_excess_rad_sum_over_joints_mean": round(float(torch.stack(total_excess).mean()), 3),
    "joints_outside_range_more_than_half_the_time": [k for k, v in per_joint.items() if v["fraction_of_time_outside_range"] > 0.5],
    "by_joint": per_joint,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  path = RESULTS / f"training_acciones_crudas_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print({k: out[k] for k in ("total_excess_rad_sum_over_joints_mean", "joints_outside_range_more_than_half_the_time")})
  print(f"Escrito: {path}")
  env.close()


if __name__ == "__main__":
  main()
