"""Mide la altura máxima que alcanzan las suelas de Danny durante un episodio, y la guarda en results/.

Para cada entorno toma la altura máxima de cada suela sobre el suelo en toda la simulación, y promedia
esas alturas. La guía pide 15 mm o más a las 1 000 iteraciones como señal de que los pies se levantan.

Uso (desde training/):
    uv run python scripts/measure_foot_peak_height.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name marcha_guia_1000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, TASK, eval_env_cfg, load_policy


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija de los servos en V; sin ella, la del entrenamiento (6 a 8,4)")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env, policy = load_policy(eval_env_cfg(args.num_envs, args.seconds, args.vin), args.checkpoint)
  device = env.unwrapped.device

  u = env.unwrapped
  robot = u.scene["robot"]
  sole_idx = [robot.site_names.index(s) for s in ("left_sole", "right_sole")]
  steps = int(args.seconds / u.step_dt)
  obs = env.get_observations()
  peak = torch.full((args.num_envs,), -1e9, device=device)
  with torch.inference_mode():
    for _ in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      peak = torch.maximum(peak, robot.data.site_pos_w[:, sole_idx, 2].amax(dim=1))

  ground = float(robot.data.site_pos_w[:, sole_idx, 2].amin())
  peak_mm = ((peak - ground) * 1000.0).clamp(min=0.0)
  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "num_envs": args.num_envs,
    "seconds_simulated": args.seconds,
    "vin_v": args.vin,
    "peak_height_mean_mm": round(float(peak_mm.mean()), 2),
    "peak_height_median_mm": round(float(peak_mm.median()), 2),
    "note": "Sin empujones. Altura de la suela sobre el suelo, máximo por entorno, promedio de entornos.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_peak_height_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {out}")
  env.close()


if __name__ == "__main__":
  main()
