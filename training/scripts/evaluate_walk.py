"""Evalúa un checkpoint de marcha de Danny sin interfaz y guarda el resultado en results/.

Mide cuánto aguanta cada entorno sin caerse (en segundos simulados) y el error de seguimiento de la
velocidad, medida en el centro de masas de `base_link` (guía 4.2). Corre con la aleatorización del entrenamiento, sin
empujones.

Uso (desde training/):
    uv run python scripts/evaluate_walk.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name smoke_30
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
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env, policy = load_policy(eval_env_cfg(args.num_envs, args.seconds), args.checkpoint)
  device = env.unwrapped.device

  steps = int(args.seconds / env.unwrapped.step_dt)
  alive = torch.ones(args.num_envs, dtype=torch.bool, device=device)
  survival_steps = torch.zeros(args.num_envs, device=device)
  track_err = []

  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(steps):
      actions = policy(obs)
      obs, _, dones, _ = env.step(actions)
      twist = env.unwrapped.command_manager.get_command("twist")
      robot = env.unwrapped.scene["robot"]
      vel_err = (robot.data.root_com_lin_vel_b[:, :2] - twist[:, :2]).norm(dim=1)
      track_err.append(vel_err[alive].mean().item() if alive.any() else float("nan"))
      survival_steps += alive.float()
      # Solo cuenta como caída la terminación, no el fin de episodio por tiempo.
      alive &= ~env.unwrapped.reset_terminated.bool()

  survival_s = survival_steps * env.unwrapped.step_dt
  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "num_envs": args.num_envs,
    "seconds_simulated": args.seconds,
    "fraction_standing_full_episode": float(alive.float().mean()),
    "survival_s": {
      "mean": float(survival_s.mean()),
      "min": float(survival_s.min()),
      "median": float(survival_s.median()),
    },
    "mean_xy_velocity_error_m_s": float(torch.tensor([e for e in track_err if e == e]).mean()),
    "note": "Sin empujones. Resto de la aleatorización activa. Velocidad del centro de masas de base_link. Actuadores HD-1910 sin confirmar.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_walk_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  env.close()


if __name__ == "__main__":
  main()
