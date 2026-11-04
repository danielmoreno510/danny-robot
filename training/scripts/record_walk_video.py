"""Graba vídeos cortos de la política en simulación: un robot por movimiento, con la tensión fija pedida y sin empujones.

Un MP4 por movimiento (adelante, atrás, lateral izquierda, giro izquierda y quieto) en `results/videos/`. Cada uno dura 8 s y pesa
bastante menos de 25 MB, el límite de la guía.

Uso (desde training/):
    uv run python scripts/record_walk_video.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase2_f_rodilla --vin 6.6
"""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, fix_command, load_policy

MOVEMENTS = (
  ("adelante", 0.25, 0.0, 0.0),
  ("atras", -0.25, 0.0, 0.0),
  ("lateral_izquierda", 0.0, 0.1, 0.0),
  ("giro_izquierda", 0.0, 0.0, 0.5),
  ("quieto", 0.0, 0.0, 0.0),
)
MAX_BYTES = 25 * 1024 * 1024


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Prefijo de los vídeos en results/videos/")
  parser.add_argument("--vin", type=float, default=7.4, help="Tensión fija de los servos en V")
  parser.add_argument("--seconds", type=float, default=8.0)
  args = parser.parse_args()

  out_dir = RESULTS / "videos"
  out_dir.mkdir(parents=True, exist_ok=True)
  for name, forward, lateral, turn in MOVEMENTS:
    cfg = eval_env_cfg(1, args.seconds + 1.0, args.vin)
    fix_command(cfg, forward, turn, lateral)
    env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array")
    path = out_dir / f"{args.name}_{name}.mp4"
    obs = env.get_observations()
    with imageio.get_writer(path, fps=int(round(env.unwrapped.metadata["render_fps"])), macro_block_size=1) as writer, torch.inference_mode():
      for _ in range(int(args.seconds / env.unwrapped.step_dt)):
        obs, _, _, _ = env.step(policy(obs))
        writer.append_data(env.unwrapped.render())
    env.close()
    size = path.stat().st_size
    print(f"{path.name}: {size / 1e6:.2f} MB {'OK' if size < MAX_BYTES else 'DEMASIADO GRANDE'}", flush=True)


if __name__ == "__main__":
  main()
