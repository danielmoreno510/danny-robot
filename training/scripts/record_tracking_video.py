"""Graba el vídeo de un movimiento de la fase 6: Danny con física, junto al vídeo de la referencia (persona y Danny cinemático).

Un robot, tensión fija, sin empujones y sin ruido de observación, siguiendo la referencia entera. Si se da `--reference-video` (el de
`results/videos/gesture_<nombre>.mp4`, 50 Hz), se pega a su derecha, fotograma a fotograma, y el resultado queda con tres paneles:
persona | Danny de la referencia | Danny con física.

Uso (desde training/):
    uv run python scripts/record_tracking_video.py --checkpoint logs/rsl_rl/danny_tracking/<run>/model_XXXX.pt --motion data/motions/saludar.npz --name saludar --vin 6.6 --reference-video ../results/videos/gesture_cmu_141_16_lento1.8.mp4
"""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy

TASK = "Mjlab-Tracking-Danny"
MAX_BYTES = 25 * 1024 * 1024


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--motion", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--azimuth", type=float, default=180.0, help="Ángulo de la cámara; 180 mira a Danny de frente (como la referencia)")
  parser.add_argument("--reference-video", type=Path, default=None)
  args = parser.parse_args()

  frames = int(np.load(args.motion)["joint_pos"].shape[0])
  cfg = eval_env_cfg(1, frames / 50.0 + 2.0, args.vin, task=TASK)
  cfg.observations["actor"].enable_corruption = False
  motion_cfg = cfg.commands["motion"]
  motion_cfg.motion_file = str(args.motion)
  motion_cfg.sampling_mode = "start"
  motion_cfg.pose_range = {}
  motion_cfg.velocity_range = {}
  motion_cfg.joint_position_range = (0.0, 0.0)
  cfg.viewer.height, cfg.viewer.width = 540, 480
  cfg.viewer.azimuth, cfg.viewer.distance, cfg.viewer.elevation = args.azimuth, 0.8, -5.0
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array", task=TASK)
  physics = []
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(frames - 1):
      obs, _, _, _ = env.step(policy(obs))
      physics.append(env.unwrapped.render())
  env.close()

  reference = []
  if args.reference_video is not None:
    reader = imageio.get_reader(str(args.reference_video))
    reference = [f for f in reader]
    reader.close()
  out_dir = RESULTS / "videos"
  out_dir.mkdir(parents=True, exist_ok=True)
  path = out_dir / f"fase6_{args.name}_fisica.mp4"
  with imageio.get_writer(path, fps=50, macro_block_size=1) as writer:
    for i, frame in enumerate(physics):
      if reference:
        ref = reference[min(i + 1, len(reference) - 1)]
        frame = np.concatenate([ref, frame[: ref.shape[0], : , :3]], axis=1)
      writer.append_data(frame)
  size = path.stat().st_size
  print(f"Escrito: {path} ({size / 1e6:.1f} MB; límite {MAX_BYTES / 1e6:.0f} MB)")
  assert size <= MAX_BYTES


if __name__ == "__main__":
  main()
