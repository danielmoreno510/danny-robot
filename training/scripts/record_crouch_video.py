"""Graba un vídeo de Danny agachándose quieto con la política de la fase 3: la orden de altura baja de 0 a -0,05 m, se mantiene y vuelve a 0.

Un robot, orden de velocidad cero, tensión fija y sin empujones. La orden de altura sigue una rampa: 0 hasta los 2 s, baja hasta -0,05 m a
los 6 s, se mantiene hasta los 9 s, sube a 0 a los 11 s y sigue en 0 hasta los 13 s. Guarda results/videos/<name>_agacharse.mp4 e imprime la
altura de la pelvis cada segundo.

Uso (desde training/):
    uv run python scripts/record_crouch_video.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase3_agacharse --vin 6.6
"""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, fix_command, load_policy

RAMP_S = ((0.0, 0.0), (2.0, 0.0), (6.0, -0.05), (9.0, -0.05), (11.0, 0.0), (13.0, 0.0))
MAX_BYTES = 25 * 1024 * 1024


def command_at(t: float) -> float:
  for (t0, v0), (t1, v1) in zip(RAMP_S, RAMP_S[1:]):
    if t <= t1:
      return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
  return RAMP_S[-1][1]


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Prefijo del vídeo en results/videos/")
  parser.add_argument("--vin", type=float, default=6.6, help="Tensión fija de los servos en V")
  args = parser.parse_args()

  seconds = RAMP_S[-1][0]
  cfg = eval_env_cfg(1, seconds + 1.0, args.vin)
  fix_command(cfg, 0.0)
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array")
  u = env.unwrapped
  height_term = u.command_manager.get_term("height")
  robot = u.scene["robot"]
  pelvis = robot.find_sites(["pelvis_bottom"])[0][0]
  out_dir = RESULTS / "videos"
  out_dir.mkdir(parents=True, exist_ok=True)
  path = out_dir / f"{args.name}_agacharse.mp4"
  obs = env.get_observations()
  fps = int(round(u.metadata["render_fps"]))
  with imageio.get_writer(path, fps=fps, macro_block_size=1) as writer, torch.inference_mode():
    for i in range(int(seconds / u.step_dt)):
      t = i * u.step_dt
      height_term.height[:] = command_at(t)
      obs = env.get_observations() if i == 0 else obs
      obs, _, _, _ = env.step(policy(obs))
      writer.append_data(u.render())
      if i % int(1.0 / u.step_dt) == 0:
        print(f"t={t:4.1f} s  orden {command_at(t):+.3f} m  pelvis {float(robot.data.site_pos_w[0, pelvis, 2]):.3f} m", flush=True)
  env.close()
  size = path.stat().st_size
  print(f"{path.name}: {size / 1e6:.2f} MB {'OK' if size < MAX_BYTES else 'DEMASIADO GRANDE'}")


if __name__ == "__main__":
  main()
