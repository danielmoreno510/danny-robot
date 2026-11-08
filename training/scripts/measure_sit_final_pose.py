"""Guarda la pose final de la política de sentarse (fase 5): articulaciones, altura y orientación de la base, para empezar la tarea de tumbarse desde ahí.

Uso (desde training/):
    uv run python scripts/measure_sit_final_pose.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt
Guarda results/training_sentarse_pose_final.json.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy

SIT_TASK = "Mjlab-SitDown-Danny"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--num-envs", type=int, default=512)
  parser.add_argument("--seconds", type=float, default=8.0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}
  env, policy = load_policy(cfg, args.checkpoint, task=SIT_TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(int(args.seconds / u.step_dt)):
      obs, _, _, _ = env.step(policy(obs))
  data = robot.data
  pos = data.joint_pos
  out = {
    "checkpoint": str(args.checkpoint),
    "joint_names": list(robot.joint_names),
    "joint_pos_rad_median": [round(float(x), 4) for x in pos.median(dim=0).values],
    "joint_pos_rad_p10": [round(float(x), 4) for x in torch.quantile(pos, 0.1, dim=0)],
    "joint_pos_rad_p90": [round(float(x), 4) for x in torch.quantile(pos, 0.9, dim=0)],
    "root_height_m_median": round(float((data.root_link_pos_w[:, 2] - u.scene.env_origins[:, 2]).median()), 4),
    "root_quat_wxyz_median": [round(float(x), 4) for x in data.root_link_quat_w.median(dim=0).values],
  }
  path = RESULTS / "training_sentarse_pose_final.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: out[k] for k in ("root_height_m_median", "root_quat_wxyz_median")}))
  print({n: round(v * 57.2958, 1) for n, v in zip(out["joint_names"], out["joint_pos_rad_median"])})


if __name__ == "__main__":
  main()
