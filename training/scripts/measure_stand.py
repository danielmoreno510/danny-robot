"""Mide la postura de pie de Danny en mjlab y guarda el resultado en results/.

Pasos 4.1.5 de la guía: altura de pie y velocidades reales, sin heredar nada de XGO-Duck.
Deja a Danny quieto en la pose `stand` con acciones cero durante unos segundos, sin aleatorización.

Uso (desde training/):
    uv run python scripts/measure_stand.py
"""

from __future__ import annotations

import json
from pathlib import Path

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

TASK = "Mjlab-Velocity-Flat-Danny"
SECONDS = 3.0
SETTLE_S = 1.0
OUTPUT = Path(__file__).resolve().parents[2] / "results" / "training_stand_measurement.json"


def main() -> None:
  cfg = load_env_cfg(TASK)
  cfg.scene.num_envs = 1
  cfg.episode_length_s = 1e9
  cfg.events.pop("push_robot", None)
  cfg.commands["twist"].ranges.lin_vel_x = (0.0, 0.0)
  cfg.commands["twist"].ranges.lin_vel_y = (0.0, 0.0)
  cfg.commands["twist"].ranges.ang_vel_z = (0.0, 0.0)
  cfg.commands["twist"].rel_standing_envs = 1.0

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  env = ManagerBasedRlEnv(cfg=cfg, device=device)
  robot = env.scene["robot"]
  env.reset()

  dt = env.step_dt
  steps = int(SECONDS / dt)
  settle = int(SETTLE_S / dt)
  action = torch.zeros((env.num_envs, env.action_manager.total_action_dim), device=device)

  heights, com_heights, lin_speeds, joint_speeds = [], [], [], []
  with torch.inference_mode():
    for step in range(steps):
      env.step(action)
      if step < settle:
        continue
      heights.append(float(robot.data.root_link_pos_w[0, 2]))
      com_heights.append(float(robot.data.root_com_pos_w[0, 2]))
      lin_speeds.append(float(robot.data.root_link_lin_vel_w[0].norm()))
      joint_speeds.append(float(robot.data.joint_vel[0].abs().max()))

  def stats(values: list[float]) -> dict[str, float]:
    t = torch.tensor(values)
    return {"mean": float(t.mean()), "max": float(t.max()), "min": float(t.min())}

  result = {
    "task": TASK,
    "device": device,
    "control_hz": 1.0 / dt,
    "seconds_measured": SECONDS - SETTLE_S,
    "action": "cero, pose por defecto (stand)",
    "root_link_height_m": stats(heights),
    "com_height_m": stats(com_heights),
    "root_lin_speed_m_s": stats(lin_speeds),
    "max_abs_joint_speed_rad_s": stats(joint_speeds),
    "note": "Medido en simulación con actuadores HD-1910 de xgoduck_rl, sin confirmar en servos reales.",
  }
  OUTPUT.parent.mkdir(parents=True, exist_ok=True)
  OUTPUT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  env.close()


if __name__ == "__main__":
  main()
