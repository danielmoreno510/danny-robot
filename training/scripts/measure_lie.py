"""Mide la tarea de tumbarse (fase 5) con el criterio del 8 de noviembre y guarda el resultado en results/.

Cada robot empieza sentado con las piernas estiradas, apoyado sin penetrar en el suelo (reinicio de `lie.py`), con la aleatorización completa de la
etapa 3 y sin empujones. Antes de contar se sujetan las articulaciones `lie.SETTLE_STEPS` pasos para que empiece quieto. Criterio y definiciones en
`danny_training/lie_measure.py` y PROJECT_HANDOFF.md (Fase 5, tumbarse): queda tumbado, tronco a 0,8 m/s o menos al tocar el suelo, cabeza sin tocar,
giro de la pelvis de 3 rad/s o menos en todos los ejes, y saturación de par. Con `--videos N` graba los N robots con el mayor giro de la pelvis.

Uso (desde training/):
    uv run python scripts/measure_lie.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase5_tumbarse_suave_6.6V --vin 6.6 --num-envs 4096
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie, lie_measure
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

LIE_TASK = "Mjlab-LieDown-Danny"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija en V; sin ella, la sorteada del entrenamiento (6 a 8,4)")
  parser.add_argument("--num-envs", type=int, default=1024)
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--settle-steps", type=int, default=0 if lie.LIE_START == "sit_end" else lie.SETTLE_STEPS)
  parser.add_argument("--videos", type=int, default=0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 2.0, args.vin, task=LIE_TASK)
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array" if args.videos else None, task=LIE_TASK)
  u = env.unwrapped
  n, dt = args.num_envs, u.step_dt
  robot = u.scene["robot"]
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)
  steps = int(args.seconds / dt)

  min_dist_step0 = lie.lowest_floor_distance(u)
  obs = lie_measure.settle(env, args.settle_steps)
  settle_speed = robot.data.root_com_lin_vel_b.norm(dim=1)
  settle_gx = robot.data.projected_gravity_b[:, 0]
  rec = lie_measure.LieRecorder(env, steps, stall)
  trajectory = torch.zeros(n, steps, u.sim.data.qpos.shape[1], device=u.device) if args.videos else None
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      rec.record(step)
      if trajectory is not None:
        trajectory[:, step] = u.sim.data.qpos

  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "reset": {
      "robots_penetrating_floor_at_step_0": int((min_dist_step0 < 0).sum()),
      "min_contact_distance_step_0_m": round(float(min_dist_step0.min()), 4) if bool((min_dist_step0 < 0.5).any()) else "sin contactos (apoyado a 1 mm)",
      "settle_steps": args.settle_steps,
      "speed_after_settle_m_s": {"median": round(float(settle_speed.median()), 4), "max": round(float(settle_speed.max()), 4)},
      "gravity_x_after_settle_median": round(float(settle_gx.median()), 3),
    },
    **lie_measure.lie_report(rec, dt),
  }
  worst = torch.argsort(rec.pelvis_rate.abs().amax(dim=(1, 2)), descending=True)[: max(args.videos, 3)].tolist()
  out["highest_pelvis_rate_robots"] = [
    {"robot": i, "peak_rad_s": round(float(rec.pelvis_rate[i].abs().max()), 2), "time_s": round(float(rec.pelvis_rate[i].abs().amax(dim=1).argmax()) * dt, 2),
     "axis": lie_measure.AXES[int(rec.pelvis_rate[i].abs().amax(dim=0).argmax())]}
    for i in worst
  ]
  if args.videos:
    videos = RESULTS / "videos"
    videos.mkdir(parents=True, exist_ok=True)
    out["videos"] = []
    for row in out["highest_pelvis_rate_robots"][: args.videos]:
      path = videos / f"{args.name}_robot{row['robot']}_giro{row['peak_rad_s']}_t{row['time_s']}s_{row['axis']}.mp4"
      with imageio.get_writer(path, fps=int(round(u.metadata["render_fps"])), macro_block_size=1) as writer:
        for t in range(steps):
          u.sim.data.qpos[0] = trajectory[row["robot"], t]
          u.sim.data.qvel[0] = 0.0
          u.sim.forward()
          writer.append_data(u.render())
      out["videos"].append(path.name)
  path = RESULTS / f"training_tumbarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items()}, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
