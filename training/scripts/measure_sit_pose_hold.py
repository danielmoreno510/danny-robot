"""Mide cuánto giro de la pelvis sale solo de que la pose sentada no se sostiene sola (fase 5, sin política).

Mismo inicio que tumbarse (los estados reales con que acaba sentarse, apoyados sin penetrar, aleatorización completa de la etapa 3, sin empujones),
pero en lugar de la política cada articulación recibe como objetivo fijo la posición con que empezó (no la pose de pie: la acción cero llevaría las
articulaciones a la pose de pie). Se registra el giro de la pelvis en sus tres ejes y se guarda en results/training_sentarse_pose_sujeta_<name>.json.

Uso (desde training/):
    uv run python scripts/measure_sit_pose_hold.py --name 6.6V --vin 6.6 --num-envs 4096
"""

from __future__ import annotations

import argparse
import json

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie, lie_measure
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

LIE_TASK = "Mjlab-LieDown-Danny"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, help="Solo para construir el entorno; la política no se usa")
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--seconds", type=float, default=3.0)
  args = parser.parse_args()

  from pathlib import Path

  cfg = eval_env_cfg(args.num_envs, args.seconds + 2.0, args.vin, task=LIE_TASK)
  env, _ = load_policy(cfg, Path(args.checkpoint), task=LIE_TASK)
  u = env.unwrapped
  n, dt = args.num_envs, u.step_dt
  robot = u.scene["robot"]
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)
  steps = int(args.seconds / dt)
  rec = lie_measure.LieRecorder(env, steps, stall)
  lie_measure.hold_fixed_pose(env, steps, rec)

  rate = rec.pelvis_rate.abs()  # (n, steps, 3)
  pitch = rate[:, :, 1]
  window = {f"first_{int(s*1000)}ms": lie_measure.quantiles(rate[:, : int(s / dt)].amax(dim=(1, 2))) for s in (0.1, 0.5, 1.0)}
  out = {
    "what": "objetivo de las articulaciones fijo en la pose sentada inicial (sin política), mismo inicio que tumbarse",
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "start": lie.LIE_START,
    "pelvis_rate_peak_rad_s": {
      "any_axis": lie_measure.quantiles(rate.amax(dim=(1, 2))),
      **{lie_measure.AXES[a]: lie_measure.quantiles(rate[:, :, a].amax(dim=1)) for a in range(3)},
    },
    "pelvis_pitch_peak_time_s_median": round(float(pitch.argmax(dim=1).float().median() * dt), 2),
    "pelvis_rate_peak_by_window_any_axis": window,
    "fraction_of_robots_with_every_axis_at_most_3_rad_s": round(float((rate.amax(dim=(1, 2)) <= lie_measure.MAX_PELVIS_RATE_RAD_S).float().mean()), 4),
    "median_abs_pitch_rate_by_step_rad_s": [round(float(pitch[:, t].median()), 3) for t in range(min(steps, 15))],
    "final": {
      "gravity_x_median": round(float(rec.gravity[:, -1, 0].median()), 3),
      "lin_speed_median_m_s": round(float(rec.speed[:, -1].median()), 3),
      "lin_speed_at_0.5s_median_m_s": round(float(rec.speed[:, int(0.5 / dt)].median()), 3),
    },
  }
  path = RESULTS / f"training_sentarse_pose_sujeta_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(out, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
