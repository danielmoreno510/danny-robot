"""Evalúa cada movimiento de Danny por separado con una orden fija y dice si cumple el criterio de la fase 2.

Siete órdenes: adelante 0,25 m/s, atrás 0,25 m/s, paso lateral de 0,1 m/s a cada lado, giro en el sitio de 0,5 rad/s a cada lado y
orden cero. Cada una con la aleatorización completa del entrenamiento y sin empujones, a la tensión pedida (por defecto 7,4 V).

Cumple un movimiento si el 95 % o más de los robots no cae en 20 s y, entre los que no caen, el error de velocidad es de 0,08 m/s
o menos y el de giro de 0,15 rad/s o menos. Con orden cero, además, la velocidad media es de 0,03 m/s o menos y el giro medio de
0,10 rad/s o menos (se queda quieto). El paso alterno se mide pero no decide.

Uso (desde training/):
    uv run python scripts/evaluate_movements.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase2_3000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from danny_training.evaluation import RESULTS
from danny_training.gait_measure import measure_gait

NOT_FALLEN_MIN = 0.95
XY_ERROR_MAX_M_S = 0.08
YAW_ERROR_MAX_RAD_S = 0.15
STAND_SPEED_MAX_M_S = 0.03
STAND_YAW_RATE_MAX_RAD_S = 0.10

# nombre, avance (m/s), lateral (m/s, positivo a la izquierda), giro (rad/s, positivo a la izquierda)
MOVEMENTS = (
  ("adelante", 0.25, 0.0, 0.0),
  ("atras", -0.25, 0.0, 0.0),
  ("lateral_izquierda", 0.0, 0.1, 0.0),
  ("lateral_derecha", 0.0, -0.1, 0.0),
  ("giro_izquierda", 0.0, 0.0, 0.5),
  ("giro_derecha", 0.0, 0.0, -0.5),
  ("quieto", 0.0, 0.0, 0.0),
)


def passes(name: str, r: dict) -> bool:
  if r["fraction_envs_not_fallen"] < NOT_FALLEN_MIN or r["mean_xy_velocity_error_m_s"] is None:
    return False
  ok = r["mean_xy_velocity_error_m_s"] <= XY_ERROR_MAX_M_S and r["mean_yaw_rate_error_rad_s"] <= YAW_ERROR_MAX_RAD_S
  if name == "quieto":
    ok = ok and r["mean_xy_speed_m_s"] <= STAND_SPEED_MAX_M_S and r["mean_abs_yaw_rate_rad_s"] <= STAND_YAW_RATE_MAX_RAD_S
  return ok


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--height", type=float, default=0.0, help="Bajada de la pelvis en m (≤ 0)")
  parser.add_argument("--vin", type=float, default=7.4, help="Tensión fija de los servos en V")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  results = {}
  for name, forward, lateral, turn in MOVEMENTS:
    r = measure_gait(args.checkpoint, forward=forward, lateral=lateral, turn=turn, height=args.height, vin=args.vin, num_envs=args.num_envs, seconds=args.seconds)
    r["passes"] = passes(name, r)
    results[name] = r
    print(
      f"{name:18s} sin caer {r['fraction_envs_not_fallen']:.3f}  error lineal {r['mean_xy_velocity_error_m_s']}  error de giro {r['mean_yaw_rate_error_rad_s']}"
      f"  giro medio {r['mean_yaw_rate_rad_s']}  giro |medio| {r['mean_abs_yaw_rate_rad_s']}  alterno {r['fraction_alternating_touchdowns']}  -> {'CUMPLE' if r['passes'] else 'NO CUMPLE'}"
    )

  summary = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin,
    "height_command_m": args.height,
    "criteria": {
      "fraction_not_fallen_min": NOT_FALLEN_MIN,
      "xy_velocity_error_max_m_s": XY_ERROR_MAX_M_S,
      "yaw_rate_error_max_rad_s": YAW_ERROR_MAX_RAD_S,
      "stand_speed_max_m_s": STAND_SPEED_MAX_M_S,
      "stand_yaw_rate_max_rad_s": STAND_YAW_RATE_MAX_RAD_S,
    },
    "all_movements_pass": all(r["passes"] for r in results.values()),
    "movements": results,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_movements_{args.name}.json"
  out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
  print(f"\nTodos los movimientos cumplen: {summary['all_movements_pass']}. Escrito: {out}")


if __name__ == "__main__":
  main()
