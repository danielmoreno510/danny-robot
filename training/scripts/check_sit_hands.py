"""Sentarse apoyando las manos detrás: altura de la pelvis a la que llegan las manos al suelo, par de hombros y codos con las manos aguantando
el 30, 40 y 50 % del peso (frente al 80 % del bloqueo a 6,6 V) y caída libre desde 0,16 m hasta el contacto de las manos.

Estático, con los pies planos o apoyados en punta o talón (`--feet`), las dos manos simétricas (balanceo de hombro 0), cajas de colisión del antebrazo como mano y el peso de las manos
repartido a partes iguales; no incluye el par dinámico del frenado. Escribe results/training_sentarse_manos_par_estatico.json.

Uso (desde training/):
    uv run python scripts/check_sit_hands.py
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from danny_training.evaluation import RESULTS  # primero: evita un import circular
from danny_training.actuator import stall_torque_nm
from danny_training.robot import DANNY_XML
from danny_training.static_pose import ARM_JOINTS, LEG_JOINTS, StaticPose

VIN_V = 6.6
LIMIT_FRACTION = 0.8
GRAVITY = 9.81
SEEDS = [(-0.9, 1.4, -0.6, 0.15, 0.5, -0.3), (-0.5, 1.0, -0.6, 0.15, 0.2, -1.0)]


def best_pose(sp: StaticPose, z: float, lean: float, support: float, flat: bool):
  best = None
  for seed in SEEDS:
    pose = sp.solve_hands(z, lean, support, seed, flat)
    if pose["residual"] < 1e-3 and all(pose["in_range"].values()):
      cost = max(pose["torque_nm"].values())
      if best is None or cost < best[0]:
        best = (cost, pose)
  return best[1] if best else None


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--heights", type=float, nargs="+", default=[round(0.20 - 0.01 * i, 2) for i in range(15)])
  parser.add_argument("--feet", choices=("flat", "edge"), default="edge", help="flat: pies planos; edge: pie apoyado en punta o talón")
  parser.add_argument("--supports", type=float, nargs="+", default=[0.3, 0.4, 0.5])
  args = parser.parse_args()
  stall = float(stall_torque_nm(VIN_V))
  sp = StaticPose(DANNY_XML)
  rows = []
  for z in args.heights:
    entry = {"pelvis_z_m": z, "by_support": {}}
    for support in args.supports:
      feasible = []
      for lean_deg in np.arange(-80.0, 1.0, 5.0):
        pose = best_pose(sp, z, float(np.radians(lean_deg)), support, args.feet == "flat")
        if pose is None:
          continue
        frac = {n: round(v / stall, 3) for n, v in pose["torque_nm"].items()}
        feasible.append({"lean_deg": float(lean_deg), "angles_deg": {n: round(float(np.degrees(v)), 1) for n, v in pose["angles_rad"].items()},
                         "fraction_of_stall": frac, "max_fraction_of_stall": max(frac.values()), "hand_x_m": round(pose["hand_x_m"], 3),
                         "foot_x_m": round(pose["foot_x_m"], 3), "com_x_m": round(pose["com_x_m"], 3)})
      best = min(feasible, key=lambda r: r["max_fraction_of_stall"]) if feasible else None
      entry["by_support"][str(support)] = {"feasible_lean_deg": [f["lean_deg"] for f in feasible], "best": best,
                                           "within_80_percent": bool(best and best["max_fraction_of_stall"] <= LIMIT_FRACTION)}
      print(f"pelvis {z:.2f} m, manos {support*100:.0f} %: " + (f"{len(feasible)} inclinaciones posibles; mejor {best['lean_deg']:.0f}°, par máx {best['max_fraction_of_stall']*100:.0f} % del bloqueo "
            f"{best['fraction_of_stall']}, ángulos {best['angles_deg']}" if best else "ninguna pose posible"))
    rows.append(entry)
  reach = [r["pelvis_z_m"] for r in rows if any(v["best"] for v in r["by_support"].values())]
  top = max(reach) if reach else None
  fall = {}
  if top is not None:
    for start in (0.16,):
      drop = start - top
      fall[str(start)] = {"drop_m": round(drop, 3), "free_fall_speed_m_s": round(float(np.sqrt(2 * GRAVITY * max(drop, 0.0))), 2),
                          "drop_for_0.4_m_s_free_fall_m": round(0.4**2 / (2 * GRAVITY), 4)}
  out = {"feet": args.feet, "vin_v": VIN_V, "stall_torque_nm": round(stall, 4), "limit_fraction": LIMIT_FRACTION, "highest_pelvis_z_with_hands_on_floor_m": top, "fall_from_start": fall, "heights": rows}
  path = RESULTS / f"training_sentarse_manos_par_estatico_{args.feet}.json"
  path.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
  print("Altura más alta de la pelvis con las manos en el suelo:", top, "| caída desde 0,16 m:", fall)
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
