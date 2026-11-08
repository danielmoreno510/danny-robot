"""Par estático de rodillas, caderas y tobillos para sostener a Danny en cuclillas con la pelvis a varias alturas, frente al 80 % del bloqueo.

Para cada altura de la pelvis (`pelvis_bottom`) barre la inclinación del tronco (pies planos, centro de masas sobre las suelas, peso repartido
por igual) y guarda en results/training_cuclillas_par_estatico.json, para cada altura: si hay alguna pose dentro del rango de las articulaciones,
y la inclinación que minimiza el mayor porcentaje de par (cadera, rodilla o tobillo) frente al bloqueo y frente al 80 % del bloqueo, a 6,6 V.
Es estático y con los pies planos: no incluye apoyar las manos ni el par dinámico.

Uso (desde training/):
    uv run python scripts/check_squat_torque.py
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from danny_training.evaluation import RESULTS  # primero: evita un import circular
from danny_training.actuator import stall_torque_nm
from danny_training.robot import DANNY_XML
from danny_training.static_pose import LEG_JOINTS, StaticPose

VIN_V = 6.6
LIMIT_FRACTION = 0.8


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--heights", type=float, nargs="+", default=[0.228, 0.20, 0.15, 0.10, 0.07], help="Altura de la pelvis en m")
  parser.add_argument("--lean-max-deg", type=float, default=90.0)
  args = parser.parse_args()
  stall = float(stall_torque_nm(VIN_V))
  sp = StaticPose(DANNY_XML)
  results = []
  for z in args.heights:
    rows, guess = [], (-0.9, 1.4, -0.6, 0.15)
    for lean_deg in np.arange(0.0, args.lean_max_deg + 1e-9, 5.0):
      pose = sp.solve(z, float(np.radians(lean_deg)), guess)
      good = pose["residual"] < 1e-3 and all(pose["in_range"].values())
      row = {
        "lean_deg": float(lean_deg),
        "converged": pose["residual"] < 1e-3,
        "in_range": pose["in_range"],
        "angles_deg": {n: round(float(np.degrees(v)), 1) for n, v in pose["angles_rad"].items()},
        "torque_nm": {n: round(v, 3) for n, v in pose["torque_nm"].items()},
        "fraction_of_stall": {n: round(v / stall, 3) for n, v in pose["torque_nm"].items()},
        "max_fraction_of_stall": round(max(pose["torque_nm"].values()) / stall, 3),
        "feasible": bool(good),
      }
      rows.append(row)
    ok = [r for r in rows if r["feasible"]]
    best = min(ok, key=lambda r: r["max_fraction_of_stall"]) if ok else None
    results.append({
      "pelvis_z_m": z,
      "any_feasible_pose": bool(ok),
      "best_lean": best,
      "best_max_fraction_of_stall": best["max_fraction_of_stall"] if best else None,
      "within_80_percent": bool(best and best["max_fraction_of_stall"] <= LIMIT_FRACTION),
      "sweep": rows,
    })
    if best:
      print(f"pelvis {z:.3f} m: mejor inclinación {best['lean_deg']:.0f}°, par máx {best['max_fraction_of_stall']*100:.0f} % del bloqueo "
            f"(rodilla {best['fraction_of_stall']['knee']*100:.0f} %, cadera {best['fraction_of_stall']['hip_pitch']*100:.0f} %, tobillo {best['fraction_of_stall']['ankle_pitch']*100:.0f} %); ángulos {best['angles_deg']}")
    else:
      print(f"pelvis {z:.3f} m: ninguna pose con pies planos dentro del rango de las articulaciones")
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / "training_cuclillas_par_estatico.json"
  out.write_text(json.dumps({"vin_v": VIN_V, "stall_torque_nm": round(stall, 4), "limit_fraction": LIMIT_FRACTION, "heights": results}, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {out}")


if __name__ == "__main__":
  main()
