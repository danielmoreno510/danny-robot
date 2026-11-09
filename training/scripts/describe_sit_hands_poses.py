"""Lista las poses válidas del barrido de manos apoyadas (results/training_sentarse_manos_par_estatico_edge.json): con qué borde de la suela se apoya el pie
(punta o talón), la inclinación del tronco y el par de cada articulación como fracción del bloqueo a 6,6 V. Escribe results/training_sentarse_manos_poses_validas.json.

Uso (desde training/, después de scripts/check_sit_hands.py --feet edge):
    uv run python scripts/describe_sit_hands_poses.py
"""

from __future__ import annotations

import json

import numpy as np

from danny_training.evaluation import RESULTS  # primero: evita un import circular
from danny_training.robot import DANNY_XML
from danny_training.static_pose import StaticPose

SEEDS = [(-0.9, 1.4, -0.6, 0.15, 0.5, -0.3), (-0.5, 1.0, -0.6, 0.15, 0.2, -1.0)]


def main() -> None:
  sweep = json.loads((RESULTS / "training_sentarse_manos_par_estatico_edge.json").read_text(encoding="utf-8"))
  sp = StaticPose(DANNY_XML)
  stall = sweep["stall_torque_nm"]
  rows = []
  for h in sweep["heights"]:
    for support, v in h["by_support"].items():
      for lean in v["feasible_lean_deg"]:
        best = None
        for seed in SEEDS:
          pose = sp.solve_hands(h["pelvis_z_m"], float(np.radians(lean)), float(support), seed, False)
          if pose["residual"] < 1e-3 and all(pose["in_range"].values()):
            cost = max(pose["torque_nm"].values())
            if best is None or cost < best[0]:
              best = (cost, pose)
        if best is None:
          continue
        pose = best[1]
        low = np.mean([sp._lowest_box_point([i])[0] for i in sp.soles])
        centre = np.mean([sp.data.geom_xpos[i][0] for i in sp.soles])
        rows.append({
          "pelvis_z_m": h["pelvis_z_m"], "hands_fraction_of_weight": float(support), "trunk_lean_back_deg": -float(lean),
          "foot_support": "talon" if low < centre else "punta", "lowest_sole_vertex_minus_sole_centre_cm": round(float((low - centre) * 100), 1),
          "fraction_of_stall": {n: round(t / stall, 3) for n, t in pose["torque_nm"].items()},
          "angles_deg": {n: round(float(np.degrees(a)), 1) for n, a in pose["angles_rad"].items()},
        })
        r = rows[-1]
        print(f"pelvis {r['pelvis_z_m']:.2f} m, manos {support}, tronco {r['trunk_lean_back_deg']:.0f}° atrás, {r['foot_support']}: {r['fraction_of_stall']}")
  out = RESULTS / "training_sentarse_manos_poses_validas.json"
  out.write_text(json.dumps({"vin_v": sweep["vin_v"], "stall_torque_nm": stall, "limit_fraction": sweep["limit_fraction"], "poses": rows}, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {out}")


if __name__ == "__main__":
  main()
