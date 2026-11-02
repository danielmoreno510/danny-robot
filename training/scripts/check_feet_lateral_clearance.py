"""Comprueba `mdp.feet_lateral_clearance` con posiciones de suela puestas a mano, sin simular. Sale con código 1 si algo falla.

Casos (suela izquierda, derecha, orientación del robot en el mundo):
- De pie: 64 mm entre centros, separación de 16 mm. No penaliza.
- 58 mm entre centros (10 mm de separación, el umbral). No penaliza.
- 54 mm entre centros (6 mm): (4 / 10)² = 0,16.
- Pies cruzados (la izquierda 20 mm a la derecha): hundimiento limitado a 3, vale 9.
- Una suela 100 mm por delante de la otra, con 20 mm entre centros: no se solapan, no penaliza.
- Con el robot girado 90°, el mismo caso que a 54 mm da 0,16: se mide en el marco del cuerpo.

Uso (desde training/):
    uv run python scripts/check_feet_lateral_clearance.py
"""

from __future__ import annotations

import math
import sys
from types import SimpleNamespace

import torch

from danny_training.mdp import feet_lateral_clearance

PARAMS = {"min_clearance": 0.010, "sole_width": 0.048, "sole_length": 0.085}


def main() -> None:
  # Cada caso: (x izquierda, y izquierda, x derecha, y derecha, orientación en rad, esperado). En el marco del robot, y es la izquierda.
  cases = [
    ("de pie, 16 mm", (0.0, 0.032, 0.0, -0.032, 0.0), 0.0),
    ("en el umbral, 10 mm", (0.0, 0.029, 0.0, -0.029, 0.0), 0.0),
    ("6 mm de separación", (0.0, 0.027, 0.0, -0.027, 0.0), 0.16),
    ("pies cruzados", (0.0, -0.010, 0.0, 0.010, 0.0), 9.0),
    ("sin solape longitudinal", (0.100, 0.010, 0.0, -0.010, 0.0), 0.0),
    ("robot girado 90°, 6 mm", (-0.027, 0.0, 0.027, 0.0, math.pi / 2), 0.16),
  ]
  pos = torch.zeros(len(cases), 2, 3)
  heading = torch.zeros(len(cases))
  for i, (_, (xl, yl, xr, yr, yaw), _) in enumerate(cases):
    pos[i, 0, :2] = torch.tensor([xl, yl])
    pos[i, 1, :2] = torch.tensor([xr, yr])
    heading[i] = yaw
  data = SimpleNamespace(site_pos_w=pos, heading_w=heading)
  env = SimpleNamespace(scene={"robot": SimpleNamespace(data=data)})
  asset_cfg = SimpleNamespace(name="robot", site_ids=[0, 1])

  got = feet_lateral_clearance(env, asset_cfg=asset_cfg, **PARAMS).tolist()  # type: ignore[arg-type]
  failures = [f"{name}: esperado {exp}, obtenido {g:.4f}" for (name, _, exp), g in zip(cases, got) if abs(g - exp) > 1e-3]
  if failures:
    print("FALLA:\n" + "\n".join(failures))
    sys.exit(1)
  print("OK: feet_lateral_clearance penaliza solo cuando las suelas se solapan y se acercan a menos de 10 mm, y mide en el marco del cuerpo.")


if __name__ == "__main__":
  main()
