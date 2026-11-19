"""Muestra cómo se detectan los apoyos de las suelas en una referencia (sin simulador).

Para cada pie imprime una línea con un carácter por fotograma (`S` apoyo, `.` vuelo, `x` apoyo descartado por corto o recortado),
la lista de apoyos medibles con su recorrido de referencia, y cuántos fotogramas habría marcado como apoyo la regla antigua
(solo altura a menos de 5 mm), para ver en qué fotogramas la nueva regla corrige.

Uso (desde training/):
    uv run python scripts/show_reference_stances.py --motion data/motions/andar_07_01.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from danny_training import reference_contacts as rc


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--motion", required=True, type=Path)
  args = parser.parse_args()
  data = np.load(args.motion)
  fps = float(data["fps"][0])
  soles = rc.reference_soles(args.motion)
  print(f"{args.motion.name}: {len(data['body_pos_w'])} fotogramas a {fps:.0f} Hz")
  print(f"Reglas: altura < {rc.HEIGHT_BAND_M * 100:.1f} cm sobre el mínimo, velocidad xy y z < {rc.MAX_SPEED_M_S * 100:.0f} cm/s, "
        f"mínimo {rc.MIN_STANCE_FRAMES} fotogramas, recorte {rc.ERODE_FRAMES} por extremo")
  for side in rc.SIDES:
    sole = soles[side]
    mask = rc.stance_mask(sole, fps)
    old = sole[:, 2] - sole[:, 2].min() < 0.005
    measured = np.zeros(len(sole), dtype=bool)
    items = rc.stances(sole, fps)
    for st in items:
      measured[st.start : st.end + 1] = True
    line = "".join("S" if measured[i] else ("x" if mask[i] else ".") for i in range(len(sole)))
    speed = np.linalg.norm(np.gradient(sole[:, :2], 1.0 / fps, axis=0), axis=1)
    print(f"\n{side}: altura mín {sole[:, 2].min() * 100:.2f} cm, máx {sole[:, 2].max() * 100:.2f} cm, velocidad xy máx {speed.max() * 100:.0f} cm/s")
    for k in range(0, len(line), 60):
      print(f"  {k:4d} {line[k:k + 60]}")
    print(f"  apoyos medibles: {len(items)}")
    for st in items:
      print(f"    pasos {st.start:4d}-{st.end:4d} ({(st.end - st.start) / fps:.2f} s)  recorrido de la referencia {st.reference_path_m * 100:.2f} cm")
    wrong = old & ~mask
    print(f"  regla antigua (solo altura): {int(old.sum())} fotogramas de apoyo; la nueva los pasa a vuelo en {int(wrong.sum())}"
          + (f" (velocidad xy en ellos hasta {speed[wrong].max() * 100:.0f} cm/s)" if wrong.any() else ""))


if __name__ == "__main__":
  main()
