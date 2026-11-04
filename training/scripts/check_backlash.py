"""Comprueba el juego mecánico de `actuator.apply_backlash` con objetivos puestos a mano, sin simular. Sale con código 1 si falla.

- Una rampa que sube: la salida sigue al objetivo con un retraso igual al semiancho del juego.
- Al cambiar de sentido, la salida se queda quieta hasta que el objetivo recorre el juego completo (2 × semiancho) y luego baja detrás de él.
- Un objetivo que oscila dentro del juego no mueve la salida.
- Con semiancho 0 la salida es el objetivo.

Uso (desde training/):
    uv run python scripts/check_backlash.py
"""

from __future__ import annotations

import sys

import torch

from danny_training.actuator import apply_backlash

HALF = 0.01


def run(targets: list[float], half: float) -> list[float]:
  output = torch.tensor([[targets[0]]])
  half_width = torch.tensor([[half]])
  out = []
  for t in targets:
    output = apply_backlash(torch.tensor([[t]]), output, half_width)
    out.append(round(float(output), 6))
  return out


def main() -> None:
  failures = []

  def check(name: str, got: list[float], expected: list[float]) -> None:
    if any(abs(g - e) > 1e-6 for g, e in zip(got, expected)) or len(got) != len(expected):
      failures.append(f"{name}: esperado {expected}, obtenido {got}")

  check("rampa que sube", run([0.0, 0.02, 0.04], HALF), [0.0, 0.01, 0.03])
  check("cambio de sentido", run([0.0, 0.04, 0.04, 0.03, 0.02, 0.01], HALF), [0.0, 0.03, 0.03, 0.03, 0.03, 0.02])
  check("oscilación dentro del juego", run([0.0, 0.005, -0.005, 0.005, 0.0], HALF), [0.0, 0.0, 0.0, 0.0, 0.0])
  check("sin juego", run([0.0, 0.02, 0.01, 0.03], 0.0), [0.0, 0.02, 0.01, 0.03])

  if failures:
    print("FALLA:\n" + "\n".join(failures))
    sys.exit(1)
  print("OK: apply_backlash retrasa hasta el semiancho, mantiene la salida en los cambios de sentido y no mueve dentro del juego.")


if __name__ == "__main__":
  main()
