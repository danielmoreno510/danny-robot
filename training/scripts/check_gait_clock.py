"""Comprueba el reloj de marcha y `mdp.gait_contact_phase` con un entorno falso, sin simular. Sale con código 1 si algo falla.

- Con orden de movimiento, la fase avanza step_dt / 0,8 s por paso y da la vuelta a 1.
- Con orden cero, la fase se congela.
- Al reiniciar un episodio, la fase vuelve a 0.
- La recompensa vale 1 con el pie correcto apoyado y el otro en el aire, 0 al revés, 0,5 con los dos apoyados, y con orden
  cero vale 1 con los dos apoyados.

Uso (desde training/):
    uv run python scripts/check_gait_clock.py
"""

from __future__ import annotations

import sys
from types import SimpleNamespace

import torch

from danny_training.gait_clock import GaitClockCommand, GaitClockCommandCfg
from danny_training.mdp import gait_contact_phase

STEP_DT = 0.02


def main() -> None:
  twist = torch.tensor([[0.25, 0.0, 0.0], [0.0, 0.0, 0.0]])
  found = torch.zeros(2, 2)
  terms: dict = {}
  env = SimpleNamespace(
    num_envs=2,
    device="cpu",
    step_dt=STEP_DT,
    scene={"feet_ground_contact": SimpleNamespace(data=SimpleNamespace(found=found))},
    command_manager=SimpleNamespace(get_command=lambda name: twist, get_term=lambda name: terms[name]),
  )
  clock = GaitClockCommand(GaitClockCommandCfg(), env)  # type: ignore[arg-type]
  terms["gait"] = clock
  failures = []

  def check(name: str, got: float, expected: float) -> None:
    if abs(got - expected) > 1e-5:
      failures.append(f"{name}: esperado {expected}, obtenido {got}")

  for _ in range(10):
    clock.compute(STEP_DT)
  check("fase con orden de 0,25 m/s tras 10 pasos", float(clock.phase[0]), 10 * STEP_DT / 0.8)
  check("fase con orden cero tras 10 pasos", float(clock.phase[1]), 0.0)
  for _ in range(35):
    clock.compute(STEP_DT)
  check("fase tras 45 pasos (da la vuelta)", float(clock.phase[0]), (45 * STEP_DT / 0.8) % 1.0)
  sincos = clock.command[0]
  check("seno² + coseno²", float(sincos.square().sum()), 1.0)

  clock.phase[0] = 0.2  # primera mitad: apoya el izquierdo.
  found[0] = torch.tensor([1.0, 0.0])
  check("izquierdo apoyado en la primera mitad", float(gait_contact_phase(env, "feet_ground_contact")[0]), 1.0)  # type: ignore[arg-type]
  found[0] = torch.tensor([0.0, 1.0])
  check("pies al revés en la primera mitad", float(gait_contact_phase(env, "feet_ground_contact")[0]), 0.0)  # type: ignore[arg-type]
  found[0] = torch.tensor([1.0, 1.0])
  check("los dos apoyados con movimiento", float(gait_contact_phase(env, "feet_ground_contact")[0]), 0.5)  # type: ignore[arg-type]
  clock.phase[0] = 0.7  # segunda mitad: apoya el derecho.
  found[0] = torch.tensor([0.0, 1.0])
  check("derecho apoyado en la segunda mitad", float(gait_contact_phase(env, "feet_ground_contact")[0]), 1.0)  # type: ignore[arg-type]
  found[1] = torch.tensor([1.0, 1.0])
  check("los dos apoyados con orden cero", float(gait_contact_phase(env, "feet_ground_contact")[1]), 1.0)  # type: ignore[arg-type]
  found[1] = torch.tensor([1.0, 0.0])
  check("un pie en el aire con orden cero", float(gait_contact_phase(env, "feet_ground_contact")[1]), 0.5)  # type: ignore[arg-type]

  clock.reset(env_ids=torch.tensor([0]))
  check("fase tras reiniciar", float(clock.phase[0]), 0.0)

  if failures:
    print("FALLA:\n" + "\n".join(failures))
    sys.exit(1)
  print("OK: el reloj avanza con orden, se congela sin ella y vuelve a 0 al reiniciar; la recompensa sigue la fase.")


if __name__ == "__main__":
  main()
