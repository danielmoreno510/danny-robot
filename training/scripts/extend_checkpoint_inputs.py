"""Amplía un punto de control con entradas nuevas del actor y del crítico, con pesos a cero, para seguir entrenando con más observaciones.

La política resultante da exactamente las mismas acciones mientras la entrada nueva no se use: el normalizador de la entrada nueva
queda con media 0 y varianza 1 y la columna de la primera capa a cero. El estado del optimizador se descarta (Adam lo rehace) y la
iteración se conserva. Las columnas nuevas van al final, como la observación nueva.

Uso (desde training/):
    uv run python scripts/extend_checkpoint_inputs.py --checkpoint <model_N.pt> --output <nuevo.pt> --actor-inputs 1 --critic-inputs 1
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


def extend(state: dict, extra: int) -> None:
  weight = state["mlp.0.weight"]
  state["mlp.0.weight"] = torch.cat([weight, torch.zeros(weight.shape[0], extra, dtype=weight.dtype)], dim=1)
  for key, fill in (("obs_normalizer._mean", 0.0), ("obs_normalizer._var", 1.0), ("obs_normalizer._std", 1.0)):
    old = state[key]
    state[key] = torch.cat([old, torch.full((old.shape[0], extra), fill, dtype=old.dtype)], dim=1)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--actor-inputs", type=int, required=True, help="Entradas nuevas del actor")
  parser.add_argument("--critic-inputs", type=int, required=True, help="Entradas nuevas del crítico")
  args = parser.parse_args()

  checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
  extend(checkpoint["actor_state_dict"], args.actor_inputs)
  extend(checkpoint["critic_state_dict"], args.critic_inputs)
  checkpoint["optimizer_state_dict"]["state"] = {}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  torch.save(checkpoint, args.output)
  actor = checkpoint["actor_state_dict"]["mlp.0.weight"].shape[1]
  critic = checkpoint["critic_state_dict"]["mlp.0.weight"].shape[1]
  print(f"Escrito {args.output}: actor con {actor} entradas, crítico con {critic}, iteración {checkpoint['iter']}.")


if __name__ == "__main__":
  main()
