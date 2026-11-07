"""Comprueba que el ONNX de una política da las mismas acciones que el punto de control, con observaciones reales de la simulación.

Corre 16 robots 10 s con las órdenes del entrenamiento y compara, en cada paso, la acción de la política de PyTorch con la de
onnxruntime. Guarda el error máximo en results/ y sale con código 1 si supera 1e-4.

Uso (desde training/):
    uv run python scripts/verify_policy_onnx.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --onnx <fichero>.onnx --name fase2_f_rodilla
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

from danny_training.evaluation import RESULTS, TASK, eval_env_cfg, load_policy

MAX_ERROR = 1e-4


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--onnx", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--num-envs", type=int, default=16)
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--task", default=TASK, help="Tarea del punto de control (por ejemplo Mjlab-GetUp-Danny)")
  args = parser.parse_args()

  env, policy = load_policy(eval_env_cfg(args.num_envs, args.seconds, task=args.task), args.checkpoint, task=args.task)
  session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
  input_name = session.get_inputs()[0].name
  obs = env.get_observations()
  worst = 0.0
  steps = int(args.seconds / env.unwrapped.step_dt)
  with torch.inference_mode():
    for _ in range(steps):
      action = policy(obs)
      x = obs["actor"] if hasattr(obs, "keys") else obs
      x = x.detach().cpu().numpy().astype(np.float32)
      onnx_action = np.concatenate([session.run(None, {input_name: x[i : i + 1]})[0] for i in range(x.shape[0])])
      worst = max(worst, float(np.abs(onnx_action - action.detach().cpu().numpy()).max()))
      obs, _, _, _ = env.step(action)

  result = {
    "checkpoint": str(args.checkpoint),
    "onnx": args.onnx.name,
    "observation_size": session.get_inputs()[0].shape[1],
    "action_size": session.get_outputs()[0].shape[1],
    "steps": steps,
    "num_envs": args.num_envs,
    "max_abs_action_difference": worst,
    "max_allowed": MAX_ERROR,
    "passes": worst <= MAX_ERROR,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_onnx_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  sys.exit(0 if result["passes"] else 1)


if __name__ == "__main__":
  main()
