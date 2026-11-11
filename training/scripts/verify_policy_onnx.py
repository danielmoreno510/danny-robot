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
  parser.add_argument("--motion", type=Path, default=None, help="Referencia .npz, solo para las políticas de seguimiento (fase 6)")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds, task=args.task)
  if args.motion is not None:
    cfg.commands["motion"].motion_file = str(args.motion)
  env, policy = load_policy(cfg, args.checkpoint, task=args.task)
  session = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
  input_name = session.get_inputs()[0].name
  tracking = len(session.get_inputs()) == 2  # entradas (obs, time_step) y salidas con la referencia empaquetada (fase 6)
  command = env.unwrapped.command_manager.get_term("motion") if tracking else None
  obs = env.get_observations()
  worst = 0.0
  steps = int(args.seconds / env.unwrapped.step_dt)
  with torch.inference_mode():
    for _ in range(steps):
      action = policy(obs)
      x = obs["actor"] if hasattr(obs, "keys") else obs
      x = x.detach().cpu().numpy().astype(np.float32)
      feeds = [{input_name: x[i : i + 1]} for i in range(x.shape[0])]
      if tracking:
        steps_now = command.time_steps.cpu().numpy().astype(np.float32)
        feeds = [{**f, "time_step": steps_now[i : i + 1].reshape(1, 1)} for i, f in enumerate(feeds)]
      outs = [session.run(None, f) for f in feeds]
      onnx_action = np.concatenate([o[0] for o in outs])
      worst = max(worst, float(np.abs(onnx_action - action.detach().cpu().numpy()).max()))
      if tracking:  # la referencia empaquetada debe ser la que lee la tarea
        onnx_ref = np.concatenate([o[1] for o in outs])
        worst = max(worst, float(np.abs(onnx_ref - command.joint_pos.cpu().numpy()).max()))
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
