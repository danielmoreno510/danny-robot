"""Exporta a ONNX la política de un punto de control (con los mismos metadatos que el entrenamiento) y guarda el fichero donde se pida.

El entrenamiento solo exporta el ONNX del último punto que guarda; este script sirve para exportar otro (por ejemplo el aprobado, que no es el último).
Después se comprueba con `verify_policy_onnx.py`.

Uso (desde training/):
    uv run python scripts/export_policy_onnx.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --task Mjlab-LieDown-Danny --output ../policies/<nombre>.onnx
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
from dataclasses import asdict
from pathlib import Path

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.rl.exporter_utils import attach_metadata_to_onnx, get_base_metadata
from mjlab.tasks.registry import load_rl_cfg, load_runner_cls

from danny_training.evaluation import eval_env_cfg


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--task", required=True, help="Tarea del punto de control (por ejemplo Mjlab-LieDown-Danny)")
  parser.add_argument("--output", required=True, type=Path, help="Fichero .onnx de salida")
  args = parser.parse_args()

  device = "cuda:0"
  agent_cfg = load_rl_cfg(args.task)
  env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg=eval_env_cfg(1, 5.0, task=args.task), device=device), clip_actions=agent_cfg.clip_actions)
  runner = (load_runner_cls(args.task) or MjlabOnPolicyRunner)(env, asdict(agent_cfg), device=device)
  runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
  with tempfile.TemporaryDirectory() as tmp:
    runner.export_policy_to_onnx(tmp, "policy.onnx")
    attach_metadata_to_onnx(str(Path(tmp) / "policy.onnx"), get_base_metadata(env.unwrapped, "local"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(tmp) / "policy.onnx", args.output)
  print(f"Escrito: {args.output}")


if __name__ == "__main__":
  main()
