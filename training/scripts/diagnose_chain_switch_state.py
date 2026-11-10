"""Compara, sin entrenar, lo que ve tumbarse en la cadena con lo que ve al entrenar (fase 5).

En la cadena (`measure_sit_lie_chain.py`) la política de tumbarse trabaja en el entorno de sentarse y empieza del estado real con que acaba sentarse; al
entrenar, trabaja en el entorno de tumbarse y empieza de un estado guardado (`training/data/sit_end_states.pt`) con 0,02 rad de ruido y guiñada aleatoria.
Este script mide dos cosas y escribe `results/training_diagnostico_cadena_cambio_<name>.json`:
  1. El estado de los robots justo al cambiar de política (articulaciones, altura de la pelvis, inclinación sin guiñada, velocidades) frente al guardado.
  2. Las diferencias de configuración entre el entorno de sentarse y el de tumbarse (acciones, observaciones, simulación, eventos, terminaciones).

Uso (desde training/):
    uv run python scripts/diagnose_chain_switch_state.py --sit-checkpoint <modelo sentarse> --name manos --vin 6.6
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

import torch
from mjlab.utils.lab_api.math import quat_inv, quat_mul, yaw_quat

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie

SIT_TASK, LIE_TASK = "Mjlab-SitDown-Danny", "Mjlab-LieDown-Danny"


def flat(obj, prefix: str = "") -> dict[str, str]:
  """Aplana una configuración (dataclasses y diccionarios) a {ruta: repr del valor}."""
  out = {}
  if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
    for f in dataclasses.fields(obj):
      out.update(flat(getattr(obj, f.name), f"{prefix}.{f.name}" if prefix else f.name))
  elif isinstance(obj, dict):
    for k, v in obj.items():
      out.update(flat(v, f"{prefix}.{k}"))
  elif isinstance(obj, (list, tuple)):
    out[prefix] = repr(obj)
  else:
    out[prefix] = repr(obj)
  return out


def summary(x: torch.Tensor) -> dict:
  return {"median": round(float(x.median()), 4), "p5": round(float(torch.quantile(x.float(), 0.05)), 4), "p95": round(float(torch.quantile(x.float(), 0.95)), 4)}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--sit-checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--sit-seconds", type=float, default=10.0)
  args = parser.parse_args()

  # 2. Diferencias de configuración.
  sit_flat = flat(eval_env_cfg(8, 10.0, args.vin, task=SIT_TASK))
  lie_flat = flat(eval_env_cfg(8, 10.0, args.vin, task=LIE_TASK))
  differences = {k: {"sentarse": sit_flat.get(k), "tumbarse": lie_flat.get(k)} for k in sorted(set(sit_flat) | set(lie_flat)) if sit_flat.get(k) != lie_flat.get(k)}

  # 1. Estado al cambiar de política.
  cfg = eval_env_cfg(args.num_envs, args.sit_seconds + 1.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}
  env, policy = load_policy(cfg, args.sit_checkpoint, task=SIT_TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(int(args.sit_seconds / u.step_dt)):
      obs, _, _, _ = env.step(policy(obs))
  data = robot.data
  tilt_quat = quat_mul(quat_inv(yaw_quat(data.root_link_quat_w)), data.root_link_quat_w)
  saved = torch.load(lie.SIT_END_STATES)
  names = list(robot.joint_names)
  assert names == saved["joint_names"]
  chain_q, saved_q = data.joint_pos.cpu(), saved["joint_pos"]
  per_joint = {
    n: {
      "chain_median": round(float(chain_q[:, j].median()), 3),
      "saved_median": round(float(saved_q[:, j].median()), 3),
      "chain_spread_p5_p95": [round(float(torch.quantile(chain_q[:, j], q)), 3) for q in (0.05, 0.95)],
      "saved_spread_p5_p95": [round(float(torch.quantile(saved_q[:, j], q)), 3) for q in (0.05, 0.95)],
      "chain_abs_velocity_median_rad_s": round(float(data.joint_vel[:, j].abs().median()), 4),
    }
    for j, n in enumerate(names)
  }
  out = {
    "sit_checkpoint": str(args.sit_checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": args.num_envs,
    "state_at_switch": {
      "root_z_chain_m": summary(data.root_link_pos_w[:, 2]),
      "root_z_saved_m": summary(saved["root_z"]),
      "tilt_quat_wxyz_chain_median": [round(float(tilt_quat[:, i].median()), 4) for i in range(4)],
      "tilt_quat_wxyz_saved_median": [round(float(saved["tilt_quat_wxyz"][:, i].median()), 4) for i in range(4)],
      "root_lin_speed_chain_m_s": summary(data.root_com_lin_vel_b.norm(dim=1)),
      "root_ang_speed_chain_rad_s": summary(data.root_link_ang_vel_b.norm(dim=1)),
      "joints": per_joint,
      "largest_joint_median_gap_rad": sorted(((n, round(abs(v["chain_median"] - v["saved_median"]), 3)) for n, v in per_joint.items()), key=lambda t: -t[1])[:5],
    },
    "env_config_differences_sit_vs_lie": differences,
  }
  path = RESULTS / f"training_diagnostico_cadena_cambio_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items() if k != "env_config_differences_sit_vs_lie"}, indent=1, ensure_ascii=False)[:6000])
  print("diferencias de configuracion:", len(differences))
  for k, v in list(differences.items())[:60]:
    print(" ", k, "| sentarse:", str(v["sentarse"])[:90], "| tumbarse:", str(v["tumbarse"])[:90])
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
