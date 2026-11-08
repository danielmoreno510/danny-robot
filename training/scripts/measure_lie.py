"""Mide la tarea de tumbarse (fase 5) y guarda el resultado en results/.

Cada robot empieza sentado con las piernas estiradas (aleatorización completa de la etapa 3, sin empujones). «Queda tumbado» cuando, antes de
`--seconds`, durante 3 s seguidos se cumple a la vez: el eje x del tronco apunta hacia arriba (`projected_gravity_b[:, 0]` de -0,9 o menos),
inclinación lateral de 0,3 o menos, velocidad lineal de 0,05 m/s o menos y angular de 0,1 rad/s o menos (PROJECT_HANDOFF.md, Fase 5). Se mide
también el pico de giro de cabeceo del tronco (suave: 2,0 rad/s o menos), la saturación de par (sobre el 90 % del bloqueo de la tensión de cada
robot) mientras se tumba y estando tumbado, y la postura final.

Uso (desde training/):
    uv run python scripts/measure_lie.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase5_tumbarse_6.6V --vin 6.6
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

LIE_TASK = "Mjlab-LieDown-Danny"
WINDOW_S, SATURATED = 3.0, 0.9


def longest_run(flags: torch.Tensor) -> torch.Tensor:
  """Racha más larga de True a lo largo de la dimensión 1 de `flags` (entornos, pasos, articulaciones); devuelve (entornos, articulaciones)."""
  current = torch.zeros_like(flags[:, 0], dtype=torch.int32)
  best = torch.zeros_like(current)
  for t in range(flags.shape[1]):
    current = (current + 1) * flags[:, t].to(torch.int32)
    best = torch.maximum(best, current)
  return best


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija en V; sin ella, la sorteada del entrenamiento (6 a 8,4)")
  parser.add_argument("--num-envs", type=int, default=1024)
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--max-pitch-rate", type=float, default=lie.MAX_PITCH_RATE_RAD_S, help="Pico de giro de cabeceo permitido en rad/s")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=LIE_TASK)
  env, policy = load_policy(cfg, args.checkpoint, task=LIE_TASK)
  u = env.unwrapped
  device, n, dt = u.device, args.num_envs, u.step_dt
  robot = u.scene["robot"]
  joint_names = list(robot.joint_names)
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)

  steps = int(args.seconds / dt)
  window = int(WINDOW_S / dt)
  sat = torch.zeros(n, steps, len(joint_names), dtype=torch.bool, device=device)
  peak_pitch = torch.zeros(n, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  motion_ring = torch.zeros(n, window, 2, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  last = {}
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      data = robot.data
      gravity = data.projected_gravity_b
      speed = data.root_com_lin_vel_b.norm(dim=1)
      angular = data.root_link_ang_vel_b.norm(dim=1)
      peak_pitch = torch.maximum(peak_pitch, data.root_link_ang_vel_b[:, 1].abs())
      sat[:, step] = (data.actuator_force.abs() / stall) > SATURATED
      ok_now = (gravity[:, 0] <= lie.LYING_GRAVITY_X) & (gravity[:, 1].abs() <= lie.LYING_SIDE_MAX)
      run = (run + 1) * ok_now.to(torch.int32)
      motion_ring[:, step % window, 0] = speed
      motion_ring[:, step % window, 1] = angular
      new = (success_step < 0) & (run >= window) & (motion_ring[:, :, 0].mean(dim=1) <= lie.LYING_SPEED_M_S) & (motion_ring[:, :, 1].mean(dim=1) <= lie.LYING_ANGULAR_RAD_S)
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)
      last = {"gx": gravity[:, 0], "gy": gravity[:, 1]}

  ok = success_step >= 0
  time_index = torch.arange(steps, device=device)[None, :]
  moving_len = torch.where(ok, success_step - window + 1, torch.full_like(success_step, steps))
  moving_mask = (time_index < moving_len[:, None])[:, :, None]
  lying_mask = ((time_index >= moving_len[:, None]) & (time_index <= success_step[:, None]) & ok[:, None])[:, :, None]
  moving_run = longest_run(sat & moving_mask).float() * dt
  lying_frac = (sat & lying_mask).sum(dim=1) / window
  worst_run = moving_run.max(dim=1).values
  worst_lying = lying_frac.max(dim=1).values
  per_joint = {
    name: {
      "moving_longest_saturated_run_s_max": round(float(moving_run[:, j].max()), 2),
      "moving_longest_saturated_run_s_p99": round(float(torch.quantile(moving_run[:, j], 0.99)), 2),
      "lying_saturated_fraction_max": round(float(lying_frac[ok][:, j].max()), 4) if ok.any() else None,
    }
    for j, name in enumerate(joint_names)
  }
  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "definition": "eje x del tronco hacia arriba (gravedad x <= -0,9), inclinacion lateral <= 0,3, velocidad lineal <= 0,05 m/s y angular <= 0,1 rad/s, 3 s seguidos",
    "fraction_lying": round(float(ok.float().mean()), 4),
    "fraction_pitch_rate_peak_at_most_limit": round(float((peak_pitch <= args.max_pitch_rate).float().mean()), 4),
    "pitch_rate_limit_rad_s": args.max_pitch_rate,
    "pitch_rate_peak_rad_s": {"median": round(float(peak_pitch.median()), 3), "p95": round(float(torch.quantile(peak_pitch, 0.95)), 3)},
    "time_to_lying_s": {"median": round(float(((success_step[ok] + 1) * dt).median()), 2)} if ok.any() else None,
    "final_gravity_x": {"median": round(float(last["gx"].median()), 3), "p10": round(float(torch.quantile(last["gx"], 0.1)), 3)},
    "final_gravity_y_abs_p90": round(float(torch.quantile(last["gy"].abs(), 0.9)), 3),
    "saturation": {
      "fraction_of_robots_moving_run_at_most_0.5s": round(float((worst_run <= 0.5).float().mean()), 4),
      "fraction_of_robots_moving_run_at_most_2s": round(float((worst_run <= 2.0).float().mean()), 4),
      "fraction_of_lying_robots_saturated_at_most_5pct": round(float((worst_lying[ok] <= 0.05).float().mean()), 4) if ok.any() else None,
      "by_joint": per_joint,
    },
  }
  path = RESULTS / f"training_tumbarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items() if k != "saturation"}, indent=1, ensure_ascii=False))
  print({k: v for k, v in out["saturation"].items() if k != "by_joint"})
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
