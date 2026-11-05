"""Mide el par que aplican los actuadores HD-1910 (modelo BAM) durante la marcha y guarda el resultado en results/.

Corre un checkpoint en simulación sin empujones, con las órdenes del entrenamiento o con una orden fija de avance, y registra, por articulación, el par aplicado
(`data.actuator_force`) frente al par de bloqueo a la tensión de la prueba (0,739 N·m a 6 V y 0,912 a 7,4 V).
Sirve para saber si los servos se quedan sin par cuando la política pide movimiento.

Uso (desde training/):
    uv run python scripts/measure_actuator_torque.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --forward-speed 0.25 --name marcha_1000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, TASK, eval_env_cfg, fix_command, load_policy
from danny_training.actuator import stall_torque_nm



def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--forward-speed", type=float, default=None, help="Orden fija de avance en m/s; sin ella, las del entrenamiento")
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--height", type=float, default=0.0, help="Bajada de la pelvis en m (≤ 0)")
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija de los servos en V; sin ella, la del entrenamiento (6 a 8,4)")
  parser.add_argument("--pushes", action="store_true", help="Mantiene los empujones del entrenamiento (±0,20 m/s)")
  parser.add_argument("--push", type=float, default=None, metavar="M_S", help="Empujones de ±M_S m/s en XY cada 3 a 6 s (la guía usa 0,12)")
  parser.add_argument("--num-envs", type=int, default=64)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env_cfg = eval_env_cfg(args.num_envs, args.seconds, args.vin, args.pushes, args.push)
  if args.forward_speed is not None or args.height != 0.0:
    fix_command(env_cfg, args.forward_speed or 0.0, height=args.height)
  env, policy = load_policy(env_cfg, args.checkpoint)
  device = env.unwrapped.device

  u = env.unwrapped
  robot = u.scene["robot"]
  joint_names = list(robot.joint_names)
  # Con tensión sorteada se compara con el bloqueo a 6 V, el peor caso: sobrestima la saturación de los robots con más tensión. Para
  # saber si un servo satura de verdad, medir aparte con --vin 6.0, 7.4 y 8.4.
  stall_nm = stall_torque_nm(args.vin if args.vin is not None else 6.0)
  steps = int(args.seconds / u.step_dt)
  settle = int(1.0 / u.step_dt)
  obs = env.get_observations()

  tau_max = None
  tau_sum = torch.zeros(len(joint_names), device=device)
  over_sum = torch.zeros(len(joint_names), device=device)
  count = 0
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      if step < settle:
        continue
      tau = robot.data.actuator_force.abs()
      tau_max = tau if tau_max is None else torch.maximum(tau_max, tau)
      tau_sum += tau.mean(dim=0)
      over_sum += (tau > 0.9 * stall_nm).float().mean(dim=0)
      count += 1

  mx = tau_max.amax(dim=0)
  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "forward_command_m_s": args.forward_speed,
    "num_envs": args.num_envs,
    "seconds_simulated": args.seconds,
    "vin_v": args.vin,
    "height_command_m": args.height,
    "pushes": args.pushes or args.push is not None,
    "push_speed_m_s": args.push if args.push is not None else (0.2 if args.pushes else None),
    "stall_torque_nm": round(stall_nm, 4),
    "joints": {
      name: {
        "max_abs_nm": round(mx[i].item(), 4),
        "mean_abs_nm": round((tau_sum[i] / count).item(), 4),
        "frac_time_above_90pct_stall": round((over_sum[i] / count).item(), 4),
      }
      for i, name in enumerate(joint_names)
    },
    "note": f"{'Con' if args.pushes or args.push is not None else 'Sin'} empujones. Resto de la aleatorización activa. Bloqueo a {'la tensión fija' if args.vin is not None else '6 V (peor caso; la tensión se sortea de 6 a 8,4 V)'}. Actuadores HD-1910 sin confirmar.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_torque_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {out}")
  env.close()


if __name__ == "__main__":
  main()
