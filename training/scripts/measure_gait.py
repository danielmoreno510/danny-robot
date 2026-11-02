"""Mide si Danny camina con paso alterno a una orden fija de avance, y guarda el resultado en results/.

Orden fija: por defecto 0,25 m/s hacia delante, sin lateral ni giro; con `--turn` se añade un giro. Descarta los primeros 2 s.
Para cada entorno que no se cae mide:
- error de velocidad respecto a la orden y velocidad media de avance, en el centro de masas de `base_link`;
- velocidad de giro media y su error respecto a la orden;
- fracción del tiempo con un solo pie apoyado;
- apoyos por pie y segundo, y fracción de apoyos que siguen a un apoyo del otro pie (1,0 es paso alterno perfecto);
- fracción de los apoyos que hace el pie izquierdo (0,5 es reparto igual).

Uso (desde training/):
    uv run python scripts/measure_gait.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name marcha_3000
    uv run python scripts/measure_gait.py --checkpoint ... --forward 0.0 --turn 0.5 --name giro_en_sitio
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, TASK, eval_env_cfg, fix_command, load_policy

FORWARD_CMD = 0.25  # m/s, la velocidad de la guía 4.2.
WARMUP_S = 2.0


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--forward", type=float, default=FORWARD_CMD, help="Orden de avance en m/s")
  parser.add_argument("--turn", type=float, default=0.0, help="Orden de giro en rad/s, positivo a la izquierda")
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija de los servos en V; sin ella, la del entrenamiento (6 a 8,4)")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env_cfg = eval_env_cfg(args.num_envs, args.seconds, args.vin)
  fix_command(env_cfg, args.forward, args.turn)
  env, policy = load_policy(env_cfg, args.checkpoint)

  u = env.unwrapped
  robot = u.scene["robot"]
  contact = u.scene["feet_ground_contact"]
  n, dt = args.num_envs, u.step_dt
  steps = int(args.seconds / dt)
  warmup = int(WARMUP_S / dt)
  device = u.device

  alive = torch.ones(n, dtype=torch.bool, device=device)
  prev = None
  last_foot = torch.full((n,), -1, dtype=torch.long, device=device)
  touchdowns = torch.zeros(n, 2, device=device)
  alternating = torch.zeros(n, device=device)
  single = torch.zeros(n, device=device)
  vx_sum = torch.zeros(n, device=device)
  err_sum = torch.zeros(n, device=device)
  yaw_sum = torch.zeros(n, device=device)
  yaw_err_sum = torch.zeros(n, device=device)
  counted = 0

  obs = env.get_observations()
  with torch.inference_mode():
    for i in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      alive &= ~u.reset_terminated.bool()
      found = contact.data.found.reshape(n, -1)[:, :2] > 0
      if i >= warmup:
        counted += 1
        cmd = u.command_manager.get_command("twist")
        vel = robot.data.root_com_lin_vel_b[:, :2]
        vx_sum += vel[:, 0]
        err_sum += (vel - cmd[:, :2]).norm(dim=1)
        yaw = robot.data.root_link_ang_vel_b[:, 2]
        yaw_sum += yaw
        yaw_err_sum += (yaw - cmd[:, 2]).abs()
        single += (found.sum(dim=1) == 1).float()
        if prev is not None:
          for foot in (0, 1):
            down = found[:, foot] & ~prev[:, foot]
            touchdowns[:, foot] += down.float()
            alternating += (down & (last_foot == 1 - foot)).float()
            last_foot = torch.where(down, torch.full_like(last_foot, foot), last_foot)
      prev = found.clone()

  ok = alive
  seconds_counted = counted * dt
  total_td = touchdowns.sum(dim=1)
  alt_frac = torch.where(total_td > 1, alternating / (total_td - 1).clamp(min=1), torch.zeros_like(total_td))

  def mean(x):
    return round(float(x[ok].mean()), 4) if ok.any() else None

  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "num_envs": n,
    "seconds_simulated": args.seconds,
    "vin_v": args.vin,
    "forward_command_m_s": args.forward,
    "turn_command_rad_s": args.turn,
    "fraction_envs_not_fallen": round(float(ok.float().mean()), 4),
    "mean_forward_velocity_m_s": mean(vx_sum / counted),
    "mean_xy_velocity_error_m_s": mean(err_sum / counted),
    "mean_yaw_rate_rad_s": mean(yaw_sum / counted),
    "mean_yaw_rate_error_rad_s": mean(yaw_err_sum / counted),
    "fraction_time_single_support": mean(single / counted),
    "touchdowns_per_foot_per_s": mean(total_td / 2 / seconds_counted),
    "fraction_alternating_touchdowns": mean(alt_frac),
    "fraction_touchdowns_left": round(float(touchdowns[ok, 0].sum() / touchdowns[ok].sum().clamp(min=1)), 4) if ok.any() else None,
    "note": f"Sin empujones. Orden fija de avance. Se descartan los primeros {WARMUP_S} s. Solo entornos que no caen. Velocidad del centro de masas de base_link.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_gait_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  env.close()


if __name__ == "__main__":
  main()
