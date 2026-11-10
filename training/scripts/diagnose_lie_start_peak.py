"""Diagnostica de dónde sale el pico de giro de cabeceo de la pelvis a los 0,06 s de empezar tumbarse (fase 5) y guarda el resultado en results/.

Pregunta: ¿el pico es de la pose sentada soltándose sola o de la primera acción de la política? Con el entorno de tumbarse (inicio de los estados con
que acaba sentarse, `training/data/sit_end_states.pt`) corre `--seconds` s en tres modos, con los mismos robots y la misma semilla:
  - `politica`: la política de tumbarse.
  - `mantener`: acción que deja el objetivo de cada articulación en la posición con que nace el robot (la pose sentada se sostiene con los servos).
  - `cero`: acción cero (objetivo = pose por defecto de pie).
Por modo mide el giro de la pelvis (valor absoluto, ejes de la pelvis): pico en cualquier eje y en cada uno, instante del pico, mediana por paso de
los primeros pasos y fracción de robots con todos los ejes a 3 rad/s o menos.

Uso (desde training/):
    uv run python scripts/diagnose_lie_start_peak.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase5_tumbarse_manos_model_14496_6.6V --vin 6.6
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie, lie_measure

LIE_TASK = "Mjlab-LieDown-Danny"
LIMIT = lie_measure.MAX_PELVIS_RATE_RAD_S
AXES = lie_measure.AXES


def quantiles(x: torch.Tensor) -> dict:
  return {"median": round(float(x.median()), 3), "p95": round(float(torch.quantile(x, 0.95)), 3), "max": round(float(x.max()), 3)}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--seconds", type=float, default=2.0)
  args = parser.parse_args()

  results = {}
  for mode in ("politica", "mantener", "cero"):
    torch.manual_seed(0)
    cfg = eval_env_cfg(args.num_envs, args.seconds + 5.0, args.vin, task=LIE_TASK)
    cfg.terminations = {}
    env, policy = load_policy(cfg, args.checkpoint, task=LIE_TASK)
    u = env.unwrapped
    robot, n, dt = u.scene["robot"], args.num_envs, u.step_dt
    term = u.action_manager.get_term("joint_pos")
    q0 = robot.data.joint_pos.clone()
    hold = ((q0 - term._offset) / term._scale)
    steps = int(args.seconds / dt)
    rate = torch.zeros(n, steps, 3, device=u.device)
    obs = env.get_observations()
    with torch.inference_mode():
      for step in range(steps):
        if mode == "politica":
          action = policy(obs)
        elif mode == "mantener":
          action = hold
        else:
          action = torch.zeros_like(hold)
        obs, _, _, _ = env.step(action)
        rate[:, step] = robot.data.root_link_ang_vel_b
    a = rate.abs()
    peak_any = a.amax(dim=(1, 2))
    peak_step = a.amax(dim=2).argmax(dim=1)
    results[mode] = {
      "hold_action_abs_max": round(float(hold.abs().max()), 3) if mode == "mantener" else None,
      "pelvis_rate_peak_any_axis_rad_s": quantiles(peak_any),
      "pelvis_rate_peak_by_axis_rad_s": {AXES[k]: quantiles(a[:, :, k].amax(dim=1)) for k in range(3)},
      "peak_time_s_median": round(float(peak_step.float().median() * dt), 2),
      "fraction_of_robots_all_axes_at_most_3_rad_s": round(float((peak_any <= LIMIT).float().mean()), 4),
      "fraction_by_axis_at_most_3_rad_s": {AXES[k]: round(float((a[:, :, k].amax(dim=1) <= LIMIT).float().mean()), 4) for k in range(3)},
      "median_abs_rate_first_10_steps_rad_s": {AXES[k]: [round(float(a[:, t, k].median()), 3) for t in range(10)] for k in range(3)},
      "step_dt_s": dt,
    }
    env.close()
  out = {"checkpoint": str(args.checkpoint), "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4", "num_envs": args.num_envs, "seconds": args.seconds, "modes": results}
  path = RESULTS / f"training_tumbarse_pico_arranque_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(out, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
