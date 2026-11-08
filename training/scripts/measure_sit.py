"""Mide la tarea de sentarse (fase 5) y guarda el resultado en results/.

Cada robot empieza de pie con la aleatorización completa de la etapa 3 y sin empujones. «Se sienta» cuando, antes de `--seconds`, durante 3 s
seguidos se cumple a la vez: pelvis a 0,07 m o menos, tronco a 35° o menos, velocidad media de 0,05 m/s o menos y ni tronco, cuello ni cabeza
tocando el suelo (PROJECT_HANDOFF.md, Fase 5). Se mide también la velocidad máxima de bajada de la pelvis (solo por debajo de la altura de pie menos 1 cm, para no contar la caída del robot al nacer), si el tronco pasa de 70° en algún
momento, la saturación de par (sobre el 90 % del bloqueo de la tensión de cada robot) mientras baja y estando sentado, y la postura final
(altura de la pelvis, inclinación, qué eslabones tocan el suelo), que sirve para ajustar el umbral de altura si es una estimación.

Uso (desde training/):
    uv run python scripts/measure_sit.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase5_sentarse_6.6V --vin 6.6
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import crouch, getup, sit
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

SIT_TASK = "Mjlab-SitDown-Danny"
WINDOW_S, SPEED_MAX, SATURATED = 3.0, sit.SEATED_SPEED_M_S, 0.9
FALL_TILT_RAD = math.radians(70.0)
SOFT_DESCENT_M_S = 0.6


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
  parser.add_argument("--seated-pelvis-m", type=float, default=sit.SEATED_PELVIS_M, help="Altura máxima de la pelvis sentado")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}  # se mide si cae, no se reinicia.
  env, policy = load_policy(cfg, args.checkpoint, task=SIT_TASK)
  u = env.unwrapped
  device, n, dt = u.device, args.num_envs, u.step_dt
  robot = u.scene["robot"]
  pelvis_id = robot.find_sites([getup.PELVIS_SITE])[0][0]
  joint_names = list(robot.joint_names)
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)
  model = u.sim.mj_model
  floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=device)
  body_of_geom = torch.as_tensor(model.geom_bodyid, device=device)
  forbidden = torch.tensor([model.body(f"robot/{name}").id for name in sit.FORBIDDEN_BODIES], device=device)

  steps = int(args.seconds / dt)
  window = int(WINDOW_S / dt)
  sat = torch.zeros(n, steps, len(joint_names), dtype=torch.bool, device=device)
  max_down = torch.zeros(n, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  last = {"z": torch.zeros(n, device=device), "tilt": torch.zeros(n, device=device)}
  touching = torch.zeros(n, model.nbody, device=device)
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      data = robot.data
      z = data.site_pos_w[:, pelvis_id, 2]
      tilt = torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      speed = data.root_com_lin_vel_b.norm(dim=1)
      # La bajada cuenta solo por debajo de la altura de pie menos 1 cm: el robot nace entre 1 y 5 cm por encima y cae hasta 1 m/s al apoyarse,
      # que no es parte de sentarse.
      descending = z < crouch.STAND_PELVIS_Z_M - 0.01
      max_down = torch.maximum(max_down, (-data.root_com_lin_vel_w[:, 2]).clamp(min=0.0) * descending)
      fell |= tilt > FALL_TILT_RAD
      sat[:, step] = (data.actuator_force.abs() / stall) > SATURATED

      sim = u.sim.data
      geom = sim.contact.geom.long()
      active = (torch.arange(geom.shape[0], device=device) < sim.nacon[0]) & (sim.contact.dist < 0.0)
      first, second = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
      hit = active & (first | second)
      other = torch.where(first, geom[:, 1], geom[:, 0])
      world = sim.contact.worldid.long()
      per_body = torch.zeros(n, model.nbody, device=device)
      per_body[world[hit], body_of_geom[other[hit]]] += 1.0
      bad = per_body[:, forbidden].sum(dim=1) > 0
      if step >= steps - int(1.0 / dt):
        touching += (per_body > 0).float()

      ok_now = (z <= args.seated_pelvis_m) & (tilt <= sit.SEATED_TILT_RAD) & ~bad
      run = (run + 1) * ok_now.to(torch.int32)
      speed_ring[:, step % window] = speed
      new = (success_step < 0) & (run >= window) & (speed_ring.mean(dim=1) <= SPEED_MAX)
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)
      last["z"], last["tilt"] = z, torch.rad2deg(tilt)

  ok = success_step >= 0
  time_index = torch.arange(steps, device=device)[None, :]
  moving_len = torch.where(ok, success_step - window + 1, torch.full_like(success_step, steps))
  moving_mask = (time_index < moving_len[:, None])[:, :, None]
  seated_mask = ((time_index >= moving_len[:, None]) & (time_index <= success_step[:, None]) & ok[:, None])[:, :, None]
  moving_run = longest_run(sat & moving_mask).float() * dt
  seated_frac = (sat & seated_mask).sum(dim=1) / window
  worst_run = moving_run.max(dim=1).values
  worst_seated = seated_frac.max(dim=1).values

  per_joint = {
    name: {
      "moving_longest_saturated_run_s_max": round(float(moving_run[:, j].max()), 2),
      "moving_longest_saturated_run_s_p99": round(float(torch.quantile(moving_run[:, j], 0.99)), 2),
      "seated_saturated_fraction_max": round(float(seated_frac[ok][:, j].max()), 4) if ok.any() else None,
    }
    for j, name in enumerate(joint_names)
  }
  touch_frac = touching.mean(dim=0) / int(1.0 / dt)
  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "seated_pelvis_threshold_m": args.seated_pelvis_m,
    "definition": "pelvis <= umbral, tronco <= 35 grados, ni tronco, cuello ni cabeza en el suelo, velocidad media <= 0,05 m/s, 3 s seguidos",
    "fraction_seated": round(float(ok.float().mean()), 4),
    "fraction_never_tilted_over_70deg": round(float((~fell).float().mean()), 4),
    "fraction_descent_speed_below_0.6_m_s": round(float((max_down <= SOFT_DESCENT_M_S).float().mean()), 4),
    "descent_speed_max_m_s": {"median": round(float(max_down.median()), 3), "p95": round(float(torch.quantile(max_down, 0.95)), 3)},
    "time_to_seated_s": {"median": round(float(((success_step[ok] + 1) * dt).median()), 2)} if ok.any() else None,
    "final_pelvis_z_m": {"median": round(float(last["z"].median()), 4), "p10": round(float(torch.quantile(last["z"], 0.1)), 4), "p90": round(float(torch.quantile(last["z"], 0.9)), 4)},
    "final_tilt_deg": {"median": round(float(last["tilt"].median()), 1), "p90": round(float(torch.quantile(last["tilt"], 0.9)), 1)},
    "final_bodies_touching_floor_fraction_of_robots": {model.body(i).name: round(float(touch_frac[i]), 2) for i in range(model.nbody) if touch_frac[i] > 0.01},
    "saturation": {
      "fraction_of_robots_moving_run_at_most_0.5s": round(float((worst_run <= 0.5).float().mean()), 4),
      "fraction_of_robots_moving_run_at_most_2s": round(float((worst_run <= 2.0).float().mean()), 4),
      "fraction_of_seated_robots_saturated_at_most_5pct": round(float((worst_seated[ok] <= 0.05).float().mean()), 4) if ok.any() else None,
      "by_joint": per_joint,
    },
  }
  path = RESULTS / f"training_sentarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items() if k != "saturation"}, indent=1, ensure_ascii=False))
  print({k: v for k, v in out["saturation"].items() if k != "by_joint"})
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
