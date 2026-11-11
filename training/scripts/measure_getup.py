"""Mide la tarea de levantarse (fase 4) y guarda el resultado en results/.

Cada robot empieza tumbado (boca abajo, boca arriba o mezclado) con la aleatorización completa de la etapa 3 y sin empujones. Se
cuenta que «se levanta» cuando, antes de `--seconds`, durante 3 s seguidos se cumple a la vez: pelvis a 0,20 m o más, tronco inclinado
15° o menos, los dos pies en el suelo, ningún eslabón distinto de los pies tocando el suelo y velocidad media de 0,05 m/s o menos
(PROJECT_HANDOFF.md, Fase 4). En el instante en que un robot cumple eso, cambia a la política de la fase 2 (`--walk-checkpoint`) con
orden cero, y se mide si no cae (tronco a más de 70°) en los 5 s siguientes.

Saturación por articulación (par sobre el 90 % del bloqueo de la tensión de cada robot): mientras se levanta (antes de la ventana de
3 s), fracción del tiempo, máximo del par respecto al bloqueo y tiempo seguido más largo; y durante la ventana de 3 s de pie.

Uso (desde training/):
    uv run python scripts/measure_getup.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase4_boca_abajo_6.6V --vin 6.6
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import getup
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

GETUP_TASK = "Mjlab-GetUp-Danny"
WALK_CHECKPOINT = Path("logs/rsl_rl/danny_walk/2026-11-05_00-00-00_fase3_inicio/model_8995.pt")
WINDOW_S, AFTER_S, SPEED_MAX, SATURATED = 3.0, 5.0, 0.05, 0.9
FALL_TILT_RAD = math.radians(70.0)


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
  parser.add_argument("--walk-checkpoint", type=Path, default=WALK_CHECKPOINT)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija de los servos en V; sin ella, la sorteada del entrenamiento (6 a 8,4)")
  parser.add_argument("--prone-fraction", type=float, default=1.0, help="Fracción de robots que empiezan boca abajo; el resto, boca arriba")
  parser.add_argument("--lie-end-states", type=Path, default=None, help="Estados con que acaba tumbarse (export_lie_end_states.py): los robots empiezan de ahí en vez de la pose de levantarse")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + AFTER_S + 1.0, args.vin, task=GETUP_TASK)
  if args.lie_end_states is not None:
    cfg.events["reset_lying"].func = getup.reset_from_lie_end
    cfg.events["reset_lying"].params = {"states_path": str(args.lie_end_states)}
  else:
    cfg.events["reset_lying"].params["prone_fraction"] = args.prone_fraction
  env, policy = load_policy(cfg, args.checkpoint, task=GETUP_TASK)
  # La política de la fase 2 se carga sobre un entorno de marcha de un solo robot, que da la forma de sus observaciones (81).
  walk_env, walk_policy = load_policy(eval_env_cfg(1, 1.0, args.vin), args.walk_checkpoint)
  u = env.unwrapped
  device = u.device
  robot = u.scene["robot"]
  n = args.num_envs
  dt = u.step_dt
  pelvis_id = robot.find_sites(["pelvis_bottom"])[0][0]
  joint_names = list(robot.joint_names)
  vin = next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor
  stall = stall_torque_nm(vin)  # (n, 1)

  model = u.sim.mj_model
  floor_geoms = [i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE]
  foot_bodies = [model.body(f"robot/{name}").id for name in ("left_foot_1", "right_foot_1")]
  foot_geoms = torch.tensor([i for i in range(model.ngeom) if model.geom_bodyid[i] in foot_bodies], device=device)
  floor = torch.tensor(floor_geoms, device=device)

  steps = int((args.seconds + AFTER_S) / dt)
  window, after = int(WINDOW_S / dt), int(AFTER_S / dt)
  sat = torch.zeros(n, int(args.seconds / dt), len(joint_names), dtype=torch.bool, device=device)
  peak_ratio = torch.zeros(n, len(joint_names), device=device)
  max_z = torch.zeros(n, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  fell_after = torch.zeros(n, dtype=torch.bool, device=device)
  post_speed = torch.zeros(n, device=device)
  post_count = 0
  post_z = torch.zeros(n, device=device)
  extra = torch.tensor([0.0, 1.0, 0.0], device=device).expand(n, 3)  # reloj de marcha parado (seno 0, coseno 1) y altura 0.

  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      switched = success_step >= 0
      action = policy(obs)
      if switched.any():
        walk_obs = obs.clone()
        walk_obs["actor"] = torch.cat([obs["actor"], extra], dim=1)
        action = torch.where(switched[:, None], walk_policy(walk_obs), action)
      obs, _, _, _ = env.step(action)

      z = robot.data.site_pos_w[:, pelvis_id, 2]
      tilt = torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      max_z = torch.maximum(max_z, z)
      speed = robot.data.root_com_lin_vel_b[:, :2].norm(dim=1)
      if step < sat.shape[1]:
        ratio = robot.data.actuator_force.abs() / stall
        sat[:, step] = ratio > SATURATED
        peak_ratio = torch.maximum(peak_ratio, ratio)

      data = u.sim.data
      geom = data.contact.geom.long()
      active = (torch.arange(geom.shape[0], device=device) < data.nacon[0]) & (data.contact.dist < 0.0)
      floor_first, floor_second = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
      other = torch.where(floor_first, geom[:, 1], geom[:, 0])
      body_hit = active & (floor_first | floor_second) & ~torch.isin(other, foot_geoms)
      body_contact = torch.zeros(n, device=device).index_add_(0, data.contact.worldid[body_hit].long(), torch.ones(int(body_hit.sum()), device=device)) > 0
      feet = (u.scene["feet_ground_contact"].data.found.reshape(n, -1)[:, :2] > 0).all(dim=1)

      stood = (z >= getup.STOOD_PELVIS_M) & (tilt <= getup.STOOD_TILT_RAD) & feet & ~body_contact
      run = (run + 1) * stood.to(torch.int32)
      speed_ring[:, step % window] = speed
      new = (success_step < 0) & (run >= window) & (speed_ring.mean(dim=1) <= SPEED_MAX) & (step < sat.shape[1])
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)

      after_switch = (success_step >= 0) & (step > success_step)
      fell_after |= after_switch & (tilt > FALL_TILT_RAD)
      in_tail = after_switch & (step - success_step >= after - int(3.0 / dt))
      post_speed += speed * in_tail
      post_z += z * in_tail
      post_count += in_tail.to(torch.int32)

  ok = success_step >= 0
  checked_after = ok & ((steps - 1 - success_step) >= after)
  rising_len = torch.where(ok, success_step - window + 1, torch.full_like(success_step, sat.shape[1]))
  time_index = torch.arange(sat.shape[1], device=device)[None, :]
  rising_mask = (time_index < rising_len[:, None])[:, :, None]
  standing_mask = ((time_index >= rising_len[:, None]) & (time_index <= success_step[:, None]) & ok[:, None])[:, :, None]
  rising_sat, standing_sat = sat & rising_mask, sat & standing_mask
  rising_frac = rising_sat.sum(dim=1) / rising_len.clamp(min=1)[:, None]
  rising_run = longest_run(rising_sat).float() * dt
  standing_frac = standing_sat.sum(dim=1) / window

  per_joint = {}
  for j, name in enumerate(joint_names):
    per_joint[name] = {
      "rising_peak_torque_over_stall": round(float(peak_ratio[:, j].max()), 3),
      "rising_saturated_fraction_mean": round(float(rising_frac[:, j].mean()), 4),
      "rising_saturated_fraction_max": round(float(rising_frac[:, j].max()), 4),
      "rising_longest_saturated_run_s_max": round(float(rising_run[:, j].max()), 2),
      "rising_longest_saturated_run_s_p95": round(float(torch.quantile(rising_run[:, j], 0.95)), 2),
      "standing_saturated_fraction_max": round(float(standing_frac[ok][:, j].max()), 4) if ok.any() else None,
    }
  worst_run = max(joint_names, key=lambda name: per_joint[name]["rising_longest_saturated_run_s_max"])
  worst_stand = max(joint_names, key=lambda name: per_joint[name]["standing_saturated_fraction_max"] or 0.0)
  success_time = (success_step[ok].float() + 1) * dt

  out = {
    "checkpoint": str(args.checkpoint),
    "walk_checkpoint": str(args.walk_checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "prone_fraction": args.prone_fraction,
    "start": f"estados con que acaba tumbarse: {args.lie_end_states}" if args.lie_end_states is not None else "pose de levantarse (reset_lying)",
    "num_envs": n,
    "seconds_to_get_up": args.seconds,
    "definition": "pelvis >= 0,20 m, tronco <= 15 grados, dos pies apoyados, ningun eslabon (salvo pies) en el suelo, velocidad media <= 0,05 m/s, 3 s seguidos",
    "fraction_got_up": round(float(ok.float().mean()), 4),
    "fraction_ever_pelvis_above_0.10m": round(float((max_z >= getup.RISEN_PELVIS_M).float().mean()), 4),
    "fraction_pelvis_0.15m_tilt_45deg_at_end": round(float(((z >= getup.UP_PELVIS_M) & (tilt <= getup.UP_TILT_RAD)).float().mean()), 4),
    "time_to_stand_s": {"median": round(float(success_time.median()), 2), "p90": round(float(torch.quantile(success_time, 0.9)), 2)} if ok.any() else None,
    "switch_to_walk": {
      "robots_that_got_up_and_have_5s_after": int(checked_after.sum()),
      "fraction_not_fallen": round(float((~fell_after[checked_after]).float().mean()), 4) if checked_after.any() else None,
      "mean_xy_speed_m_s_last_3s": round(float((post_speed / post_count.clamp(min=1))[checked_after].mean()), 4) if checked_after.any() else None,
      "mean_pelvis_height_m_last_3s": round(float((post_z / post_count.clamp(min=1))[checked_after].mean()), 4) if checked_after.any() else None,
      "note": "Cambio a la politica de la fase 2 con orden cero en el instante en que el robot cumple 3 s de pie; cae si el tronco pasa de 70 grados.",
    },
    "joint_with_longest_saturated_run_while_rising": {"joint": worst_run, "seconds": per_joint[worst_run]["rising_longest_saturated_run_s_max"]},
    "joint_with_most_saturation_while_standing": {"joint": worst_stand, "fraction": per_joint[worst_stand]["standing_saturated_fraction_max"]},
    "saturation_threshold": "par > 0,9 x bloqueo de la tension de cada robot",
    "by_joint": per_joint,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  path = RESULTS / f"training_getup_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print({k: out[k] for k in ("fraction_got_up", "fraction_ever_pelvis_above_0.10m", "fraction_pelvis_0.15m_tilt_45deg_at_end", "time_to_stand_s", "switch_to_walk", "joint_with_longest_saturated_run_while_rising", "joint_with_most_saturation_while_standing")})
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
