"""Mide la cadena de la fase 5: sentarse con su política y, desde donde termina, tumbarse con la suya. Guarda el resultado en results/.

Un solo entorno de sentarse (empieza de pie, aleatorización completa de la etapa 3, sin empujones): `--sit-seconds` s con la política de sentarse y luego
`--lie-seconds` s con la de tumbarse, que usa la misma observación (78) y las mismas acciones. No hay reinicio entre las dos: tumbarse empieza en el
estado real en que deja el robot sentarse (con brazos, cintura y cuello donde los deja esa política, no la pose limpia con la que se entrenó tumbarse).
Se aplican los dos criterios escritos: el de sentarse (`measure_sit.py`, criterio del 9-10 de noviembre con las manos detrás, tronco a 80°) sobre la primera parte y el de tumbarse (`danny_training/lie_measure.py`)
sobre la segunda. La cadena pasa si cumple los dos.

Uso (desde training/):
    uv run python scripts/measure_sit_lie_chain.py --sit-checkpoint <modelo sentarse> --lie-checkpoint <modelo tumbarse> --name cadena_6.6V --vin 6.6 --num-envs 4096
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy, load_policy_on
from danny_training import crouch, getup, lie_measure, sit
from danny_training.servo_speed import joint_speed_report
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

SIT_TASK, LIE_TASK = "Mjlab-SitDown-Danny", "Mjlab-LieDown-Danny"
WINDOW_S, SATURATED = 3.0, 0.9
FALL_TILT_RAD = math.radians(80.0)
# Criterio de sentarse del 9-10 de noviembre (mismos valores que measure_sit.py; un script no importa de otro).
MIN_DESCENT_S = 0.6
DESCENT_END_Z_M = 0.16
MAX_DESCENT_M_S = 0.3
MAX_TOUCH_M_S = 0.4
MAX_TRUNK_RATE_RAD_S = 3.0


def switch_analysis(sit_rate: torch.Tensor, lie_rate: torch.Tensor, dt: float) -> dict:
  """Giro de la pelvis (valor absoluto, mediana de los robots) por paso alrededor del cambio de política, por eje, y cuándo ocurre el pico de cabeceo."""
  before, after = sit_rate[:, -5:].abs(), lie_rate[:, :20].abs()
  axes = lie_measure.AXES
  trace = lambda x, a: [round(float(x[:, t, a].median()), 3) for t in range(x.shape[1])]
  peak = lie_rate.abs()
  out = {
    "step_dt_s": dt,
    "last_5_steps_of_sit_median_abs_rate_rad_s": {axes[a]: trace(before, a) for a in range(3)},
    "first_20_steps_of_lie_median_abs_rate_rad_s": {axes[a]: trace(after, a) for a in range(3)},
    "lie_peak_axis_fraction_of_robots": {axes[a]: round(float((peak.amax(dim=1).argmax(dim=1) == a).float().mean()), 4) for a in range(3)},
    "lie_peak_step_by_axis_median": {axes[a]: int(peak[:, :, a].argmax(dim=1).float().median()) for a in range(3)},
    "lie_peak_time_after_switch_s_by_axis_median": {axes[a]: round(float(peak[:, :, a].argmax(dim=1).float().median() * dt), 2) for a in range(3)},
    "sit_peak_in_last_second_any_axis": lie_measure.quantiles(sit_rate[:, -int(1.0 / dt):].abs().amax(dim=(1, 2))),
    "lie_peak_pitch_rad_s": lie_measure.quantiles(peak[:, :, 1].amax(dim=1)),
    "lie_peak_pitch_after_first_0.5s_rad_s": lie_measure.quantiles(peak[:, int(0.5 / dt):, 1].amax(dim=1)),
  }
  return out


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--sit-checkpoint", required=True, type=Path)
  parser.add_argument("--lie-checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=1024)
  parser.add_argument("--sit-seconds", type=float, default=10.0)
  parser.add_argument("--lie-seconds", type=float, default=10.0)
  parser.add_argument("--zero-last-action", action="store_true", help="Al cambiar de política, pone a cero las últimas acciones que ve la observación (diagnóstico, 10 nov)")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.sit_seconds + args.lie_seconds + 2.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}
  env, sit_policy = load_policy(cfg, args.sit_checkpoint, task=SIT_TASK)
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

  # Primera parte: sentarse, con el criterio del 9-10 de noviembre (mismas definiciones que measure_sit.py, new_criterion_2026_11_09_manos).
  pelvis_body = int(model.site_bodyid[model.site(f"robot/{getup.PELVIS_SITE}").id])
  hand_bodies = torch.tensor([i for i in range(model.nbody) if "forearm" in model.body(i).name], device=device)
  support_bodies = torch.tensor([pelvis_body] + [i for i in range(model.nbody) if "thigh" in model.body(i).name or "shin" in model.body(i).name], device=device)
  neg1 = lambda: torch.full((n,), -1, dtype=torch.int32, device=device)
  sit_steps, window = int(args.sit_seconds / dt), int(WINDOW_S / dt)
  sat = torch.zeros(n, sit_steps, len(joint_names), dtype=torch.bool, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = neg1()
  max_dq_sit = torch.zeros(n, len(joint_names), device=device)
  sit_pelvis_rate = torch.zeros(n, sit_steps, 3, device=device)
  descent_start, reach_step, reach160_step = neg1(), neg1(), neg1()
  max_down_to160 = torch.zeros(n, device=device)
  hands_first, pelvis_first, support_first, cross_step = neg1(), neg1(), neg1(), neg1()
  support_touch_speed, cross_speed, prev_down = (torch.zeros(n, device=device) for _ in range(3))
  rate_max = torch.zeros(n, 3, device=device)
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(sit_steps):
      obs, _, _, _ = env.step(sit_policy(obs))
      data = robot.data
      max_dq_sit = torch.maximum(max_dq_sit, data.joint_vel.abs())
      sit_pelvis_rate[:, step] = data.root_link_ang_vel_b
      z = data.site_pos_w[:, pelvis_id, 2]
      tilt = torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      speed = data.root_com_lin_vel_b.norm(dim=1)
      descending = z < crouch.STAND_PELVIS_Z_M - 0.01
      down = (-data.root_com_lin_vel_w[:, 2]).clamp(min=0.0)
      above160 = z >= DESCENT_END_Z_M
      max_down_to160 = torch.maximum(max_down_to160, down * descending * above160)
      reach160_step = torch.where((reach160_step < 0) & (descent_start >= 0) & ~above160, torch.full_like(reach160_step, step), reach160_step)
      descent_start = torch.where((descent_start < 0) & descending, torch.full_like(descent_start, step), descent_start)
      began = descent_start >= 0
      reach_step = torch.where(began & (reach_step < 0) & (z <= sit.SEATED_PELVIS_M), torch.full_like(reach_step, step), reach_step)
      rate_max = torch.maximum(rate_max, data.root_link_ang_vel_b.abs() * began[:, None])
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
      hands_now = per_body[:, hand_bodies].sum(dim=1) > 0
      pelvis_now = per_body[:, pelvis_body] > 0
      hands_first = torch.where((hands_first < 0) & hands_now, torch.full_like(hands_first, step), hands_first)
      pelvis_first = torch.where((pelvis_first < 0) & pelvis_now, torch.full_like(pelvis_first, step), pelvis_first)
      new_support = (support_first < 0) & (per_body[:, support_bodies].sum(dim=1) > 0)
      support_touch_speed = torch.where(new_support, prev_down, support_touch_speed)
      support_first = torch.where(new_support, torch.full_like(support_first, step), support_first)
      new_cross = (cross_step < 0) & (z <= sit.SEATED_PELVIS_M) & began
      cross_speed = torch.where(new_cross, prev_down, cross_speed)
      cross_step = torch.where(new_cross, torch.full_like(cross_step, step), cross_step)
      prev_down = down
      ok_now = (z <= sit.SEATED_PELVIS_M) & (tilt <= sit.SEATED_TILT_RAD) & ~bad
      run = (run + 1) * ok_now.to(torch.int32)
      speed_ring[:, step % window] = speed
      new = (success_step < 0) & (run >= window) & (speed_ring.mean(dim=1) <= sit.SEATED_SPEED_M_S)
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)

  ok = success_step >= 0
  time_index = torch.arange(sit_steps, device=device)[None, :]
  moving_len = torch.where(ok, success_step - window + 1, torch.full_like(success_step, sit_steps))
  moving_mask = (time_index < moving_len[:, None])[:, :, None]
  seated_mask = ((time_index >= moving_len[:, None]) & (time_index <= success_step[:, None]) & ok[:, None])[:, :, None]
  moving_run = lie_measure.longest_run(sat & moving_mask).float() * dt
  seated_frac = (sat & seated_mask).sum(dim=1) / window
  worst_run, worst_seated = moving_run.max(dim=1).values, seated_frac.max(dim=1).values
  descent_s = torch.where((descent_start >= 0) & (reach_step >= 0), (reach_step - descent_start).float() * dt, torch.zeros(n, device=device))
  hands_before_pelvis = (hands_first >= 0) & ((pelvis_first < 0) | (hands_first < pelvis_first))
  touch_ok = (support_first >= 0) & (support_touch_speed <= MAX_TOUCH_M_S) & (cross_step >= 0) & (cross_speed <= MAX_TOUCH_M_S)
  f = lambda x: round(float(x), 4)
  sit_criteria = {
    "seated": {"value": f(ok.float().mean()), "required": 0.95},
    "full_descent_0.218_to_0.07_m_lasts_at_least_0.6_s": {"value": f((descent_s >= MIN_DESCENT_S).float().mean()), "required": 0.95},
    "pelvis_descent_speed_to_0.16_m_at_most_0.3_m_s": {"value": f((max_down_to160 <= MAX_DESCENT_M_S).float().mean()), "required": 0.95},
    "hands_touch_floor_before_pelvis": {"value": f(hands_before_pelvis.float().mean()), "required": 0.95},
    "pelvis_thighs_or_shins_touch_floor_and_pelvis_crosses_0.07_m_at_most_0.4_m_s": {"value": f(touch_ok.float().mean()), "required": 0.95},
    "trunk_rate_at_most_3_rad_s_every_axis": {"value": f((rate_max.amax(dim=1) <= MAX_TRUNK_RATE_RAD_S).float().mean()), "required": 0.99},
    "never_tilted_over_80deg": {"value": f((~fell).float().mean()), "required": 0.98},
    "saturation_moving_run_at_most_0.5s": {"value": f((worst_run <= 0.5).float().mean()), "required": 0.99},
    "saturation_seated_at_most_5pct": {"value": f((worst_seated[ok] <= 0.05).float().mean()) if ok.any() else None, "required": 0.99},
    "saturation_no_robot_over_2s": {"value": f((worst_run <= 2.0).float().mean()), "required": 1.0},
  }
  for c in sit_criteria.values():
    c["passes"] = c["value"] is not None and c["value"] >= c["required"]
  sit_pass = all(c["passes"] for c in sit_criteria.values())

  # Segunda parte: tumbarse con su política, desde el estado en que quedó el robot.
  lie_policy = load_policy_on(env, args.lie_checkpoint, LIE_TASK)
  lie_steps = int(args.lie_seconds / dt)
  rec = lie_measure.LieRecorder(env, lie_steps, stall)
  start_pelvis_z = robot.data.site_pos_w[:, pelvis_id, 2].clone()
  max_dq_lie = torch.zeros(n, len(joint_names), device=device)
  if args.zero_last_action:
    am = u.action_manager
    am._action.zero_(); am._prev_action.zero_(); am._prev_prev_action.zero_()
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(lie_steps):
      obs, _, _, _ = env.step(lie_policy(obs))
      rec.record(step)
      max_dq_lie = torch.maximum(max_dq_lie, robot.data.joint_vel.abs())
  lie = lie_measure.lie_report(rec, dt)
  out = {
    "sit_checkpoint": str(args.sit_checkpoint),
    "lie_checkpoint": str(args.lie_checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "sit_seconds": args.sit_seconds,
    "lie_seconds": args.lie_seconds,
    "chain_passes": bool(sit_pass and lie["passes_all"]),
    "sit_part": {
      "passes_all": sit_pass,
      "criteria": sit_criteria,
      "pelvis_z_at_switch_m": {"median": round(float(start_pelvis_z.median()), 4), "max": round(float(start_pelvis_z.max()), 4)},
    },
    "lie_part": lie,
    "switch_analysis": switch_analysis(sit_pelvis_rate, rec.pelvis_rate, dt),
    "joint_speed": {
      "sit_part": joint_speed_report(max_dq_sit, joint_names, args.vin, robot),
      "lie_part": joint_speed_report(max_dq_lie, joint_names, args.vin, robot),
    },
  }
  path = RESULTS / f"training_cadena_sentarse_tumbarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(out, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
