"""Diagnostica por qué caen algunos robots tras el cambio de la política de levantarse a la de marcha (fase 4), sin entrenar.

Repite el cambio de `measure_getup.py` (3 s de pie, orden cero, cae si el tronco pasa de 70° en los 5 s siguientes) y guarda en
results/training_getup_caidas_<name>.json: cuándo cae cada robot tras el cambio, hacia dónde (gravedad proyectada en el tronco: delante,
atrás, izquierda, derecha), y qué diferencia el estado en el instante del cambio entre los que caen y los que no (altura de la pelvis,
inclinación, velocidad de las articulaciones y ángulo de cada articulación, con la diferencia de medias en desviaciones típicas).

Uso (desde training/):
    uv run python scripts/diagnose_getup_switch_falls.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase4_pose_sin_umbral --vin 6.6
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
from danny_training.actuator import FrictionDRBamActuator

GETUP_TASK = "Mjlab-GetUp-Danny"
WALK_CHECKPOINT = Path("logs/rsl_rl/danny_walk/2026-11-05_00-00-00_fase3_inicio/model_8995.pt")
WINDOW_S, AFTER_S, SPEED_MAX = 3.0, 5.0, 0.05
FALL_TILT_RAD = math.radians(70.0)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--walk-checkpoint", type=Path, default=WALK_CHECKPOINT)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=512)
  parser.add_argument("--seconds", type=float, default=20.0)
  parser.add_argument("--blend-s", type=float, default=0.0, help="Segundos en que la acción pasa de la política de levantarse a la de marcha (0: cambio seco)")
  parser.add_argument("--window-s", type=float, default=WINDOW_S, help="Segundos seguidos de pie antes del cambio (el criterio es 3)")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + AFTER_S + 1.0, args.vin, task=GETUP_TASK)
  cfg.events["reset_lying"].params["prone_fraction"] = 1.0
  env, policy = load_policy(cfg, args.checkpoint, task=GETUP_TASK)
  walk_env, walk_policy = load_policy(eval_env_cfg(1, 1.0, args.vin), args.walk_checkpoint)
  u = env.unwrapped
  device, n, dt = u.device, args.num_envs, u.step_dt
  robot = u.scene["robot"]
  pelvis_id = robot.find_sites(["pelvis_bottom"])[0][0]
  joint_names = list(robot.joint_names)
  model = u.sim.mj_model
  floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=device)
  foot_bodies = [model.body(f"robot/{name}").id for name in ("left_foot_1", "right_foot_1")]
  foot_geoms = torch.tensor([i for i in range(model.ngeom) if model.geom_bodyid[i] in foot_bodies], device=device)

  steps = int((args.seconds + AFTER_S) / dt)
  window, after, rise_steps = int(args.window_s / dt), int(AFTER_S / dt), int(args.seconds / dt)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  fall_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  at_switch_q = torch.zeros(n, len(joint_names), device=device)
  at_switch_qd = torch.zeros(n, len(joint_names), device=device)
  at_switch_scalar = torch.zeros(n, 3, device=device)  # altura de la pelvis, inclinación, velocidad lineal
  fall_gravity = torch.zeros(n, 3, device=device)  # gravedad proyectada en el tronco al caer
  peak_tilt = torch.zeros(n, device=device)
  extra = torch.tensor([0.0, 1.0, 0.0], device=device).expand(n, 3)

  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      switched = success_step >= 0
      action = policy(obs)
      if switched.any():
        walk_obs = obs.clone()
        walk_obs["actor"] = torch.cat([obs["actor"], extra], dim=1)
        mix = ((step - success_step).float() / max(args.blend_s / dt, 1.0)).clamp(0.0, 1.0)[:, None]
        action = torch.where(switched[:, None], (1.0 - mix) * action + mix * walk_policy(walk_obs), action)
      obs, _, _, _ = env.step(action)

      z = robot.data.site_pos_w[:, pelvis_id, 2]
      gravity = robot.data.projected_gravity_b
      tilt = torch.acos((-gravity[:, 2]).clamp(-1.0, 1.0))
      speed = robot.data.root_com_lin_vel_b[:, :2].norm(dim=1)
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
      new = (success_step < 0) & (run >= window) & (speed_ring.mean(dim=1) <= SPEED_MAX) & (step < rise_steps)
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)
      at_switch_q[new] = robot.data.joint_pos[new]
      at_switch_qd[new] = robot.data.joint_vel[new]
      at_switch_scalar[new] = torch.stack([z, torch.rad2deg(tilt), speed], dim=1)[new]

      after_switch = (success_step >= 0) & (step > success_step) & (step - success_step <= after)
      peak_tilt = torch.where(after_switch, torch.maximum(peak_tilt, tilt), peak_tilt)
      fall_now = after_switch & (tilt > FALL_TILT_RAD) & (fall_step < 0)
      fall_step = torch.where(fall_now, torch.full_like(fall_step, step), fall_step)
      fall_gravity[fall_now] = gravity[fall_now]

  checked = (success_step >= 0) & ((steps - 1 - success_step) >= after)
  fell = checked & (fall_step >= 0)
  stayed = checked & (fall_step < 0)
  time_to_fall = (fall_step[fell] - success_step[fell]).float() * dt

  def direction(g: torch.Tensor) -> dict:
    # Con el cuerpo vertical g = (0, 0, -1). x del cuerpo hacia delante: g_x > 0 significa que la gravedad tira hacia delante.
    gx, gy = g[:, 0], g[:, 1]
    forward = (gx.abs() >= gy.abs()) & (gx > 0)
    back = (gx.abs() >= gy.abs()) & (gx <= 0)
    left = (gx.abs() < gy.abs()) & (gy > 0)
    right = (gx.abs() < gy.abs()) & (gy <= 0)
    return {k: int(v.sum()) for k, v in (("hacia_delante", forward), ("hacia_atras", back), ("hacia_la_izquierda", left), ("hacia_la_derecha", right))}

  def diff_in_std(x: torch.Tensor) -> float:
    pooled = torch.cat([x[fell], x[stayed]]).std().clamp(min=1e-6)
    return round(float((x[fell].mean() - x[stayed].mean()) / pooled), 2)

  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "window_s": args.window_s,
    "blend_s": args.blend_s,
    "num_envs": n,
    "robots_that_got_up_and_have_5s_after": int(checked.sum()),
    "fell": int(fell.sum()),
    "fraction_not_fallen": round(float(stayed.sum() / checked.sum().clamp(min=1)), 4),
    "time_to_fall_s": {"min": round(float(time_to_fall.min()), 2), "median": round(float(time_to_fall.median()), 2), "max": round(float(time_to_fall.max()), 2)} if fell.any() else None,
    "fall_direction_robot_frame": direction(fall_gravity[fell]) if fell.any() else None,
    "peak_tilt_deg_after_switch": {
      "fell_median": round(math.degrees(float(peak_tilt[fell].median())), 1) if fell.any() else None,
      "stayed_median": round(math.degrees(float(peak_tilt[stayed].median())), 1) if stayed.any() else None,
      "stayed_p95": round(math.degrees(float(torch.quantile(peak_tilt[stayed], 0.95))), 1) if stayed.any() else None,
    },
    "state_at_switch_fell_minus_stayed_in_std": {
      "pelvis_height_m": diff_in_std(at_switch_scalar[:, 0]),
      "tilt_deg": diff_in_std(at_switch_scalar[:, 1]),
      "speed_m_s": diff_in_std(at_switch_scalar[:, 2]),
    },
    "joint_angle_at_switch_fell_minus_stayed": {},
    "joint_speed_at_switch_fell_minus_stayed_in_std": {},
  }
  if fell.any() and stayed.any():
    for j, name in enumerate(joint_names):
      out["joint_angle_at_switch_fell_minus_stayed"][name] = {
        "deg_mean_fell": round(math.degrees(float(at_switch_q[fell, j].mean())), 1),
        "deg_mean_stayed": round(math.degrees(float(at_switch_q[stayed, j].mean())), 1),
        "diff_in_std": diff_in_std(at_switch_q[:, j]),
      }
      out["joint_speed_at_switch_fell_minus_stayed_in_std"][name] = diff_in_std(at_switch_qd[:, j])
    out["largest_angle_differences"] = sorted(out["joint_angle_at_switch_fell_minus_stayed"].items(), key=lambda kv: -abs(kv[1]["diff_in_std"]))[:5]
  RESULTS.mkdir(parents=True, exist_ok=True)
  path = RESULTS / f"training_getup_caidas_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print({k: out[k] for k in list(out)[3:9]})
  print(out.get("largest_angle_differences"))
  print(f"Escrito: {path}")
  env.close()


if __name__ == "__main__":
  main()
