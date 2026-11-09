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

import imageio
import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import crouch, getup, sit
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

SIT_TASK = "Mjlab-SitDown-Danny"
WINDOW_S, SPEED_MAX, SATURATED = 3.0, sit.SEATED_SPEED_M_S, 0.9
SOFT_DESCENT_M_S = 0.6
# Criterio del 9 de noviembre, sentarse apoyando las manos detrás (PROJECT_HANDOFF.md, Fase 5, sentarse, tercer intento).
MIN_DESCENT_S = 1.0
DESCENT_END_Z_M = 0.16  # la bajada medida va de la altura de pie menos 1 cm a esta altura
MAX_DESCENT_M_S = 0.3
MAX_PELVIS_TOUCH_M_S = 0.4
MAX_TRUNK_RATE_RAD_S = 3.0
ARM_REST_TOLERANCE_RAD = math.radians(20.0)  # solo informativo desde el 9 de noviembre: las manos pueden quedar apoyadas
ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")


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
  parser.add_argument("--videos", action="store_true", help="Graba en results/videos/ un robot tipico (bajada mediana) y el de la bajada mas rapida")
  parser.add_argument("--fall-tilt-deg", type=float, default=70.0, help="Inclinación del tronco a partir de la cual se cuenta caída")
  parser.add_argument("--seated-pelvis-m", type=float, default=sit.SEATED_PELVIS_M, help="Altura máxima de la pelvis sentado")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=SIT_TASK)
  cfg.terminations = {}  # se mide si cae, no se reinicia.
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array" if args.videos else None, task=SIT_TASK)
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
  pelvis_body = int(model.site_bodyid[model.site(f"robot/{getup.PELVIS_SITE}").id])
  hand_bodies = torch.tensor([i for i in range(model.nbody) if "forearm" in model.body(i).name], device=device)
  fall_tilt = math.radians(args.fall_tilt_deg)

  steps = int(args.seconds / dt)
  window = int(WINDOW_S / dt)
  sat = torch.zeros(n, steps, len(joint_names), dtype=torch.bool, device=device)
  max_down = torch.zeros(n, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  arm_ids = [i for i, name in enumerate(joint_names) if any(name.endswith(a) for a in ARM_JOINTS)]
  descent_start = torch.full((n,), -1, dtype=torch.int32, device=device)
  reach_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  rate_max = torch.zeros(n, 3, device=device)
  reach160_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  max_down_to160 = torch.zeros(n, device=device)
  hands_first = torch.full((n,), -1, dtype=torch.int32, device=device)
  pelvis_first = torch.full((n,), -1, dtype=torch.int32, device=device)
  pelvis_touch_speed = torch.zeros(n, device=device)
  max_tilt = torch.zeros(n, device=device)
  hand_z_at_contact = torch.full((n,), float("nan"), device=device)
  prev_down = torch.zeros(n, device=device)
  last = {"z": torch.zeros(n, device=device), "tilt": torch.zeros(n, device=device)}
  touching = torch.zeros(n, model.nbody, device=device)
  trajectory = torch.zeros(n, steps, u.sim.data.qpos.shape[1], device=device) if args.videos else None
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
      fell |= tilt > fall_tilt
      max_tilt = torch.maximum(max_tilt, tilt)
      down = (-data.root_com_lin_vel_w[:, 2]).clamp(min=0.0)
      above160 = z >= DESCENT_END_Z_M
      max_down_to160 = torch.maximum(max_down_to160, down * descending * above160)
      reach160_step = torch.where((reach160_step < 0) & (descent_start >= 0) & ~above160, torch.full_like(reach160_step, step), reach160_step)
      began = descent_start >= 0
      descent_start = torch.where(~began & descending, torch.full_like(descent_start, step), descent_start)
      began = descent_start >= 0
      reach_step = torch.where(began & (reach_step < 0) & (z <= args.seated_pelvis_m), torch.full_like(reach_step, step), reach_step)
      rate_max = torch.maximum(rate_max, data.root_link_ang_vel_b.abs() * began[:, None])
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
      hand_z_at_contact = torch.where(torch.isnan(hand_z_at_contact) & hands_now, z, hand_z_at_contact)
      new_pelvis = (pelvis_first < 0) & pelvis_now
      pelvis_touch_speed = torch.where(new_pelvis, prev_down, pelvis_touch_speed)
      pelvis_first = torch.where(new_pelvis, torch.full_like(pelvis_first, step), pelvis_first)
      prev_down = down
      if step >= steps - int(1.0 / dt):
        touching += (per_body > 0).float()

      ok_now = (z <= args.seated_pelvis_m) & (tilt <= sit.SEATED_TILT_RAD) & ~bad
      run = (run + 1) * ok_now.to(torch.int32)
      speed_ring[:, step % window] = speed
      new = (success_step < 0) & (run >= window) & (speed_ring.mean(dim=1) <= SPEED_MAX)
      success_step = torch.where(new, torch.full_like(success_step, step), success_step)
      last["z"], last["tilt"] = z, torch.rad2deg(tilt)
      if trajectory is not None:
        trajectory[:, step] = u.sim.data.qpos

  arm_error = (robot.data.joint_pos[:, arm_ids] - robot.data.default_joint_pos[:, arm_ids]).abs()
  arm_worst = arm_error.max(dim=1).values
  descent_s = torch.where((descent_start >= 0) & (reach_step >= 0), (reach_step - descent_start).float() * dt, torch.zeros(n, device=device))
  descent160_s = torch.where((descent_start >= 0) & (reach160_step >= 0), (reach160_step - descent_start).float() * dt, torch.zeros(n, device=device))
  hands_before_pelvis = (hands_first >= 0) & ((pelvis_first < 0) | (hands_first < pelvis_first))
  touch_ok = (pelvis_first < 0) | (pelvis_touch_speed <= MAX_PELVIS_TOUCH_M_S)
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
  new_checks = {
    "seated": (ok.float().mean(), 0.95),
    "descent_to_0.16_m_lasts_at_least_1_s": ((descent160_s >= MIN_DESCENT_S).float().mean(), 0.95),
    "pelvis_descent_speed_to_0.16_m_at_most_0.3_m_s": ((max_down_to160 <= MAX_DESCENT_M_S).float().mean(), 0.95),
    "hands_touch_floor_before_pelvis": (hands_before_pelvis.float().mean(), 0.95),
    "pelvis_touches_floor_at_most_0.4_m_s": (touch_ok.float().mean(), 0.95),
    "trunk_rate_at_most_3_rad_s_every_axis": ((rate_max.amax(dim=1) <= MAX_TRUNK_RATE_RAD_S).float().mean(), 0.99),
    f"never_tilted_over_{args.fall_tilt_deg:g}deg": ((~fell).float().mean(), 0.98),
    "saturation_moving_run_at_most_0.5s": ((worst_run <= 0.5).float().mean(), 0.99),
    "saturation_seated_at_most_5pct": ((worst_seated[ok] <= 0.05).float().mean() if ok.any() else torch.tensor(0.0), 0.99),
    "saturation_no_robot_over_2s": ((worst_run <= 2.0).float().mean(), 1.0),
  }
  new_criterion = {k: {"value": round(float(v), 4), "required": r, "passes": bool(float(v) >= r)} for k, (v, r) in new_checks.items()}
  touch_frac = touching.mean(dim=0) / int(1.0 / dt)
  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "seated_pelvis_threshold_m": args.seated_pelvis_m,
    "definition": "pelvis <= umbral, tronco <= 35 grados, ni tronco, cuello ni cabeza en el suelo, velocidad media <= 0,05 m/s, 3 s seguidos",
    "new_criterion_2026_11_09_manos": {"passes_all": all(c["passes"] for c in new_criterion.values()), "criteria": new_criterion},
    "descent_duration_s": {"median": round(float(descent_s[ok].median()), 2), "p5": round(float(torch.quantile(descent_s[ok], 0.05)), 2), "p95": round(float(torch.quantile(descent_s[ok], 0.95)), 2)} if ok.any() else None,
    "trunk_rate_peak_rad_s_by_axis_after_descent_start": {
      axis: {"median": round(float(rate_max[:, a].median()), 3), "p95": round(float(torch.quantile(rate_max[:, a], 0.95)), 3), "max": round(float(rate_max[:, a].max()), 3)}
      for a, axis in enumerate(("balanceo", "cabeceo", "guinada"))
    },
    "descent_to_0.16_m_duration_s": {"median": round(float(descent160_s[reach160_step >= 0].median()), 2), "p5": round(float(torch.quantile(descent160_s[reach160_step >= 0], 0.05)), 2), "p95": round(float(torch.quantile(descent160_s[reach160_step >= 0], 0.95)), 2)} if (reach160_step >= 0).any() else None,
    "descent_speed_to_0.16_m_max_m_s": {"median": round(float(max_down_to160.median()), 3), "p95": round(float(torch.quantile(max_down_to160, 0.95)), 3)},
    "hands_first_touch_fraction_of_robots": round(float((hands_first >= 0).float().mean()), 4),
    "pelvis_z_at_first_hand_contact_m": {"median": round(float(hand_z_at_contact[~torch.isnan(hand_z_at_contact)].median()), 3), "max": round(float(hand_z_at_contact[~torch.isnan(hand_z_at_contact)].max()), 3)} if (~torch.isnan(hand_z_at_contact)).any() else None,
    "pelvis_touch_speed_m_s": {"median": round(float(pelvis_touch_speed[pelvis_first >= 0].median()), 3), "p95": round(float(torch.quantile(pelvis_touch_speed[pelvis_first >= 0], 0.95)), 3)} if (pelvis_first >= 0).any() else None,
    "pelvis_touched_floor_fraction_of_robots": round(float((pelvis_first >= 0).float().mean()), 4),
    "max_trunk_tilt_deg": {"median": round(math.degrees(float(max_tilt.median())), 1), "p95": round(math.degrees(float(torch.quantile(max_tilt, 0.95))), 1), "max": round(math.degrees(float(max_tilt.max())), 1)},
    "arms_max_deviation_from_rest_deg_at_end": {"median": round(math.degrees(float(arm_worst.median())), 1), "p95": round(math.degrees(float(torch.quantile(arm_worst, 0.95))), 1), "max": round(math.degrees(float(arm_worst.max())), 1)},
    "fraction_seated": round(float(ok.float().mean()), 4),
    "fraction_never_tilted_over_fall_threshold": round(float((~fell).float().mean()), 4),
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
  if args.videos:
    candidates = torch.nonzero(ok & ~fell).flatten()
    typical = int(candidates[torch.argmin((max_down[candidates] - max_down[candidates].median()).abs())])
    fastest = int(candidates[torch.argmax(max_down[candidates])])
    videos = RESULTS / "videos"
    videos.mkdir(parents=True, exist_ok=True)
    out["videos"] = []
    for label, i in (("tipico", typical), ("bajada_mas_rapida", fastest)):
      path = videos / f"{args.name}_{label}_robot{i}_bajada{float(max_down[i]):.2f}ms.mp4"
      with imageio.get_writer(path, fps=int(round(u.metadata["render_fps"])), macro_block_size=1) as writer:
        for t in range(steps):
          u.sim.data.qpos[0] = trajectory[i, t]
          u.sim.data.qvel[0] = 0.0
          u.sim.forward()
          writer.append_data(u.render())
      out["videos"].append({"robot": i, "kind": label, "max_descent_speed_m_s": round(float(max_down[i]), 3), "file": path.name})
      print(path.name)
  path = RESULTS / f"training_sentarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items() if k != "saturation"}, indent=1, ensure_ascii=False))
  print({k: v for k, v in out["saturation"].items() if k != "by_joint"})
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
