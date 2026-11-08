"""Mide la cadena de la fase 5: sentarse con su política y, desde donde termina, tumbarse con la suya. Guarda el resultado en results/.

Un solo entorno de sentarse (empieza de pie, aleatorización completa de la etapa 3, sin empujones): `--sit-seconds` s con la política de sentarse y luego
`--lie-seconds` s con la de tumbarse, que usa la misma observación (78) y las mismas acciones. No hay reinicio entre las dos: tumbarse empieza en el
estado real en que deja el robot sentarse (con brazos, cintura y cuello donde los deja esa política, no la pose limpia con la que se entrenó tumbarse).
Se aplican los dos criterios escritos: el de sentarse (`measure_sit.py`) sobre la primera parte y el de tumbarse (`danny_training/lie_measure.py`)
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
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

SIT_TASK, LIE_TASK = "Mjlab-SitDown-Danny", "Mjlab-LieDown-Danny"
WINDOW_S, SATURATED = 3.0, 0.9
FALL_TILT_RAD = math.radians(70.0)
SOFT_DESCENT_M_S = 0.6


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--sit-checkpoint", required=True, type=Path)
  parser.add_argument("--lie-checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=1024)
  parser.add_argument("--sit-seconds", type=float, default=10.0)
  parser.add_argument("--lie-seconds", type=float, default=10.0)
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

  # Primera parte: sentarse (mismas definiciones que measure_sit.py).
  sit_steps, window = int(args.sit_seconds / dt), int(WINDOW_S / dt)
  sat = torch.zeros(n, sit_steps, len(joint_names), dtype=torch.bool, device=device)
  max_down = torch.zeros(n, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  speed_ring = torch.zeros(n, window, device=device)
  success_step = torch.full((n,), -1, dtype=torch.int32, device=device)
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(sit_steps):
      obs, _, _, _ = env.step(sit_policy(obs))
      data = robot.data
      z = data.site_pos_w[:, pelvis_id, 2]
      tilt = torch.acos((-data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      speed = data.root_com_lin_vel_b.norm(dim=1)
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
  f = lambda x: round(float(x), 4)
  sit_criteria = {
    "seated": {"value": f(ok.float().mean()), "required": 0.95},
    "descent_at_most_0.6_m_s": {"value": f((max_down <= SOFT_DESCENT_M_S).float().mean()), "required": 0.95},
    "never_tilted_over_70deg": {"value": f((~fell).float().mean()), "required": 0.98},
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
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(lie_steps):
      obs, _, _, _ = env.step(lie_policy(obs))
      rec.record(step)
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
  }
  path = RESULTS / f"training_cadena_sentarse_tumbarse_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(out, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
