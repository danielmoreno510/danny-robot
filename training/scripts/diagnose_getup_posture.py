"""Diagnostica en qué postura se queda la política de levantarse (fase 4) y graba vídeos, sin entrenar.

Corre `--num-envs` robots tumbados durante `--seconds` a tensión fija y guarda en results/training_getup_postura_<name>.json:
postura media de los últimos `--last-seconds` s (altura de la pelvis, inclinación, fracción del tiempo que cada eslabón toca el suelo) y,
por articulación, ángulo pedido, ángulo real, límites, fracción del tiempo a menos de 3° del límite y par respecto al bloqueo. Además
graba `--videos` robots de uno en uno en results/videos/<name>_robot<i>.mp4.

Uso (desde training/):
    uv run python scripts/diagnose_getup_posture.py --checkpoint ../puntos_de_control/fase4_boca_abajo_model_3100.pt --name fase4_boca_abajo_it3000 --vin 6.6
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import getup
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

GETUP_TASK = "Mjlab-GetUp-Danny"
NEAR_LIMIT_RAD = math.radians(3.0)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  parser.add_argument("--last-seconds", type=float, default=10.0)
  parser.add_argument("--videos", type=int, default=3)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=GETUP_TASK)
  env, policy = load_policy(cfg, args.checkpoint, task=GETUP_TASK)
  u = env.unwrapped
  device, n, dt = u.device, args.num_envs, u.step_dt
  robot = u.scene["robot"]
  pelvis_id = robot.find_sites(["pelvis_bottom"])[0][0]
  joint_names = list(robot.joint_names)
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)
  limits = robot.data.soft_joint_pos_limits[0]  # (joints, 2)

  model = u.sim.mj_model
  floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=device)
  geom_body = torch.tensor(model.geom_bodyid, device=device).long()
  nbody = model.nbody
  body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(nbody)]

  steps = int(args.seconds / dt)
  last = int(args.last_seconds / dt)
  contact_steps = torch.zeros(n * nbody, device=device)
  z_hist, tilt_hist = [], []
  q_sum = torch.zeros(n, len(joint_names), device=device)
  target_sum, torque_sum, near_lo, near_hi = (torch.zeros_like(q_sum) for _ in range(4))
  seconds_series = {"pelvis_z_mean": [], "tilt_deg_mean": [], "left_hip_pitch": [], "right_hip_pitch": []}
  lhp, rhp = joint_names.index("left_hip_pitch"), joint_names.index("right_hip_pitch")

  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      z = robot.data.site_pos_w[:, pelvis_id, 2]
      tilt = torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      q, target = robot.data.joint_pos, robot.data.joint_pos_target
      torque = robot.data.actuator_force.abs() / stall
      if step % int(1.0 / dt) == 0:
        seconds_series["pelvis_z_mean"].append(round(float(z.mean()), 4))
        seconds_series["tilt_deg_mean"].append(round(float(torch.rad2deg(tilt).mean()), 1))
        for key, j in (("left_hip_pitch", lhp), ("right_hip_pitch", rhp)):
          seconds_series[key].append({
            "target_deg": round(math.degrees(float(target[:, j].mean())), 1),
            "real_deg": round(math.degrees(float(q[:, j].mean())), 1),
            "torque_over_stall": round(float(torque[:, j].mean()), 3),
          })
      if step >= steps - last:
        z_hist.append(z)
        tilt_hist.append(tilt)
        q_sum += q
        target_sum += target
        torque_sum += torque
        near_lo += (q - limits[:, 0] < NEAR_LIMIT_RAD).float()
        near_hi += (limits[:, 1] - q < NEAR_LIMIT_RAD).float()
        data = u.sim.data
        geom = data.contact.geom.long()
        active = (torch.arange(geom.shape[0], device=device) < data.nacon[0]) & (data.contact.dist < 0.0)
        floor_first, floor_second = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
        hit = active & (floor_first | floor_second)
        other = torch.where(floor_first, geom[:, 1], geom[:, 0])[hit]
        world = data.contact.worldid[hit].long()
        pair = torch.unique(world * nbody + geom_body[other])  # un eslabón cuenta una vez por paso
        contact_steps.index_add_(0, pair, torch.ones(len(pair), device=device))

  z_last = torch.stack(z_hist).mean(dim=0)  # (n,)
  tilt_last = torch.rad2deg(torch.stack(tilt_hist).mean(dim=0))
  contact_frac = (contact_steps.reshape(n, nbody) / last).mean(dim=0)
  touching = {body_names[i].removeprefix("robot/"): round(float(contact_frac[i]), 3) for i in range(nbody) if contact_frac[i] > 0.01}

  def stats(x: torch.Tensor) -> dict:
    return {"mean": round(float(x.mean()), 4), "p10": round(float(torch.quantile(x, 0.1)), 4), "p90": round(float(torch.quantile(x, 0.9)), 4)}

  per_joint = {}
  for j, name in enumerate(joint_names):
    per_joint[name] = {
      "limit_deg": [round(math.degrees(float(limits[j, 0])), 1), round(math.degrees(float(limits[j, 1])), 1)],
      "target_deg_mean": round(math.degrees(float((target_sum[:, j] / last).mean())), 1),
      "real_deg_mean": round(math.degrees(float((q_sum[:, j] / last).mean())), 1),
      "real_deg_std_between_robots": round(math.degrees(float((q_sum[:, j] / last).std())), 1),
      "fraction_within_3deg_of_lower_limit": round(float((near_lo[:, j] / last).mean()), 3),
      "fraction_within_3deg_of_upper_limit": round(float((near_hi[:, j] / last).mean()), 3),
      "torque_over_stall_mean": round(float((torque_sum[:, j] / last).mean()), 3),
    }
  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin,
    "num_envs": n,
    "window_s": [args.seconds - args.last_seconds, args.seconds],
    "pelvis_height_m": stats(z_last),
    "trunk_tilt_deg": stats(tilt_last),
    "fraction_of_robots_with_pelvis_above_0.15m_and_tilt_below_45deg": round(float(((z_last >= 0.15) & (tilt_last <= 45)).float().mean()), 4),
    "fraction_of_robots_with_pelvis_above_0.15m": round(float((z_last >= 0.15).float().mean()), 4),
    "fraction_of_time_link_touches_floor": touching,
    "by_second": seconds_series,
    "by_joint": per_joint,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  path = RESULTS / f"training_getup_postura_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: out[k] for k in list(out)[4:9]}, indent=1, ensure_ascii=False))
  print({k: per_joint[k] for k in ("left_hip_pitch", "right_hip_pitch")})
  print(f"Escrito: {path}")
  env.close()

  videos = RESULTS / "videos"
  videos.mkdir(parents=True, exist_ok=True)
  for i in range(args.videos):
    vcfg = eval_env_cfg(1, args.seconds + 1.0, args.vin, task=GETUP_TASK)
    vcfg.seed = 100 + i
    venv, vpolicy = load_policy(vcfg, args.checkpoint, render_mode="rgb_array", task=GETUP_TASK)
    vu = venv.unwrapped
    vpath = videos / f"{args.name}_robot{i}.mp4"
    obs = venv.get_observations()
    with imageio.get_writer(vpath, fps=int(round(vu.metadata["render_fps"])), macro_block_size=1) as writer, torch.inference_mode():
      for _ in range(int(args.seconds / vu.step_dt)):
        obs, _, _, _ = venv.step(vpolicy(obs))
        writer.append_data(vu.render())
    venv.close()
    print(f"{vpath.name}: {vpath.stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
  main()
