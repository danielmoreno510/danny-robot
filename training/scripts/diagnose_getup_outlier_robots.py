"""Busca qué tienen en común los robots que saturan una rodilla (fase 4, boca arriba) y guarda el resultado en results/.

Corre la política de levantarse sin cambiar a la marcha, con muchos robots, y marca como atípicos los que pasan más de `--run-limit` s seguidos
con una rodilla sobre el 90 % del bloqueo de su tensión, o más del 5 % de los últimos `--tail-s` s. Compara los parámetros sorteados de cada robot
(masas, centros de masa, fricciones, armadura, holgura, tensión, posición inicial) entre atípicos y el resto y ordena por distancia en
desviaciones típicas del resto.

Uso (desde training/):
    uv run python scripts/diagnose_getup_outlier_robots.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase4_rodillas_boca_arriba
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio
import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm

GETUP_TASK = "Mjlab-GetUp-Danny"
SATURATED = 0.9
KNEES = ("left_knee", "right_knee")
MODEL_FIELDS = ("body_mass", "body_ipos", "body_inertia", "geom_friction", "dof_armature", "dof_frictionloss", "dof_damping")
ACTUATOR_FIELDS = ("vin_tensor", "kp_scale", "kd_scale", "friction_scale", "backlash_half_width")


def longest_run(flags: torch.Tensor) -> torch.Tensor:
  """Racha más larga de True a lo largo de la dimensión 1 de `flags` (entornos, pasos)."""
  current = torch.zeros_like(flags[:, 0], dtype=torch.int32)
  best = torch.zeros_like(current)
  for t in range(flags.shape[1]):
    current = (current + 1) * flags[:, t].to(torch.int32)
    best = torch.maximum(best, current)
  return best


def per_robot_features(env, robot, joint_names) -> dict[str, torch.Tensor]:
  """Parámetros sorteados de cada robot: (entornos, k) por campo, con k valores aplanados."""
  n = env.num_envs
  model = env.sim.model
  out = {}
  for name in MODEL_FIELDS:
    value = torch.as_tensor(getattr(model, name)).float().reshape(n, -1)
    for k in range(value.shape[1]):
      out[f"{name}[{k}]"] = value[:, k]
  actuator = next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator))
  for name in ACTUATOR_FIELDS:
    out[name] = getattr(actuator, name).float().reshape(n)
  for j, joint in enumerate(joint_names):
    out[f"joint_pos_inicial_{joint}"] = robot.data.joint_pos[:, j].float()
  quat = robot.data.root_link_quat_w.float()
  for k in range(4):
    out[f"quat_inicial[{k}]"] = quat[:, k]
  return out


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--prone-fraction", type=float, default=0.0)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--seconds", type=float, default=20.0)
  parser.add_argument("--run-limit", type=float, default=0.5)
  parser.add_argument("--tail-s", type=float, default=10.0)
  parser.add_argument("--tail-fraction", type=float, default=0.05)
  parser.add_argument("--top", type=int, default=15)
  parser.add_argument("--videos", type=int, default=0, help="Graba en results/videos/ los N robots atipicos con la racha mas larga")
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=GETUP_TASK)
  cfg.events["reset_lying"].params["prone_fraction"] = args.prone_fraction
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array" if args.videos else None, task=GETUP_TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  n, dt = args.num_envs, u.step_dt
  joint_names = list(robot.joint_names)
  knee_ids = [joint_names.index(k) for k in KNEES]
  vin = next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor
  stall = stall_torque_nm(vin)

  model = u.sim.mj_model
  floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=u.device)
  foot_ids = {name: model.body(f"robot/{name}").id for name in ("left_foot_1", "right_foot_1")}
  foot_geoms = {name: torch.tensor([i for i in range(model.ngeom) if model.geom_bodyid[i] == b], device=u.device) for name, b in foot_ids.items()}
  all_foot_geoms = torch.cat(list(foot_geoms.values()))
  pelvis_id = robot.find_sites(["pelvis_bottom"])[0][0]

  obs = env.get_observations()
  features = per_robot_features(u, robot, joint_names)
  steps = int(args.seconds / dt)
  sat = torch.zeros(n, steps, len(knee_ids), dtype=torch.bool, device=u.device)
  # Por paso y robot: altura de la pelvis, inclinacion, pie izq., pie dcho., eslabon no-pie en el suelo, contactos entre pies, otros autocontactos,
  # angulo de las dos rodillas.
  state = torch.zeros(n, steps, 8, device=u.device)
  trajectory = torch.zeros(n, steps, u.sim.data.qpos.shape[1], device=u.device) if args.videos else None
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      sat[:, step] = (robot.data.actuator_force[:, knee_ids].abs() / stall) > SATURATED
      data = u.sim.data
      geom = data.contact.geom.long()
      active = (torch.arange(geom.shape[0], device=u.device) < data.nacon[0]) & (data.contact.dist < 0.0)
      first_floor, second_floor = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
      other = torch.where(first_floor, geom[:, 1], geom[:, 0])
      floor_hit = active & (first_floor | second_floor)
      body_hit = floor_hit & ~torch.isin(other, all_foot_geoms)
      in_left, in_right = torch.isin(geom, foot_geoms["left_foot_1"]), torch.isin(geom, foot_geoms["right_foot_1"])
      feet_hit = active & ((in_left[:, 0] & in_right[:, 1]) | (in_right[:, 0] & in_left[:, 1]))
      self_hit = active & ~(first_floor | second_floor) & ~feet_hit
      world = data.contact.worldid.long()

      def per_env(mask: torch.Tensor) -> torch.Tensor:
        return torch.zeros(n, device=u.device).index_add_(0, world[mask], torch.ones(int(mask.sum()), device=u.device))

      tilt = torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))
      found = u.scene["feet_ground_contact"].data.found.reshape(n, -1)[:, :2].float()
      state[:, step, 0] = robot.data.site_pos_w[:, pelvis_id, 2]
      state[:, step, 1] = torch.rad2deg(tilt)
      state[:, step, 2:4] = found
      state[:, step, 4] = per_env(body_hit)
      state[:, step, 5] = per_env(feet_hit)
      state[:, step, 6] = per_env(self_hit)
      state[:, step, 7] = torch.rad2deg(robot.data.joint_pos[:, knee_ids[1]])
      if trajectory is not None:
        trajectory[:, step] = data.qpos

  tail = int(args.tail_s / dt)
  run_s = torch.stack([longest_run(sat[:, :, k]).float() * dt for k in range(len(knee_ids))], dim=1)  # (n, 2)
  tail_fraction = sat[:, -tail:, :].float().mean(dim=1)  # (n, 2)
  flagged = (run_s > args.run_limit) | (tail_fraction > args.tail_fraction)
  report = {"checkpoint": str(args.checkpoint), "vin_v": args.vin, "prone_fraction": args.prone_fraction, "num_envs": n,
            "outliers_per_knee": {}, "criterion": f"racha > {args.run_limit} s o fraccion de los ultimos {args.tail_s} s > {args.tail_fraction}"}
  for k, knee in enumerate(KNEES):
    mask = flagged[:, k]
    entry = {"outliers": int(mask.sum()), "of": n,
             "longest_run_s_of_outliers": [round(float(x), 2) for x in run_s[mask, k][:20]],
             "tail_fraction_of_outliers": [round(float(x), 3) for x in tail_fraction[mask, k][:20]],
             "top_features_by_zscore_of_outlier_mean": []}
    if 2 <= int(mask.sum()) < n - 2:
      rows = []
      for name, value in features.items():
        rest = value[~mask]
        std = float(rest.std())
        if std < 1e-9:
          continue
        z = (float(value[mask].mean()) - float(rest.mean())) / std
        rows.append((abs(z), name, z, float(value[mask].mean()), float(rest.mean()), std))
      rows.sort(reverse=True)
      entry["top_features_by_zscore_of_outlier_mean"] = [
        {"feature": r[1], "z": round(r[2], 2), "outlier_mean": round(r[3], 5), "rest_mean": round(r[4], 5), "rest_std": round(r[5], 5)}
        for r in rows[: args.top]
      ]
    report["outliers_per_knee"][knee] = entry
    print(knee, entry["outliers"], "de", n, "atipicos")
    for row in entry["top_features_by_zscore_of_outlier_mean"][:8]:
      print("  ", row)

  # Que pasa en cada atipico mientras la rodilla esta al tope (la racha mas larga): medias de estado en esa racha.
  worst = []
  for k, knee in enumerate(KNEES):
    for i in torch.nonzero(flagged[:, k]).flatten().tolist():
      flags = sat[i, :, k]
      best_len, best_end, current = 0, 0, 0
      for t in range(steps):
        current = current + 1 if bool(flags[t]) else 0
        if current > best_len:
          best_len, best_end = current, t
      start = best_end - best_len + 1
      window = state[i, start : best_end + 1]
      worst.append({
        "robot": i, "knee": knee, "run_s": round(best_len * dt, 2), "from_s": round(start * dt, 2), "to_s": round((best_end + 1) * dt, 2),
        "pelvis_z_m": round(float(window[:, 0].mean()), 3), "tilt_deg": round(float(window[:, 1].mean()), 1),
        "left_foot_on_floor": round(float(window[:, 2].mean()), 2), "right_foot_on_floor": round(float(window[:, 3].mean()), 2),
        "non_foot_body_on_floor_contacts": round(float(window[:, 4].mean()), 2), "foot_foot_contacts": round(float(window[:, 5].mean()), 2),
        "other_self_contacts": round(float(window[:, 6].mean()), 2), "right_knee_deg": round(float(window[:, 7].mean()), 1),
      })
  worst.sort(key=lambda r: -r["run_s"])
  report["outlier_state_during_longest_run"] = worst
  population = state[~flagged.any(dim=1)]
  report["rest_state_mean_whole_run"] = {
    "pelvis_z_m": round(float(population[:, :, 0].mean()), 3), "tilt_deg": round(float(population[:, :, 1].mean()), 1),
    "non_foot_body_on_floor_contacts": round(float(population[:, :, 4].mean()), 2), "foot_foot_contacts": round(float(population[:, :, 5].mean()), 2),
    "other_self_contacts": round(float(population[:, :, 6].mean()), 2),
  }
  for row in worst[:6]:
    print(row)

  if args.videos:
    videos = RESULTS / "videos"
    videos.mkdir(parents=True, exist_ok=True)
    for row in worst[: args.videos]:
      path = videos / f"{args.name}_robot{row['robot']}_{row['knee']}_{row['run_s']}s.mp4"
      with imageio.get_writer(path, fps=int(round(u.metadata["render_fps"])), macro_block_size=1) as writer:
        for t in range(steps):
          u.sim.data.qpos[0] = trajectory[row["robot"], t]
          u.sim.data.qvel[0] = 0.0
          u.sim.forward()
          writer.append_data(u.render())
      print(path.name, f"{path.stat().st_size / 1e6:.2f} MB")
      row["video"] = str(path.name)

  out = RESULTS / f"training_getup_atipicos_{args.name}.json"
  out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
  print("Escrito:", out)


if __name__ == "__main__":
  main()
