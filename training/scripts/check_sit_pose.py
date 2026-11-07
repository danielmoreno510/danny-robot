"""Comprueba en el simulador qué poses sentadas en el suelo se sostienen (fase 5, sentarse) y da la altura de la pelvis sentado.

Coloca el robot sentado (pelvis a 12 cm del suelo (cae y se asienta), tronco recto, piernas hacia delante) y le pide durante `--seconds` s la pose de articulaciones
de cada candidata, sin política. Mide la altura de `pelvis_bottom`, la inclinación del tronco, la velocidad media y los eslabones que tocan el
suelo en el último segundo. Guarda results/training_sit_pose.json. Es la comprobación de que el criterio de la fase 5 es físicamente posible.

Uso (desde training/):
    uv run python scripts/check_sit_pose.py
"""

from __future__ import annotations

import argparse
import json
import math

import mujoco
import torch

from danny_training.evaluation import RESULTS, eval_env_cfg
from danny_training import getup
from mjlab.envs import ManagerBasedRlEnv

TASK = "Mjlab-GetUp-Danny"
# Candidatas (grados): cadera de cabeceo, rodilla y tobillo de cabeceo de las dos piernas; los brazos cuelgan.
CANDIDATES = {
  "piernas_adelante_recta": {"hip_pitch": -85.0, "knee": 10.0, "ankle_pitch": 0.0},
  "piernas_adelante_flexion": {"hip_pitch": -80.0, "knee": 40.0, "ankle_pitch": -10.0},
  "piernas_adelante_cadera_94": {"hip_pitch": -90.0, "knee": 20.0, "ankle_pitch": 0.0},
  "sobre_talones": {"hip_pitch": -90.0, "knee": 120.0, "ankle_pitch": 30.0},
}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--seconds", type=float, default=4.0)
  parser.add_argument("--num-envs", type=int, default=64)
  parser.add_argument("--vin", type=float, default=6.6)
  args = parser.parse_args()

  results = {}
  for name, pose in CANDIDATES.items():
    cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=TASK)
    cfg.events.pop("reset_lying")
    cfg.terminations = {}
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    robot = env.scene["robot"]
    names = list(robot.joint_names)
    target = robot.data.default_joint_pos.clone()
    for j, joint in enumerate(names):
      for key, deg in pose.items():
        if joint.endswith(key):
          target[:, j] = math.radians(deg)
    env.reset()
    n = args.num_envs
    # Sentado: tronco recto, pelvis a 12 cm del suelo, articulaciones ya en la pose pedida.
    ids = torch.arange(n, device=env.device, dtype=torch.int)
    quat = robot.data.default_root_state[:, 3:7].clone()
    pos = torch.zeros(n, 3, device=env.device)
    pos[:, 2] = 0.12
    pos += env.scene.env_origins[ids]
    robot.write_root_link_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=ids)
    robot.write_root_link_velocity_to_sim(torch.zeros(n, 6, device=env.device), env_ids=ids)
    robot.write_joint_state_to_sim(target, torch.zeros_like(target), env_ids=ids)
    action = target - robot.data.default_joint_pos
    pelvis = robot.find_sites([getup.PELVIS_SITE])[0][0]
    steps = int(args.seconds / env.step_dt)
    keep = int(1.0 / env.step_dt)
    z_log, tilt_log, speed_log = [], [], []
    model = env.sim.mj_model
    floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=env.device)
    touching = torch.zeros(len(model.body_names) if hasattr(model, "body_names") else model.nbody, device=env.device)
    for step in range(steps):
      env.step(action)
      if step >= steps - keep:
        z_log.append(robot.data.site_pos_w[:, pelvis, 2].clone())
        tilt_log.append(torch.rad2deg(torch.acos((-robot.data.projected_gravity_b[:, 2]).clamp(-1.0, 1.0))))
        speed_log.append(robot.data.root_com_lin_vel_b.norm(dim=1))
        data = env.sim.data
        geom = data.contact.geom.long()
        active = (torch.arange(geom.shape[0], device=env.device) < data.nacon[0]) & (data.contact.dist < 0.0)
        first, second = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
        other = torch.where(first, geom[:, 1], geom[:, 0])[active & (first | second)]
        body = torch.as_tensor(model.geom_bodyid, device=env.device)[other]
        touching += torch.bincount(body, minlength=touching.numel()).float()
    z, tilt, speed = torch.stack(z_log), torch.stack(tilt_log), torch.stack(speed_log)
    bodies = {model.body(int(i)).name: round(float(touching[i]) / (keep * n), 2) for i in torch.nonzero(touching).flatten().tolist()}
    results[name] = {
      "pose_deg": pose,
      "pelvis_z_m": {"mean": round(float(z.mean()), 4), "p10": round(float(torch.quantile(z.mean(0), 0.1)), 4), "p90": round(float(torch.quantile(z.mean(0), 0.9)), 4)},
      "trunk_tilt_deg_mean": round(float(tilt.mean()), 1),
      "speed_m_s_mean": round(float(speed.mean()), 4),
      "fraction_of_robots_tilt_below_25deg": round(float((tilt.mean(0) <= 25.0).float().mean()), 3),
      "bodies_touching_floor_contacts_per_step_per_robot": bodies,
    }
    print(name, json.dumps({k: v for k, v in results[name].items() if k != "pose_deg"}, ensure_ascii=False))
    env.close()

  out = RESULTS / "training_sit_pose.json"
  out.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print("Escrito:", out)


if __name__ == "__main__":
  main()
