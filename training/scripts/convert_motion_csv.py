"""Pasa una referencia de `results/retargeted/*.csv` (50 Hz) al formato de movimiento de mjlab (.npz) con el modelo de Danny (fase 6).

Es el `csv_to_npz` de mjlab, pero con Danny (el de mjlab es del G1 y sube el resultado a wandb). Además sube la base hasta que las suelas
tocan el suelo: el CSV trae la altura de la clave `stand` del MJCF, no la de la pose de inicio del entrenamiento (`robot.py`).

Uso (desde training/):
    uv run python scripts/convert_motion_csv.py --csv ../results/retargeted/cmu_141_16_lento1.8.csv --output data/motions/saludar.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
from mjlab.scene import Scene
from mjlab.scripts.csv_to_npz import MotionLoader
from mjlab.sim.sim import Simulation, SimulationCfg

from danny_training import robot as danny_robot
from danny_training.tracking import danny_tracking_env_cfg

JOINTS = [
  "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "left_ankle_roll",
  "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch", "right_ankle_roll",
  "waist_yaw", "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
  "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "neck_yaw", "neck_pitch",
]
FPS = 50


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--csv", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  args = parser.parse_args()
  device = "cuda:0" if torch.cuda.is_available() else "cpu"

  # Subida de la base: altura de inicio del entrenamiento menos la de la clave `stand`.
  key_z = float(danny_robot.get_danny_spec().compile().key("stand").qpos[2])
  lift = danny_robot._STAND_Z - key_z

  sim_cfg = SimulationCfg()
  sim_cfg.mujoco.timestep = 1.0 / FPS
  scene = Scene(danny_tracking_env_cfg().scene, device=device)
  model = scene.compile()
  sim = Simulation(num_envs=1, cfg=sim_cfg, model=model, device=device)
  scene.initialize(sim.mj_model, sim.model, sim.data)
  robot = scene["robot"]
  idx = robot.find_joints(JOINTS, preserve_order=True)[0]

  motion = MotionLoader(motion_file=str(args.csv), input_fps=FPS, output_fps=FPS, device=device)
  log: dict[str, list] = {k: [] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w")}
  scene.reset()
  for i in range(motion.output_frames):
    root = robot.data.default_root_state.clone()
    root[:, 0:3] = motion.motion_base_poss[i : i + 1]
    root[:, 2] += lift
    root[:, 3:7] = motion.motion_base_rots[i : i + 1]
    root[:, 7:10] = motion.motion_base_lin_vels[i : i + 1]
    root[:, 10:] = motion.motion_base_ang_vels[i : i + 1]
    robot.write_root_state_to_sim(root)
    q = robot.data.default_joint_pos.clone()
    dq = robot.data.default_joint_vel.clone()
    q[:, idx] = motion.motion_dof_poss[i : i + 1]
    dq[:, idx] = motion.motion_dof_vels[i : i + 1]
    robot.write_joint_state_to_sim(q, dq)
    sim.forward()
    scene.update(sim.mj_model.opt.timestep)
    log["joint_pos"].append(robot.data.joint_pos[0].cpu().numpy().copy())
    log["joint_vel"].append(robot.data.joint_vel[0].cpu().numpy().copy())
    log["body_pos_w"].append(robot.data.body_link_pos_w[0].cpu().numpy().copy())
    log["body_quat_w"].append(robot.data.body_link_quat_w[0].cpu().numpy().copy())
    log["body_lin_vel_w"].append(robot.data.body_link_lin_vel_w[0].cpu().numpy().copy())
    log["body_ang_vel_w"].append(robot.data.body_link_ang_vel_w[0].cpu().numpy().copy())
  out = {k: np.stack(v, axis=0) for k, v in log.items()}
  out["fps"] = np.array([FPS])
  args.output.parent.mkdir(parents=True, exist_ok=True)
  np.savez(args.output, **out)
  print(f"Escrito {args.output}: {motion.output_frames} fotogramas a {FPS} Hz, base subida {lift * 1000:.1f} mm; cuerpos {out['body_pos_w'].shape[1]}")


if __name__ == "__main__":
  main()
