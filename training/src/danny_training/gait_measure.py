"""Mide la marcha de Danny con una orden fija: avance, giro, paso alterno y reparto entre pies. Lo usan los scripts de medida.

Para cada entorno que no se cae mide, tras descartar los primeros `WARMUP_S` s:
- error de velocidad respecto a la orden y velocidad media de avance, en el centro de masas de `base_link`;
- velocidad de giro media y su error respecto a la orden;
- velocidad lineal y de giro medias en valor absoluto (con orden cero, cuánto se mueve);
- fracción del tiempo con un solo pie apoyado;
- apoyos por pie y segundo, y fracción de apoyos que siguen a un apoyo del otro pie (1,0 es paso alterno perfecto);
- fracción de los apoyos que hace el pie izquierdo (0,5 es reparto igual).
"""

from __future__ import annotations

import re
from pathlib import Path

import torch

from danny_training.crouch import STAND_PELVIS_Z_M
from danny_training.evaluation import TASK, eval_env_cfg, fix_command, load_policy

WARMUP_S = 2.0


def measure_gait(
  checkpoint: Path,
  *,
  forward: float = 0.25,
  turn: float = 0.0,
  lateral: float = 0.0,
  height: float | None = 0.0,
  vin: float | None = None,
  nominal_mass: bool = False,
  drop_link_contacts: str | None = None,
  pushes: bool = False,
  push_speed: float | None = None,
  num_envs: int = 256,
  seconds: float = 20.0,
) -> dict:
  """Corre `checkpoint` con una orden fija y devuelve las medidas. Las opciones de diagnóstico quitan partes del modelo."""
  env_cfg = eval_env_cfg(num_envs, seconds, vin, pushes, push_speed)
  fix_command(env_cfg, forward, turn, lateral, height)
  if nominal_mass:
    for event in ("randomize_body_mass", "base_com"):
      env_cfg.events.pop(event)
  if drop_link_contacts is not None:
    robot_cfg = env_cfg.scene.entities["robot"]
    base_spec_fn = robot_cfg.spec_fn

    def spec_without_link_contacts():
      spec = base_spec_fn()
      for pair in list(spec.pairs):
        if re.search(drop_link_contacts, pair.geomname1) or re.search(drop_link_contacts, pair.geomname2):
          spec.delete(pair)
      return spec

    robot_cfg.spec_fn = spec_without_link_contacts
  env, policy = load_policy(env_cfg, checkpoint)

  u = env.unwrapped
  robot = u.scene["robot"]
  contact = u.scene["feet_ground_contact"]
  n, dt = num_envs, u.step_dt
  steps = int(seconds / dt)
  warmup = int(WARMUP_S / dt)
  device = u.device

  alive = torch.ones(n, dtype=torch.bool, device=device)
  prev = None
  last_foot = torch.full((n,), -1, dtype=torch.long, device=device)
  touchdowns = torch.zeros(n, 2, device=device)
  alternating = torch.zeros(n, device=device)
  single = torch.zeros(n, device=device)
  vx_sum = torch.zeros(n, device=device)
  err_sum = torch.zeros(n, device=device)
  speed_sum = torch.zeros(n, device=device)
  yaw_sum = torch.zeros(n, device=device)
  yaw_err_sum = torch.zeros(n, device=device)
  yaw_abs_sum = torch.zeros(n, device=device)
  knee_ids = robot.find_joints(["left_knee", "right_knee"], preserve_order=True)[0]
  knee_stance_sum = torch.zeros(n, 2, device=device)
  knee_stance_steps = torch.zeros(n, 2, device=device)
  pelvis_ids = robot.find_sites(["pelvis_bottom"])[0]
  pelvis_sum = torch.zeros(n, device=device)
  lean_sum = torch.zeros(n, device=device)
  counted = 0

  obs = env.get_observations()
  with torch.inference_mode():
    for i in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      alive &= ~u.reset_terminated.bool()
      found = contact.data.found.reshape(n, -1)[:, :2] > 0
      if i >= warmup:
        counted += 1
        cmd = u.command_manager.get_command("twist")
        vel = robot.data.root_com_lin_vel_b[:, :2]
        vx_sum += vel[:, 0]
        err_sum += (vel - cmd[:, :2]).norm(dim=1)
        speed_sum += vel.norm(dim=1)
        yaw = robot.data.root_link_ang_vel_b[:, 2]
        yaw_sum += yaw
        yaw_err_sum += (yaw - cmd[:, 2]).abs()
        yaw_abs_sum += yaw.abs()
        pelvis_sum += robot.data.site_pos_w[:, pelvis_ids[0], 2]
        lean_sum += torch.asin(robot.data.projected_gravity_b[:, 0].clamp(-1.0, 1.0))
        single += (found.sum(dim=1) == 1).float()
        knee_stance_sum += robot.data.joint_pos[:, knee_ids] * found.float()
        knee_stance_steps += found.float()
        if prev is not None:
          for foot in (0, 1):
            down = found[:, foot] & ~prev[:, foot]
            touchdowns[:, foot] += down.float()
            alternating += (down & (last_foot == 1 - foot)).float()
            last_foot = torch.where(down, torch.full_like(last_foot, foot), last_foot)
      prev = found.clone()

  ok = alive
  seconds_counted = counted * dt
  total_td = touchdowns.sum(dim=1)
  alt_frac = torch.where(total_td > 1, alternating / (total_td - 1).clamp(min=1), torch.zeros_like(total_td))

  def mean(x):
    return round(float(x[ok].mean()), 4) if ok.any() else None

  result = {
    "checkpoint": str(checkpoint),
    "task": TASK,
    "num_envs": n,
    "seconds_simulated": seconds,
    "vin_v": vin,
    "pushes": pushes or push_speed is not None,
    "push_speed_m_s": push_speed if push_speed is not None else (0.2 if pushes else None),
    "nominal_mass": nominal_mass,
    "dropped_link_contacts": drop_link_contacts,
    "forward_command_m_s": forward,
    "lateral_command_m_s": lateral,
    "turn_command_rad_s": turn,
    "height_command_m": height,
    "fraction_envs_not_fallen": round(float(ok.float().mean()), 4),
    "mean_forward_velocity_m_s": mean(vx_sum / counted),
    "mean_xy_velocity_error_m_s": mean(err_sum / counted),
    "mean_xy_speed_m_s": mean(speed_sum / counted),
    "mean_yaw_rate_rad_s": mean(yaw_sum / counted),
    "mean_yaw_rate_error_rad_s": mean(yaw_err_sum / counted),
    "mean_abs_yaw_rate_rad_s": mean(yaw_abs_sum / counted),
    "fraction_time_single_support": mean(single / counted),
    "touchdowns_per_foot_per_s": mean(total_td / 2 / seconds_counted),
    "fraction_alternating_touchdowns": mean(alt_frac),
    "mean_pelvis_height_m": mean(pelvis_sum / counted),
    "mean_pelvis_height_error_m": mean((pelvis_sum / counted - (STAND_PELVIS_Z_M + height)).abs()) if height is not None else None,
    "mean_trunk_lean_deg": mean(torch.rad2deg(lean_sum / counted)),
    "mean_knee_angle_stance_deg": mean(torch.rad2deg(knee_stance_sum.sum(dim=1) / knee_stance_steps.sum(dim=1).clamp(min=1))),
    "fraction_touchdowns_left": round(float(touchdowns[ok, 0].sum() / touchdowns[ok].sum().clamp(min=1)), 4) if ok.any() else None,
    "note": f"{'Con' if pushes or push_speed is not None else 'Sin'} empujones. Orden fija. Se descartan los primeros {WARMUP_S} s. Solo entornos que no caen. Velocidad del centro de masas de base_link.",
  }
  env.close()
  return result
