"""Mide un movimiento de la fase 6 (seguimiento con física) y guarda el resultado en results/training_tracking_<name>.json.

Cada robot empieza en el primer fotograma de la referencia (sin ruido de inicio), con la aleatorización completa de la etapa 3 y los
empujones de ±0,20 m/s en XY de la fase 2, y sigue la referencia entera (`--motion` es el .npz de `convert_motion_csv.py`). Mide, con la política
determinista y el ruido de observación del entrenamiento:
  1. No cae: fracción de robots sin ninguna terminación del entrenamiento (altura del ancla o de un extremo a más de 8 cm de la referencia,
     orientación del ancla) durante todo el movimiento. Informa además de cuántos pasan de 60° de inclinación del tronco.
  2. Sigue la referencia: error de manos, codos y pies respecto a la referencia, en el marco de la pelvis (posición y guiñada de la
     pelvis del robot alineadas con las de la referencia, así que el desplazamiento de la base no cuenta), mediana y p95 de todos los robots
     y fotogramas. Mano = sitio `left_hand`/`right_hand`; codo = cuerpo `*_elbow_link_1`; pie = cuerpo `*_foot_1`.
  3. Saturación: par por encima del 90 % del bloqueo de la tensión de cada robot; racha más larga por robot y articulación.
  4. Pies: en los movimientos de pie (`--legs fixed`), fracción de robots sin ningún paso (ninguna suela sube más de 1 cm sobre el suelo ni se
     desplaza más de 3 cm de su sitio); en los de piernas (`--legs moving`), el deslizamiento máximo de la suela mientras apoya (acumulado por
     apoyo; limite 1 cm). Definiciones mías, marcadas como interpretación del criterio.
Con `--vin` fija la tensión (V); sin ella se sortea de 6 a 8,4 V.

Uso (desde training/):
    uv run python scripts/measure_tracking.py --checkpoint logs/rsl_rl/danny_tracking/<run>/model_XXXX.pt --motion data/motions/saludar.npz --name saludar_6.6V --vin 6.6 --legs fixed
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import torch
from mjlab.utils.lab_api.math import quat_apply, quat_apply_inverse, quat_inv, quat_mul, yaw_quat

from danny_training.actuator import FrictionDRBamActuator, stall_torque_nm
from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training.lie_measure import longest_run

TASK = "Mjlab-Tracking-Danny"
SATURATED = 0.9
HAND_SITES = {"left": "left_hand", "right": "right_hand"}
HAND_BODIES = {"left": "left_forearm_1", "right": "right_forearm_1"}
ELBOW_BODIES = {"left": "left_elbow_link_1", "right": "right_elbow_link_1"}
FOOT_BODIES = {"left": "left_foot_1", "right": "right_foot_1"}
SOLE_SITES = {"left": "left_sole", "right": "right_sole"}
TILT_FALL_DEG = 60.0
STEP_LIFT_M, STEP_SLIDE_M, STANCE_SLIDE_M = 0.01, 0.03, 0.01
POS_MEDIAN_M, POS_P95_M = 0.02, 0.04


def quantiles(x: torch.Tensor) -> dict:
  x = x[~torch.isnan(x)].float()
  return {"median": round(float(x.median()), 4), "p95": round(float(torch.quantile(x[:: max(1, x.numel() // 2_000_000)], 0.95)), 4), "max": round(float(x.max()), 4)}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--motion", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--legs", choices=("fixed", "moving"), required=True)
  parser.add_argument("--push-speed", type=float, default=0.2)
  args = parser.parse_args()

  frames = int(np.load(args.motion)["joint_pos"].shape[0])
  steps = frames - 1
  cfg = eval_env_cfg(args.num_envs, steps / 50.0 + 2.0, args.vin, push_speed=args.push_speed, task=TASK)
  motion_cfg = cfg.commands["motion"]
  motion_cfg.motion_file = str(args.motion)
  motion_cfg.sampling_mode = "start"
  motion_cfg.pose_range = {}
  motion_cfg.velocity_range = {}
  motion_cfg.joint_position_range = (0.0, 0.0)
  env, policy = load_policy(cfg, args.checkpoint, task=TASK)
  u = env.unwrapped
  robot = u.scene["robot"]
  cmd = u.command_manager.get_term("motion")
  n, dt, device = args.num_envs, u.step_dt, u.device
  origins = u.scene.env_origins

  names = list(robot.body_names)
  body = lambda b: names.index(b)  # noqa: E731
  site_idx = {s: robot.site_names.index(s) for s in (*HAND_SITES.values(), *SOLE_SITES.values())}
  stall = stall_torque_nm(next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator)).vin_tensor)
  ref = {k: torch.tensor(np.load(args.motion)[k], device=device) for k in ("body_pos_w", "body_quat_w")}

  # Posición de la mano en el marco de su cuerpo (constante): se toma del robot al empezar.
  obs = env.get_observations()
  data = robot.data
  local_hand = {}
  for side in ("left", "right"):
    b = body(HAND_BODIES[side])
    site = data.site_pos_w[:, site_idx[HAND_SITES[side]]]
    local_hand[side] = quat_apply_inverse(data.body_link_quat_w[:, b], site - data.body_link_pos_w[:, b])[:1]

  keys = [f"{s}_{p}" for p in ("hand", "elbow", "foot") for s in ("left", "right")]
  err = {k: torch.full((n, steps), float("nan"), device=device) for k in keys}
  sat = torch.zeros(n, steps, data.actuator_force.shape[1], dtype=torch.bool, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  tilted = torch.zeros(n, dtype=torch.bool, device=device)
  alive_steps = torch.zeros(n, steps, dtype=torch.bool, device=device)
  sole_lift = {s: torch.zeros(n, device=device) for s in ("left", "right")}
  sole_move = {s: torch.zeros(n, device=device) for s in ("left", "right")}
  sole_start = {}
  stance_slide = {s: torch.zeros(n, device=device) for s in ("left", "right")}
  slide_acc = {s: torch.zeros(n, device=device) for s in ("left", "right")}
  prev_sole = {}
  contact = u.scene["feet_ground_contact"]

  with torch.inference_mode():
    for t in range(steps):
      obs, _, dones, extras = env.step(policy(obs))
      terminated = u.termination_manager.terminated
      alive = ~fell
      alive_steps[:, t] = alive
      fell |= terminated
      idx = min(t + 1, frames - 1)
      d = robot.data
      base_pos = d.body_link_pos_w[:, 0] - origins
      base_quat = d.body_link_quat_w[:, 0]
      rb_pos, rb_quat = ref["body_pos_w"][idx, 0], ref["body_quat_w"][idx, 0]
      # Alinear la referencia con la pelvis del robot: misma posición XY y guiñada.
      dyaw = yaw_quat(quat_mul(base_quat, quat_inv(rb_quat.expand_as(base_quat))))

      def aligned_ref(pos_ref: torch.Tensor) -> torch.Tensor:
        rel = pos_ref - rb_pos
        return base_pos + quat_apply(dyaw, rel.expand_as(base_pos))

      for side in ("left", "right"):
        hb, eb, fb = body(HAND_BODIES[side]), body(ELBOW_BODIES[side]), body(FOOT_BODIES[side])
        hand_r = d.site_pos_w[:, site_idx[HAND_SITES[side]]] - origins
        hand_ref = ref["body_pos_w"][idx, hb] + quat_apply(ref["body_quat_w"][idx, hb].expand(1, 4), local_hand[side])[0]
        err[f"{side}_hand"][:, t] = (hand_r - aligned_ref(hand_ref)).norm(dim=-1)
        err[f"{side}_elbow"][:, t] = (d.body_link_pos_w[:, eb] - origins - aligned_ref(ref["body_pos_w"][idx, eb])).norm(dim=-1)
        err[f"{side}_foot"][:, t] = (d.body_link_pos_w[:, fb] - origins - aligned_ref(ref["body_pos_w"][idx, fb])).norm(dim=-1)
        sole = d.site_pos_w[:, site_idx[SOLE_SITES[side]]] - origins
        if t == 0:
          sole_start[side] = sole.clone()
          prev_sole[side] = sole.clone()
        sole_lift[side] = torch.maximum(sole_lift[side], sole[:, 2] - sole_start[side][:, 2])
        sole_move[side] = torch.maximum(sole_move[side], (sole[:, :2] - sole_start[side][:, :2]).norm(dim=-1))
        on_ground = contact.data.found.reshape(n, -1)[:, 0 if side == "left" else 1] > 0
        step_xy = (sole[:, :2] - prev_sole[side][:, :2]).norm(dim=-1)
        slide_acc[side] = torch.where(on_ground, slide_acc[side] + step_xy, torch.zeros_like(step_xy))
        stance_slide[side] = torch.maximum(stance_slide[side], slide_acc[side])
        prev_sole[side] = sole.clone()
      sat[:, t] = (d.actuator_force.abs() / stall) > SATURATED
      gravity_z = quat_apply_inverse(base_quat, torch.tensor([0.0, 0.0, -1.0], device=device).expand(n, 3))[:, 2]
      tilted |= torch.acos((-gravity_z).clamp(-1, 1)) > math.radians(TILT_FALL_DEG)
      # Los robots caídos se dejan de contar: sus errores ya no son del movimiento.
      for k in keys:
        err[k][fell, t] = float("nan")
      sat[fell, t] = False

  ok = ~fell
  joint_names = list(robot.joint_names)
  runs = longest_run(sat).float() * dt
  worst = runs.max(dim=1).values
  per_joint = {j: round(float(runs[ok, i].max()), 2) for i, j in enumerate(joint_names) if float(runs[ok, i].max()) > 0.5}
  all_err = {k: quantiles(err[k]) for k in keys}
  groups = {g: quantiles(torch.cat([err[f"left_{g}"].flatten(), err[f"right_{g}"].flatten()])) for g in ("hand", "elbow", "foot")}
  no_fall = float(ok.float().mean())
  sat_ok = float((worst[ok] <= 0.5).float().mean()) if bool(ok.any()) else 0.0
  sat_2s = float((worst[ok] <= 2.0).float().mean()) if bool(ok.any()) else 0.0
  if args.legs == "fixed":
    stepped = torch.stack([(sole_lift[s] > STEP_LIFT_M) | (sole_move[s] > STEP_SLIDE_M) for s in ("left", "right")]).any(dim=0)
    feet = {"definition": f"paso = suela a más de {STEP_LIFT_M * 100:.0f} cm de altura o a más de {STEP_SLIDE_M * 100:.0f} cm de su sitio", "fraction_without_step": round(float((~stepped[ok]).float().mean()), 4), "required": 0.95}
    feet_pass = feet["fraction_without_step"] >= 0.95
  else:
    slide = torch.maximum(stance_slide["left"], stance_slide["right"])
    feet = {"definition": "deslizamiento de la suela acumulado en cada apoyo (m)", "stance_slide_m": quantiles(slide[ok]), "required_max_m": STANCE_SLIDE_M}
    feet_pass = float(slide[ok].max()) <= STANCE_SLIDE_M
  criteria = {
    "no_fall_fraction": {"value": round(no_fall, 4), "required": 0.95, "passes": no_fall >= 0.95},
    "hands_elbows_feet_median_m": {"value": {g: groups[g]["median"] for g in groups}, "required_max": POS_MEDIAN_M, "passes": all(groups[g]["median"] <= POS_MEDIAN_M for g in groups)},
    "hands_elbows_feet_p95_m": {"value": {g: groups[g]["p95"] for g in groups}, "required_max": POS_P95_M, "passes": all(groups[g]["p95"] <= POS_P95_M for g in groups)},
    "saturation_run_at_most_0.5s": {"value": round(sat_ok, 4), "required": 0.99, "passes": sat_ok >= 0.99},
    "saturation_no_robot_over_2s": {"value": round(sat_2s, 4), "required": 1.0, "passes": sat_2s >= 1.0},
    "feet": {"value": feet, "passes": feet_pass},
  }
  out = {
    "checkpoint": str(args.checkpoint), "motion": str(args.motion), "legs": args.legs, "num_envs": n, "seconds": round(steps * dt, 2),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4", "push_speed_m_s": args.push_speed,
    "tilt_over_60deg_fraction": round(float(tilted.float().mean()), 4),
    "error_by_body_m": all_err, "error_by_group_m": groups,
    "saturated_joints_run_over_0.5s_max_s": per_joint,
    "criteria": criteria, "passes_measurable_criteria": all(c["passes"] for c in criteria.values()),
    "not_checked": "vídeo aprobado por el dueño; ONNX verificado; robot real",
  }
  path = RESULTS / f"training_tracking_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: v for k, v in out.items() if k != "error_by_body_m"}, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
