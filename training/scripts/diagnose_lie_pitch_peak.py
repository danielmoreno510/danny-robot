"""Diagnostica de dónde sale el pico de giro de cabeceo al tumbarse (fase 5) y guarda el resultado en results/.

Corre la política de tumbarse (sin entrenar) con muchos robots y, para cada uno, mide:
  1. Cuándo ocurre el pico de giro de cabeceo de la pelvis (el que se mide en el criterio) respecto al primer contacto con el suelo de tronco
     (`torso_1`), cuello o cabeza: `antes` (más de 0,1 s antes), `golpe` (de 0,1 s antes a 0,2 s después), `despues` (rebote, más de 0,2 s
     después) o `sin_contacto`. Los 0,1 s previos cubren que el contacto se detecta al final del paso de 0,02 s. También si cae en el primer
     medio segundo del episodio (el arranque).
  2. De qué cuerpo (pelvis, tronco, cuello, cabeza) y de qué eje es el mayor giro de todo el episodio. Los ejes son los de la pelvis: x balanceo,
     y cabeceo, z guiñada. Si es de balanceo o guiñada, el robot se tuerce y no es la bajada hacia atrás.
  3. La velocidad lineal del centro de masa de tronco, cuello y cabeza en su primer contacto con el suelo (máximo de los 3 pasos anteriores al
     contacto incluido, para no medir la velocidad ya frenada por el golpe) y su componente vertical de bajada, en m/s.
  4. Fracción de robots con pico de cabeceo ≤ límite si se mide solo antes del contacto, y si se excluye la ventana del golpe.
Con `--videos N` graba en results/videos/ los N robots con el pico de cabeceo más alto.

Uso (desde training/):
    uv run python scripts/diagnose_lie_pitch_peak.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase5_tumbarse_suave_6.6V --vin 6.6 --videos 2
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio
import mujoco
import torch
from mjlab.utils.lab_api.math import quat_apply_inverse

from danny_training.evaluation import RESULTS, eval_env_cfg, load_policy
from danny_training import lie

LIE_TASK = "Mjlab-LieDown-Danny"
IMPACT_BODIES = ("torso_1", "neck_link_1", "head_1")
AXES = ("balanceo", "cabeceo", "guinada")
LEAD_S, AFTER_S, START_S, SPEED_STEPS = 0.1, 0.2, 0.5, 3


def quantiles(x: torch.Tensor) -> dict:
  return {"median": round(float(x.median()), 3), "p95": round(float(torch.quantile(x, 0.95)), 3), "max": round(float(x.max()), 3)}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True)
  parser.add_argument("--vin", type=float, default=None)
  parser.add_argument("--num-envs", type=int, default=4096)
  parser.add_argument("--seconds", type=float, default=10.0)
  parser.add_argument("--limit", type=float, default=lie.MAX_PITCH_RATE_RAD_S)
  parser.add_argument("--videos", type=int, default=0)
  args = parser.parse_args()

  cfg = eval_env_cfg(args.num_envs, args.seconds + 1.0, args.vin, task=LIE_TASK)
  env, policy = load_policy(cfg, args.checkpoint, render_mode="rgb_array" if args.videos else None, task=LIE_TASK)
  u = env.unwrapped
  device, n, dt = u.device, args.num_envs, u.step_dt
  robot = u.scene["robot"]
  steps = int(args.seconds / dt)
  model = u.sim.mj_model
  floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=device)
  body_of_geom = torch.as_tensor(model.geom_bodyid, device=device)
  model_ids = torch.tensor([model.body(f"robot/{name}").id for name in IMPACT_BODIES], device=device)
  robot_ids = robot.find_bodies(list(IMPACT_BODIES), preserve_order=True)[0]

  # Cuerpos con giro: 0 pelvis (raíz), 1 tronco, 2 cuello, 3 cabeza. Ejes de la pelvis.
  ang = torch.zeros(n, steps, 1 + len(IMPACT_BODIES), 3, device=device)
  speed = torch.zeros(n, steps, len(IMPACT_BODIES), device=device)
  down = torch.zeros(n, steps, len(IMPACT_BODIES), device=device)
  touch = torch.zeros(n, steps, len(IMPACT_BODIES), dtype=torch.bool, device=device)
  trajectory = torch.zeros(n, steps, u.sim.data.qpos.shape[1], device=device) if args.videos else None
  obs = env.get_observations()
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      data = robot.data
      ang[:, step, 0] = data.root_link_ang_vel_b
      world_ang = data.body_link_ang_vel_w[:, robot_ids]
      for k in range(len(IMPACT_BODIES)):
        ang[:, step, 1 + k] = quat_apply_inverse(data.root_link_quat_w, world_ang[:, k])
      lin = data.body_com_lin_vel_w[:, robot_ids]
      speed[:, step] = lin.norm(dim=2)
      down[:, step] = (-lin[..., 2]).clamp(min=0.0)

      sim = u.sim.data
      geom = sim.contact.geom.long()
      active = (torch.arange(geom.shape[0], device=device) < sim.nacon[0]) & (sim.contact.dist < 0.0)
      first, second = torch.isin(geom[:, 0], floor), torch.isin(geom[:, 1], floor)
      hit = active & (first | second)
      other = torch.where(first, geom[:, 1], geom[:, 0])
      world = sim.contact.worldid.long()
      per_body = torch.zeros(n, model.nbody, device=device)
      per_body[world[hit], body_of_geom[other[hit]]] += 1.0
      touch[:, step] = per_body[:, model_ids] > 0
      if trajectory is not None:
        trajectory[:, step] = sim.qpos

  idx = torch.arange(steps, device=device)[None, :]
  pitch = ang[:, :, 0, 1].abs()
  peak, peak_step = pitch.max(dim=1)
  any_touch = touch.any(dim=2)
  has_contact = any_touch.any(dim=1)
  first_contact = torch.where(has_contact, any_touch.int().argmax(dim=1), torch.full((n,), steps, device=device))
  lead, after = int(round(LEAD_S / dt)), int(round(AFTER_S / dt))
  rel = peak_step - first_contact
  before_mask = has_contact & (rel < -lead)
  impact_mask = has_contact & (rel >= -lead) & (rel <= after)
  later_mask = has_contact & (rel > after)
  start_mask = peak_step < int(round(START_S / dt))

  # Mayor giro de todo el episodio por cuerpo y eje.
  per_axis_peak = ang.abs().amax(dim=1)  # (n, cuerpos, 3)
  flat_peak, flat_arg = per_axis_peak.reshape(n, -1).max(dim=1)
  body_names = ["pelvis", *IMPACT_BODIES]
  worst_body, worst_axis = flat_arg // 3, flat_arg % 3
  axis_counts = {AXES[a]: round(float((worst_axis == a).float().mean()), 4) for a in range(3)}
  body_counts = {body_names[b]: round(float((worst_body == b).float().mean()), 4) for b in range(len(body_names))}
  pelvis_peak_axis = per_axis_peak[:, 0, :]
  pelvis_axis_counts = {AXES[a]: round(float((pelvis_peak_axis.argmax(dim=1) == a).float().mean()), 4) for a in range(3)}

  # Velocidad al primer contacto de cada cuerpo (máximo de los pasos anteriores, contacto incluido).
  impact = {}
  for k, name in enumerate(IMPACT_BODIES):
    touched = touch[:, :, k]
    has = touched.any(dim=1)
    t0 = touched.int().argmax(dim=1)
    window = (t0[:, None] - torch.arange(SPEED_STEPS, device=device)[None, :]).clamp(min=0)
    v = torch.gather(speed[:, :, k], 1, window).max(dim=1).values
    vz = torch.gather(down[:, :, k], 1, window).max(dim=1).values
    impact[name] = {
      "fraction_of_robots_that_touch_floor": round(float(has.float().mean()), 4),
      "first_contact_time_s_median": round(float((t0[has].float() * dt).median()), 2) if has.any() else None,
      "speed_m_s": quantiles(v[has]) if has.any() else None,
      "downward_speed_m_s": quantiles(vz[has]) if has.any() else None,
    }

  near = (idx >= (first_contact[:, None] - lead)) & (idx <= (first_contact[:, None] + after))
  peak_before = (pitch * (idx < (first_contact[:, None] - lead))).max(dim=1).values
  peak_no_impact = (pitch * ~near).max(dim=1).values
  peak_after_start = (pitch * (idx >= int(round(START_S / dt)))).max(dim=1).values
  out = {
    "checkpoint": str(args.checkpoint),
    "vin_v": args.vin if args.vin is not None else "aleatoria 6 a 8,4",
    "num_envs": n,
    "seconds": args.seconds,
    "pitch_rate_limit_rad_s": args.limit,
    "pitch_peak_rad_s": quantiles(peak),
    "fraction_pitch_peak_at_most_limit": round(float((peak <= args.limit).float().mean()), 4),
    "fraction_robots_touching_floor_with_torso_neck_or_head": round(float(has_contact.float().mean()), 4),
    "first_contact_time_s": {"median": round(float((first_contact[has_contact].float() * dt).median()), 2)} if has_contact.any() else None,
    "peak_time_s": {"median": round(float(peak_step.float().median() * dt), 2)},
    "when_is_the_pitch_peak": {
      "windows": f"antes: >{LEAD_S} s antes del primer contacto; golpe: de {LEAD_S} s antes a {AFTER_S} s despues; despues: mas tarde; sin_contacto: nunca toca",
      "antes": round(float(before_mask.float().mean()), 4),
      "golpe": round(float(impact_mask.float().mean()), 4),
      "despues_rebote": round(float(later_mask.float().mean()), 4),
      "sin_contacto": round(float((~has_contact).float().mean()), 4),
      "in_first_0.5s_start": round(float(start_mask.float().mean()), 4),
    },
    "alternative_criteria_fraction_at_most_limit": {
      "pitch_peak_only_before_contact": round(float((peak_before <= args.limit).float().mean()), 4),
      "pitch_peak_excluding_impact_window": round(float((peak_no_impact <= args.limit).float().mean()), 4),
      "pitch_peak_excluding_first_0.5s": round(float((peak_after_start <= args.limit).float().mean()), 4),
    },
    "pitch_peak_before_contact_rad_s": quantiles(peak_before),
    "pitch_peak_excluding_impact_window_rad_s": quantiles(peak_no_impact),
    "largest_rotation_of_whole_episode": {
      "note": "mayor |giro| entre pelvis, tronco, cuello y cabeza y los tres ejes de la pelvis (x balanceo, y cabeceo, z guinada)",
      "value_rad_s": quantiles(flat_peak),
      "axis_fraction_of_robots": axis_counts,
      "body_fraction_of_robots": body_counts,
      "pelvis_largest_axis_fraction_of_robots": pelvis_axis_counts,
      "pelvis_peak_by_axis_rad_s": {AXES[a]: quantiles(pelvis_peak_axis[:, a]) for a in range(3)},
    },
    "impact_speed_at_first_floor_contact": impact,
  }
  # Los robots con el pico más alto: cuándo y dónde.
  order = torch.argsort(peak, descending=True)
  top = []
  for i in order[:5].tolist():
    top.append({
      "robot": i, "pitch_peak_rad_s": round(float(peak[i]), 2), "peak_time_s": round(float(peak_step[i]) * dt, 2),
      "first_contact_time_s": round(float(first_contact[i]) * dt, 2) if bool(has_contact[i]) else None,
      "largest_rotation": f"{body_names[int(worst_body[i])]}/{AXES[int(worst_axis[i])]} {float(flat_peak[i]):.2f} rad/s",
    })
  out["top_pitch_peak_robots"] = top

  if args.videos:
    videos = RESULTS / "videos"
    videos.mkdir(parents=True, exist_ok=True)
    out["videos"] = []
    for row in top[: args.videos]:
      path = videos / f"{args.name}_robot{row['robot']}_pico{row['pitch_peak_rad_s']}_t{row['peak_time_s']}s.mp4"
      with imageio.get_writer(path, fps=int(round(u.metadata["render_fps"])), macro_block_size=1) as writer:
        for t in range(steps):
          u.sim.data.qpos[0] = trajectory[row["robot"], t]
          u.sim.data.qvel[0] = 0.0
          u.sim.forward()
          writer.append_data(u.render())
      out["videos"].append(path.name)
      print(path.name, f"{path.stat().st_size / 1e6:.2f} MB")

  path = RESULTS / f"training_tumbarse_pico_{args.name}.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(out, indent=1, ensure_ascii=False))
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
