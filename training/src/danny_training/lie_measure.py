"""Medida de tumbarse (fase 5) con el criterio de impacto y giro del 8 de noviembre; la usan `measure_lie.py` y `measure_sit_lie_chain.py`.

`LieRecorder` guarda por paso lo que hace falta y `lie_report` calcula el informe. Criterio (PROJECT_HANDOFF.md, Fase 5, tumbarse):
  - queda tumbado el 95 % o más (eje x del tronco a -0,9 o menos, inclinación lateral de 0,3 o menos, velocidad lineal media de 0,05 m/s o
    menos y angular de 0,1 rad/s o menos, 3 s seguidos);
  - el 95 % o más con el tronco tocando el suelo a 0,8 m/s o menos (máximo de los 3 pasos anteriores al primer contacto, contacto incluido);
  - el 99 % o más sin que la cabeza toque el suelo en ningún momento;
  - ningún giro de la pelvis de más de 3 rad/s en ningún eje una vez pasado el reinicio (se informa también el porcentaje de robots que lo cumple);
  - saturación de par (sobre el 90 % del bloqueo): el 99 % o más de los robots sin pasar de 0,5 s seguidos mientras se tumban y con 5 % o menos
    estando tumbados, y ningún robot más de 2 s.
"""

from __future__ import annotations

import mujoco
import torch
from mjlab.utils.lab_api.math import quat_apply_inverse

from danny_training import lie

TRUNK, NECK, HEAD = "torso_1", "neck_link_1", "head_1"
BODIES = (TRUNK, NECK, HEAD)
AXES = ("balanceo", "cabeceo", "guinada")
WINDOW_S, SATURATED = 3.0, 0.9
MAX_TRUNK_IMPACT_M_S = 0.8
MAX_PELVIS_RATE_RAD_S = 3.0
SPEED_STEPS = 3
TRUNK_IMPACT_FRACTION, NO_HEAD_FRACTION, LYING_FRACTION, SATURATION_FRACTION = 0.95, 0.99, 0.95, 0.99


def longest_run(flags: torch.Tensor) -> torch.Tensor:
  """Racha más larga de True a lo largo de la dimensión 1 de `flags` (entornos, pasos, articulaciones); devuelve (entornos, articulaciones)."""
  current = torch.zeros_like(flags[:, 0], dtype=torch.int32)
  best = torch.zeros_like(current)
  for t in range(flags.shape[1]):
    current = (current + 1) * flags[:, t].to(torch.int32)
    best = torch.maximum(best, current)
  return best


class LieRecorder:
  """Guarda por paso: saturación, gravedad, velocidades, giro de la pelvis en sus ejes y contactos de tronco, cuello y cabeza con el suelo."""

  def __init__(self, env, steps: int, stall: torch.Tensor):
    self.u = env.unwrapped
    self.robot = self.u.scene["robot"]
    device, n = self.u.device, self.u.num_envs
    self.n, self.steps, self.stall, self.device = n, steps, stall, device
    self.sat = torch.zeros(n, steps, len(self.robot.joint_names), dtype=torch.bool, device=device)
    self.gravity = torch.zeros(n, steps, 2, device=device)
    self.speed = torch.zeros(n, steps, device=device)
    self.angular = torch.zeros(n, steps, device=device)
    self.pelvis_rate = torch.zeros(n, steps, 3, device=device)
    self.body_speed = torch.zeros(n, steps, len(BODIES), device=device)
    self.touch = torch.zeros(n, steps, len(BODIES), dtype=torch.bool, device=device)
    model = self.u.sim.mj_model
    self.model = model
    self.floor = torch.tensor([i for i in range(model.ngeom) if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE], device=device)
    self.body_of_geom = torch.as_tensor(model.geom_bodyid, device=device)
    self.model_ids = torch.tensor([model.body(f"robot/{name}").id for name in BODIES], device=device)
    self.robot_ids = self.robot.find_bodies(list(BODIES), preserve_order=True)[0]

  def record(self, t: int) -> None:
    data = self.robot.data
    self.sat[:, t] = (data.actuator_force.abs() / self.stall) > SATURATED
    self.gravity[:, t] = data.projected_gravity_b[:, :2]
    self.speed[:, t] = data.root_com_lin_vel_b.norm(dim=1)
    self.angular[:, t] = data.root_link_ang_vel_b.norm(dim=1)
    self.pelvis_rate[:, t] = data.root_link_ang_vel_b
    self.body_speed[:, t] = data.body_com_lin_vel_w[:, self.robot_ids].norm(dim=2)
    sim = self.u.sim.data
    geom = sim.contact.geom.long()
    active = (torch.arange(geom.shape[0], device=self.device) < sim.nacon[0]) & (sim.contact.dist < 0.0)
    first, second = torch.isin(geom[:, 0], self.floor), torch.isin(geom[:, 1], self.floor)
    hit = active & (first | second)
    other = torch.where(first, geom[:, 1], geom[:, 0])
    world = sim.contact.worldid.long()
    per_body = torch.zeros(self.n, self.model.nbody, device=self.device)
    per_body[world[hit], self.body_of_geom[other[hit]]] += 1.0
    self.touch[:, t] = per_body[:, self.model_ids] > 0


def settle(env, steps: int):
  """Deja pasar `steps` pasos sujetando cada articulación en la posición que tiene (sin política), para que el robot empiece apoyado y quieto. Devuelve la observación."""
  term = env.unwrapped.action_manager.get_term("joint_pos")
  robot = env.unwrapped.scene["robot"]
  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(steps):
      hold = (robot.data.joint_pos - term.offset) / term.scale
      obs, _, _, _ = env.step(hold)
  return obs


def quantiles(x: torch.Tensor) -> dict:
  return {"median": round(float(x.median()), 3), "p95": round(float(torch.quantile(x, 0.95)), 3), "max": round(float(x.max()), 3)}


def lie_report(rec: LieRecorder, dt: float) -> dict:
  """Informe de tumbarse con las cifras del criterio y sus aprobados."""
  n, steps, device = rec.n, rec.steps, rec.device
  window = int(WINDOW_S / dt)
  ok_now = (rec.gravity[:, :, 0] <= lie.LYING_GRAVITY_X) & (rec.gravity[:, :, 1].abs() <= lie.LYING_SIDE_MAX)
  success = torch.full((n,), -1, dtype=torch.int32, device=device)
  run = torch.zeros(n, dtype=torch.int32, device=device)
  for t in range(steps):
    run = (run + 1) * ok_now[:, t].to(torch.int32)
    if t + 1 >= window:
      still = (rec.speed[:, t + 1 - window : t + 1].mean(dim=1) <= lie.LYING_SPEED_M_S) & (rec.angular[:, t + 1 - window : t + 1].mean(dim=1) <= lie.LYING_ANGULAR_RAD_S)
      new = (success < 0) & (run >= window) & still
      success = torch.where(new, torch.full_like(success, t), success)
  ok = success >= 0

  index = torch.arange(steps, device=device)[None, :]
  moving_len = torch.where(ok, success - window + 1, torch.full_like(success, steps))
  moving_mask = (index < moving_len[:, None])[:, :, None]
  lying_mask = ((index >= moving_len[:, None]) & (index <= success[:, None]) & ok[:, None])[:, :, None]
  moving_run = longest_run(rec.sat & moving_mask).float() * dt
  lying_frac = (rec.sat & lying_mask).sum(dim=1) / window
  worst_run, worst_lying = moving_run.max(dim=1).values, lying_frac.max(dim=1).values

  # Velocidad al primer contacto de cada cuerpo (máximo de los pasos previos, contacto incluido).
  impact, speeds = {}, {}
  for k, name in enumerate(BODIES):
    touched = rec.touch[:, :, k]
    has = touched.any(dim=1)
    t0 = touched.int().argmax(dim=1)
    prior = (t0[:, None] - torch.arange(SPEED_STEPS, device=device)[None, :]).clamp(min=0)
    v = torch.gather(rec.body_speed[:, :, k], 1, prior).max(dim=1).values * has
    speeds[name] = (v, has)
    impact[name] = {
      "fraction_of_robots_that_touch_floor": round(float(has.float().mean()), 4),
      "first_contact_time_s_median": round(float((t0[has].float() * dt).median()), 2) if has.any() else None,
      "speed_m_s": quantiles(v[has]) if has.any() else None,
    }
  trunk_v, trunk_has = speeds[TRUNK]
  head_touch = rec.touch[:, :, BODIES.index(HEAD)].any(dim=1)
  rate_by_axis = rec.pelvis_rate.abs().amax(dim=1)  # (n, 3)
  rate_peak = rate_by_axis.max(dim=1).values
  rate_ok = rate_peak <= MAX_PELVIS_RATE_RAD_S
  peak_step = rec.pelvis_rate.abs().amax(dim=2).argmax(dim=1)

  f = lambda x: round(float(x), 4)
  lying_pass = f(ok.float().mean())
  trunk_pass = f((trunk_v <= MAX_TRUNK_IMPACT_M_S).float().mean())
  head_pass = f((~head_touch).float().mean())
  sat_moving = f((worst_run <= 0.5).float().mean())
  sat_lying = f((worst_lying[ok] <= 0.05).float().mean()) if ok.any() else None
  sat_2s = f((worst_run <= 2.0).float().mean())
  report = {
    "definition_lying": "eje x del tronco hacia arriba (gravedad x <= -0,9), inclinacion lateral <= 0,3, velocidad lineal <= 0,05 m/s y angular <= 0,1 rad/s, 3 s seguidos",
    "criteria": {
      "lying": {"value": lying_pass, "required": LYING_FRACTION, "passes": lying_pass >= LYING_FRACTION},
      "trunk_impact_at_most_0.8_m_s": {"value": trunk_pass, "required": TRUNK_IMPACT_FRACTION, "passes": trunk_pass >= TRUNK_IMPACT_FRACTION},
      "head_never_touches_floor": {"value": head_pass, "required": NO_HEAD_FRACTION, "passes": head_pass >= NO_HEAD_FRACTION},
      "pelvis_rate_at_most_3_rad_s_every_axis": {"value": f(rate_ok.float().mean()), "required": 1.0, "passes": bool(rate_ok.all())},
      "saturation_moving_run_at_most_0.5s": {"value": sat_moving, "required": SATURATION_FRACTION, "passes": sat_moving >= SATURATION_FRACTION},
      "saturation_lying_at_most_5pct": {"value": sat_lying, "required": SATURATION_FRACTION, "passes": sat_lying is not None and sat_lying >= SATURATION_FRACTION},
      "saturation_no_robot_over_2s": {"value": sat_2s, "required": 1.0, "passes": sat_2s >= 1.0},
    },
    "time_to_lying_s": {"median": round(float(((success[ok] + 1) * dt).median()), 2)} if ok.any() else None,
    "trunk_neck_head_impact_speed_at_first_floor_contact": impact,
    "fraction_trunk_touches_floor": f(trunk_has.float().mean()),
    "pelvis_rate_peak_rad_s": {"any_axis": quantiles(rate_peak), **{AXES[a]: quantiles(rate_by_axis[:, a]) for a in range(3)}},
    "pelvis_rate_peak_time_s_median": round(float(peak_step.float().median() * dt), 2),
    "pelvis_rate_fraction_of_robots_at_most_3_rad_s_by_axis": {AXES[a]: f((rate_by_axis[:, a] <= MAX_PELVIS_RATE_RAD_S).float().mean()) for a in range(3)},
    "final_gravity_x": {"median": round(float(rec.gravity[:, -1, 0].median()), 3), "p10": round(float(torch.quantile(rec.gravity[:, -1, 0], 0.1)), 3)},
  }
  report["passes_all"] = all(c["passes"] for c in report["criteria"].values())
  return report
