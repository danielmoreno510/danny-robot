"""Cálculo estático de una pose en cuclillas: pies planos, centro de masas sobre las suelas y par de cada articulación de la pierna.

Para una altura de la pelvis (`pelvis_bottom`) y una inclinación del tronco dadas, resuelve cadera, rodilla, tobillo y altura de la base
con los pies planos en el suelo y el centro de masas sobre el centro de las suelas, y calcula el par estático repartiendo el peso a partes
iguales entre los dos pies. Es estático: no incluye el par dinámico ni la aleatorización de masa.
"""

from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import least_squares

GRAVITY = 9.81
PELVIS_SITE = "pelvis_bottom"
LEG_JOINTS = ("hip_pitch", "knee", "ankle_pitch")
ARM_JOINTS = ("shoulder_pitch", "elbow")


class StaticPose:
  """Modelo de MuJoCo preparado para resolver poses estáticas con los pies planos."""

  def __init__(self, xml_path) -> None:
    self.model = mujoco.MjModel.from_xml_path(str(xml_path))
    self.data = mujoco.MjData(self.model)
    m = self.model
    self.mass = float(m.body_subtreemass[0])
    self.soles = [m.geom("left_sole").id, m.geom("right_sole").id]
    self.pelvis = m.site(PELVIS_SITE).id
    self.key = m.key("stand")
    self.dof = {n: int(m.jnt_dofadr[m.joint("left_" + n).id]) for n in LEG_JOINTS}
    self.range_deg = {n: np.degrees(m.jnt_range[m.joint("left_" + n).id]) for n in LEG_JOINTS + ARM_JOINTS}
    self.dof.update({n: int(m.jnt_dofadr[m.joint("left_" + n).id]) for n in ARM_JOINTS})
    # cajas de colisión del antebrazo (la mano): las únicas piezas del brazo que se apoyan en el suelo en la comprobación de manos
    self.hand_geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[g] and "left_forearm" in m.body(m.geom_bodyid[g]).name]

  def _set(self, x, lean: float) -> None:
    m, d = self.model, self.data
    h, q, a, z = x
    d.qpos[:] = self.key.qpos
    d.qpos[2] = z
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(quat, np.array([0.0, 1.0, 0.0]), lean)
    d.qpos[3:7] = quat
    for side in ("left", "right"):
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_hip_pitch").id]] = h
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_knee").id]] = q
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_ankle_pitch").id]] = a
    mujoco.mj_forward(m, d)

  def _sole_low(self, i: int) -> float:
    return self.data.geom_xpos[i][2] - (np.abs(self.data.geom_xmat[i].reshape(3, 3)) @ self.model.geom_size[i])[2]

  def _residual(self, x, lean: float, pelvis_z: float):
    self._set(x, lean)
    d = self.data
    sole_x = np.mean([d.geom_xpos[i][0] for i in self.soles])
    pitch = np.mean([d.geom_xmat[i].reshape(3, 3)[2, 0] for i in self.soles])
    return [d.subtree_com[1][0] - sole_x, pitch * 0.3, d.site_xpos[self.pelvis][2] - pelvis_z, min(self._sole_low(i) for i in self.soles)]

  def solve(self, pelvis_z: float, lean: float, guess=(-0.9, 1.4, -0.6, 0.15)) -> dict:
    """Pose para `pelvis_z` (m) y `lean` (rad, inclinación del tronco hacia delante). Devuelve ángulos (rad), residuo y pares (N·m)."""
    sol = least_squares(self._residual, list(guess), args=(lean, pelvis_z), x_scale=[0.1, 0.1, 0.1, 0.01])
    self._set(sol.x, lean)
    d, m = self.data, self.model
    com = d.subtree_com[1].copy()
    reaction = np.zeros(m.nv)
    for side in ("left", "right"):
      sole = m.geom(f"{side}_sole")
      point = np.array([com[0], d.geom_xpos[sole.id][1], d.geom_xpos[sole.id][2]])
      jacp = np.zeros((3, m.nv))
      mujoco.mj_jac(m, d, jacp, None, point, int(m.geom_bodyid[sole.id]))
      reaction += jacp.T @ np.array([0.0, 0.0, self.mass * GRAVITY / 2])
    torque = d.qfrc_bias - reaction
    return {
      "angles_rad": {"hip_pitch": float(sol.x[0]), "knee": float(sol.x[1]), "ankle_pitch": float(sol.x[2])},
      "residual": float(np.abs(sol.fun).max()),
      "com_error_m": float(abs(com[0] - np.mean([d.geom_xpos[i][0] for i in self.soles]))),
      "torque_nm": {n: float(abs(torque[self.dof[n]])) for n in LEG_JOINTS},
      "in_range": {n: bool(self.range_deg[n][0] <= np.degrees(v) <= self.range_deg[n][1]) for n, v in zip(LEG_JOINTS, sol.x[:3])},
    }

  # ---- apoyo de las manos detrás -----------------------------------------------------------------------------------------
  def _set_hands(self, x, lean: float) -> None:
    self._set(x[:4], lean)
    m, d = self.model, self.data
    for side in ("left", "right"):
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_shoulder_pitch").id]] = x[4]
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_elbow").id]] = x[5]
      d.qpos[m.jnt_qposadr[m.joint(f"{side}_shoulder_roll").id]] = 0.0
    mujoco.mj_forward(m, d)

  def _forearm_boxes(self, side: str) -> list[int]:
    m = self.model
    return [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[g] and f"{side}_forearm" in m.body(m.geom_bodyid[g]).name]

  def _lowest_box_point(self, geoms: list[int]) -> np.ndarray:
    """Vértice más bajo de las cajas `geoms` (en el mundo)."""
    m, d = self.model, self.data
    best = None
    for g in geoms:
      rot, size = d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
      for sx in (-1, 1):
        for sy in (-1, 1):
          for sz in (-1, 1):
            p = d.geom_xpos[g] + rot @ (np.array([sx, sy, sz]) * size)
            if best is None or p[2] < best[2]:
              best = p
    return best

  def _hand_low_point(self, side: str = "left") -> np.ndarray:
    return self._lowest_box_point(self._forearm_boxes(side))

  def _foot_contact_x(self, flat: bool) -> float:
    """Dónde actúa el apoyo de los pies: el centro de las suelas (pies planos) o el vértice más bajo de la suela (pie apoyado en punta o talón)."""
    d = self.data
    if flat:
      return float(np.mean([d.geom_xpos[i][0] for i in self.soles]))
    return float(np.mean([self._lowest_box_point([i])[0] for i in self.soles]))

  def _residual_hands(self, x, lean: float, pelvis_z: float, support: float, flat: bool):
    self._set_hands(x, lean)
    d = self.data
    foot_x = self._foot_contact_x(flat)
    hand = self._hand_low_point()
    pitch = np.mean([d.geom_xmat[i].reshape(3, 3)[2, 0] for i in self.soles])
    com_target = (1.0 - support) * foot_x + support * hand[0]
    out = [d.subtree_com[1][0] - com_target, d.site_xpos[self.pelvis][2] - pelvis_z, min(self._lowest_box_point([i])[2] for i in self.soles), hand[2]]
    return out + [pitch * 0.3] if flat else out

  def solve_hands(self, pelvis_z: float, lean: float, support: float, guess=(-0.9, 1.4, -0.6, 0.15, 0.3, -0.3), flat: bool = True) -> dict:
    """Pose con las manos (las dos, simétricas, balanceo de hombro 0) en el suelo detrás y los pies planos (`flat`) o apoyados en punta o talón.

    `support`: fracción del peso que aguantan las manos (el resto, los pies). `lean`: inclinación del tronco (rad, negativa = hacia atrás). Las
    articulaciones están acotadas a su rango. Devuelve ángulos, residuo, posiciones y pares de pierna y brazo (N·m, por lado).
    """
    m = self.model
    names = LEG_JOINTS + ARM_JOINTS
    lo = [np.radians(self.range_deg[n][0]) for n in names[:3]] + [-1.0] + [np.radians(self.range_deg[n][0]) for n in names[3:]]
    hi = [np.radians(self.range_deg[n][1]) for n in names[:3]] + [1.0] + [np.radians(self.range_deg[n][1]) for n in names[3:]]
    x0 = np.clip(np.array(guess, dtype=float), np.array(lo) + 1e-6, np.array(hi) - 1e-6)
    sol = least_squares(self._residual_hands, x0, args=(lean, pelvis_z, support, flat), bounds=(lo, hi), x_scale=[0.1, 0.1, 0.1, 0.01, 0.1, 0.1])
    self._set_hands(sol.x, lean)
    d = self.data
    com = d.subtree_com[1].copy()
    reaction = np.zeros(m.nv)
    for side in ("left", "right"):
      sole = m.geom(f"{side}_sole")
      foot = com[0] if flat else self._lowest_box_point([sole.id])[0]
      point = np.array([com[0] if flat else foot, d.geom_xpos[sole.id][1], d.geom_xpos[sole.id][2] if flat else self._lowest_box_point([sole.id])[2]])
      jacp = np.zeros((3, m.nv))
      mujoco.mj_jac(m, d, jacp, None, point, int(m.geom_bodyid[sole.id]))
      reaction += jacp.T @ np.array([0.0, 0.0, self.mass * GRAVITY * (1.0 - support) / 2])
      boxes = self._forearm_boxes(side)
      jacp = np.zeros((3, m.nv))
      mujoco.mj_jac(m, d, jacp, None, self._lowest_box_point(boxes), int(m.geom_bodyid[boxes[0]]))
      reaction += jacp.T @ np.array([0.0, 0.0, self.mass * GRAVITY * support / 2])
    torque = d.qfrc_bias - reaction
    angles = dict(zip(names, [sol.x[0], sol.x[1], sol.x[2], sol.x[4], sol.x[5]]))
    return {
      "angles_rad": {n: float(v) for n, v in angles.items()},
      "residual": float(np.abs(sol.fun).max()),
      "hand_x_m": float(self._hand_low_point()[0]),
      "foot_x_m": self._foot_contact_x(flat),
      "com_x_m": float(com[0]),
      "shoulder_height_above_pelvis_m": float(d.xanchor[m.joint("left_shoulder_pitch").id][2] - d.site_xpos[self.pelvis][2]),
      "torque_nm": {n: float(abs(torque[self.dof[n]])) for n in names},
      "in_range": {n: bool(self.range_deg[n][0] - 1e-3 <= np.degrees(v) <= self.range_deg[n][1] + 1e-3) for n, v in angles.items()},
    }
