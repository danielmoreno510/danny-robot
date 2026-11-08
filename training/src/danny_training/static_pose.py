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
    self.range_deg = {n: np.degrees(m.jnt_range[m.joint("left_" + n).id]) for n in LEG_JOINTS}

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
