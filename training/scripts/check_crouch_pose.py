"""Comprueba con un cálculo estático que Danny puede agacharse a cada altura pedida: tobillo, rodilla y cadera dentro de su rango y centro de masas sobre los pies.

Para cada bajada de la pelvis (`pelvis_bottom`, respecto a la pose de pie con el tronco recto) y su inclinación de tronco, que crece
en proporción (`--lean-at-max` rad a `--max-drop` m), resuelve cadera, rodilla, tobillo y altura de la base con los pies planos en el
suelo y el centro de masas sobre el centro de las suelas. Con eso calcula el par estático de cada articulación de la pierna, repartiendo
el peso a partes iguales entre los dos pies. Guarda el resultado en results/ y sale con código 1 si alguna pose se sale del rango de las articulaciones, descentra el centro de masas más de 5 mm o pide más del 50 % del bloqueo a 6,6 V.

Uso (desde training/):
    uv run python scripts/check_crouch_pose.py
"""

from __future__ import annotations

import argparse
import json
import sys

import mujoco
import numpy as np
from scipy.optimize import least_squares

from danny_training.evaluation import RESULTS
from danny_training.actuator import stall_torque_nm
from danny_training.robot import DANNY_XML

COM_TOLERANCE_M = 0.005
STALL_FRACTION_MAX = 0.5
VIN_V = 6.6
GRAVITY = 9.81


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--drops", type=float, nargs="+", default=[0.0, 0.025, 0.045, 0.05, 0.06], help="Bajadas de la pelvis en m")
  parser.add_argument("--max-drop", type=float, default=0.05)
  parser.add_argument("--lean-at-max", type=float, default=0.35, help="Inclinación del tronco en rad a --max-drop")
  args = parser.parse_args()

  model = mujoco.MjModel.from_xml_path(str(DANNY_XML))
  data = mujoco.MjData(model)
  mass = float(model.body_subtreemass[0])
  soles = [model.geom("left_sole").id, model.geom("right_sole").id]
  pelvis = model.site("pelvis_bottom").id
  dof = {n: int(model.jnt_dofadr[model.joint("left_" + n).id]) for n in ("hip_pitch", "knee", "ankle_pitch")}
  ankle_range = np.degrees(model.jnt_range[model.joint("left_ankle_pitch").id])
  knee_range = np.degrees(model.jnt_range[model.joint("left_knee").id])
  hip_range = np.degrees(model.jnt_range[model.joint("left_hip_pitch").id])
  key = model.key("stand")

  def set_pose(x, lean):
    h, q, a, z = x
    data.qpos[:] = key.qpos
    data.qpos[2] = z
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(quat, np.array([0.0, 1.0, 0.0]), lean)
    data.qpos[3:7] = quat
    for side in ("left", "right"):
      data.qpos[model.jnt_qposadr[model.joint(f"{side}_hip_pitch").id]] = h
      data.qpos[model.jnt_qposadr[model.joint(f"{side}_knee").id]] = q
      data.qpos[model.jnt_qposadr[model.joint(f"{side}_ankle_pitch").id]] = a
    mujoco.mj_forward(model, data)

  def sole_low(i):
    return data.geom_xpos[i][2] - (np.abs(data.geom_xmat[i].reshape(3, 3)) @ model.geom_size[i])[2]

  def residual(x, lean, pelvis_z):
    set_pose(x, lean)
    sole_x = np.mean([data.geom_xpos[i][0] for i in soles])
    pitch = np.mean([data.geom_xmat[i].reshape(3, 3)[2, 0] for i in soles])  # 0 con la suela plana
    return [data.subtree_com[1][0] - sole_x, pitch * 0.3, data.site_xpos[pelvis][2] - pelvis_z, min(sole_low(i) for i in soles)]

  poses, ok_all = [], True
  stand_pelvis = None
  for drop in args.drops:
    lean = args.lean_at_max * drop / args.max_drop
    if stand_pelvis is None:  # la primera bajada es 0: fija la pelvis de pie con el tronco recto
      guess = [-0.364, 0.827, -0.463, 0.0]
      probe = least_squares(lambda x: [residual(x, 0.0, 0.0)[0], residual(x, 0.0, 0.0)[1], residual(x, 0.0, 0.0)[3]] + [0.0], guess)
      set_pose(probe.x, 0.0)
      stand_pelvis = float(data.site_xpos[pelvis][2])
    guess = [-0.364 - 0.4 * drop / 0.02 * 0.1, 0.827 + 0.6 * drop / 0.02 * 0.5, -0.463, key.qpos[2]]
    sol = least_squares(residual, guess, args=(lean, stand_pelvis - drop), x_scale=[0.1, 0.1, 0.1, 0.01])
    set_pose(sol.x, lean)
    mujoco.mj_forward(model, data)
    com = data.subtree_com[1].copy()
    com_error = float(abs(com[0] - np.mean([data.geom_xpos[i][0] for i in soles])))
    pelvis_drop = stand_pelvis - float(data.site_xpos[pelvis][2])
    # Par estático: gravedad menos la reacción del suelo, mitad en cada pie, con la fuerza aplicada bajo el centro de masas.
    bias = data.qfrc_bias.copy()
    reaction = np.zeros(model.nv)
    for side in ("left", "right"):
      sole = model.geom(f"{side}_sole")
      point = np.array([com[0], data.geom_xpos[sole.id][1], data.geom_xpos[sole.id][2]])
      jacp = np.zeros((3, model.nv))
      mujoco.mj_jac(model, data, jacp, None, point, int(model.geom_bodyid[sole.id]))
      reaction += jacp.T @ np.array([0.0, 0.0, mass * GRAVITY / 2])
    torque = bias - reaction
    stall = float(stall_torque_nm(VIN_V))
    pose = {
      "pelvis_drop_m": round(pelvis_drop, 4),
      "trunk_lean_deg": round(float(np.degrees(lean)), 1),
      "hip_pitch_deg": round(float(np.degrees(sol.x[0])), 1),
      "knee_deg": round(float(np.degrees(sol.x[1])), 1),
      "ankle_pitch_deg": round(float(np.degrees(sol.x[2])), 1),
      "ankle_range_deg": [round(float(v), 1) for v in ankle_range],
      "hip_range_deg": [round(float(v), 1) for v in hip_range],
      "knee_range_deg": [round(float(v), 1) for v in knee_range],
      "com_error_mm": round(com_error * 1000, 2),
      "knee_static_torque_nm": round(float(abs(torque[dof["knee"]])), 3),
      "ankle_static_torque_nm": round(float(abs(torque[dof["ankle_pitch"]])), 3),
      "hip_static_torque_nm": round(float(abs(torque[dof["hip_pitch"]])), 3),
      "knee_fraction_of_stall_6.6V": round(float(abs(torque[dof["knee"]]) / stall), 3),
      "solver_residual": round(float(np.abs(sol.fun).max()), 5),
    }
    pose["passes"] = bool(
      ankle_range[0] <= np.degrees(sol.x[2]) <= ankle_range[1]
      and knee_range[0] <= np.degrees(sol.x[1]) <= knee_range[1]
      and hip_range[0] <= np.degrees(sol.x[0]) <= hip_range[1]
      and com_error <= COM_TOLERANCE_M
      and pose["knee_fraction_of_stall_6.6V"] <= STALL_FRACTION_MAX
      and pose["solver_residual"] < 1e-3
      and abs(pelvis_drop - drop) < 0.001
    )
    ok_all &= pose["passes"]
    poses.append(pose)
    print(pose)

  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / "training_crouch_static.json"
  out.write_text(json.dumps({"vin_v": VIN_V, "stand_pelvis_height_m": stand_pelvis, "poses": poses, "all_pass": ok_all}, indent=2) + "\n", encoding="utf-8")
  sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
  main()
