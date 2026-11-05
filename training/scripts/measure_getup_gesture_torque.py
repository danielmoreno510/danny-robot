"""Mide el par que exige el gesto de levantarse boca abajo de la búsqueda (results/get_up_prone.json), sin entrenar.

Repite la secuencia en el modelo de análisis (`analysis/danny_sim.py`, de solo lectura) con el mismo control de posición de la búsqueda
(rigidez 20 N·m/rad, amortiguación 0,5), primero sin límite de par para ver lo que pide cada articulación, y después con el bloqueo de
cada tensión (6,0, 6,6, 7,4 y 8,4 V) para ver si con ese par se pone de pie. Escribe results/training_getup_gesto_par.json.

Límite: es el modelo de análisis, no el de entrenamiento (sin las fricciones ni la aleatorización de mjlab), y la sacudida del PD que
sigue una trayectoria con interpolación lineal no es la de una política.

Uso (desde training/):
    uv run python scripts/measure_getup_gesture_torque.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis"))
import danny_sim  # noqa: E402

KEYFRAMES, DIMENSIONS = 8, ("hip_pitch", "knee", "ankle_pitch", "shoulder_pitch", "shoulder_roll", "elbow", "hip_roll")
FINAL_POSE = {"hip_pitch": -0.3, "knee": 0.6, "ankle_pitch": -0.3}
SETTLE_S, HOLD_S, SEGMENT_S, CONTROL_STEPS = 0.8, 1.5, (0.2, 2.0), 10
GAIN, DAMPING, PITCH, HEIGHT = 20.0, 0.5, 1.5, 0.045
STAND_COM_Z, MAX_TILT = 0.19, 0.3
VOLTS = (6.0, 6.6, 7.4, 8.4)
SATURATED = 0.9


def main() -> None:
  report = json.loads((ROOT / "results" / "get_up_prone.json").read_text(encoding="utf-8"))
  vector = np.array(report["best"]["vector"])
  model = danny_sim.build_model()
  model.actuator_gainprm[:, 0] = GAIN
  model.actuator_biasprm[:, 1] = -GAIN
  model.actuator_biasprm[:, 2] = -DAMPING
  names = danny_sim.hinge_names(model)
  index = {name: i for i, name in enumerate(names)}

  def targets_of(pose):
    target = np.zeros(model.nu)
    for name, value in pose.items():
      for side, sign in (("left", 1), ("right", -1)):
        i = index[f"{side}_{name}"]
        low, high = model.jnt_range[i + 1]
        target[i] = np.clip((sign if "roll" in name else 1) * value, low, high)
    return target

  width = len(DIMENSIONS) + 1
  keyframes = [dict(zip(DIMENSIONS, vector[k * width:(k + 1) * width][:-1])) for k in range(KEYFRAMES)]
  durations = [float(np.clip(vector[k * width:(k + 1) * width][-1], *SEGMENT_S)) for k in range(KEYFRAMES)] + [0.8]

  def run(limit):
    model.actuator_forcerange[:] = [-limit, limit]
    data = mujoco.MjData(model)
    rotation = np.array([[np.cos(PITCH), 0, np.sin(PITCH)], [0, 1, 0], [-np.sin(PITCH), 0, np.cos(PITCH)]])
    mujoco.mju_mat2Quat(data.qpos[3:7], rotation.flatten())
    data.qpos[2] = HEIGHT
    for _ in range(int(SETTLE_S / model.opt.timestep)):
      mujoco.mj_step(model, data)
    targets = [data.qpos[7:].copy()] + [targets_of(p) for p in keyframes] + [targets_of(FINAL_POSE)]
    forces = []
    for segment, seconds in enumerate(durations):
      steps = max(1, int(seconds / (model.opt.timestep * CONTROL_STEPS)))
      for step in range(steps):
        data.ctrl[:] = targets[segment] + (targets[segment + 1] - targets[segment]) * (step + 1) / steps
        for _ in range(CONTROL_STEPS):
          mujoco.mj_step(model, data)
        forces.append(data.actuator_force.copy())
    data.ctrl[:] = targets[-1]
    heights, tilts = [], []
    for _ in range(int(HOLD_S / model.opt.timestep)):
      mujoco.mj_step(model, data)
      heights.append(float(data.subtree_com[1][2]))
      up = np.zeros(3)
      mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), data.qpos[3:7])
      tilts.append(float(np.arccos(np.clip(up[2], -1, 1))))
    return np.abs(np.array(forces)), float(np.mean(heights[-250:])), float(np.mean(tilts[-250:]))

  step_s = model.opt.timestep * CONTROL_STEPS
  forces, height, tilt = run(100.0)  # sin límite práctico
  out = {
    "gesture": "results/get_up_prone.json, mejor secuencia, 8 puntos, control de posición kp 20 kd 0,5",
    "duration_s": round(len(forces) * step_s, 2),
    "stall_torque_nm": {str(v): round(float(danny_sim.stall_torque(v)), 4) for v in VOLTS},
    "unlimited": {"com_height_m": round(height, 3), "tilt_rad": round(tilt, 3), "stands": height >= STAND_COM_Z and tilt <= MAX_TILT},
    "demanded_torque_nm_by_joint": {},
    "with_stall_limit": {},
  }
  for j, name in enumerate(names):
    column = forces[:, j]
    run_lengths = {}
    for v in VOLTS:
      flag = column > SATURATED * danny_sim.stall_torque(v)
      best = current = 0
      for f in flag:
        current = current + 1 if f else 0
        best = max(best, current)
      run_lengths[str(v)] = round(best * step_s, 2)
    out["demanded_torque_nm_by_joint"][name] = {
      "peak": round(float(column.max()), 3),
      "rms": round(float(np.sqrt((column**2).mean())), 3),
      "peak_over_stall": {str(v): round(float(column.max() / danny_sim.stall_torque(v)), 2) for v in VOLTS},
      "longest_time_above_0.9_stall_s": run_lengths,
    }
  for v in VOLTS:
    f, h, t = run(float(danny_sim.stall_torque(v)))
    out["with_stall_limit"][str(v)] = {"com_height_m": round(h, 3), "tilt_rad": round(t, 3), "stands": h >= STAND_COM_Z and t <= MAX_TILT}
  worst = sorted(out["demanded_torque_nm_by_joint"].items(), key=lambda kv: -kv[1]["peak"])[:6]
  path = ROOT / "results" / "training_getup_gesto_par.json"
  path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps({k: out[k] for k in ("stall_torque_nm", "unlimited", "with_stall_limit")}, indent=1))
  for name, d in worst:
    print(name, d["peak"], d["peak_over_stall"], d["longest_time_above_0.9_stall_s"])
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
