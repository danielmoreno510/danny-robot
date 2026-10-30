"""
Danny - Comprobación de la cinemática inversa (etapas 9 y 10).

Compara leg_ik y joint_targets con la cinemática directa de MuJoCo: se piden
posiciones de los pies, se ponen los ángulos calculados en el modelo y se mide
dónde quedan los pies de verdad.

Uso:
  python simulation/scripts/check_kinematics.py
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

from biped_control import Biped, Pose
from leg_control import leg_ik


ROOT = Path(__file__).resolve().parents[1]
LEG_MODEL = ROOT / "models" / "leg_6dof.xml"
BIPED_MODEL = ROOT / "models" / "danny.xml"

LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")

# Tobillo respecto a la cadera [m]
LEG_CASES = [
    (0.0, 0.0, -0.45),
    (0.08, 0.05, -0.40),
    (-0.10, -0.08, -0.35),
    (0.05, 0.12, -0.42),
    (0.0, 0.15, -0.40),
]

# (pelvis x, pelvis yaw, pie izq x, pie izq yaw, pie der x, pie der yaw)
BIPED_CASES = [
    (0.10, 0.6, 0.13, 0.0, 0.07, 0.3),
    (0.05, -0.4, 0.07, -0.2, 0.02, 0.2),
    (0.20, 0.785, 0.22, 0.5, 0.17, 0.785),
]

MAX_POSITION_ERROR = 1e-6  # m
MAX_ANGLE_ERROR_DEG = 1e-3


def check_leg() -> list[str]:
    model = mujoco.MjModel.from_xml_path(str(LEG_MODEL))
    data = mujoco.MjData(model)
    addresses = [model.joint(f"{name}_joint").qposadr[0] for name in LEG_JOINTS]
    failures = []
    for case in LEG_CASES:
        data.qpos[:] = 0.0
        data.qpos[addresses] = leg_ik(np.array(case))
        mujoco.mj_kinematics(model, data)
        ankle = data.xpos[model.body("foot").id] - data.xpos[model.body("thigh").id]
        error = float(np.linalg.norm(ankle - case))
        tilt = math.degrees(math.acos(min(1.0, data.xmat[model.body("foot").id].reshape(3, 3)[2, 2])))
        print(f"  pierna {case}: error {1000 * error:.4f} mm, inclinación {tilt:.4f}°")
        if error > MAX_POSITION_ERROR or tilt > MAX_ANGLE_ERROR_DEG:
            failures.append(f"pierna {case}")
    return failures


def check_biped() -> list[str]:
    model = mujoco.MjModel.from_xml_path(str(BIPED_MODEL))
    data = mujoco.MjData(model)
    biped = Biped(model, data)
    failures = []
    for pelvis_x, pelvis_yaw, left_x, left_yaw, right_x, right_yaw in BIPED_CASES:
        pose = Pose(
            np.array([pelvis_x, 0.01, 0.34]),
            np.array([left_x, 0.07, 0.0]),
            np.array([right_x, -0.07, 0.02]),
            pelvis_yaw,
            left_yaw,
            right_yaw,
        )
        biped.set_pose(data, pose)
        for index, (sole, yaw) in enumerate(((pose.left_sole, left_yaw), (pose.right_sole, right_yaw))):
            rotation = data.site_xmat[biped.sole_ids[index]].reshape(3, 3)
            error = float(np.linalg.norm(data.site_xpos[biped.sole_ids[index]] - sole))
            yaw_error = abs(math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]) - yaw))
            tilt = math.degrees(math.acos(min(1.0, rotation[2, 2])))
            print(f"  bípedo, pie {index}: error {1000 * error:.4f} mm, giro {yaw_error:.4f}°, "
                  f"inclinación {tilt:.4f}°")
            if error > MAX_POSITION_ERROR or yaw_error > MAX_ANGLE_ERROR_DEG or tilt > MAX_ANGLE_ERROR_DEG:
                failures.append(f"bípedo, pie {index}, yaw {yaw:.2f}")
    return failures


def main() -> int:
    print("Cinemática inversa contra MuJoCo")
    failures = check_leg() + check_biped()
    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: los pies quedan donde se pide, planos y con el giro pedido")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
