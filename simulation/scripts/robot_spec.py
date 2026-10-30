"""
Danny - Especificación del robot: dimensiones, masas, articulaciones y motores.

Es la única fuente de estos datos. build_model.py genera el modelo con ellos, y
los controladores los importan. Todo en unidades SI, y provisional hasta elegir
motores (etapa 31). Las proporciones del humanoide completo están en
documentation/phase-3.5-proposal.md.
"""

from dataclasses import dataclass

import numpy as np


SIDES = ("left", "right")

TIMESTEP = 0.002  # s
GRAVITY = 9.81  # m/s²
FLOOR_FRICTION = (1.0, 0.005, 0.0001)

# Geometría [m]
PELVIS_START_HEIGHT = 0.4
PELVIS_HALF = (0.06, 0.10, 0.05)
TORSO_OFFSET = 0.17  # centro del torso sobre la pelvis
TORSO_HALF = (0.06, 0.10, 0.12)
HIP_Y = 0.07  # caderas en y = ±0.07
HIP_BELOW_PELVIS = 0.05
THIGH_LENGTH = 0.16
SHIN_LENGTH = 0.16
LEG_RADIUS = 0.025
SOLE_BELOW_ANKLE = 0.04
FOOT_HALF = (0.07, 0.04, 0.01)
LEG_LENGTH = THIGH_LENGTH + SHIN_LENGTH

# Masas [kg]
PELVIS_MASS = 1.2
TORSO_MASS = 4.0
THIGH_MASS = 0.7
SHIN_MASS = 0.5
FOOT_MASS = 0.2

# Articulaciones. Armature: inercia del motor reflejada por la reductora. Sin
# ella, los tobillos son tan ligeros que resuenan a 12 Hz con el contacto.
JOINT_DAMPING = 0.5  # N·m·s/rad
JOINT_ARMATURE = 0.01  # kg·m²

# Motores de posición
MOTOR_KP = 100.0  # N·m/rad
MOTOR_TORQUE_LIMIT = 20.0  # N·m, provisional


@dataclass(frozen=True)
class Joint:
    """
    kv = 2 * zeta * sqrt(kp * (I + armature)), zeta ~ 0,8, con I medida con
    mj_fullM y la pierna recta. Excepción, los tobillos: de pie sostienen todo el
    cuerpo, así que su kv se calcula para esa carga, con zeta ~ 0,6. Con el
    valor de la pierna colgando, el cuerpo rebotaba sobre ellos.
    """

    part: str
    axis: tuple[float, float, float]
    limits: tuple[float, float]  # rad, tope mecánico
    kv: float  # N·m·s/rad
    command_limits: tuple[float, float] | None = None  # rad; None = los del tope


# Orden de los ejes en cada cuerpo: importa. El orden del tuple es el del vector
# de 6 ángulos de cada pierna.
LEG_JOINTS = (
    Joint("hip_yaw", (0, 0, 1), (-0.6981, 0.6981), 1.66),
    Joint("hip_roll", (1, 0, 0), (-0.5236, 0.5236), 4.3),
    Joint("hip_pitch", (0, 1, 0), (-1.5708, 1.5708), 4.3),
    # El mando llega a -5°: con la pierna recta, la compensación puede pedir un
    # objetivo algo negativo. El tope sigue en 0°.
    Joint("knee", (0, 1, 0), (0.0, 2.2689), 2.4, (-0.0873, 2.2689)),
    Joint("ankle_pitch", (0, 1, 0), (-1.1345, 0.7854), 8.5),
    Joint("ankle_roll", (1, 0, 0), (-0.5236, 0.5236), 8.5),
)
HIP_JOINTS = LEG_JOINTS[:3]
KNEE_JOINT = LEG_JOINTS[3]
ANKLE_JOINTS = LEG_JOINTS[4:]

JOINT_PARTS = tuple(joint.part for joint in LEG_JOINTS)
JOINTS = tuple(f"{side}_{part}_joint" for side in SIDES for part in JOINT_PARTS)
ACTUATORS = tuple(f"{side}_{part}_motor" for side in SIDES for part in JOINT_PARTS)
POSITION_SENSORS = tuple(f"{side}_{part}_pos" for side in SIDES for part in JOINT_PARTS)
TORQUE_SENSORS = tuple(f"{side}_{part}_torque" for side in SIDES for part in JOINT_PARTS)

LEG_JOINT_LIMITS = np.array([joint.limits for joint in LEG_JOINTS])
JOINT_LIMITS = np.vstack([LEG_JOINT_LIMITS, LEG_JOINT_LIMITS])
