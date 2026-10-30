"""
Danny - Especificación del robot: dimensiones, masas, articulaciones y motores.

Es la única fuente de estos datos. build_model.py genera el modelo con ellos, y
los controladores los importan. Todo en unidades SI, y provisional hasta elegir
motores (etapa 31). Las proporciones del humanoide completo están en
documentation/phase-3.5-proposal.md.
"""

from dataclasses import dataclass, field

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


@dataclass(frozen=True)
class Motor:
    """
    Un tipo de motor. Par y velocidad vienen del catálogo
    (documentation/phase-3.5-mass-budget.md); los del STS3215 están sin confirmar.
    kp sigue una regla: unas 6 veces el par de pico, sin redondeo fino.
    Armature (inercia reflejada por la reductora) y damping son ESTIMACIONES sin
    medir: la fase 4 debe variarlas. Sin armature, los tobillos resuenan a 12 Hz
    con el contacto.
    """

    peak_torque: float  # N·m
    no_load_speed: float  # rad/s
    kp: float  # N·m/rad
    armature: float  # kg·m²
    damping: float  # N·m·s/rad


GENERIC_MOTOR = Motor(20.0, 20.0, 100.0, 0.01, 0.5)  # el del bípedo: provisional
ROBSTRIDE_01 = Motor(17.0, 33.0, 100.0, 0.01, 0.1)
ROBSTRIDE_00 = Motor(14.0, 33.0, 85.0, 0.01, 0.1)
XM540 = Motor(7.3, 5.55, 45.0, 0.01, 0.2)
XM430 = Motor(4.1, 4.8, 25.0, 0.005, 0.15)
STS3215 = Motor(1.9, 4.7, 12.0, 0.002, 0.05)


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

# Cintura: solo en el humanoide. Yaw sobre la pelvis, y encima roll y pitch. El roll
# deja el torso vertical mientras la pelvis se inclina al andar.
WAIST_JOINTS = (
    Joint("waist_yaw", (0, 0, 1), (-0.7854, 0.7854), 0.0),  # ±45°
    Joint("waist_roll", (1, 0, 0), (-0.3491, 0.3491), 0.0),  # ±20°
    Joint("waist_pitch", (0, 1, 0), (-0.2618, 0.7854), 0.0),  # -15° a 45°
)

# Cuello (etapa 3.5.6): yaw y, encima, pitch. Pitch positivo: hacia abajo.
NECK_JOINTS = (
    Joint("neck_yaw", (0, 0, 1), (-1.3963, 1.3963), 0.0),  # ±80°
    Joint("neck_pitch", (0, 1, 0), (-0.5236, 0.7854), 0.0),  # -30° a 45°
)

# Brazo (etapa 3.5.4), en orden desde el hombro. El brazo derecho refleja los ejes
# (ver mirrored_axis): un ángulo positivo significa lo mismo en los dos lados.
# Hombro pitch negativo: hacia delante. Hombro roll positivo: hacia fuera.
ARM_JOINTS = (
    Joint("shoulder_pitch", (0, 1, 0), (-2.618, 1.0472), 0.0),  # -150° a 60°
    Joint("shoulder_roll", (1, 0, 0), (-0.1745, 2.0944), 0.0),  # -10° a 120°
    Joint("shoulder_yaw", (0, 0, 1), (-1.5708, 1.5708), 0.0),  # ±90°
    Joint("elbow", (0, -1, 0), (0.0, 2.4435), 0.0),  # 0° a 140°
    Joint("wrist_yaw", (0, 0, 1), (-1.5708, 1.5708), 0.0),  # ±90°
    Joint("wrist_pitch", (0, 1, 0), (-0.7854, 0.7854), 0.0),  # ±45°
    Joint("wrist_roll", (1, 0, 0), (-0.7854, 0.7854), 0.0),  # ±45°
)
# Postura fija de los brazos: con 15° el codo libra la esquina de la pelvis en todo el
# giro de cintura (con 10°, chocaba a partir de unos 30°)
ARM_HOLD_ANGLES = {"shoulder_roll": 0.2618, "elbow": 0.3491}  # rad

# Humanoide: alturas respecto a la pelvis [m]
WAIST_ABOVE_PELVIS = 0.04
WAIST_LINK_HALF = (0.03, 0.05, 0.02)
WAIST_ROLL_LINK_HALF = (0.03, 0.04, 0.015)
UPPER_TORSO_HALF = (0.06, 0.085, 0.07)
UPPER_TORSO_OFFSET = 0.07  # centro, sobre la cintura
SHOULDER_Y = 0.11
SHOULDER_ABOVE_WAIST = 0.14
SHOULDER_RADIUS = 0.035
UPPER_ARM_LENGTH = 0.13
FOREARM_LENGTH = 0.12
ARM_RADIUS = 0.02
HAND_HALF = (0.015, 0.025, 0.025)  # espesor, ancho y largo: rígida, sin dedos
NECK_BASE_ABOVE_WAIST = 0.155
NECK_RADIUS = 0.02
NECK_PITCH_ABOVE_BASE = 0.015
HEAD_RADIUS = 0.045
HEAD_ABOVE_NECK_PITCH = 0.035  # el centro de la cabeza, a 65,5 cm del suelo


@dataclass(frozen=True)
class Robot:
    """
    Una configuración del robot. Las masas del humanoide son las del presupuesto de
    documentation/phase-3.5-mass-budget.md, con el margen de 0,31 kg en el torso.
    """

    file: str
    pelvis_mass: float
    thigh_mass: float
    shin_mass: float
    foot_mass: float
    upper_body_mass: float  # torso del bípedo o torso superior del humanoide [kg]
    leg_kv: dict[str, float]
    motors: dict[str, Motor] = field(default_factory=dict)  # por articulación
    waist_mass: float = 0.0
    waist_roll_mass: float = 0.0
    shoulder_mass: float = 0.0
    upper_arm_mass: float = 0.0
    forearm_mass: float = 0.0
    hand_mass: float = 0.0
    neck_mass: float = 0.0
    head_mass: float = 0.0
    waist_kv: dict[str, float] | None = None
    arm_kv: dict[str, float] | None = None
    neck_kv: dict[str, float] | None = None

    def motor(self, part: str) -> Motor:
        return self.motors.get(part, GENERIC_MOTOR)

    @property
    def has_waist(self) -> bool:
        return self.waist_kv is not None

    @property
    def has_arms(self) -> bool:
        return self.arm_kv is not None


BIPED = Robot(
    file="danny_biped.xml",
    pelvis_mass=1.2,
    thigh_mass=0.7,
    shin_mass=0.5,
    foot_mass=0.2,
    upper_body_mass=4.0,
    leg_kv={joint.part: joint.kv for joint in LEG_JOINTS},
)
HUMANOID = Robot(
    file="danny_humanoid.xml",
    pelvis_mass=2.0,
    thigh_mass=1.0,
    shin_mass=0.77,
    foot_mass=0.2,
    upper_body_mass=2.48,
    leg_kv={"hip_yaw": 1.8, "hip_roll": 4.83, "hip_pitch": 4.84, "knee": 2.53,
            "ankle_pitch": 8.4, "ankle_roll": 8.4},
    motors={
        "hip_yaw": ROBSTRIDE_00, "hip_roll": ROBSTRIDE_01, "hip_pitch": ROBSTRIDE_01,
        "knee": ROBSTRIDE_01, "ankle_pitch": ROBSTRIDE_00, "ankle_roll": ROBSTRIDE_00,
        "waist_yaw": ROBSTRIDE_00, "waist_roll": ROBSTRIDE_00, "waist_pitch": ROBSTRIDE_00,
        "shoulder_pitch": XM540, "shoulder_roll": XM540, "shoulder_yaw": XM540, "elbow": XM430,
        "wrist_yaw": STS3215, "wrist_pitch": STS3215, "wrist_roll": STS3215,
        "neck_yaw": STS3215, "neck_pitch": STS3215,
    },
    waist_mass=0.2,
    waist_roll_mass=0.31,
    shoulder_mass=0.215,
    upper_arm_mass=0.347,
    forearm_mass=0.115,
    hand_mass=0.16,
    neck_mass=0.09,
    head_mass=0.31,
    waist_kv={"waist_yaw": 3.42, "waist_roll": 4.66, "waist_pitch": 3.77},
    arm_kv={"shoulder_pitch": 1.77, "shoulder_roll": 1.79, "shoulder_yaw": 1.11, "elbow": 0.76,
            "wrist_yaw": 0.25, "wrist_pitch": 0.26, "wrist_roll": 0.26},
    neck_kv={"neck_yaw": 0.26, "neck_pitch": 0.28},
)
ROBOTS = {"biped": BIPED, "humanoid": HUMANOID}

JOINT_PARTS = tuple(joint.part for joint in LEG_JOINTS)
ARM_PARTS = tuple(f"{side}_{joint.part}" for side in SIDES for joint in ARM_JOINTS)
NECK_PARTS = tuple(joint.part for joint in NECK_JOINTS)
HELD_PARTS = (*(joint.part for joint in WAIST_JOINTS), *ARM_PARTS, *NECK_PARTS)  # lo que no es pierna
JOINTS = tuple(f"{side}_{part}_joint" for side in SIDES for part in JOINT_PARTS)
ACTUATORS = tuple(f"{side}_{part}_motor" for side in SIDES for part in JOINT_PARTS)
POSITION_SENSORS = tuple(f"{side}_{part}_pos" for side in SIDES for part in JOINT_PARTS)
TORQUE_SENSORS = tuple(f"{side}_{part}_torque" for side in SIDES for part in JOINT_PARTS)



def mirrored_axis(joint: Joint, side: str) -> tuple[float, float, float]:
    """Eje en el lado 'side'. Reflejar en el plano xz invierte los giros en x y z."""
    x, y, z = joint.axis
    return joint.axis if side == "left" else (-x, y, -z)


def short_part(part: str) -> str:
    """La articulación sin su lado: 'left_elbow' -> 'elbow'."""
    side, _, rest = part.partition("_")
    return rest if side in SIDES else part


def held_angle(part: str) -> float:
    """Ángulo [rad] en que se sujeta una articulación que no es de pierna."""
    return ARM_HOLD_ANGLES.get(short_part(part), 0.0)


LEG_JOINT_LIMITS = np.array([joint.limits for joint in LEG_JOINTS])
JOINT_LIMITS = np.vstack([LEG_JOINT_LIMITS, LEG_JOINT_LIMITS])
