"""
Danny - Capa de simulación sobre el MJCF exportado de Fusion, etapa 3.

No edita cad/export: carga el modelo con MjSpec y añade suelo, base libre, pies de contacto,
puntos de medida, actuadores y los parámetros del servo. Módulo compartido por los scripts
de esta etapa.
"""

import json
import pathlib

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL = ROOT / "cad" / "export" / "danny" / "danny.xml"
PARAMETERS = ROOT / "analysis" / "hd1910_bam_parameters.json"

SERVO = json.loads(PARAMETERS.read_text(encoding="utf-8"))
SPEC_STALL_NM = 1.47  # ficha, a 7,4 V
BATTERY_VOLTS = (7.4, 6.0)
POSITION_GAIN = 8.0  # N·m/rad (E): rigidez de postura, sin identificar
POSITION_DAMPING = 0.3

IMU_POSITION = [0.0295, 0.016, 0.326]  # centro de la BNO085 en el torso, en m

HIP_Y = 0.029
FOOT_CENTER_X = 0.0125
FOOT_Y_OFFSET = 0.003
FOOT_HALF = (0.0425, 0.024)
SOLE_HEIGHT = 0.002
HAND_X = 0.0075
HAND_Y = 0.0584
HAND_Z = 0.148


# Colisiones simples en coordenadas de pose cero: (mínimo, máximo) en X, Y, Z, en mm. Solo lado izquierdo
BODY_COLLIDERS = {
    "base_link": [((-29.5, 27.0), (-40.5, 40.5), (241.0, 269.0))],
    "torso_1": [((-29.0, 44.0), (-39.0, 39.0), (272.5, 341.0))],
    "neck_link_1": [((-1.5, 16.5), (-17.5, 17.5), (341.4, 358.0))],
    "head_1": [((-13.0, 34.4), (-25.0, 25.0), (358.0, 395.0)), ((-7.7, 22.7), (-14.4, 14.4), (344.8, 358.0))],
}
SIDE_COLLIDERS = {
    "hip_yaw_link_1": [((-30.0, 15.0), (12.7, 39.0), (213.5, 240.5)), ((-6.0, 6.0), (12.7, 39.0), (240.5, 272.0))],
    "hip_roll_link_1": [((-26.5, 11.5), (4.3, 44.4), (188.0, 232.5))],
    "thigh_1": [((-15.0, 19.0), (11.5, 46.5), (106.0, 209.0))],
    "shin_1": [((-13.0, 20.0), (11.5, 46.5), (24.0, 126.0))],
    "ankle_link_1": [((-9.0, 53.2), (11.0, 46.5), (7.0, 44.0))],
    "foot_1": [((21.5, 49.5), (19.5, 56.0), (5.0, 28.0)), ((-30.0, 55.0), (8.0, 56.0), (2.0, 5.0))],
    "shoulder_link_1": [((-10.0, 25.0), (39.3, 65.5), (316.4, 334.4))],
    "upper_arm_1": [((-20.9, 26.9), (42.1, 66.9), (271.9, 339.4))],
    "elbow_link_1": [((-2.5, 17.5), (44.5, 73.5), (221.0, 271.3))],
    "forearm_1": [((-4.4, 34.4), (41.0, 69.8), (218.6, 243.4)), ((-5.7, 20.7), (47.0, 69.9), (148.0, 218.6))],
}
# Choques entre eslabones que no son vecinos: (eslabón izquierdo, caja), (eslabón derecho, caja)
LEFT_RIGHT_CONTACTS = [
    ("hip_roll_link_1", 0), ("thigh_1", 0), ("shin_1", 0), ("foot_1", 0), ("foot_1", 1), ("forearm_1", 1),
]
ARM_LEG_CONTACTS = [("forearm_1", 1, leg, 0) for leg in ("hip_roll_link_1", "thigh_1", "shin_1")]
BODY_FRICTION = 0.35  # PETG sobre el suelo (E)
HARDWARE_MASS_KG = 0.100  # tornillos y cables, repartidos por igual sobre lo modelado (E)


def add_collider(body, name, extent):
    geom = body.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.pos = [(low + high) / 2000 for low, high in extent]
    geom.size = [(high - low) / 2000 for low, high in extent]
    geom.rgba = [0.2, 0.6, 0.9, 0.3]
    geom.group = 3
    geom.friction = [BODY_FRICTION, 0.01, 0.001]
    geom.priority = 1
    geom.conaffinity = 0  # entre eslabones solo chocan los pares de add_link_contacts


def add_link_contacts(spec):
    """Contactos explícitos: siguen activos aunque el entrenamiento apague los demás entre eslabones."""
    pairs = [(f"left_{link}_box_{box}", f"right_{link}_box_{box}") for link, box in LEFT_RIGHT_CONTACTS]
    pairs += [("left_sole", "right_sole")]
    for side in ("left", "right"):
        pairs += [(f"{side}_{arm}_box_{arm_box}", f"{side}_{leg}_box_{leg_box}")
                  for arm, arm_box, leg, leg_box in ARM_LEG_CONTACTS]
    for one, two in pairs:
        pair = spec.add_pair(geomname1=one, geomname2=two)
        pair.condim = 3
        pair.friction = [BODY_FRICTION, BODY_FRICTION, 0.01, 0.001, 0.001]


def add_hardware_mass(spec):
    bodies = [body for body in spec.bodies if body.name != "world"]
    factor = 1 + HARDWARE_MASS_KG / sum(body.mass for body in bodies)
    for body in bodies:
        body.mass *= factor
        body.fullinertia = body.fullinertia * factor


def foot_center_y(side):
    return side * (HIP_Y + FOOT_Y_OFFSET)


def build_spec(actuated=True):
    """Devuelve el MjSpec de simulación: el exportado más lo que no sale de Fusion."""
    spec = mujoco.MjSpec.from_file(str(MODEL))
    spec.body("base_link").add_freejoint(name="base")
    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [2, 2, 0.1]
    floor.friction = [0.8, 0.01, 0.001]
    spec.worldbody.add_site(name="base_origin", pos=[0, 0, 0])

    for name, side in (("left", 1), ("right", -1)):
        foot = spec.body(f"{name}_foot_1")
        center = [FOOT_CENTER_X, foot_center_y(side), SOLE_HEIGHT / 2]
        sole = foot.add_geom()
        sole.name = f"{name}_sole"
        sole.type = mujoco.mjtGeom.mjGEOM_BOX
        sole.pos = center
        sole.size = [FOOT_HALF[0], FOOT_HALF[1], SOLE_HEIGHT / 2]
        sole.friction = [0.8, 0.01, 0.001]
        sole.priority = 1  # manda sobre el suelo: si no, MuJoCo toma el mayor rozamiento de los dos
        sole.conaffinity = 0
        foot.add_site(name=f"{name}_sole", pos=[center[0], center[1], 0])
        foot.add_site(name=f"{name}_toe", pos=[0.055, center[1], 0])
        foot.add_site(name=f"{name}_heel", pos=[-0.03, center[1], 0])
        spec.body(f"{name}_forearm_1").add_site(name=f"{name}_hand", pos=[HAND_X, side * HAND_Y, HAND_Z])
        spec.body(f"{name}_shin_1").add_site(name=f"{name}_knee", pos=[0.02, side * HIP_Y, 0.117])

    spec.body("torso_1").add_site(name="torso", pos=[0.0075, 0, 0.3])
    spec.body("torso_1").add_site(name="imu", pos=IMU_POSITION)
    spec.body("base_link").add_site(name="pelvis_bottom", pos=[0, 0, 0.241])
    spec.body("base_link").add_site(name="pelvis_back", pos=[-0.0295, 0, 0.255])

    for body_name, extents in BODY_COLLIDERS.items():
        for index, extent in enumerate(extents):
            add_collider(spec.body(body_name), f"{body_name}_box_{index}", extent)
    for name, side in (("left", 1), ("right", -1)):
        for link, extents in SIDE_COLLIDERS.items():
            for index, (x, y, z) in enumerate(extents):
                mirrored = y if side > 0 else (-y[1], -y[0])
                add_collider(spec.body(f"{name}_{link}"), f"{name}_{link}_box_{index}", (x, mirrored, z))

    add_link_contacts(spec)
    add_hardware_mass(spec)

    joints = [joint for joint in spec.joints if joint.type == mujoco.mjtJoint.mjJNT_HINGE]
    back_emf = SERVO["kt"] ** 2 / SERVO["R"]
    for joint in joints:
        joint.armature = SERVO["armature"]
        joint.damping = [SERVO["friction_viscous"] + back_emf, 0, 0]
        joint.frictionloss = SERVO["friction_base"] + SERVO["friction_stribeck"]
        if actuated:
            actuator = spec.add_actuator()
            actuator.name = joint.name
            actuator.target = joint.name
            actuator.trntype = mujoco.mjtTrn.mjTRN_JOINT
            actuator.set_to_position(kp=POSITION_GAIN, kv=POSITION_DAMPING)
            limit = stall_torque(BATTERY_VOLTS[0])
            actuator.forcerange = [-limit, limit]
            actuator.forcelimited = 1
    return spec


def build_model(actuated=True):
    return build_spec(actuated).compile()


def stall_torque(volts):
    return SERVO["max_pwm"] * SERVO["kt"] * volts / SERVO["R"]


def no_load_speed(volts):
    return SERVO["max_pwm"] * volts / SERVO["kt"]


def available_torque(volts, speed):
    """Par máximo del modelo eléctrico a esa velocidad, sin rozamiento."""
    return np.maximum(0.0, SERVO["kt"] * (SERVO["max_pwm"] * volts - SERVO["kt"] * np.abs(speed)) / SERVO["R"])


def hinge_names(model):
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        for joint in range(model.njnt) if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE
    ]
