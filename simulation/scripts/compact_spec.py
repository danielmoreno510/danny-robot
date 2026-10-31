"""
Danny - Humanoide compacto de unos 40 cm con servos de bus (fase 5).

Convive con robot_spec.py, que conserva los modelos de 70 cm validados: los nombres, ejes
y topes de las articulaciones salen de allí. Aquí las longitudes no escalan sin más: el
pie, la cadera y el pecho tienen mínimos que imponen los componentes, y las masas se
suman pieza a pieza. Todo es provisional hasta medir un servo y tener el CAD.

Fuentes en documentation/phase-5.2-compact-design.md.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

import robot_spec as spec


SERVO_MASS = 0.055  # kg, STS3215 C018 (tienda)
SERVO_SIZE = (0.0452, 0.0247, 0.035)  # m
OPEN_DUCK_STRUCTURE = 1.19  # kg a 42 cm: 2,107 kg menos servos, baterías y electrónica (estimado)
OPEN_DUCK_HEIGHT = 0.42
ELECTRONICS_MASS = {"bus": 0.016, "cables": 0.05, "regulador": 0.02}  # kg
BATTERY_MASS = 0.115  # kg, 3S 1300 mAh, 14,4 Wh (tienda)
JETSON_MASS = 0.20  # kg, estimado: la masa del módulo no se ha encontrado
PI_ZERO_MASS = 0.016  # kg

FOOT_CENTER = 0.01  # m, centro del pie por delante del tobillo

# 22 GDL: piernas completas, cintura yaw, brazos de 4 y cuello yaw
WAIST_JOINTS = spec.WAIST_JOINTS[:1]
ARM_JOINTS = spec.ARM_JOINTS[:4]
NECK_JOINTS = spec.NECK_JOINTS[:1]


@dataclass(frozen=True)
class Motor:
    """Par de bloqueo y vacío de tienda; kp, damping, armature y frictionloss de Open Duck."""

    name: str
    stall_torque: float  # N·m
    rated_torque: float  # N·m, sostenible según catálogo: sin confirmar
    no_load_speed: float  # rad/s
    mass: float  # kg
    kp: float = 17.8
    damping: float = 0.6
    armature: float = 0.028
    friction: float = 0.052


STS3215_C018 = Motor("STS3215 C018 12 V", 2.94, 0.98, 4.71, 0.055)
STS3215_C001 = Motor("STS3215 C001 7,4 V", 1.91, 0.49, 5.45, 0.055)
STS3215_C044 = Motor("STS3215 C044 7,4 V", 2.69, 0.88, 9.0, 0.055)
ST3215_HS = Motor("ST3215-HS 12 V", 1.96, 0.0, 11.1, 0.068)


@dataclass(frozen=True)
class Design:
    height: float
    motor: Motor = STS3215_C018
    onboard_mass: float = JETSON_MASS

    @property
    def ankle(self) -> float:
        return 0.03  # placa, servo de roll y holgura: no baja con la altura

    @property
    def thigh(self) -> float:
        return 0.2125 * self.height

    @property
    def shin(self) -> float:
        return self.thigh

    @property
    def leg(self) -> float:
        return self.ankle + self.shin + self.thigh

    @property
    def hip_y(self) -> float:
        return 0.04  # servo, paredes y holgura entre piernas

    @property
    def hip_below_pelvis(self) -> float:
        return 0.025

    @property
    def waist_above_pelvis(self) -> float:
        return 0.02

    @property
    def neck_above_waist(self) -> float:
        return self.height - self.leg - self.hip_below_pelvis - self.waist_above_pelvis - 2 * self.head_radius - 0.005

    @property
    def shoulder_above_waist(self) -> float:
        return self.neck_above_waist - 0.005

    @property
    def head_radius(self) -> float:
        return 0.0875 * self.height

    @property
    def shoulder_y(self) -> float:
        return 0.08  # el pecho lo fija la placa del Jetson; el brazo libra el torso

    @property
    def upper_arm(self) -> float:
        return 0.15 * self.height

    @property
    def forearm(self) -> float:
        return 0.1375 * self.height

    @property
    def hand(self) -> float:
        return 0.0625 * self.height

    @property
    def foot_half(self) -> tuple[float, float, float]:
        return (0.1125 * self.height, 0.025, 0.005)

    @property
    def torso_half(self) -> tuple[float, float, float]:
        return (0.04, 0.06, (self.neck_above_waist) / 2)

    @property
    def structure_mass(self) -> float:
        return OPEN_DUCK_STRUCTURE * (self.height / OPEN_DUCK_HEIGHT) ** 2

    def masses(self) -> dict[str, float]:
        """Cada segmento: sus servos y su parte de la estructura."""
        servo = self.motor.mass
        share = self.structure_mass
        electronics = sum(ELECTRONICS_MASS.values()) + BATTERY_MASS + self.onboard_mass
        return {
            "pelvis": 3 * servo + 0.24 * share,
            "torso": 3 * servo + 0.27 * share + electronics,  # hombros y cuello, con electrónica
            "thigh": 3 * servo + 0.08 * share,
            "shin": 2 * servo + 0.06 * share,
            "foot": 0.03 * share,
            "upper_arm": 3 * servo + 0.03 * share,  # hombro roll y yaw, y codo
            "forearm": 0.02 * share,
            "hand": 0.01 * share,
            "head": 0.03 * share,
        }

    def total_mass(self) -> float:
        m = self.masses()
        return m["pelvis"] + m["torso"] + m["head"] + 2 * sum(
            m[part] for part in ("thigh", "shin", "foot", "upper_arm", "forearm", "hand"))


def add_joint(body, prefix: str, joint: spec.Joint, motor: Motor, axis=None) -> None:
    limits = spec.HUMANOID.limits(joint)
    body.add_joint(name=f"{prefix}{joint.part}_joint", type=mujoco.mjtJoint.mjJNT_HINGE,
                   axis=axis or joint.axis, range=limits, limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                   damping=motor.damping, armature=motor.armature, frictionloss=motor.friction)


def add_capsule(body, name: str, length: float, mass: float, radius: float) -> None:
    geom = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=(radius, 0, 0), mass=mass)
    geom.fromto = (0, 0, 0, 0, 0, -length)


def build(design: Design) -> mujoco.MjSpec:
    model = mujoco.MjSpec()
    model.modelname = f"Danny compacto {round(design.height * 100)} cm"
    model.compiler.degree = False
    model.option.timestep = spec.TIMESTEP
    model.option.gravity = (0, 0, -spec.GRAVITY)
    model.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    masses = design.masses()
    motor = design.motor

    world = model.worldbody
    world.add_light(pos=(0, 0, 3), dir=(0, 0, -1))
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=(2, 2, 0.1), friction=spec.FLOOR_FRICTION)

    pelvis = world.add_body(name="pelvis", pos=(0, 0, design.leg + design.hip_below_pelvis))
    pelvis.add_freejoint(name="pelvis_free")
    pelvis.add_geom(name="pelvis_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=(0.035, 0.055, 0.025), mass=masses["pelvis"])
    pelvis.add_site(name="imu_site", size=(0.005, 0.005, 0.005))

    waist = pelvis.add_body(name="torso", pos=(0, 0, design.waist_above_pelvis))
    add_joint(waist, "", WAIST_JOINTS[0], motor)
    half = design.torso_half
    waist.add_geom(name="torso_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, half[2]), size=half, mass=masses["torso"])

    neck = waist.add_body(name="head", pos=(0, 0, design.neck_above_waist))
    add_joint(neck, "", NECK_JOINTS[0], motor)
    neck.add_geom(name="head_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=(0, 0, 0.005 + design.head_radius),
                  size=(design.head_radius, 0, 0), mass=masses["head"])

    for side, sign in zip(spec.SIDES, (1, -1)):
        add_arm(waist, design, side, sign, masses)
        add_leg(pelvis, design, side, sign, masses)

    for side in spec.SIDES:
        model.add_exclude(bodyname1="pelvis", bodyname2=f"{side}_thigh")
    model.add_exclude(bodyname1="pelvis", bodyname2="torso")

    add_motors(model, motor)
    add_stand_keyframe(model)
    return model


def add_leg(pelvis, design: Design, side: str, sign: int, masses: dict) -> None:
    motor = design.motor
    thigh = pelvis.add_body(name=f"{side}_thigh", pos=(0, sign * design.hip_y, -design.hip_below_pelvis))
    for joint in spec.HIP_JOINTS:
        add_joint(thigh, f"{side}_", joint, motor)
    add_capsule(thigh, f"{side}_thigh_geom", design.thigh, masses["thigh"], 0.016)
    shin = thigh.add_body(name=f"{side}_shin", pos=(0, 0, -design.thigh))
    add_joint(shin, f"{side}_", spec.KNEE_JOINT, motor)
    add_capsule(shin, f"{side}_shin_geom", design.shin, masses["shin"], 0.014)
    foot = shin.add_body(name=f"{side}_foot", pos=(0, 0, -design.shin))
    for joint in spec.ANKLE_JOINTS:
        add_joint(foot, f"{side}_", joint, motor)
    half = design.foot_half
    foot.add_geom(name=f"{side}_foot_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                  pos=(FOOT_CENTER, 0, -design.ankle + half[2]), size=half, mass=masses["foot"])
    foot.add_site(name=f"{side}_sole_site", pos=(0, 0, -design.ankle), size=(0.005, 0.005, 0.005))


def add_arm(torso, design: Design, side: str, sign: int, masses: dict) -> None:
    motor = design.motor
    pitch, roll, yaw, elbow = ARM_JOINTS

    def add(body, joint):
        add_joint(body, f"{side}_", joint, motor, spec.mirrored_axis(joint, side))

    shoulder = torso.add_body(name=f"{side}_shoulder", pos=(0, sign * design.shoulder_y, design.shoulder_above_waist))
    add(shoulder, pitch)
    add(shoulder, roll)
    add(shoulder, yaw)
    add_capsule(shoulder, f"{side}_upper_arm_geom", design.upper_arm, masses["upper_arm"], 0.014)
    forearm = shoulder.add_body(name=f"{side}_forearm", pos=(0, 0, -design.upper_arm))
    add(forearm, elbow)
    add_capsule(forearm, f"{side}_forearm_geom", design.forearm, masses["forearm"], 0.012)
    hand = forearm.add_body(name=f"{side}_hand", pos=(0, 0, -design.forearm))
    hand.add_geom(name=f"{side}_hand_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, -design.hand / 2),
                  size=(0.008, 0.015, design.hand / 2), mass=masses["hand"])


def add_motors(model, motor: Motor) -> None:
    joints = [(f"{side}_{j.part}", j) for side in spec.SIDES for j in spec.LEG_JOINTS]
    joints += [(j.part, j) for j in WAIST_JOINTS]
    joints += [(f"{side}_{j.part}", j) for side in spec.SIDES for j in ARM_JOINTS]
    joints += [(j.part, j) for j in NECK_JOINTS]
    for name, joint in joints:
        actuator = model.add_actuator(
            name=f"{name}_motor", target=f"{name}_joint", trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=spec.HUMANOID.limits(joint), ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            forcerange=(-motor.stall_torque, motor.stall_torque), forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
        actuator.set_to_position(kp=motor.kp, kv=0.0)


def add_stand_keyframe(model) -> None:
    compiled = model.compile()
    data = mujoco.MjData(compiled)
    balance_on(compiled, data, np.radians(spec.STAND_KNEE_DEG), ("left", "right"))
    ctrl = data.qpos[compiled.jnt_qposadr[compiled.actuator_trnid[:, 0]]].copy()
    model.add_key(name="stand", qpos=data.qpos.copy(), ctrl=np.clip(ctrl, *compiled.actuator_ctrlrange.T))


def set_stand(model, data, knee: float, lean: float = 0.0, shift: float = 0.0, lift: float = 0.0) -> None:
    """
    Pose de pie: 'lean' inclina las espinillas adelante y 'shift' las lleva de lado, con la
    pelvis y los pies horizontales. 'lift' dobla la pierna derecha para sacarla del suelo.
    """
    def put(joint, value):
        data.qpos[model.joint(f"{joint}_joint").qposadr[0]] = value

    data.qpos[:] = 0.0
    data.qpos[3] = 1.0
    for side in spec.SIDES:
        put(f"{side}_hip_pitch", -knee / 2 + lean)
        put(f"{side}_knee", knee)
        put(f"{side}_ankle_pitch", -knee / 2 - lean)
        put(f"{side}_hip_roll", shift)
        put(f"{side}_ankle_roll", -shift)
    if lift:
        put("right_hip_pitch", -knee / 2 + lean - lift)
        put("right_knee", knee + 2 * lift)
        put("right_ankle_pitch", -knee / 2 - lean - lift)
        put("right_hip_roll", 0.0)
        put("right_ankle_roll", 0.0)
    mujoco.mj_kinematics(model, data)
    sole = data.site("left_sole_site").xpos.copy()
    data.qpos[:3] = [-sole[0], 0.0, -sole[2]]
    mujoco.mj_forward(model, data)


def balance_on(model, data, knee: float, feet: tuple[str, ...], lift: float = 0.0) -> None:
    """Busca la pose con el CdM sobre los pies: Newton sobre 'lean' y 'shift'."""
    def error(values):
        set_stand(model, data, knee, values[0], values[1], lift)
        center = np.mean([data.site(f"{side}_sole_site").xpos for side in feet], axis=0)
        return data.subtree_com[1][:2] - center[:2] - np.array([FOOT_CENTER, 0.0])

    values = np.zeros(2)
    for _ in range(12):
        current = error(values)
        if np.linalg.norm(current) < 1e-6:
            break
        jacobian = np.column_stack([(error(values + step) - current) / 1e-4 for step in np.eye(2) * 1e-4])
        values -= np.linalg.solve(jacobian, current)
    error(values)
