"""
Danny - Genera los modelos MuJoCo desde robot_spec.py.

Izquierda y derecha salen de una sola descripción. El resultado se versiona en
simulation/models/ y una prueba comprueba que coincide con la especificación:
ese XML no se edita a mano.

Uso:
  python simulation/scripts/build_model.py
"""

from pathlib import Path

import mujoco
import numpy as np

import compact_spec
import robot_spec as spec
from biped_control import Biped, Pose, center_com_over, pelvis_height


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "models"
COMPACT_FILE = "danny_compact.xml"

BLUE = (0.1, 0.4, 0.9, 1)
GREY = (0.3, 0.3, 0.3, 1)
LIGHT_GREY = (0.8, 0.8, 0.85, 1)
FLOOR_GREY = (0.7, 0.7, 0.7, 1)
ORANGE = (0.9, 0.5, 0.1, 1)
INVISIBLE = (0, 0, 0, 0)
SITE_SIZE = (0.01, 0.005, 0.005)
SHIN_COLOR = {"left": (0.1, 0.7, 0.5, 1), "right": (0.7, 0.5, 0.1, 1)}


def add_joint(body, prefix: str, joint: spec.Joint, robot: spec.Robot, axis=None) -> None:
    motor = robot.motor(joint.part)
    body.add_joint(
        name=f"{prefix}{joint.part}_joint",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=axis or joint.axis,
        range=robot.limits(joint),
        limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        damping=motor.damping,
        armature=motor.armature,
    )


def add_segment(body, name: str, length: float, mass: float, rgba, radius: float = spec.LEG_RADIUS) -> None:
    geom = body.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=(radius, 0, 0),
        mass=mass,
        rgba=rgba,
    )
    geom.fromto = (0, 0, 0, 0, 0, -length)


def add_leg(pelvis, robot: spec.Robot, side: str) -> None:
    sign = 1 if side == "left" else -1
    thigh = pelvis.add_body(name=f"{side}_thigh", pos=(0, sign * spec.HIP_Y, -spec.HIP_BELOW_PELVIS))
    for joint in spec.HIP_JOINTS:
        add_joint(thigh, f"{side}_", joint, robot)
    add_segment(thigh, f"{side}_thigh_geom", spec.THIGH_LENGTH, robot.thigh_mass, BLUE)

    shin = thigh.add_body(name=f"{side}_shin", pos=(0, 0, -spec.THIGH_LENGTH))
    add_joint(shin, f"{side}_", spec.KNEE_JOINT, robot)
    add_segment(shin, f"{side}_shin_geom", spec.SHIN_LENGTH, robot.shin_mass, SHIN_COLOR[side])

    foot = shin.add_body(name=f"{side}_foot", pos=(0, 0, -spec.SHIN_LENGTH))
    for joint in spec.ANKLE_JOINTS:
        add_joint(foot, f"{side}_", joint, robot)
    sole_height = -spec.SOLE_BELOW_ANKLE + spec.FOOT_HALF[2]
    foot.add_geom(
        name=f"{side}_foot_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(0, 0, sole_height),
        size=spec.FOOT_HALF,
        mass=robot.foot_mass,
        rgba=ORANGE,
    )
    foot.add_site(name=f"{side}_sole_site", pos=(0, 0, -spec.SOLE_BELOW_ANKLE), size=SITE_SIZE, rgba=INVISIBLE)
    foot.add_site(
        name=f"{side}_touch_site",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(0, 0, sole_height),
        size=(spec.FOOT_HALF[0] + 0.005, spec.FOOT_HALF[1] + 0.005, spec.FOOT_HALF[2] + 0.005),
        rgba=INVISIBLE,
    )


def add_arm(torso, robot: spec.Robot, side: str) -> None:
    """Hombro, brazo, antebrazo y muñeca: 7 GDL. La mano es un bloque rígido."""
    sign = 1 if side == "left" else -1
    pitch, roll, yaw, elbow, *wrist = spec.ARM_JOINTS

    def add(body, joint):
        add_joint(body, f"{side}_", joint, robot, spec.mirrored_axis(joint, side))

    shoulder = torso.add_body(name=f"{side}_shoulder", pos=(0, sign * spec.SHOULDER_Y, spec.SHOULDER_ABOVE_WAIST))
    add(shoulder, pitch)
    add(shoulder, roll)
    shoulder.add_geom(name=f"{side}_shoulder_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                      size=(spec.SHOULDER_RADIUS, 0, 0), mass=robot.shoulder_mass, rgba=GREY)

    upper_arm = shoulder.add_body(name=f"{side}_upper_arm")
    add(upper_arm, yaw)
    add_segment(upper_arm, f"{side}_upper_arm_geom", spec.UPPER_ARM_LENGTH, robot.upper_arm_mass, BLUE, spec.ARM_RADIUS)

    forearm = upper_arm.add_body(name=f"{side}_forearm", pos=(0, 0, -spec.UPPER_ARM_LENGTH))
    add(forearm, elbow)
    add_segment(forearm, f"{side}_forearm_geom", spec.FOREARM_LENGTH, robot.forearm_mass, SHIN_COLOR[side], spec.ARM_RADIUS)

    hand = forearm.add_body(name=f"{side}_hand", pos=(0, 0, -spec.FOREARM_LENGTH))
    for joint in wrist:
        add(hand, joint)
    hand.add_geom(name=f"{side}_hand_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, -spec.HAND_HALF[2]),
                  size=spec.HAND_HALF, mass=robot.hand_mass, rgba=ORANGE)
    hand.add_site(name=f"{side}_wrist_site", size=SITE_SIZE, rgba=INVISIBLE)
    hand.add_site(name=f"{side}_fingertip_site", pos=(0, 0, -2 * spec.HAND_HALF[2]), size=SITE_SIZE, rgba=INVISIBLE)


def add_neck_and_head(torso, robot: spec.Robot) -> None:
    yaw, pitch = spec.NECK_JOINTS
    neck = torso.add_body(name="neck", pos=(0, 0, spec.NECK_BASE_ABOVE_WAIST))
    add_joint(neck, "", yaw, robot)
    add_segment(neck, "neck_geom", -spec.NECK_PITCH_ABOVE_BASE, robot.neck_mass, GREY, spec.NECK_RADIUS)

    head = neck.add_body(name="head", pos=(0, 0, spec.NECK_PITCH_ABOVE_BASE))
    add_joint(head, "", pitch, robot)
    head.add_geom(name="head_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=(0, 0, spec.HEAD_ABOVE_NECK_PITCH),
                  size=(spec.HEAD_RADIUS, 0, 0), mass=robot.head_mass, rgba=ORANGE)
    head.add_site(name="head_site", pos=(0, 0, spec.HEAD_ABOVE_NECK_PITCH), size=SITE_SIZE, rgba=INVISIBLE)


def add_motor(model, name: str, joint: spec.Joint, robot: spec.Robot, kv: float) -> None:
    catalog = robot.motor(joint.part)
    motor = model.add_actuator(
        name=f"{name}_motor",
        target=f"{name}_joint",
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        ctrlrange=joint.command_limits or robot.limits(joint),
        ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
        forcerange=(-catalog.peak_torque, catalog.peak_torque),
        forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
    )
    motor.set_to_position(kp=catalog.kp, kv=kv)


def add_motors(model, robot: spec.Robot) -> None:
    for side in spec.SIDES:
        for joint in spec.LEG_JOINTS:
            add_motor(model, f"{side}_{joint.part}", joint, robot, robot.leg_kv[joint.part])
    if robot.has_waist:
        for joint in spec.WAIST_JOINTS:
            add_motor(model, joint.part, joint, robot, robot.waist_kv[joint.part])
    if robot.has_arms:
        for side in spec.SIDES:
            for joint in spec.ARM_JOINTS:
                add_motor(model, f"{side}_{joint.part}", joint, robot, robot.arm_kv[joint.part])
        for joint in spec.NECK_JOINTS:
            add_motor(model, joint.part, joint, robot, robot.neck_kv[joint.part])


def add_waist_and_torso(pelvis, robot: spec.Robot) -> None:
    """Cintura de 3 ejes, y sobre ella el torso superior con brazos, cuello y cabeza."""
    yaw, roll, pitch = spec.WAIST_JOINTS
    waist = pelvis.add_body(name="waist", pos=(0, 0, spec.WAIST_ABOVE_PELVIS))
    add_joint(waist, "", yaw, robot)
    waist.add_geom(name="waist_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=spec.WAIST_LINK_HALF,
                   mass=robot.waist_mass, rgba=GREY)
    roll_link = waist.add_body(name="waist_roll_link")
    add_joint(roll_link, "", roll, robot)
    roll_link.add_geom(name="waist_roll_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=spec.WAIST_ROLL_LINK_HALF,
                       mass=robot.waist_roll_mass, rgba=GREY)
    torso = roll_link.add_body(name="torso")
    add_joint(torso, "", pitch, robot)
    torso.add_geom(name="torso_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, spec.UPPER_TORSO_OFFSET),
                   size=spec.UPPER_TORSO_HALF, mass=robot.upper_body_mass, rgba=LIGHT_GREY)
    if robot.has_arms:
        for side in spec.SIDES:
            add_arm(torso, robot, side)
        add_neck_and_head(torso, robot)


def add_sensors(model, robot: spec.Robot) -> None:
    model.add_sensor(name="pelvis_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                     objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_site")
    model.add_sensor(name="pelvis_pos", type=mujoco.mjtSensor.mjSENS_FRAMEPOS,
                     objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_site")
    for side in spec.SIDES:
        for part in spec.JOINT_PARTS:
            model.add_sensor(name=f"{side}_{part}_pos", type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                             objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"{side}_{part}_joint")
    for side in spec.SIDES:
        for part in spec.JOINT_PARTS:
            model.add_sensor(name=f"{side}_{part}_torque", type=mujoco.mjtSensor.mjSENS_ACTUATORFRC,
                             objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f"{side}_{part}_motor")
    held = [joint.part for joint in spec.WAIST_JOINTS] if robot.has_waist else []
    if robot.has_arms:
        held += [*spec.ARM_PARTS, *spec.NECK_PARTS]
    for part in held:
        model.add_sensor(name=f"{part}_pos", type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                         objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"{part}_joint")
        model.add_sensor(name=f"{part}_torque", type=mujoco.mjtSensor.mjSENS_ACTUATORFRC,
                         objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f"{part}_motor")
    for side in spec.SIDES:
        model.add_sensor(name=f"{side}_foot_force", type=mujoco.mjtSensor.mjSENS_TOUCH,
                         objtype=mujoco.mjtObj.mjOBJ_SITE, objname=f"{side}_touch_site")
    if robot.has_arms:  # lo que necesita una política: IMU y velocidades
        model.add_sensor(name="pelvis_gyro", type=mujoco.mjtSensor.mjSENS_GYRO,
                         objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_site")
        model.add_sensor(name="pelvis_accel", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                         objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu_site")
        for part in (*(f"{side}_{leg}" for side in spec.SIDES for leg in spec.JOINT_PARTS), *held):
            model.add_sensor(name=f"{part}_vel", type=mujoco.mjtSensor.mjSENS_JOINTVEL,
                             objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"{part}_joint")


def add_stand_keyframe(model) -> None:
    """Postura de pie para arrancar: CdM centrado entre los pies y motores en esa postura."""
    compiled = model.compile()
    data = mujoco.MjData(compiled)
    biped = Biped(compiled, data)
    height = pelvis_height(spec.STAND_KNEE_DEG)
    stand = Pose(np.array([0.0, 0.0, height]), np.array([0.0, spec.HIP_Y, 0.0]), np.array([0.0, -spec.HIP_Y, 0.0]))
    biped.set_pose(data, center_com_over(biped, stand, "left", inset=spec.HIP_Y))
    ctrl = np.clip(data.qpos[compiled.actuator_trnid[:, 0] + 6], *compiled.actuator_ctrlrange.T)
    model.add_key(name="stand", qpos=data.qpos.copy(), ctrl=ctrl)


def build_robot(robot: spec.Robot) -> mujoco.MjSpec:
    model = mujoco.MjSpec()
    model.modelname = f"Danny {Path(robot.file).stem}"
    model.compiler.degree = False
    model.option.timestep = spec.TIMESTEP
    model.option.gravity = (0, 0, -spec.GRAVITY)
    model.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    visual = getattr(model.visual, "global")
    visual.azimuth = 150
    visual.elevation = -15
    visual.offwidth = 1920
    visual.offheight = 1080

    world = model.worldbody
    world.add_light(pos=(0, 0, 3), dir=(0, 0, -1))
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(2, 2, 0.1),
        rgba=FLOOR_GREY,
        friction=spec.FLOOR_FRICTION,
    )

    pelvis = world.add_body(name="pelvis", pos=(0, 0, spec.PELVIS_START_HEIGHT))
    pelvis.add_freejoint(name="pelvis_free")
    pelvis.add_geom(
        name="pelvis_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=spec.PELVIS_HALF,
        mass=robot.pelvis_mass,
        rgba=GREY,
    )
    if robot.has_waist:
        add_waist_and_torso(pelvis, robot)
    else:
        # Torso provisional: sin brazos ni articulación, solo su masa y su altura
        pelvis.add_geom(
            name="torso_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=(0, 0, spec.TORSO_OFFSET),
            size=spec.TORSO_HALF,
            mass=robot.upper_body_mass,
            rgba=LIGHT_GREY,
        )
    pelvis.add_site(name="imu_site", size=SITE_SIZE, rgba=INVISIBLE)

    for side in spec.SIDES:
        add_leg(pelvis, robot, side)

    # La cadera está dentro de la pelvis a propósito. MuJoCo excluye los cuerpos
    # unidos, pero no si el padre está fijo al mundo.
    for side in spec.SIDES:
        model.add_exclude(bodyname1="pelvis", bodyname2=f"{side}_thigh")
    if robot.has_waist:
        # La cintura entra en la pelvis y en el torso
        model.add_exclude(bodyname1="pelvis", bodyname2="torso")
        model.add_exclude(bodyname1="pelvis", bodyname2="waist_roll_link")
        model.add_exclude(bodyname1="waist", bodyname2="torso")

    add_motors(model, robot)
    add_sensors(model, robot)
    if robot.has_arms:
        add_stand_keyframe(model)
    return model


def build_biped() -> mujoco.MjSpec:
    return build_robot(spec.BIPED)


def model_path(robot: spec.Robot) -> Path:
    return MODELS / robot.file


def main() -> None:
    for robot in spec.ROBOTS.values():
        model_path(robot).write_text(build_robot(robot).to_xml())
        print(f"Generado {robot.file}")
    (MODELS / COMPACT_FILE).write_text(compact_spec.build(compact_spec.Design(0.40)).to_xml())
    print(f"Generado {COMPACT_FILE}")


if __name__ == "__main__":
    main()
