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

import robot_spec as spec


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "models"

BLUE = (0.1, 0.4, 0.9, 1)
GREY = (0.3, 0.3, 0.3, 1)
LIGHT_GREY = (0.8, 0.8, 0.85, 1)
FLOOR_GREY = (0.7, 0.7, 0.7, 1)
ORANGE = (0.9, 0.5, 0.1, 1)
GHOST = (0.8, 0.3, 0.3, 0.4)
INVISIBLE = (0, 0, 0, 0)
SITE_SIZE = (0.01, 0.005, 0.005)
SHIN_COLOR = {"left": (0.1, 0.7, 0.5, 1), "right": (0.7, 0.5, 0.1, 1)}


def add_joint(body, prefix: str, joint: spec.Joint) -> None:
    body.add_joint(
        name=f"{prefix}{joint.part}_joint",
        type=mujoco.mjtJoint.mjJNT_HINGE,
        axis=joint.axis,
        range=joint.limits,
        limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        damping=spec.JOINT_DAMPING,
        armature=spec.JOINT_ARMATURE,
    )


def add_segment(body, name: str, length: float, mass: float, rgba) -> None:
    geom = body.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=(spec.LEG_RADIUS, 0, 0),
        mass=mass,
        rgba=rgba,
    )
    geom.fromto = (0, 0, 0, 0, 0, -length)


def add_leg(pelvis, robot: spec.Robot, side: str) -> None:
    sign = 1 if side == "left" else -1
    thigh = pelvis.add_body(name=f"{side}_thigh", pos=(0, sign * spec.HIP_Y, -spec.HIP_BELOW_PELVIS))
    for joint in spec.HIP_JOINTS:
        add_joint(thigh, f"{side}_", joint)
    add_segment(thigh, f"{side}_thigh_geom", spec.THIGH_LENGTH, robot.thigh_mass, BLUE)

    shin = thigh.add_body(name=f"{side}_shin", pos=(0, 0, -spec.THIGH_LENGTH))
    add_joint(shin, f"{side}_", spec.KNEE_JOINT)
    add_segment(shin, f"{side}_shin_geom", spec.SHIN_LENGTH, robot.shin_mass, SHIN_COLOR[side])

    foot = shin.add_body(name=f"{side}_foot", pos=(0, 0, -spec.SHIN_LENGTH))
    for joint in spec.ANKLE_JOINTS:
        add_joint(foot, f"{side}_", joint)
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


def add_motor(model, name: str, joint: spec.Joint, kv: float) -> None:
    motor = model.add_actuator(
        name=f"{name}_motor",
        target=f"{name}_joint",
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        ctrlrange=joint.command_limits or joint.limits,
        ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
        forcerange=(-spec.MOTOR_TORQUE_LIMIT, spec.MOTOR_TORQUE_LIMIT),
        forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
    )
    motor.set_to_position(kp=spec.MOTOR_KP, kv=kv)


def add_motors(model, robot: spec.Robot) -> None:
    for side in spec.SIDES:
        for joint in spec.LEG_JOINTS:
            add_motor(model, f"{side}_{joint.part}", joint, robot.leg_kv[joint.part])
    if robot.has_waist:
        for joint in spec.WAIST_JOINTS:
            add_motor(model, joint.part, joint, robot.waist_kv[joint.part])


def add_waist_and_torso(pelvis, robot: spec.Robot) -> None:
    """Cintura de 2 ejes y torso superior, con lastres en lugar de brazos y cabeza."""
    yaw, pitch = spec.WAIST_JOINTS
    waist = pelvis.add_body(name="waist", pos=(0, 0, spec.WAIST_ABOVE_PELVIS))
    add_joint(waist, "", yaw)
    waist.add_geom(name="waist_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=spec.WAIST_LINK_HALF,
                   mass=robot.waist_mass, rgba=GREY)
    torso = waist.add_body(name="torso")
    add_joint(torso, "", pitch)
    torso.add_geom(name="torso_geom", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, spec.UPPER_TORSO_OFFSET),
                   size=spec.UPPER_TORSO_HALF, mass=robot.upper_body_mass, rgba=LIGHT_GREY)
    # Lastres sin colisión: masa e inercia, hasta que existan brazos y cabeza
    torso.add_geom(name="arm_ballast", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0, 0, spec.ARM_BALLAST_OFFSET),
                   size=spec.ARM_BALLAST_HALF, mass=robot.arm_ballast, rgba=GHOST, contype=0, conaffinity=0)
    torso.add_geom(name="head_ballast", type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=(0, 0, spec.HEAD_BALLAST_OFFSET),
                   size=(spec.HEAD_BALLAST_RADIUS, 0, 0), mass=robot.head_ballast, rgba=GHOST, contype=0, conaffinity=0)


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
    if robot.has_waist:
        for joint in spec.WAIST_JOINTS:
            model.add_sensor(name=f"{joint.part}_pos", type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                             objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=f"{joint.part}_joint")
            model.add_sensor(name=f"{joint.part}_torque", type=mujoco.mjtSensor.mjSENS_ACTUATORFRC,
                             objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f"{joint.part}_motor")
    for side in spec.SIDES:
        model.add_sensor(name=f"{side}_foot_force", type=mujoco.mjtSensor.mjSENS_TOUCH,
                         objtype=mujoco.mjtObj.mjOBJ_SITE, objname=f"{side}_touch_site")


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
        model.add_exclude(bodyname1="pelvis", bodyname2="torso")  # la cintura entra en la pelvis

    add_motors(model, robot)
    add_sensors(model, robot)
    return model


def build_biped() -> mujoco.MjSpec:
    return build_robot(spec.BIPED)


def model_path(robot: spec.Robot) -> Path:
    return MODELS / robot.file


def main() -> None:
    for robot in spec.ROBOTS.values():
        model_path(robot).write_text(build_robot(robot).to_xml())
        print(f"Generado {robot.file}")


if __name__ == "__main__":
    main()
