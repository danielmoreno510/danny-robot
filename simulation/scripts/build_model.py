"""
Danny - Genera el modelo MuJoCo desde robot_spec.py.

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
MODEL_PATH = ROOT / "models" / "danny_biped.xml"

BLUE = (0.1, 0.4, 0.9, 1)
GREY = (0.3, 0.3, 0.3, 1)
LIGHT_GREY = (0.8, 0.8, 0.85, 1)
FLOOR_GREY = (0.7, 0.7, 0.7, 1)
ORANGE = (0.9, 0.5, 0.1, 1)
INVISIBLE = (0, 0, 0, 0)
SITE_SIZE = (0.01, 0.005, 0.005)
SHIN_COLOR = {"left": (0.1, 0.7, 0.5, 1), "right": (0.7, 0.5, 0.1, 1)}


def add_joint(body, side: str, joint: spec.Joint) -> None:
    body.add_joint(
        name=f"{side}_{joint.part}_joint",
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


def add_leg(pelvis, side: str) -> None:
    sign = 1 if side == "left" else -1
    thigh = pelvis.add_body(name=f"{side}_thigh", pos=(0, sign * spec.HIP_Y, -spec.HIP_BELOW_PELVIS))
    for joint in spec.HIP_JOINTS:
        add_joint(thigh, side, joint)
    add_segment(thigh, f"{side}_thigh_geom", spec.THIGH_LENGTH, spec.THIGH_MASS, BLUE)

    shin = thigh.add_body(name=f"{side}_shin", pos=(0, 0, -spec.THIGH_LENGTH))
    add_joint(shin, side, spec.KNEE_JOINT)
    add_segment(shin, f"{side}_shin_geom", spec.SHIN_LENGTH, spec.SHIN_MASS, SHIN_COLOR[side])

    foot = shin.add_body(name=f"{side}_foot", pos=(0, 0, -spec.SHIN_LENGTH))
    for joint in spec.ANKLE_JOINTS:
        add_joint(foot, side, joint)
    sole_height = -spec.SOLE_BELOW_ANKLE + spec.FOOT_HALF[2]
    foot.add_geom(
        name=f"{side}_foot_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(0, 0, sole_height),
        size=spec.FOOT_HALF,
        mass=spec.FOOT_MASS,
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


def add_motors(model) -> None:
    for side in spec.SIDES:
        for joint in spec.LEG_JOINTS:
            motor = model.add_actuator(
                name=f"{side}_{joint.part}_motor",
                target=f"{side}_{joint.part}_joint",
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                ctrlrange=joint.command_limits or joint.limits,
                ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                forcerange=(-spec.MOTOR_TORQUE_LIMIT, spec.MOTOR_TORQUE_LIMIT),
                forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            )
            motor.set_to_position(kp=spec.MOTOR_KP, kv=joint.kv)


def add_sensors(model) -> None:
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
    for side in spec.SIDES:
        model.add_sensor(name=f"{side}_foot_force", type=mujoco.mjtSensor.mjSENS_TOUCH,
                         objtype=mujoco.mjtObj.mjOBJ_SITE, objname=f"{side}_touch_site")


def build_biped() -> mujoco.MjSpec:
    model = mujoco.MjSpec()
    model.modelname = "Danny Biped"
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
        mass=spec.PELVIS_MASS,
        rgba=GREY,
    )
    # Torso provisional: sin brazos ni articulación, solo su masa y su altura
    pelvis.add_geom(
        name="torso_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(0, 0, spec.TORSO_OFFSET),
        size=spec.TORSO_HALF,
        mass=spec.TORSO_MASS,
        rgba=LIGHT_GREY,
    )
    pelvis.add_site(name="imu_site", size=SITE_SIZE, rgba=INVISIBLE)

    for side in spec.SIDES:
        add_leg(pelvis, side)

    # La cadera está dentro de la pelvis a propósito. MuJoCo excluye los cuerpos
    # unidos, pero no si el padre está fijo al mundo.
    for side in spec.SIDES:
        model.add_exclude(bodyname1="pelvis", bodyname2=f"{side}_thigh")

    add_motors(model)
    add_sensors(model)
    return model


def main() -> None:
    MODEL_PATH.write_text(build_biped().to_xml())
    print(f"Generado {MODEL_PATH.name}")


if __name__ == "__main__":
    main()
