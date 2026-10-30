"""
Auditoría de preparación para la fase 4: comprueba el humanoide articulación por
articulación y su física básica. Resultados en documentation/phase-4-readiness-audit.md.
"""

import math

import mujoco
import numpy as np

import robot_spec as spec
from biped_control import Biped, Pose, center_com_over, pelvis_height
from conftest import ROOT, record

PATH = ROOT / "simulation" / "models" / spec.HUMANOID.file
FORCE_FACTOR = 2.0  # veces el par del motor, para empujar contra el tope
MAX_LIMIT_OVERSHOOT_DEG = 3.0


def humanoid():
    return mujoco.MjModel.from_xml_path(str(PATH))


def fixed_base():
    model = mujoco.MjSpec.from_file(str(PATH))
    model.delete(model.joint("pelvis_free"))
    return model.compile()


def test_every_joint_is_limited_and_has_motor_and_sensors():
    model = humanoid()
    names = [model.joint(i).name.removesuffix("_joint") for i in range(1, model.njnt)]
    record("preparacion.gdl", len(names), "= 31")
    assert len(names) == 31 == model.nu
    assert len(set(names)) == 31
    for i, name in enumerate(names, start=1):
        assert model.jnt_limited[i]
        assert model.jnt_range[i, 0] < model.jnt_range[i, 1]
        for kind, suffix in ((mujoco.mjtObj.mjOBJ_ACTUATOR, "_motor"), (mujoco.mjtObj.mjOBJ_SENSOR, "_pos"),
                             (mujoco.mjtObj.mjOBJ_SENSOR, "_torque")):
            assert mujoco.mj_name2id(model, kind, name + suffix) >= 0, name + suffix


def test_right_side_mirrors_the_left():
    model = humanoid()
    for i in range(1, model.njnt):
        name = model.joint(i).name
        if name.startswith("left_"):
            other = model.joint(name.replace("left_", "right_", 1)).id
            assert np.allclose(model.jnt_range[i], model.jnt_range[other])
            x, y, z = model.jnt_axis[i]
            if not any(part in name for part in spec.JOINT_PARTS):  # brazos: ejes reflejados
                assert np.allclose(model.jnt_axis[other], [-x, y, -z])


def test_all_bodies_have_mass_and_valid_inertia():
    model = humanoid()
    for body in range(1, model.nbody):
        a, b, c = np.sort(model.body_inertia[body])
        assert model.body_mass[body] > 0, model.body(body).name
        assert a > 0 and a + b >= c * (1 - 1e-9), model.body(body).name


def test_all_geoms_collide_and_exclusions_are_the_declared_ones():
    model = humanoid()
    assert all(model.geom_contype[g] and model.geom_conaffinity[g] for g in range(model.ngeom))
    pairs = {frozenset((model.body(int(s >> 16)).name, model.body(int(s & 0xFFFF)).name)) for s in model.exclude_signature}
    assert pairs == {frozenset(p) for p in (("pelvis", "left_thigh"), ("pelvis", "right_thigh"), ("pelvis", "torso"),
                                            ("pelvis", "waist_roll_link"), ("waist", "torso"))}


def test_free_fall_matches_gravity():
    model = humanoid()
    data = mujoco.MjData(model)
    data.qpos[2] = 5.0
    data.qpos[3] = 1.0
    mujoco.mj_forward(model, data)
    start = data.subtree_com[1][2]
    steps = 100
    for _ in range(steps):
        mujoco.mj_step(model, data)
    # Euler semi-implícito: tras n pasos, z = -g * dt² * n * (n - 1) / 2
    expected = -spec.GRAVITY * model.opt.timestep**2 * steps * (steps - 1) / 2
    assert abs(data.subtree_com[1][2] - start - expected) < 1e-5


def test_feet_carry_the_whole_weight():
    model = humanoid()
    data = mujoco.MjData(model)
    biped = Biped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(56)]), np.array([0.0, 0.05, 0.0]), np.array([0.0, -0.05, 0.0]))
    stand = center_com_over(biped, stand, "left", inset=0.05)
    biped.set_pose(data, stand)
    for _ in range(1500):
        biped.step(stand, None)
    measured = sum(data.sensor(f"{side}_foot_force").data[0] for side in spec.SIDES)
    weight = model.body_subtreemass[1] * spec.GRAVITY
    record("preparacion.error_peso_pies", 100 * abs(measured / weight - 1), "< 1", "%")
    assert abs(measured / weight - 1) < 0.01


def test_joint_limits_hold_against_twice_the_motor_torque():
    model = fixed_base()
    data = mujoco.MjData(model)
    worst = 0.0
    for joint in range(model.njnt):
        name = model.joint(joint).name.removesuffix("_joint")
        force = FORCE_FACTOR * model.actuator(f"{name}_motor").forcerange[1]
        for index, sign in ((1, 1.0), (0, -1.0)):
            mujoco.mj_resetData(model, data)
            qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
            data.qpos[qpos] = model.jnt_range[joint, index]
            for _ in range(500):
                data.qfrc_applied[dof] = sign * force
                mujoco.mj_step(model, data)
                worst = max(worst, sign * (data.qpos[qpos] - model.jnt_range[joint, index]))
    record("preparacion.sobrepaso_tope", math.degrees(worst), f"< {MAX_LIMIT_OVERSHOOT_DEG}", "°")
    assert math.degrees(worst) < MAX_LIMIT_OVERSHOOT_DEG


def test_motors_never_exceed_their_torque_limit():
    model = fixed_base()
    data = mujoco.MjData(model)
    for _ in range(300):
        data.ctrl[:] = model.actuator_ctrlrange[:, 1]
        mujoco.mj_step(model, data)
        assert np.all(np.abs(data.actuator_force) <= model.actuator_forcerange[:, 1] + 1e-9)
