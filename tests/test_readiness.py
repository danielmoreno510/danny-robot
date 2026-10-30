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
    model.delete(model.key("stand"))
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
            data.ctrl[model.actuator(f"{name}_motor").id] = model.jnt_range[joint, index]  # no se opone
            for _ in range(500):
                data.qfrc_applied[dof] = sign * force
                mujoco.mj_step(model, data)
                worst = max(worst, sign * (data.qpos[qpos] - model.jnt_range[joint, index]))
    record("preparacion.sobrepaso_tope", math.degrees(worst), f"< {MAX_LIMIT_OVERSHOOT_DEG}", "°")
    assert math.degrees(worst) < MAX_LIMIT_OVERSHOOT_DEG


def test_policy_sensors_exist():
    model = humanoid()
    for name in ("pelvis_gyro", "pelvis_accel", "pelvis_quat"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) >= 0
    for i in range(1, model.njnt):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, model.joint(i).name[:-6] + "_vel") >= 0


def test_accelerometer_reads_gravity_when_still():
    model = humanoid()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    for _ in range(1000):
        data.ctrl[:] = model.key("stand").ctrl
        mujoco.mj_step(model, data)
    reading = data.sensor("pelvis_accel").data
    record("preparacion.acelerometro_en_reposo", np.linalg.norm(reading), "9,81 ± 0,2", "m/s²")
    assert abs(np.linalg.norm(reading) - spec.GRAVITY) < 0.2
    assert np.linalg.norm(data.sensor("pelvis_gyro").data) < 0.05


def test_stand_keyframe_is_a_stable_start():
    """Sin controlador: solo los motores sujetando la postura guardada."""
    model = humanoid()
    data = mujoco.MjData(model)
    key = model.key("stand")
    mujoco.mj_resetDataKeyframe(model, data, key.id)
    mujoco.mj_forward(model, data)
    floor = model.geom("floor").id
    assert all(floor in (c.geom1, c.geom2) for c in data.contact[: data.ncon])
    worst_tilt = 0.0
    for _ in range(round(2.0 / model.opt.timestep)):
        data.ctrl[:] = key.ctrl
        mujoco.mj_step(model, data)
        worst_tilt = max(worst_tilt, math.degrees(2 * math.asin(min(1.0, np.linalg.norm(data.qpos[4:6])))))
        assert all(floor in (c.geom1, c.geom2) for c in data.contact[: data.ncon])
    drop = key.qpos[2] - data.qpos[2]
    record("preparacion.reposo.incl_max", worst_tilt, "< 5", "°")
    record("preparacion.reposo.bajada_pelvis", 1000 * drop, "< 20", "mm")
    assert worst_tilt < 5.0
    assert drop < 0.02


