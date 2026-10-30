"""
Etapas 3.5.3 a 3.5.6: humanoide con torso, cintura, brazos, manos, cuello y cabeza.
Mismos límites que el bípedo. Mide estados reales; la cintura también se mueve.
"""

import math

import mujoco
import numpy as np
import pytest

import push_biped
import robot_spec as spec
import walk_biped
from biped_control import Pose, center_com_over, pelvis_height
from build_model import build_robot
from conftest import ROOT, RecordingBiped, foot_corners, polygon_margin, record
from helpers import E11, E13, run_walk

KNEE = 56
WIDTH = 0.05
WAIST_FREQUENCY = 0.25  # Hz
HAND_LOAD = 0.3  # kg, en el presupuesto de masa
ARM_SWING_DEG = 20


def humanoid():
    return mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / spec.HUMANOID.file))


def test_mass_height_and_com():
    model = humanoid()
    data = mujoco.MjData(model)
    data.qpos[2] = 0.4
    data.qpos[3] = 1.0
    mujoco.mj_forward(model, data)
    mass = model.body_subtreemass[model.body("pelvis").id]
    sole = data.geom_xpos[model.geom("left_foot_geom").id][2] - spec.FOOT_HALF[2]
    top = data.geom_xpos[model.geom("head_geom").id][2] + spec.HEAD_RADIUS
    com = data.subtree_com[model.body("pelvis").id]
    record("humanoide.masa", mass, "11,0 ± 0,01", "kg")
    record("humanoide.altura", top - sole, "0,70 ± 0,005", "m")
    record("humanoide.cdm_altura", com[2] - sole, "0,36 a 0,42", "m")
    assert abs(mass - 11.0) < 0.01
    assert abs(top - sole - 0.70) < 0.005
    assert 0.36 < com[2] - sole < 0.42
    assert abs(com[1]) < 1e-9


def test_inertias_valid_and_joint_conventions():
    model = humanoid()
    for body in range(1, model.nbody):
        a, b, c = np.sort(model.body_inertia[body])
        assert a > 0 and a + b >= c * (1 - 1e-9), model.body(body).name
    for joint in spec.WAIST_JOINTS:
        item = model.joint(f"{joint.part}_joint")
        assert np.allclose(item.axis, joint.axis)
        assert np.allclose(item.range, joint.limits)


def test_no_false_self_contact_in_working_postures():
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    floor = model.geom("floor").id
    for knee in (20, 40, 56, 80, 120):
        half = math.radians(knee) / 2
        pose = Pose(np.array([0, 0, 0.09 + 0.32 * math.cos(half)]), np.array([0, 0.07, 0.0]), np.array([0, -0.07, 0.0]))
        biped.set_pose(data, pose)
        pairs = [(c.geom1, c.geom2) for c in data.contact[: data.ncon] if floor not in (c.geom1, c.geom2)]
        assert not pairs, knee


def torso_thigh_contact(hip, knee):
    model = humanoid()
    data = mujoco.MjData(model)
    data.qpos[2] = 0.8
    data.qpos[3] = 1.0
    for side in spec.SIDES:
        data.qpos[model.joint(f"{side}_hip_pitch_joint").qposadr[0]] = hip
        data.qpos[model.joint(f"{side}_knee_joint").qposadr[0]] = knee
    data.qpos[model.joint("waist_pitch_joint").qposadr[0]] = spec.WAIST_JOINTS[1].limits[1]
    mujoco.mj_forward(model, data)
    return "torso_geom" in {model.geom(int(g)).name for c in data.contact[: data.ncon] for g in (c.geom1, c.geom2)}


def test_torso_collides_only_beyond_the_hip_limit():
    """Dentro de los rangos, torso y muslo no se alcanzan; pasado el límite, se detecta."""
    assert not torso_thigh_contact(-1.5708, 0.0)
    assert torso_thigh_contact(-2.5, 1.0)


@pytest.mark.parametrize("name,gait,speed,hold", [
    ("e11", E11, 1.0, 1.5),
    ("e12", E11, 1.27, 0.30),
    ("e13", E13, 1.27, 0.25),
])
def test_walking_with_waist_held(name, gait, speed, hold, monkeypatch):
    biped, results, saturation, collided, model, error = run_walk(monkeypatch, gait, speed, hold, model=humanoid())
    assert not error, error
    failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
    a = biped.arrays()
    record(f"humanoide.{name}.fallos", len(failures), "= 0")
    record(f"humanoide.{name}.duracion", a["t"][-1], "informativa", "s")
    record(f"humanoide.{name}.incl_max", a["tilt"].max(), "< 5", "°")
    record(f"humanoide.{name}.pie_fuera", 1000 * max(np.nanmax(r.foot_slip) for r in results), "< 5", "mm")
    record(f"humanoide.{name}.error_movimiento", max(r.tracking_error_deg for r in results), "< 1", "°")
    record(f"humanoide.{name}.par_max", np.abs(a["torque"]).max(), "< 20", "N·m")
    assert not failures, failures[:3]


@pytest.mark.parametrize("side", ["left", "right"])
def test_stand_on_one_foot(side):
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    swing = "right" if side == "left" else "left"
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    pose = center_com_over(biped, stand.with_foot(swing, stand.sole(swing) + [0, 0, 0.03]), side, 0.012)
    biped.set_pose(data, pose)
    start = round(1.0 / model.opt.timestep)
    for _ in range(round(6.0 / model.opt.timestep)):
        biped.step(pose, swing)
    a = biped.arrays()
    index = 0 if side == "left" else 1
    margin = min(polygon_margin(c[:2], foot_corners(s[index], 0)) for c, s in zip(a["com"][start:], a["soles"][start:]))
    record(f"humanoide.pie_{side}.margen_cdm", 1000 * margin, "> 15", "mm")
    record(f"humanoide.pie_{side}.incl_max", a["tilt"][start:].max(), "< 2", "°")
    assert margin > 0.015
    assert a["tilt"][start:].max() < 2.0


def waist_motion(t, yaw_deg, pitch_deg):
    phase = 2 * math.pi * WAIST_FREQUENCY * t
    return np.radians([yaw_deg * math.sin(phase), pitch_deg * (1 - math.cos(phase)) / 2])


@pytest.mark.parametrize("yaw,pitch", [(30, 0), (0, 30), (30, 30)])
def test_moving_waist_while_standing(yaw, pitch):
    """Mover el torso de pie: los pies no se mueven y el CdM sigue dentro."""
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    stand = center_com_over(biped, stand, "left", inset=WIDTH)
    biped.set_pose(data, stand)
    dt = model.opt.timestep
    for _ in range(500):
        biped.step(stand, None)
    start = len(biped.log["t"])
    waist = [model.joint(f"{j.part}_joint").qposadr[0] for j in spec.WAIST_JOINTS]
    tracking = 0.0
    for i in range(round(8.0 / dt)):
        biped.held_target[:2] = waist_motion(i * dt, yaw, pitch)
        biped.step(stand, None)
        tracking = max(tracking, np.abs(data.qpos[waist] - biped.held_target[:2]).max())
    a = biped.arrays()
    drift = np.linalg.norm(a["soles"][start:, :, :2] - a["soles"][start, :, :2], axis=2).max()
    polygon = np.vstack([foot_corners(a["soles"][start, 0], 0), foot_corners(a["soles"][start, 1], 0)])
    margin = min(polygon_margin(c[:2], polygon) for c in a["com"][start:])
    prefix = f"humanoide.cintura_yaw{yaw}_pitch{pitch}"
    record(f"{prefix}.deriva_pies", 1000 * drift, "< 5", "mm")
    record(f"{prefix}.margen_cdm", 1000 * margin, "> 15", "mm")
    record(f"{prefix}.par_max", np.abs(a["torque"][start:]).max(), "< 20", "N·m")
    record(f"{prefix}.error_cintura", math.degrees(tracking), "< 1", "°")
    assert drift < 0.005
    assert margin > 0.015
    assert math.degrees(tracking) < 1.0


def walk_moving_waist(monkeypatch, yaw, pitch):
    class Swinging(RecordingBiped):
        def step(self, pose, swing):
            self.held_target[:2] = waist_motion(self.data.time, yaw, pitch)
            super().step(pose, swing)

    _, results, saturation, collided, model, error = run_walk(
        monkeypatch, E13, 1.27, 0.25, biped_class=Swinging, model=humanoid())
    if error:
        return error
    margins = [r.com_margin for r in results if r.com_margin is not None and not np.isnan(r.com_margin)]
    record(f"humanoide.marcha_cintura_yaw{yaw}_pitch{pitch}.margen_cdm", 1000 * min(margins), "informativa", "mm")
    return walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)


def test_walking_while_turning_the_waist(monkeypatch):
    """La misma marcha E13 con el torso girando ±10°: debe cumplir los criterios."""
    failures = walk_moving_waist(monkeypatch, 10, 0)
    record("humanoide.marcha_cintura_yaw10.fallos", len(failures), "= 0")
    assert not failures, failures[:3]


@pytest.mark.parametrize("yaw,pitch", [(20, 0), (30, 0), (0, 10), (0, 20), (0, 30)])
def test_measure_walking_with_larger_waist_motion(yaw, pitch, monkeypatch):
    """Sin exigir: el plan no conoce la cintura, así que mide dónde deja de valer."""
    failures = walk_moving_waist(monkeypatch, yaw, pitch)
    value = failures if isinstance(failures, str) else len(failures)
    record(f"humanoide.marcha_cintura_yaw{yaw}_pitch{pitch}.fallos", value, "informativa")


@pytest.mark.parametrize("name", list(push_biped.DIRECTIONS))
def test_push_limits(name):
    model = humanoid()
    data = mujoco.MjData(model)
    direction = push_biped.DIRECTIONS[name]
    force = push_biped.REQUIRED_FORCE["humanoid"][name]
    trial = push_biped.run_push(model, data, direction, force)
    record(f"humanoide.empujon_{name}", force, "se recupera", "N")
    assert trial.recovered


def arm_pose(model, data, side="left", **angles):
    """Robot lejos del suelo, con el brazo 'side' en los ángulos dados."""
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 1.0
    data.qpos[3] = 1.0
    for part, angle in angles.items():
        data.qpos[model.joint(f"{side}_{part}_joint").qposadr[0]] = angle
    mujoco.mj_forward(model, data)


def test_hand_is_a_rigid_block_with_the_expected_inertia():
    model = humanoid()
    a, b, c = (2 * half for half in spec.HAND_HALF)
    mass = spec.HUMANOID.hand_mass
    expected = sorted(mass / 12 * np.array([b**2 + c**2, a**2 + c**2, a**2 + b**2]))
    for side in spec.SIDES:
        body = model.body(f"{side}_hand")
        assert abs(body.mass[0] - mass) < 1e-9
        assert np.allclose(sorted(body.inertia), expected, rtol=0.01)
        assert model.geom(f"{side}_hand_geom").contype[0] == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_fingertip_reach(side):
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data, side, shoulder_pitch=-1.5708)
    tip = data.site_xpos[model.site(f"{side}_fingertip_site").id] - data.xpos[model.body(f"{side}_shoulder").id]
    record(f"humanoide.mano_{side}.alcance", np.linalg.norm(tip), "0,30 ± 0,001", "m")
    assert abs(np.linalg.norm(tip) - 0.30) < 0.001


def test_hand_reaches_the_torso_and_collides():
    """Doblar el brazo lleva la mano al torso, y ese contacto se detecta."""
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data, shoulder_pitch=0.4, shoulder_roll=0.7, shoulder_yaw=-0.54, elbow=2.3)
    pairs = [{model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name} for c in data.contact[: data.ncon]]
    assert {"left_hand_geom", "torso_geom"} in pairs, pairs


def test_arm_joints_follow_the_approved_ranges():
    model = humanoid()
    assert model.nu == 30
    for joint in spec.ARM_JOINTS:
        for side in spec.SIDES:
            item = model.joint(f"{side}_{joint.part}_joint")
            assert np.allclose(item.range, joint.limits)
            assert np.allclose(item.axis, spec.mirrored_axis(joint, side))


@pytest.mark.parametrize("side,sign", [("left", 1), ("right", -1)])
def test_positive_angles_mean_the_same_on_both_sides(side, sign):
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data, side, shoulder_roll=1.5708)
    wrist = data.site_xpos[model.site(f"{side}_wrist_site").id] - data.xpos[model.body(f"{side}_shoulder").id]
    assert np.allclose(wrist, [0, sign * 0.25, 0], atol=1e-5)
    arm_pose(model, data, side, shoulder_pitch=-1.5708)
    wrist = data.site_xpos[model.site(f"{side}_wrist_site").id] - data.xpos[model.body(f"{side}_shoulder").id]
    assert np.allclose(wrist, [0.25, 0, 0], atol=1e-5)


def test_seven_axes_keep_the_hand_still_while_the_elbow_moves():
    """Rango 6 y un eje de sobra en una postura de trabajo."""
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data, shoulder_pitch=-0.8, shoulder_roll=0.4, shoulder_yaw=0.3, elbow=1.2,
             wrist_yaw=0.3, wrist_pitch=0.2, wrist_roll=0.2)
    position, rotation = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, position, rotation, model.body("left_hand").id)
    dofs = [model.joint(f"left_{j.part}_joint").dofadr[0] for j in spec.ARM_JOINTS]
    jacobian = np.vstack([position, rotation])[:, dofs]
    record("humanoide.brazo.rango_jacobiano", int(np.linalg.matrix_rank(jacobian)), "= 6")
    assert np.linalg.matrix_rank(jacobian) == 6


@pytest.mark.parametrize("load", [0.0, HAND_LOAD])
def test_holding_the_arm_out_is_within_catalog_torque(load):
    """Brazo horizontal, sin carga y con carga en la punta de la mano."""
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data, shoulder_pitch=-1.5708)
    data.qacc[:] = 0.0
    mujoco.mj_inverse(model, data)
    tip = data.site_xpos[model.site("left_fingertip_site").id]
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacobian, None, tip, model.body("left_hand").id)
    torque = data.qfrc_inverse + jacobian[2] * load * spec.GRAVITY
    # Pico de catálogo: XM540, XM430 y STS3215 (documentation/phase-3.5-mass-budget.md)
    for part, catalog in (("shoulder_pitch", 7.3), ("elbow", 4.1), ("wrist_pitch", 1.9)):
        value = abs(torque[model.joint(f"left_{part}_joint").dofadr[0]])
        record(f"humanoide.brazo.par_{part}_carga_{load:g}", value, f"< {catalog}", "N·m")
        assert value < catalog


def test_arm_sweep_finds_only_real_collisions():
    """Posturas al azar dentro de los rangos: hay choques, y todos son del brazo con el cuerpo."""
    model = humanoid()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    body = {"torso_geom", "pelvis_geom", "waist_geom", "left_thigh_geom"}
    away = model.joint("right_shoulder_roll_joint").qposadr[0]  # el otro brazo, apartado
    hits, trials = 0, 2000
    for _ in range(trials):
        arm_pose(model, data, **{j.part: rng.uniform(*j.limits) for j in spec.ARM_JOINTS})
        data.qpos[away] = 1.5708
        mujoco.mj_forward(model, data)
        pairs = [{model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name} for c in data.contact[: data.ncon]]
        for pair in pairs:
            assert any(name.startswith("left_") for name in pair - body), pair
        hits += bool(pairs)
    record("humanoide.brazo.posturas_con_choque", 100 * hits / trials, "informativa", "%")
    assert 0 < hits < trials


def test_arms_clear_the_body_across_the_whole_waist_range():
    """Postura fija de los brazos, de pie, con la cintura en sus extremos."""
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    floor = model.geom("floor").id
    yaw_limits, pitch_limits = (j.limits for j in spec.WAIST_JOINTS)
    for yaw in np.linspace(*yaw_limits, 19):
        for pitch in np.linspace(*pitch_limits, 13):
            biped.held_target[:2] = yaw, pitch
            biped.set_pose(data, stand)
            pairs = [(model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name)
                     for c in data.contact[: data.ncon] if floor not in (c.geom1, c.geom2)]
            assert not pairs, (yaw, pitch, pairs)


def test_head_position_and_inertia():
    model = humanoid()
    data = mujoco.MjData(model)
    data.qpos[2] = 0.4
    data.qpos[3] = 1.0
    mujoco.mj_forward(model, data)
    sole = data.geom_xpos[model.geom("left_foot_geom").id][2] - spec.FOOT_HALF[2]
    center = data.geom_xpos[model.geom("head_geom").id]
    record("humanoide.cabeza.altura_centro", center[2] - sole, "0,655 ± 0,005", "m")
    assert abs(center[2] - sole - 0.655) < 0.005
    expected = 0.4 * spec.HUMANOID.head_mass * spec.HEAD_RADIUS**2  # esfera maciza
    assert np.allclose(model.body("head").inertia, expected, rtol=0.01)


def test_neck_joints_and_looking_down():
    model = humanoid()
    data = mujoco.MjData(model)
    for joint in spec.NECK_JOINTS:
        item = model.joint(f"{joint.part}_joint")
        assert np.allclose(item.axis, joint.axis)
        assert np.allclose(item.range, joint.limits)
    arm_pose(model, data)
    data.qpos[model.joint("neck_pitch_joint").qposadr[0]] = spec.NECK_JOINTS[1].limits[1]
    mujoco.mj_forward(model, data)
    head = data.site_xpos[model.site("head_site").id] - data.xpos[model.body("head").id]
    assert head[0] > 0.02  # la cabeza va hacia delante y abajo


def test_holding_the_head_is_within_catalog_torque():
    model = humanoid()
    data = mujoco.MjData(model)
    arm_pose(model, data)
    data.qpos[model.joint("neck_pitch_joint").qposadr[0]] = spec.NECK_JOINTS[1].limits[1]
    mujoco.mj_forward(model, data)
    data.qacc[:] = 0.0
    mujoco.mj_inverse(model, data)
    torque = abs(data.qfrc_inverse[model.joint("neck_pitch_joint").dofadr[0]])
    record("humanoide.cuello.par_45", torque, "< 1,9", "N·m")
    assert torque < 1.9


def test_neck_range_is_free_and_the_hand_can_touch_the_head():
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    floor = model.geom("floor").id
    for yaw in np.linspace(*spec.NECK_JOINTS[0].limits, 11):
        for pitch in np.linspace(*spec.NECK_JOINTS[1].limits, 7):
            biped.held_target[-2:] = yaw, pitch
            biped.set_pose(data, stand)
            assert all(floor in (c.geom1, c.geom2) for c in data.contact[: data.ncon]), (yaw, pitch)
    rng = np.random.default_rng(1)
    touches = 0
    for _ in range(4000):
        arm_pose(model, data, **{j.part: rng.uniform(*j.limits) for j in spec.ARM_JOINTS})
        pairs = [{model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name} for c in data.contact[: data.ncon]]
        touches += any("head_geom" in pair and any("left" in name for name in pair) for pair in pairs)
    record("humanoide.cabeza.posturas_brazo_tocando", touches, "> 0")
    assert touches > 0


def test_walking_while_moving_the_neck(monkeypatch):
    """E13 mirando alrededor: ±80° de yaw y hasta 30° de pitch."""

    class Looking(RecordingBiped):
        def step(self, pose, swing):
            motion = waist_motion(self.data.time, 80, 30)
            self.held_target[-2:] = motion
            super().step(pose, swing)

    _, results, saturation, collided, model, error = run_walk(
        monkeypatch, E13, 1.27, 0.25, biped_class=Looking, model=humanoid())
    assert not error, error
    failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
    record("humanoide.marcha_cuello.fallos", len(failures), "= 0")
    assert not failures, failures[:3]


def swing_arms(biped, t, frequency):
    """Balanceo de hombros de ±20° en contrafase: validación sencilla, no una marcha natural."""
    angle = math.radians(ARM_SWING_DEG) * math.sin(2 * math.pi * frequency * t)
    parts = list(spec.HELD_PARTS)
    biped.held_target[parts.index("left_shoulder_pitch")] = angle
    biped.held_target[parts.index("right_shoulder_pitch")] = -angle
    return angle


def test_arm_swing_standing_is_within_catalog_limits():
    model = humanoid()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    stand = center_com_over(biped, stand, "left", inset=WIDTH)
    biped.set_pose(data, stand)
    for _ in range(500):
        biped.step(stand, None)
    motor = model.actuator("left_shoulder_pitch_motor").id
    dof = model.joint("left_shoulder_pitch_joint").dofadr[0]
    start, torque, speed = data.time, 0.0, 0.0
    for _ in range(round(4.0 / model.opt.timestep)):
        swing_arms(biped, data.time - start, 1.5)
        biped.step(stand, None)
        if data.time - start > 1.0:
            torque = max(torque, abs(data.actuator_force[motor]))
            speed = max(speed, abs(data.qvel[dof]))
    record("humanoide.balanceo_brazos.par", torque, "< 7,3", "N·m")
    record("humanoide.balanceo_brazos.velocidad", speed, "< 5,55", "rad/s")
    assert torque < 7.3
    assert speed < 5.55


def test_walking_with_arm_swing(monkeypatch):
    class Swinging(RecordingBiped):
        def step(self, pose, swing):
            swing_arms(self, self.data.time, 1.0)
            super().step(pose, swing)

    _, results, saturation, collided, model, error = run_walk(
        monkeypatch, E13, 1.27, 0.25, biped_class=Swinging, model=humanoid())
    assert not error, error
    failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
    record("humanoide.marcha_balanceo_brazos.fallos", len(failures), "= 0")
    assert not failures, failures[:3]


def test_generated_humanoid_matches_specification():
    path = ROOT / "simulation" / "models" / spec.HUMANOID.file
    assert path.read_text() == build_robot(spec.HUMANOID).to_xml()
