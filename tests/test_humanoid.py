"""
Etapa 3.5.3: humanoide con torso y cintura, con lastres en lugar de brazos y cabeza.
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
    top = data.geom_xpos[model.geom("head_ballast").id][2] + spec.HEAD_BALLAST_RADIUS
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
        biped.held_target = waist_motion(i * dt, yaw, pitch)
        biped.step(stand, None)
        tracking = max(tracking, np.abs(data.qpos[waist] - biped.held_target).max())
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
            self.held_target = waist_motion(self.data.time, yaw, pitch)
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


def test_generated_humanoid_matches_specification():
    path = ROOT / "simulation" / "models" / spec.HUMANOID.file
    assert path.read_text() == build_robot(spec.HUMANOID).to_xml()
