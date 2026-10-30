"""Equilibrio (auditoría 4), con estados medidos, CdM y polígono de apoyo reales."""

import mujoco
import numpy as np
import pytest

from biped_control import Pose, center_com_over, joint_targets, pelvis_height
from conftest import RecordingBiped, foot_corners, load_model, polygon_margin, record
from leg_control import interpolate

KNEE = 56
WIDTH = 0.05
LIFT = 0.03
HOLD = 5.0
INSET = 0.012

MAX_TILT = 2.0
MAX_DRIFT = 0.005
MIN_COM_MARGIN = 0.015
MIN_COP_MARGIN = 0.005
FORCE_TOLERANCE = 0.05


def setup():
    model = load_model()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    stand = Pose(np.array([0.0, 0.0, pelvis_height(KNEE)]), np.array([0.0, WIDTH, 0.0]), np.array([0.0, -WIDTH, 0.0]))
    stand = center_com_over(biped, stand, "left", inset=WIDTH)
    biped.set_pose(data, stand)
    return model, data, biped, stand


def move(biped, start, end, swing, time=1.5):
    dt = biped.model.opt.timestep
    for i in range(round((time + 0.5) / dt)):
        pose = Pose.from_vector(interpolate(start.vector(), end.vector(), i * dt, "smooth", time))
        biped.step(pose, swing)


def segment(biped, start):
    """Datos registrados a partir del paso 'start'."""
    a = biped.arrays()
    return {k: v[start:] for k, v in a.items()}, a


def report(prefix, log, biped, polygon_of, expected):
    com_margin = min(polygon_margin(c[:2], polygon_of(s)) for c, s in zip(log["com"], log["soles"]))
    cops = log["cop"][~np.isnan(log["cop"]).any(1)]
    cop_margin = min(polygon_margin(c, polygon_of(log["soles"][0])) for c in cops)
    force = log["left_force"] + log["right_force"]
    tilt = log["tilt"].max()
    drift = np.linalg.norm(log["soles"][:, :, :2] - log["soles"][0, :, :2], axis=2).max()
    record(f"{prefix}.incl_max", tilt, f"< {MAX_TILT}", "°")
    record(f"{prefix}.margen_cdm", 1000 * com_margin, f"> {1000 * MIN_COM_MARGIN:g}", "mm")
    record(f"{prefix}.margen_cop", 1000 * cop_margin, f"> {1000 * MIN_COP_MARGIN:g}", "mm")
    record(f"{prefix}.deriva_pies", 1000 * drift, f"< {1000 * MAX_DRIFT:g}", "mm")
    record(f"{prefix}.fuerza_media", force.mean(), f"{expected:.1f} ± 5 %", "N")
    assert tilt < MAX_TILT
    assert com_margin > MIN_COM_MARGIN
    assert cop_margin > MIN_COP_MARGIN
    assert drift < MAX_DRIFT
    assert abs(force.mean() - expected) < FORCE_TOLERANCE * expected


def test_stand_on_both_feet():
    model, data, biped, stand = setup()
    weight = 8.0 * 9.81
    for _ in range(500):
        biped.step(stand, None)
    start = len(biped.log["t"])
    for _ in range(round(HOLD / model.opt.timestep)):
        biped.step(stand, None)
    log, _ = segment(biped, start)

    def polygon(soles):
        return np.vstack([foot_corners(soles[0], 0), foot_corners(soles[1], 0)])

    report("equilibrio.dos_pies", log, biped, polygon, weight)
    share = log["left_force"].mean() / (log["left_force"] + log["right_force"]).mean()
    record("equilibrio.dos_pies.reparto_izq", share, "0,50 ± 0,02")
    assert abs(share - 0.5) < 0.02


@pytest.mark.parametrize("side", ["left", "right"])
def test_stand_on_one_foot(side):
    model, data, biped, stand = setup()
    swing = "right" if side == "left" else "left"
    for _ in range(500):
        biped.step(stand, None)
    lifted = center_com_over(biped, stand.with_foot(swing, stand.sole(swing) + [0, 0, LIFT]), side, INSET)
    shifted = lifted.with_foot(swing, stand.sole(swing))
    move(biped, stand, shifted, None)
    move(biped, shifted, lifted, swing)
    start = len(biped.log["t"])
    for _ in range(round(HOLD / model.opt.timestep)):
        biped.step(lifted, swing)
    log, _ = segment(biped, start)
    index = 0 if side == "left" else 1

    def polygon(soles):
        return foot_corners(soles[index], 0)

    report(f"equilibrio.pie_{side}", log, biped, polygon, 8.0 * 9.81)
    other = log["right_force" if side == "left" else "left_force"].max()
    clearance = log["soles"][:, 1 - index, 2].min()
    record(f"equilibrio.pie_{side}.pie_libre", 1000 * clearance, "> 25", "mm")
    assert other == 0.0
    assert clearance > 0.025


@pytest.mark.parametrize("swing", ["left", "right"])
def test_weight_transfer_is_smooth(swing):
    """Fuerza de cada pie durante la transferencia: sin saltos ni pérdida de contacto."""
    model, data, biped, stand = setup()
    stance = "right" if swing == "left" else "left"
    for _ in range(500):
        biped.step(stand, None)
    shifted = center_com_over(biped, stand, stance, INSET)
    start = len(biped.log["t"])
    move(biped, stand, shifted, None)
    log, _ = segment(biped, start)
    forces = log[f"{stance}_force"]
    step = np.abs(np.diff(forces)).max()
    final = forces[-200:].mean() / (8.0 * 9.81)
    tilt = log["tilt"].max()
    record(f"equilibrio.transferencia_{swing}.salto_fuerza", step, "< 10 por paso", "N")
    record(f"equilibrio.transferencia_{swing}.carga_final", final, "> 0,8")
    record(f"equilibrio.transferencia_{swing}.incl_max", tilt, f"< {MAX_TILT}", "°")
    assert step < 10.0
    assert final > 0.8
    assert tilt < MAX_TILT


def test_torso_orientation_is_stable_in_stance():
    model, data, biped, stand = setup()
    for _ in range(round(5.0 / model.opt.timestep)):
        biped.step(stand, None)
    tilt = np.array(biped.log["tilt"])[500:]
    record("equilibrio.torso_incl_std", tilt.std(), "< 0,05", "°")
    assert tilt.std() < 0.05


def test_joint_targets_match_measured_pose_in_stance():
    model, data, biped, stand = setup()
    for _ in range(round(2.0 / model.opt.timestep)):
        biped.step(stand, None)
    error = np.degrees(np.abs(joint_targets(stand) - data.qpos[biped.joint_qpos]).max())
    record("equilibrio.error_articular_reposo", error, "< 0,5", "°")
    assert error < 0.5
