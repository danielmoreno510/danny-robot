"""
Cinemática (auditoría 2): implementación analítica independiente, escrita aquí
desde cero con matrices de rotación, frente a los cuerpos que calcula MuJoCo.
"""

import math

import mujoco
import numpy as np
import pytest

from biped_control import HIP_Y, JOINT_PARTS, Biped, Pose, joint_targets, rotation_z
from conftest import load_model, record
from leg_control import leg_ik

THIGH, SHIN, HIP_DROP, SOLE = 0.16, 0.16, 0.05, 0.04
SEED = 2026


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_angle(relative):
    """Ángulo de una rotación casi nula. acos no resuelve por debajo de ~1e-6°."""
    return 0.5 * math.sqrt((relative[2, 1] - relative[1, 2]) ** 2
                           + (relative[0, 2] - relative[2, 0]) ** 2
                           + (relative[1, 0] - relative[0, 1]) ** 2)


def analytic_fk(q, side):
    """Pelvis en el origen y horizontal. Devuelve (posición de planta, rotación del pie)."""
    yaw, roll, pitch, knee, ankle_pitch, ankle_roll = q
    hip = np.array([0, HIP_Y if side == "left" else -HIP_Y, -HIP_DROP])
    thigh = rotation_z(yaw) @ rx(roll) @ ry(pitch)
    knee_pos = hip + thigh @ [0, 0, -THIGH]
    shin = thigh @ ry(knee)
    ankle = knee_pos + shin @ [0, 0, -SHIN]
    foot = shin @ ry(ankle_pitch) @ rx(ankle_roll)
    return ankle + foot @ [0, 0, -SOLE], foot


def set_leg(model, data, side, q, other=None):
    data.qpos[:] = 0
    data.qpos[3] = 1
    for part, angle in zip(JOINT_PARTS, q):
        data.qpos[model.joint(f"{side}_{part}_joint").qposadr[0]] = angle
    if other is not None:
        other_side = "right" if side == "left" else "left"
        for part, angle in zip(JOINT_PARTS, other):
            data.qpos[model.joint(f"{other_side}_{part}_joint").qposadr[0]] = angle
    mujoco.mj_kinematics(model, data)


def random_config(rng, model, side):
    q = []
    for part in JOINT_PARTS:
        low, high = model.joint(f"{side}_{part}_joint").range
        q.append(rng.uniform(low, high))
    return np.array(q)


@pytest.mark.parametrize("side", ["left", "right"])
def test_forward_kinematics_matches_mujoco(side):
    """Cada pierna por separado, en todo su rango, con la otra pierna en otra postura."""
    model = load_model()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(SEED)
    site = model.site(f"{side}_sole_site").id
    other = "right" if side == "left" else "left"
    position_error, angle_error = 0.0, 0.0
    for _ in range(2000):
        q = random_config(rng, model, side)
        set_leg(model, data, side, q, random_config(rng, model, other))
        position, rotation = analytic_fk(q, side)
        position_error = max(position_error, np.linalg.norm(position - data.site_xpos[site]))
        relative = rotation.T @ data.site_xmat[site].reshape(3, 3)
        angle_error = max(angle_error, rotation_angle(relative))
    record(f"cinematica.fk_error_pos_{side}", position_error, "< 1e-9", "m")
    record(f"cinematica.fk_error_ang_{side}", math.degrees(angle_error), "< 1e-6", "°")
    assert position_error < 1e-9
    assert math.degrees(angle_error) < 1e-6


@pytest.mark.parametrize("side", ["left", "right"])
def test_inverse_kinematics_round_trip(side):
    """FK(IK(p)) = p en todo el espacio alcanzable, con el pie plano."""
    rng = np.random.default_rng(SEED)
    worst, flat, tested, outside = 0.0, 0.0, 0, 0
    model = load_model()
    ranges = [model.joint(f"{side}_{p}_joint").range for p in JOINT_PARTS]
    while tested < 2000:
        target = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.15, 0.15), rng.uniform(-0.31, -0.12)])
        if np.linalg.norm(target) > 0.315 or np.linalg.norm(target) < 0.1:
            continue
        tested += 1
        q = leg_ik(target, THIGH, SHIN)
        outside += any(not low <= a <= high for a, (low, high) in zip(q, ranges))
        hip = np.array([0, HIP_Y if side == "left" else -HIP_Y, -HIP_DROP])
        sole, foot = analytic_fk(q, side)
        ankle = sole - foot @ [0, 0, -SOLE]
        worst = max(worst, np.linalg.norm(ankle - hip - target))
        flat = max(flat, math.degrees(math.hypot(foot[0, 2], foot[1, 2])))
    record(f"cinematica.ik_error_{side}", worst, "< 1e-9", "m")
    record(f"cinematica.ik_incl_pie_{side}", flat, "< 1e-6", "°")
    record(f"cinematica.ik_fuera_de_rango_{side}", outside / tested, "informativa: leg_ik no aplica rangos")
    assert worst < 1e-9
    assert flat < 1e-6


def test_biped_targets_place_feet_with_yaw_on_mujoco():
    model = load_model()
    data = mujoco.MjData(model)
    biped = Biped(model, data)
    rng = np.random.default_rng(SEED)
    worst, worst_yaw = 0.0, 0.0
    for _ in range(500):
        pose = Pose(
            np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.03, 0.03), rng.uniform(0.30, 0.36)]),
            np.array([rng.uniform(-0.08, 0.08), 0.07 + rng.uniform(-0.02, 0.02), rng.uniform(0, 0.03)]),
            np.array([rng.uniform(-0.08, 0.08), -0.07 + rng.uniform(-0.02, 0.02), rng.uniform(0, 0.03)]),
            rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5),
        )
        pose.pelvis[:2] += 0.0
        biped.set_pose(data, pose)
        for index, (sole, yaw) in enumerate(((pose.left_sole, pose.left_yaw), (pose.right_sole, pose.right_yaw))):
            site = biped.sole_ids[index]
            worst = max(worst, np.linalg.norm(data.site_xpos[site] - sole))
            rotation = data.site_xmat[site].reshape(3, 3)
            worst_yaw = max(worst_yaw, abs(math.atan2(rotation[1, 0], rotation[0, 0]) - yaw))
    record("cinematica.biped_error_pos", worst, "< 1e-6", "m")
    record("cinematica.biped_error_yaw", math.degrees(worst_yaw), "< 1e-4", "°")
    assert worst < 1e-6
    assert math.degrees(worst_yaw) < 1e-4


def test_analytic_jacobian_near_singularity():
    """Con la pierna casi recta, la IK sigue siendo estable y no da NaN."""
    results = []
    for reach in (0.30, 0.31, 0.318, 0.3199, 0.32, 0.325, 0.40):
        q = leg_ik(np.array([0.0, 0.0, -reach]), THIGH, SHIN)
        results.append(q)
        assert np.all(np.isfinite(q)), reach
    record("cinematica.rodilla_con_pierna_recta", math.degrees(results[4][3]), "0 sin NaN", "°")
    step = np.degrees(np.abs(np.diff([q[3] for q in results[:4]])).max())
    record("cinematica.salto_rodilla_cerca_singularidad", step, "informativa", "°")


def test_unreachable_target_is_not_detected_by_leg_ik():
    """Riesgo documentado: leg_ik recorta sin avisar; solo walk_biped lo comprueba."""
    target = np.array([0.0, 0.0, -0.40])
    q = leg_ik(target, THIGH, SHIN)
    sole, foot = analytic_fk(q, "left")
    ankle = sole - foot @ [0, 0, -SOLE]
    error = np.linalg.norm(ankle - np.array([0, HIP_Y, -HIP_DROP]) - target)
    record("cinematica.error_objetivo_inalcanzable", error, "informativa: 8 cm sin aviso", "m")
    assert error > 0.05


def test_joint_targets_do_not_change_with_side_swap_symmetry():
    """Simetría: reflejar la pose en y refleja roll y yaw, y conserva pitch y rodilla."""
    pose = Pose(np.array([0.02, 0.03, 0.33]), np.array([0.05, 0.08, 0.0]), np.array([-0.03, -0.06, 0.01]))
    mirror = Pose(np.array([0.02, -0.03, 0.33]), np.array([-0.03, 0.06, 0.01]), np.array([0.05, -0.08, 0.0]))
    a, b = joint_targets(pose), joint_targets(mirror)
    sign = np.array([-1, -1, 1, 1, 1, -1])
    assert np.allclose(a[:6], sign * b[6:], atol=1e-9)
