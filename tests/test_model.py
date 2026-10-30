"""Validación física del modelo danny.xml (auditoría 1)."""

import mujoco
import numpy as np

from biped_control import Biped, Pose
from conftest import load_model, record

TOTAL_MASS = 8.0
HEIGHT = 0.70


def forward(model):
    data = mujoco.MjData(model)
    data.qpos[2] = 0.5
    mujoco.mj_forward(model, data)
    return data


def test_total_mass(model):
    mass = model.body_subtreemass[model.body("pelvis").id]
    record("modelo.masa_total", mass, "8,0 ± 0,01", "kg")
    assert abs(mass - TOTAL_MASS) < 0.01


def test_body_masses_positive_and_symmetric(model):
    for body in range(1, model.nbody):
        assert model.body_mass[body] > 0, model.body(body).name
    for part in ("thigh", "shin", "foot"):
        left, right = model.body(f"left_{part}").mass[0], model.body(f"right_{part}").mass[0]
        assert abs(left - right) < 1e-9, part


def test_height_and_proportions(model):
    data = forward(model)
    torso = model.geom("torso_geom").id
    foot = model.geom("left_foot_geom").id
    top = data.geom_xpos[torso][2] + model.geom_size[torso][2]
    sole = data.geom_xpos[foot][2] - model.geom_size[foot][2]
    record("modelo.altura", top - sole, "0,70 ± 0,005", "m")
    assert abs(top - sole - HEIGHT) < 0.005


def test_center_of_mass_matches_manual_sum(model):
    """CdM de MuJoCo contra una suma manual masa-posición cuerpo a cuerpo."""
    data = forward(model)
    masses = model.body_mass[1:]
    manual = (masses[:, None] * data.xipos[1:]).sum(0) / masses.sum()
    reported = data.subtree_com[model.body("pelvis").id]
    error = np.linalg.norm(manual - reported)
    record("modelo.error_cdm_manual", error, "< 1e-9", "m")
    record("modelo.cdm_lateral_recto", reported[1], "|y| < 1e-9 (simetría)", "m")
    assert error < 1e-9
    assert abs(reported[1]) < 1e-9


def test_inertias_physically_valid(model):
    """Inercias principales positivas y con la desigualdad triangular."""
    worst = np.inf
    for body in range(1, model.nbody):
        a, b, c = np.sort(model.body_inertia[body])
        assert a > 0, model.body(body).name
        worst = min(worst, (a + b - c) / c)
        assert a + b >= c * (1 - 1e-9), model.body(body).name
    record("modelo.margen_triangular_inercias", worst, "> 0")


def test_capsule_inertia_matches_analytic(model):
    """Inercia del muslo contra la fórmula de una cápsula maciza."""
    geom = model.geom("left_thigh_geom").id
    radius, half = model.geom_size[geom][:2]
    mass = model.body("left_thigh").mass[0]  # un solo geom en el cuerpo
    density = mass / (np.pi * radius**2 * (2 * half + 4 / 3 * radius))
    cylinder = density * np.pi * radius**2 * 2 * half
    sphere = density * 4 / 3 * np.pi * radius**3
    axial = cylinder * radius**2 / 2 + sphere * 2 * radius**2 / 5
    transverse = (cylinder * (half**2 / 3 + radius**2 / 4)
                  + sphere * (2 * radius**2 / 5 + half**2 + 3 * half * radius / 4))
    measured = np.sort(model.body_inertia[model.body("left_thigh").id])  # un solo geom en el cuerpo
    expected = np.sort([axial, transverse, transverse])
    error = float(np.max(np.abs(measured - expected) / expected))
    record("modelo.error_inercia_capsula", error, "< 0,02 relativo")
    assert error < 0.02


def test_joint_axes_and_ranges_match_convention(model):
    expected = {
        "hip_yaw": ((0, 0, 1), -40, 40), "hip_roll": ((1, 0, 0), -30, 30),
        "hip_pitch": ((0, 1, 0), -90, 90), "knee": ((0, 1, 0), 0, 130),
        "ankle_pitch": ((0, 1, 0), -65, 45), "ankle_roll": ((1, 0, 0), -30, 30),
    }
    for side in ("left", "right"):
        for part, (axis, low, high) in expected.items():
            joint = model.joint(f"{side}_{part}_joint")
            assert np.allclose(joint.axis, axis), joint.name
            assert np.allclose(np.degrees(joint.range), (low, high), atol=0.01), joint.name


def test_gravity_and_timestep(model):
    assert np.allclose(model.opt.gravity, (0, 0, -9.81))
    record("modelo.timestep", model.opt.timestep, "0,002 (500 Hz)", "s")
    assert model.opt.timestep == 0.002


def test_whole_body_collides(model):
    colliders = [i for i in range(model.ngeom) if model.geom_contype[i] and model.geom_conaffinity[i]]
    record("modelo.geoms_con_colision", len(colliders), "todos", "de " + str(model.ngeom))
    assert len(colliders) == model.ngeom


def test_no_self_collision_standing_or_in_the_walk_range(model):
    """Ninguna postura de trabajo debe nacer con piezas solapadas."""
    data = mujoco.MjData(model)
    biped = Biped(model, data)
    for knee in (0, 20, 56, 80, 120):  # pies bajo las caderas: 0° es la pierna recta
        half = np.radians(knee) / 2
        height = 0.09 + 0.32 * np.cos(half)
        pose = Pose(np.array([0.0, 0.0, height]), np.array([0.0, 0.07, 0.0]), np.array([0.0, -0.07, 0.0]))
        biped.set_pose(data, pose)
        pairs = [(int(c.geom1), int(c.geom2)) for c in data.contact[: data.ncon]
                 if model.geom("floor").id not in (c.geom1, c.geom2)]
        assert not pairs, (knee, [(model.geom(a).name, model.geom(b).name) for a, b in pairs])


def test_crossed_legs_are_detected(model):
    """Con la pierna derecha cruzada sobre la izquierda hay contacto entre piernas."""
    data = mujoco.MjData(model)
    data.qpos[2] = 0.8
    data.qpos[3] = 1.0
    data.qpos[model.joint("right_hip_roll_joint").qposadr[0]] = np.radians(30)
    data.qpos[model.joint("left_hip_roll_joint").qposadr[0]] = np.radians(-30)
    mujoco.mj_forward(model, data)
    names = {(model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name) for c in data.contact[: data.ncon]}
    record("modelo.contactos_piernas_cruzadas", len(names), "> 0")
    assert names


def test_contact_parameters(model):
    floor = model.geom("floor").id
    foot = model.geom("left_foot_geom").id
    record("modelo.friccion_suelo", model.geom_friction[floor][0], "informativa, sin medir en real")
    record("modelo.friccion_pie", model.geom_friction[foot][0], "informativa, sin medir en real")
    record("modelo.solref_timeconst", model.geom_solref[foot][0], "informativa, por defecto")
    assert model.geom_friction[floor][0] > 0


def test_motors_are_mirrored_and_bounded(model):
    for side in ("left", "right"):
        for i, name in enumerate(("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")):
            actuator = model.actuator(f"{side}_{name}_motor")
            assert actuator.forcelimited[0], actuator.name
            assert np.allclose(actuator.forcerange, (-20, 20)), actuator.name
            assert actuator.gainprm[0] == 100


def test_model_does_not_declare_real_motor_effects():
    """Registra lo que falta: fricción seca, backlash y dinámica de motor."""
    model = load_model()
    record("modelo.friccion_seca_articular", float(model.dof_frictionloss.max()), "0 = sin modelar", "N·m")
    record("modelo.dinamica_actuador", int(model.actuator_dyntype.max()), "0 = instantáneo, sin modelar")
    assert model.dof_frictionloss.max() == 0  # documenta el estado actual
