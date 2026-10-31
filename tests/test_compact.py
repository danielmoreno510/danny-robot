"""
Fase 5: humanoide compacto. Comprueba que el modelo es coherente; las cifras de par son
estimaciones, y ninguna prueba demuestra que el servo real las aguante.
"""

import math

import mujoco
import numpy as np
import pytest

import compact_spec as compact
import compare_compact_sizes as sizes
import robot_spec as spec
from build_model import COMPACT_FILE
from conftest import ROOT
from robot_loads import fastest_walk, support_torques


@pytest.fixture(scope="module", params=sizes.HEIGHTS)
def design(request):
    return compact.Design(request.param)


def compile_model(design):
    model = compact.build(design).compile()
    return model, mujoco.MjData(model)


def test_generated_file_matches_the_specification():
    xml = (ROOT / "simulation" / "models" / COMPACT_FILE).read_text()
    assert xml == compact.build(compact.Design(0.40)).to_xml()


def test_height_mass_and_degrees_of_freedom(design):
    model, data = compile_model(design)
    compact.set_stand(model, data, 0.0)
    top = data.geom("head_geom").xpos[2] + design.head_radius
    assert abs(top - design.height) < 1e-6
    assert abs(model.body_subtreemass[1] - design.total_mass()) < 1e-6
    assert model.nu == 22 and model.nv == 28


def test_mass_shares_cover_the_whole_structure():
    design = compact.Design(0.40)
    servos = 22 * design.motor.mass
    other = sum(compact.ELECTRONICS_MASS.values()) + compact.BATTERY_MASS + compact.JETSON_MASS
    assert abs(design.total_mass() - servos - other - design.structure_mass) < 1e-9


def test_joints_keep_the_names_axes_and_limits_of_the_full_humanoid():
    model, _ = compile_model(compact.Design(0.40))
    full = mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / spec.HUMANOID.file))
    for joint in range(1, model.njnt):
        item = model.joint(joint)
        other = full.joint(item.name)
        assert np.allclose(item.axis, other.axis), item.name
        assert np.allclose(item.range, other.range), item.name


def test_support_carries_the_weight_on_one_foot(design):
    model, data = compile_model(design)
    compact.balance_on(model, data, math.radians(30), ("left",), 0.3)
    _, vertical = support_torques(model, data, ("left",))
    assert abs(vertical - model.body_subtreemass[1] * spec.GRAVITY) < 0.01
    com, sole = data.subtree_com[1], data.site("left_sole_site").xpos
    assert np.linalg.norm(com[:2] - sole[:2] - [compact.FOOT_CENTER, 0.0]) < 1e-5
    assert abs(data.site("right_sole_site").xpos[2] - sole[2]) > 0.005


def test_keyframe_holds_standing_with_the_servos_and_no_false_contact():
    model, data = compile_model(compact.Design(0.40))
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    pairs = [{model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name} for c in data.contact[:data.ncon]]
    assert not [pair for pair in pairs if "floor" not in pair]
    height = data.qpos[2]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert abs(height - data.qpos[2]) < 0.003
    assert data.body("pelvis").xmat[8] > 0.998


def test_motor_speed_limits_the_walk():
    slow = fastest_walk(4.7, 0.2)
    fast = fastest_walk(9.0, 0.2)
    assert 0 < slow["velocidad"] < fast["velocidad"] < 1.0
    assert fastest_walk(1000.0, 0.9)["velocidad"] == pytest.approx(1.4)
