"""
Etapa 5.1: parámetros del CAD y script de Fusion, con una API falsa. Prueba la lógica
del script. Que funcione en Fusion solo lo demuestra skeleton_report.json.
"""

import importlib.util
import json
import shutil

import mujoco
import numpy as np
import pytest

import estimate_joint_loads
import export_cad_parameters as export
import robot_spec as spec
from conftest import ROOT
from fake_fusion import install

SCRIPT = export.CAD_DIR / "DannyMasterSkeleton.py"


def load_script(monkeypatch, folder, orientation="zup"):
    modules, app, design, messages = install(orientation)
    for name, module in modules.items():
        monkeypatch.setitem(__import__("sys").modules, name, module)
    copy = folder / SCRIPT.name
    shutil.copy(SCRIPT, copy)
    shutil.copy(export.PARAMETERS_FILE, folder / export.PARAMETERS_FILE.name)
    specification = importlib.util.spec_from_file_location("danny_fusion_script", copy)
    script = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(script)
    return script, app, design, messages


def model_points():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / spec.HUMANOID.file))
    data = mujoco.MjData(model)
    data.qpos[2] = spec.SOLE_BELOW_ANKLE + spec.LEG_LENGTH + spec.HIP_BELOW_PELVIS
    data.qpos[3] = 1.0
    mujoco.mj_kinematics(model, data)
    return lambda name: 1000 * data.xpos[model.body(name).id].copy()


def test_parameters_file_is_up_to_date():
    assert json.loads(export.PARAMETERS_FILE.read_text()) == json.loads(json.dumps(export.build()))


@pytest.mark.parametrize("sketch,point,body", [
    ("Sagittal_XZ", "pelvis", "pelvis"),
    ("Sagittal_XZ", "waist", "waist"),
    ("Sagittal_XZ", "neck_yaw", "neck"),
    ("Sagittal_XZ", "neck_pitch", "head"),
    ("Frontal_YZ", "left_hip_joint", "left_thigh"),
    ("Frontal_YZ", "right_hip_joint", "right_thigh"),
    ("Frontal_YZ", "left_knee_joint", "left_shin"),
    ("Frontal_YZ", "right_ankle_joint", "right_foot"),
    ("Frontal_YZ", "left_shoulder", "left_shoulder"),
    ("Frontal_YZ", "right_shoulder", "right_shoulder"),
    ("Frontal_YZ", "left_elbow", "left_forearm"),
    ("Frontal_YZ", "right_wrist", "right_hand"),
])
def test_joint_points_match_the_simulated_robot(sketch, point, body):
    points = next(s for s in export.build()["sketches"] if s["name"] == sketch)["points"]
    assert np.allclose(points[point], model_points()(body), atol=1e-3)


def test_robot_height_is_seventy_centimetres():
    points = export.build()["sketches"][0]["points"]
    assert abs(points["head_top"][2] - 700.0) < 1.0


def test_script_builds_the_skeleton_and_passes_its_report(monkeypatch, tmp_path):
    script, app, design, messages = load_script(monkeypatch, tmp_path)
    script.run(None)
    assert "Fallo" not in messages[0], messages[0]
    assert design.designType == "parametric"
    assert len(design.userParameters) == len(export.parameters())

    report_path = tmp_path / "skeleton_report.json"
    monkeypatch.setattr(export, "REPORT_FILE", report_path)
    assert export.check_report() == []

    skeleton = design.rootComponent.children[0]
    assert skeleton.name == "00_Master_Skeleton"
    assert [c.name for c in design.rootComponent.children] == list(export.COMPONENTS)
    named = {item.name for item in skeleton.constructionPoints.items}
    assert {"Sagittal_XZ_left_hip", "Frontal_YZ_right_shoulder"} <= named
    assert len(named) == len(skeleton.constructionPoints.items)


def test_every_dimension_is_driven_by_a_parameter(monkeypatch, tmp_path):
    script, app, design, _ = load_script(monkeypatch, tmp_path)
    script.run(None)
    for sketch in design.sketches():
        for dimension in sketch.dimensions:
            assert dimension.parameter.expression
            assert not any(character.isdigit() for character in dimension.parameter.expression)


def test_check_detects_a_wrong_report(monkeypatch, tmp_path):
    script, *_ = load_script(monkeypatch, tmp_path)
    script.run(None)
    report_path = tmp_path / "skeleton_report.json"
    report = json.loads(report_path.read_text())
    report["sketches"]["Sagittal_XZ"]["points"]["left_hip"][2] += 1.0
    report["self_test"]["moved_mm"] = 0.0
    report_path.write_text(json.dumps(report))
    monkeypatch.setattr(export, "REPORT_FILE", report_path)
    failures = export.check_report()
    assert any("left_hip" in failure for failure in failures)
    assert any("no mueve" in failure for failure in failures)


def test_script_stops_if_fusion_is_not_z_up(monkeypatch, tmp_path):
    script, _, design, messages = load_script(monkeypatch, tmp_path, orientation="yup")
    script.run(None)
    assert "Z arriba" in messages[0]
    assert not design.userParameters


def test_load_estimate_carries_the_whole_weight():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / spec.HUMANOID.file))
    data = mujoco.MjData(model)
    weight = float(model.body_subtreemass[1]) * -model.opt.gravity[2]
    for case in estimate_joint_loads.leg_cases(model, data).values():
        assert abs(case["fuerza_vertical"] - weight) < 0.01
