"""Arquitectura (auditoría 7): reglas del proyecto y que nada se haya roto."""

import ast
import subprocess
import sys

import numpy as np

import biped_control
import build_model
from conftest import ROOT, load_model, record
from leg_control import leg_ik

SCRIPTS = ROOT / "simulation" / "scripts"
MODULES = {"leg_control", "biped_control", "robot_spec"}


def test_scripts_only_import_modules():
    """Un script de análisis nunca importa de otro."""
    offenders = []
    local = {p.stem for p in SCRIPTS.glob("*.py")}
    for path in SCRIPTS.glob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom) and node.module in local - MODULES:
                offenders.append(f"{path.stem} -> {node.module}")
    record("software.imports_entre_scripts", len(offenders), "= 0: " + ", ".join(offenders))
    assert not offenders


def test_geometry_constants_match_model():
    model = load_model()
    assert abs(model.body("left_shin").pos[2] + biped_control.THIGH_LENGTH) < 1e-9
    assert abs(model.body("left_foot").pos[2] + biped_control.SHIN_LENGTH) < 1e-9
    assert abs(model.body("left_thigh").pos[1] - biped_control.HIP_Y) < 1e-9
    assert abs(model.body("left_thigh").pos[2] + biped_control.HIP_BELOW_PELVIS) < 1e-9
    assert abs(model.site("left_sole_site").pos[2] + biped_control.SOLE_BELOW_ANKLE) < 1e-9
    foot = model.geom("left_foot_geom").id
    assert np.allclose(model.geom_size[foot][:2], biped_control.FOOT_HALF)


def test_versioned_model_matches_the_specification():
    """danny_biped.xml no se edita a mano: debe salir igual de robot_spec.py."""
    assert build_model.MODEL_PATH.read_text() == build_model.build_biped().to_xml()


def test_no_robot_dimensions_are_duplicated_in_scripts():
    """Las medidas del bípedo solo se definen en robot_spec.py."""
    pattern = ("THIGH_LENGTH = ", "SHIN_LENGTH = ", "HIP_Y = ", "SOLE_BELOW_ANKLE = ", "LEG_LENGTH = ")
    offenders = [p.name for p in (SCRIPTS / n for n in ("biped_control.py", "walk_biped.py", "balance_biped.py", "push_biped.py"))
                 if any(text in p.read_text() for text in pattern)]
    assert not offenders


def test_single_inverse_kinematics_implementation():
    hits = [p.name for p in SCRIPTS.glob("*.py") if "def leg_ik" in p.read_text()]
    record("software.implementaciones_ik", len(hits), "= 1")
    assert hits == ["leg_control.py"]
    assert np.all(np.isfinite(leg_ik(np.array([0.0, 0.0, -0.3]), 0.16, 0.16)))


def test_all_previous_experiments_still_pass():
    run = subprocess.run([sys.executable, str(SCRIPTS / "run_checks.py")], capture_output=True, text=True)
    line = [x for x in run.stdout.splitlines() if "comprobaciones" in x][-1]
    record("software.run_checks", 0 if run.returncode == 0 else 1, "0: " + line)
    assert run.returncode == 0, run.stdout
