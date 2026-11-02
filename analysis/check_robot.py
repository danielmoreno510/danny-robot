"""
Danny - Comprueba el robot exportado de Fusion, etapas 2.2 a 2.4.

Compara cad/export/danny con los criterios de documentation/lower-body-design.md y
documentation/upper-body-design.md y documentation/panels-and-protection.md, usando results/robot_cad.json y
results/robot_interference.json. Escribe results/robot_check.json y sale con código 1 si
algo falla o si algún resultado no corresponde al BuildRobot.py actual.

Uso: .venv/bin/python analysis/check_robot.py
"""

import hashlib
import json
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL = ROOT / "cad" / "export" / "danny" / "danny.xml"
CAD = ROOT / "results" / "robot_cad.json"
INTERFERENCE = ROOT / "results" / "robot_interference.json"
OUTPUT = ROOT / "results" / "robot_check.json"
SCRIPT = ROOT / "cad" / "fusion" / "BuildRobot" / "BuildRobot.py"
EXPORT_HASH = MODEL.parent / "source_hash.txt"

MASS_TOLERANCE = 0.01
AXIS_TOLERANCE_DEG = 1.0
RANGE_TOLERANCE_DEG = 0.01
STEPS = 9
POSE_SAMPLES = 25
BODIES = 24
JOINTS = 23

# R4 y R10, de documentation/research-and-requirements.md
MAXIMUM_MASS_KG = 1.8
HEIGHT_MM = (380.0, 410.0)
MINIMUM_LEG_RATIO = 0.49
MAXIMUM_SHOULDER_RATIO = 0.34
MAXIMUM_PANEL_KG = 0.100
# Masa sin modelar, en kg (E): cables y tornillos, del desglose
UNMODELED_KG = {"cables": 0.040, "screws": 0.060}
# Apoyo en pose cero: pies de x = -30 a 55 mm y hasta y = ±56 mm
SUPPORT_X_MM = (-30.0, 55.0)
SUPPORT_Y_MM = 56.0
HIP_PITCH_JOINT = "left_hip_pitch"
SHOULDER_BODIES = ("shoulder_link", "upper_arm")

AXES = {"yaw": (0, 0, 1), "roll": (1, 0, 0), "pitch": (0, 1, 0), "knee": (0, 1, 0), "elbow": (0, 1, 0)}
RANGES = {
    "left_hip_yaw": (-15, 30),
    "left_hip_roll": (-15, 35),
    "left_hip_pitch": (-100, 20),
    "left_knee": (0, 130),
    "left_ankle_pitch": (-45, 40),
    "left_ankle_roll": (-20, 20),
    "right_hip_yaw": (-30, 15),
    "right_hip_roll": (-35, 15),
    "right_hip_pitch": (-100, 20),
    "right_knee": (0, 130),
    "right_ankle_pitch": (-45, 40),
    "right_ankle_roll": (-20, 20),
    "waist_yaw": (-60, 60),
    "left_shoulder_pitch": (-170, 45),
    "left_shoulder_roll": (-10, 120),
    "left_shoulder_yaw": (-80, 80),
    "left_elbow": (-130, 0),
    "right_shoulder_pitch": (-170, 45),
    "right_shoulder_roll": (-120, 10),
    "right_shoulder_yaw": (-80, 80),
    "right_elbow": (-130, 0),
    "neck_yaw": (-70, 70),
    "neck_pitch": (-30, 40),
}


def check_interference(report):
    failures = []
    if set(report["joints"]) != set(RANGES):
        failures.append(f"el barrido no cubre las {JOINTS} articulaciones")
    samples = sum(len(sides) for sides in report["poses"].values())
    if samples != POSE_SAMPLES:
        failures.append(f"{samples} poses en el barrido, no {POSE_SAMPLES}")
    for item in report["zero_pose"]:
        failures.append(f"pose cero: {item['one']} / {item['two']}")
    for pose, sides in report["poses"].items():
        for side, found in sides.items():
            failures += [f"{pose} {side}: {item['one']} / {item['two']}, {item['volume_mm3']} mm³" for item in found]
    for joint, steps in report["joints"].items():
        if len(steps) != STEPS:
            failures.append(f"{joint}: {len(steps)} ángulos, no {STEPS}")
        for step in steps:
            failures += [
                f"{joint}@{step['degrees']:g}: {item['one']} / {item['two']}, {item['volume_mm3']} mm³"
                for item in step["interference"]
            ]
    return failures


def check_freshness(cad, interference):
    current = hashlib.sha256(SCRIPT.read_bytes()).hexdigest()
    sources = {
        "robot_cad.json": cad.get("source_hash"),
        "robot_interference.json": interference.get("source_hash"),
        "danny.xml": EXPORT_HASH.read_text(encoding="utf-8") if EXPORT_HASH.exists() else None,
    }
    return [f"{name} no corresponde al script actual" for name, value in sources.items() if value != current]


def geom_vertices(model, data, geom):
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    return vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]


def measure_extents(model, data):
    """Devuelve altura y anchos en mm, medidos en las mallas en pose cero."""
    highest = 0.0
    widest = 0.0
    shoulders = 0.0
    for geom in range(model.ngeom):
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        vertices = geom_vertices(model, data, geom)
        half_width = float(np.abs(vertices[:, 1]).max())
        highest = max(highest, float(vertices[:, 2].max()))
        widest = max(widest, half_width)
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom])
        if any(name in body for name in SHOULDER_BODIES):
            shoulders = max(shoulders, half_width)
    return 1000 * highest, 2000 * widest, 2000 * shoulders


def main():
    cad = json.loads(CAD.read_text(encoding="utf-8"))
    interference = json.loads(INTERFERENCE.read_text(encoding="utf-8"))

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    failures = []
    if model.nbody - 1 != BODIES:
        failures.append(f"{model.nbody - 1} cuerpos, no {BODIES}")
    if model.njnt != JOINTS:
        failures.append(f"{model.njnt} articulaciones, no {JOINTS}")

    total = float(model.body_mass.sum())
    mass_error = abs(total - cad["total_mass_kg"]) / cad["total_mass_kg"]
    if mass_error > MASS_TOLERANCE:
        failures.append(f"la masa difiere un {100 * mass_error:.2f} %")

    worst_axis = 0.0
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        expected = next(axis for key, axis in AXES.items() if key in name)
        angle = np.degrees(np.arccos(np.clip(data.xaxis[joint] @ np.array(expected), -1, 1)))
        worst_axis = max(worst_axis, float(angle))
        if angle > AXIS_TOLERANCE_DEG:
            failures.append(f"{name}: eje a {angle:.2f}° del esperado")
        limits = np.degrees(model.jnt_range[joint])
        if np.abs(limits - np.array(RANGES[name])).max() > RANGE_TOLERANCE_DEG:
            failures.append(f"{name}: tope {limits.round(2).tolist()}, esperado {RANGES[name]}")

    estimated_mass = total + sum(UNMODELED_KG.values())
    if estimated_mass > MAXIMUM_MASS_KG:
        failures.append(f"R4: {estimated_mass:.3f} kg estimados, máximo {MAXIMUM_MASS_KG}")

    if cad["panel_mass_kg"] > MAXIMUM_PANEL_KG:
        failures.append(f"paneles: {1000 * cad['panel_mass_kg']:.0f} g")

    height, widest, shoulders = measure_extents(model, data)
    leg = 1000 * float(data.xanchor[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, HIP_PITCH_JOINT)][2])
    if not HEIGHT_MM[0] <= height <= HEIGHT_MM[1]:
        failures.append(f"R10: altura de {height:.1f} mm")
    if leg / height < MINIMUM_LEG_RATIO:
        failures.append(f"R10: pierna al {100 * leg / height:.1f} %")
    if shoulders / height > MAXIMUM_SHOULDER_RATIO:
        failures.append(f"R10: hombros al {100 * shoulders / height:.1f} %")

    center = 1000 * data.subtree_com[0]
    inside_x = SUPPORT_X_MM[0] < center[0] < SUPPORT_X_MM[1]
    if not inside_x or abs(center[1]) > SUPPORT_Y_MM:
        failures.append(f"centro de masas fuera del apoyo: {center.round(1).tolist()} mm")

    for pair, volume in cad["fit_overlaps_mm3"].items():
        failures.append(f"encaje: {pair}, {volume} mm³")
    failures += check_interference(interference)
    failures += check_freshness(cad, interference)

    result = {
        "bodies": int(model.nbody - 1),
        "joints": int(model.njnt),
        "modeled_mass_kg": total,
        "panel_mass_kg": cad["panel_mass_kg"],
        "unmodeled_mass_kg": UNMODELED_KG,
        "estimated_mass_kg": estimated_mass,
        "mass_margin_g": 1000 * (MAXIMUM_MASS_KG - estimated_mass),
        "mass_error_percent": 100 * mass_error,
        "worst_axis_error_deg": worst_axis,
        "height_mm": height,
        "leg_mm": leg,
        "leg_percent": 100 * leg / height,
        "shoulder_width_mm": shoulders,
        "shoulder_percent": 100 * shoulders / height,
        "widest_mm": widest,
        "center_of_mass_mm": center.round(1).tolist(),
        "failures": failures,
    }
    OUTPUT.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1, ensure_ascii=False))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
