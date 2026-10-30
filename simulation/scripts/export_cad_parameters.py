"""
Danny - Parámetros del esqueleto de CAD (etapa 5.1).

robot_spec.py sigue siendo la única fuente de dimensiones: este script escribe en
cad/fusion/DannyMasterSkeleton/skeleton_parameters.json lo que necesita el script de
Fusion, y comprueba el informe que ese script deja al ejecutarse en Fusion.

Ejes REP-103 con el robot de pie: x adelante, y a la izquierda, z arriba. El origen
está en el suelo, entre los pies. Todas las articulaciones a 0: piernas y brazos rectos.

Uso:
  python simulation/scripts/export_cad_parameters.py           # escribe los parámetros
  python simulation/scripts/export_cad_parameters.py --check   # valida skeleton_report.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import robot_spec as spec


ROOT = Path(__file__).resolve().parents[2]
CAD_DIR = ROOT / "cad" / "fusion" / "DannyMasterSkeleton"
PARAMETERS_FILE = CAD_DIR / "skeleton_parameters.json"
REPORT_FILE = CAD_DIR / "skeleton_report.json"
TOLERANCE_MM = 0.01

COMPONENTS = {
    "00_Master_Skeleton": {},
    "01_Pelvis": {},
    "02_Left_Leg": {"Left_Hip": {}, "Left_Thigh": {}, "Left_Knee": {}, "Left_Shin": {}, "Left_Ankle": {}, "Left_Foot": {}},
    "03_Right_Leg": {"Right_Hip": {}, "Right_Thigh": {}, "Right_Knee": {}, "Right_Shin": {}, "Right_Ankle": {}, "Right_Foot": {}},
    "04_Waist": {},
    "05_Torso": {},
    "06_Left_Arm": {"Left_Shoulder": {}, "Left_Upper_Arm": {}, "Left_Elbow": {}, "Left_Forearm": {}, "Left_Wrist": {}},
    "07_Right_Arm": {"Right_Shoulder": {}, "Right_Upper_Arm": {}, "Right_Elbow": {}, "Right_Forearm": {}, "Right_Wrist": {}},
    "08_Hands": {},
    "09_Neck": {},
    "10_Head": {},
    "11_Electronics": {},
    "12_Battery": {},
    "13_Exterior_Panels": {},
}


def parameters() -> dict[str, tuple[float, str]]:
    """Nombre: (valor en mm, de dónde sale)."""
    meters = {
        "sole_to_ankle": (spec.SOLE_BELOW_ANKLE, "suelo a tobillo"),
        "shin_length": (spec.SHIN_LENGTH, "tobillo a rodilla"),
        "thigh_length": (spec.THIGH_LENGTH, "rodilla a cadera"),
        "hip_below_pelvis": (spec.HIP_BELOW_PELVIS, "cadera a centro de la pelvis"),
        "hip_half_spacing": (spec.HIP_Y, "cada cadera, desde el plano medio"),
        "waist_above_pelvis": (spec.WAIST_ABOVE_PELVIS, "centro de la pelvis a cintura"),
        "shoulder_above_waist": (spec.SHOULDER_ABOVE_WAIST, "cintura a hombros"),
        "shoulder_half_spacing": (spec.SHOULDER_Y, "cada hombro, desde el plano medio"),
        "upper_arm_length": (spec.UPPER_ARM_LENGTH, "hombro a codo"),
        "forearm_length": (spec.FOREARM_LENGTH, "codo a muñeca"),
        "hand_length": (2 * spec.HAND_HALF[2], "muñeca a punta de la mano"),
        "neck_base_above_waist": (spec.NECK_BASE_ABOVE_WAIST, "cintura a cuello yaw"),
        "neck_pitch_above_base": (spec.NECK_PITCH_ABOVE_BASE, "cuello yaw a cuello pitch"),
        "head_above_neck_pitch": (spec.HEAD_ABOVE_NECK_PITCH, "cuello pitch a centro de la cabeza"),
        "head_radius": (spec.HEAD_RADIUS, "radio de la cabeza"),
        "foot_half_length": (spec.FOOT_HALF[0], "tobillo a punta del pie"),
    }
    return {name: (round(1000 * value, 3), comment) for name, (value, comment) in meters.items()}


def chain(start: str, *links: tuple[str, str, tuple[float, float, float]]) -> list[dict]:
    """Segmentos desde 'start': (punto nuevo, expresión de su longitud, dirección)."""
    segments = []
    for name, expression, direction in links:
        segments.append({"from": start, "to": name, "expression": expression, "direction": direction})
        start = name
    return segments


UP, DOWN, FORWARD = (0, 0, 1), (0, 0, -1), (1, 0, 0)
LEFT, RIGHT = (0, 1, 0), (0, -1, 0)
PELVIS_HEIGHT = "sole_to_ankle + shin_length + thigh_length + hip_below_pelvis"


def sagittal_sketch() -> dict:
    """Plano XZ, con y = 0: la columna de articulaciones del lado izquierdo y el cuerpo."""
    return {"name": "Sagittal_XZ", "plane": "xZ", "segments": [
        *chain("origin",
               ("left_ankle", "sole_to_ankle", UP),
               ("left_knee", "shin_length", UP),
               ("left_hip", "thigh_length", UP),
               ("pelvis", "hip_below_pelvis", UP),
               ("waist", "waist_above_pelvis", UP),
               ("neck_yaw", "neck_base_above_waist", UP),
               ("neck_pitch", "neck_pitch_above_base", UP),
               ("head_center", "head_above_neck_pitch", UP),
               ("head_top", "head_radius", UP)),
        *chain("left_ankle", ("toe", "foot_half_length", FORWARD)),
    ]}


def frontal_sketch() -> dict:
    """Plano YZ, con x = 0: separación de caderas y hombros, y los dos brazos."""
    segments = chain("origin", ("hip_center", "sole_to_ankle + shin_length + thigh_length", UP),
                     ("pelvis_center", "hip_below_pelvis", UP),
                     ("waist_center", "waist_above_pelvis", UP),
                     ("shoulder_center", "shoulder_above_waist", UP))
    for side, direction in (("left", LEFT), ("right", RIGHT)):
        segments += chain("hip_center",
                          (f"{side}_hip_joint", "hip_half_spacing", direction),
                          (f"{side}_knee_joint", "thigh_length", DOWN),
                          (f"{side}_ankle_joint", "shin_length", DOWN))
        segments += chain("shoulder_center",
                          (f"{side}_shoulder", "shoulder_half_spacing", direction),
                          (f"{side}_elbow", "upper_arm_length", DOWN),
                          (f"{side}_wrist", "forearm_length", DOWN),
                          (f"{side}_fingertip", "hand_length", DOWN))
    return {"name": "Frontal_YZ", "plane": "yZ", "segments": segments}


def expected_points(sketch: dict, values: dict[str, float]) -> dict[str, list[float]]:
    """Coordenadas [mm] de cada punto del boceto, recorriendo sus segmentos."""
    points = {"origin": [0.0, 0.0, 0.0]}
    for segment in sketch["segments"]:
        length = eval(segment["expression"], {"__builtins__": {}}, values)  # noqa: S307
        start = points[segment["from"]]
        points[segment["to"]] = [round(s + length * d, 3) for s, d in zip(start, segment["direction"])]
    return points


def build() -> dict:
    values = parameters()
    numbers = {name: value for name, (value, _) in values.items()}
    sketches = [sagittal_sketch(), frontal_sketch()]
    for sketch in sketches:
        sketch["points"] = expected_points(sketch, numbers)
    return {
        "parameters": {name: {"value_mm": value, "comment": comment} for name, (value, comment) in values.items()},
        "components": COMPONENTS,
        "skeleton_component": "00_Master_Skeleton",
        "sketches": sketches,
        "self_test": {"parameter": "thigh_length", "delta_mm": 10.0, "point": "left_hip", "sketch": "Sagittal_XZ"},
        "joints": joint_table(),
    }


def joint_table() -> list[dict]:
    """Ejes y topes de referencia: los del lado izquierdo, y cuello y cintura."""
    rows = []
    for group, prefix in ((spec.LEG_JOINTS, "left_"), (spec.WAIST_JOINTS, ""), (spec.ARM_JOINTS, "left_"), (spec.NECK_JOINTS, "")):
        for joint in group:
            low, high = spec.HUMANOID.limits(joint)
            rows.append({"name": f"{prefix}{joint.part}", "axis": joint.axis,
                         "limits_deg": [round(math.degrees(low), 1), round(math.degrees(high), 1)]})
    return rows


def check_report() -> list[str]:
    """Compara lo que Fusion dice haber construido con lo pedido. Devuelve los fallos."""
    if not REPORT_FILE.exists():
        return [f"falta {REPORT_FILE.name}: ejecuta el script en Fusion"]
    wanted = json.loads(PARAMETERS_FILE.read_text())
    report = json.loads(REPORT_FILE.read_text())
    failures = []
    for sketch in wanted["sketches"]:
        built = report["sketches"].get(sketch["name"])
        if built is None:
            failures.append(f"{sketch['name']}: no está en el informe")
            continue
        if not built["fully_constrained"]:
            failures.append(f"{sketch['name']}: no está totalmente restringido")
        for name, point in sketch["points"].items():
            got = built["points"].get(name)
            if got is None or max(abs(a - b) for a, b in zip(got, point)) > TOLERANCE_MM:
                failures.append(f"{sketch['name']}.{name}: {got} frente a {point}")
    test = report.get("self_test", {})
    if abs(test.get("moved_mm", math.inf) - wanted["self_test"]["delta_mm"]) > TOLERANCE_MM:
        failures.append(f"el parámetro no mueve el esqueleto: {test}")
    if test.get("restored") is not True:
        failures.append("el parámetro de prueba no se restauró")
    missing = set(wanted["parameters"]) - set(report.get("parameters", {}))
    if missing:
        failures.append(f"faltan parámetros: {sorted(missing)}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="valida skeleton_report.json")
    args = parser.parse_args()
    if args.check:
        failures = check_report()
        for failure in failures:
            print(f"  FAIL: {failure}")
        print("FAIL" if failures else "PASS: Fusion construyó el esqueleto pedido")
        return 1 if failures else 0
    CAD_DIR.mkdir(parents=True, exist_ok=True)
    PARAMETERS_FILE.write_text(json.dumps(build(), indent=1, ensure_ascii=False) + "\n")
    print(f"Escrito {PARAMETERS_FILE.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
