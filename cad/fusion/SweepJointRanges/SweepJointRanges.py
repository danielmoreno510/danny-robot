"""
Danny - Busca interferencias en el diseño activo por todo el rango de cada articulación.

Mueve cada articulación sola, desde su pose de partida, y analiza 9 ángulos de cada rango.
La pose de partida existe porque un brazo colgando está a menos de 1 mm de la pelvis: las
piernas y la cintura se prueban con los brazos abiertos, y parte del brazo con el brazo
separado del cuerpo. Después prueba unas poses combinadas. Deja la pose cero y escribe el
resultado en results/robot_interference.json.
"""

import json
import math
import os

import adsk.core
import adsk.fusion


RESULTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
    "results", "robot_interference.json")
STEPS = 9
MINIMUM_VOLUME_MM3 = 1.0
# Grados con el signo del lado izquierdo: el lado derecho invierte roll y yaw
ARMS_OPEN = {"shoulder_roll": 60}
# Pose de partida de cada grupo de articulaciones. Las que no aparecen usan ARMS_OPEN.
START_POSES = {
    "neck": {},
    "shoulder_pitch": {},
    "shoulder_roll": {"shoulder_pitch": -170},
    "shoulder_yaw": {"shoulder_roll": 45},
    "elbow": {},
    # Las dos piernas son un rango por lado: la contraria se abre para que los pies no se toquen
    "left_hip_roll": {"right_hip_roll": -20, "left_shoulder_roll": 60, "right_shoulder_roll": -60},
    "right_hip_roll": {"left_hip_roll": 20, "left_shoulder_roll": 60, "right_shoulder_roll": -60},
}
# Poses combinadas. Las articulaciones que no aparecen quedan a cero.
POSES = {
    "deep_squat": {**ARMS_OPEN, "hip_pitch": -100, "knee": 130, "ankle_pitch": -45},
    "knee_to_chest": {**ARMS_OPEN, "hip_pitch": -100, "knee": 130},
    "leg_back_knee_bent": {**ARMS_OPEN, "hip_pitch": 20, "knee": 130},
    "hip_out_and_forward": {**ARMS_OPEN, "hip_roll": 35, "hip_pitch": -100},
    "hip_turned_and_forward": {**ARMS_OPEN, "hip_yaw": 30, "hip_pitch": -100, "knee": 130},
    "arm_raised_to_side": {"shoulder_roll": 120},
    "wave": {"shoulder_roll": 120, "elbow": -130},
    "arm_forward_bent": {"shoulder_pitch": -90, "elbow": -130},
    "arm_up": {"shoulder_pitch": -170},
    "arm_back_bent": {"shoulder_pitch": 45, "elbow": -130},
}
BOTH_SIDES = ("deep_squat", "leg_back_knee_bent", "wave", "arm_forward_bent", "arm_up")
MIRRORED = ("hip_roll", "hip_yaw", "shoulder_roll")


def find_interference(design, entities):
    entities_input = design.createInterferenceInput(entities)
    entities_input.areCoincidentFacesIncluded = False
    found = []
    for result in design.analyzeInterference(entities_input):
        volume = result.interferenceBody.volume * 1000
        if volume >= MINIMUM_VOLUME_MM3:
            found.append({
                "one": result.entityOne.name,
                "two": result.entityTwo.name,
                "volume_mm3": round(volume, 2),
            })
    return found


def side_angles(prefixes, angles):
    named = {}
    for prefix in prefixes:
        for name, degrees in angles.items():
            named[f"{prefix}_{name}"] = -degrees if prefix == "right" and name in MIRRORED else degrees
    return named


def set_pose(root, named):
    """Pone los ángulos dentro de sus topes y devuelve las articulaciones movidas."""
    moved = []
    for name, degrees in named.items():
        joint = root.asBuiltJoints.itemByName(name)
        limits = joint.jointMotion.rotationLimits
        angle = math.radians(degrees)
        joint.jointMotion.rotationValue = max(limits.minimumValue, min(limits.maximumValue, angle))
        moved.append(joint)
    return moved


def start_pose(joint_name):
    side = joint_name.split("_")[0]
    prefixes = (side,) if side in ("left", "right") else ("left", "right")
    if joint_name in START_POSES:
        return dict(START_POSES[joint_name])
    for group, angles in START_POSES.items():
        if group in joint_name:
            return side_angles(prefixes, angles)
    return side_angles(("left", "right"), ARMS_OPEN)


def build(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    entities = adsk.core.ObjectCollection.create()
    for occurrence in root.occurrences:
        entities.add(occurrence)

    source_hash = design.attributes.itemByName("danny", "source_hash").value
    report = {"source_hash": source_hash, "zero_pose": find_interference(design, entities), "joints": {}}
    for joint in root.asBuiltJoints:
        moved = set_pose(root, start_pose(joint.name))
        motion = joint.jointMotion
        low, high = motion.rotationLimits.minimumValue, motion.rotationLimits.maximumValue
        report["joints"][joint.name] = []
        for step in range(STEPS):
            angle = low + (high - low) * step / (STEPS - 1)
            motion.rotationValue = angle
            adsk.doEvents()
            report["joints"][joint.name].append({
                "degrees": round(math.degrees(angle), 1),
                "interference": find_interference(design, entities),
            })
        motion.rotationValue = 0
        for other in moved:
            other.jointMotion.rotationValue = 0

    report["poses"] = {}
    for pose, angles in POSES.items():
        report["poses"][pose] = {}
        sides = [("left",), ("right",)] + ([("left", "right")] if pose in BOTH_SIDES else [])
        for prefixes in sides:
            # Los brazos abiertos de las poses de pierna van siempre en los dos lados
            named = side_angles(("left", "right"), ARMS_OPEN) if "shoulder_roll" in angles and "hip_pitch" in angles else {}
            named.update(side_angles(prefixes, angles))
            moved = set_pose(root, named)
            adsk.doEvents()
            report["poses"][pose]["+".join(prefixes)] = find_interference(design, entities)
            for joint in moved:
                joint.jointMotion.rotationValue = 0

    os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
    with open(RESULTS, "w", encoding="utf-8") as file:
        json.dump(report, file, indent=1, ensure_ascii=False)

    clashes = {}
    for name, steps in report["joints"].items():
        for step in steps:
            for found in step["interference"]:
                key = f"{found['one']} / {found['two']}"
                clashes.setdefault(key, {"joints": set(), "max_mm3": 0})
                clashes[key]["joints"].add(f"{name}@{step['degrees']:g}")
                clashes[key]["max_mm3"] = max(clashes[key]["max_mm3"], found["volume_mm3"])
    return {
        "zero_pose": report["zero_pose"],
        "poses": report["poses"],
        "clashes": {key: {"max_mm3": value["max_mm3"], "joints": sorted(value["joints"])}
                    for key, value in clashes.items()},
    }
