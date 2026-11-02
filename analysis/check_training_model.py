"""
Danny - Comprueba danny_training.xml antes de entrenar.

Busca los fallos del modelo que un entrenamiento no avisa: articulaciones cerca del límite en
reposo, piezas sin colisión, choques falsos, masa, sensores y contacto de los pies. Escribe
results/training_model_check.json y sale con código 1 si algo falla.

Uso: .venv/bin/python analysis/check_training_model.py
"""

import json
import sys

import mujoco
import numpy as np

import danny_sim

MODEL = danny_sim.ROOT / "cad" / "export" / "danny" / "danny_training.xml"
OUTPUT = danny_sim.ROOT / "results" / "training_model_check.json"
SOFT_LIMIT_FACTOR = 0.9  # el de la configuración de training/
MINIMUM_REST_MARGIN_DEG = 3.0
MESH_OUTSIDE_LIMIT_MM = 12.0
MASS_RANGE_KG = (1.27, 1.33)
MAXIMUM_PENETRATION_MM = 1.0
MINIMUM_SOLE_CONTACTS = 8
JOINT_ORDER = [
    *[f"{side}_{joint}" for side in ("left", "right")
      for joint in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")],
    "waist_yaw",
    *[f"{side}_{joint}" for side in ("left", "right")
      for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")],
    "neck_yaw", "neck_pitch",
]


def rest_margins(model, stand):
    """Grados que separan `stand` de cada límite blando."""
    margins = {}
    for joint in range(1, model.njnt):
        low, high = model.jnt_range[joint]
        middle, half = (low + high) / 2, (high - low) / 2 * SOFT_LIMIT_FACTOR
        angle = stand[model.jnt_qposadr[joint]]
        margins[model.joint(joint).name] = float(np.degrees(min(angle - (middle - half), middle + half - angle)))
    return margins


def uncovered_mesh(model, data):
    """Mayor distancia, en mm, de un vértice de cada cuerpo fuera de sus cajas, en pose cero."""
    worst = {}
    for body in range(1, model.nbody):
        boxes = [g for g in range(model.ngeom)
                 if model.geom_bodyid[g] == body and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
        meshes = [g for g in range(model.ngeom)
                  if model.geom_bodyid[g] == body and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        if not meshes:
            continue
        if not boxes:
            worst[model.body(body).name] = float("inf")
            continue
        distance = np.inf
        points = []
        for geom in meshes:
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            points.append(data.geom_xpos[geom] + model.mesh_vert[start:start + count] @ data.geom_xmat[geom].reshape(3, 3).T)
        points = np.concatenate(points)
        outside = []
        for geom in boxes:
            local = (points - data.geom_xpos[geom]) @ data.geom_xmat[geom].reshape(3, 3)
            outside.append(np.linalg.norm(np.maximum(np.abs(local) - model.geom_size[geom], 0), axis=1))
        distance = np.min(outside, axis=0).max()
        worst[model.body(body).name] = float(1000 * distance)
    return worst


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    names = [model.joint(joint).name for joint in range(1, model.njnt)]
    actuators = [model.joint(model.actuator_trnid[index, 0]).name for index in range(model.nu)]
    failures = []
    if names != JOINT_ORDER or actuators != JOINT_ORDER:
        failures.append("el orden de articulaciones o actuadores no es el del handbook")

    mass = float(model.body_mass.sum())
    if not MASS_RANGE_KG[0] <= mass <= MASS_RANGE_KG[1]:
        failures.append(f"masa de {mass:.3f} kg")

    imu = model.site("imu").id
    if model.body(model.site_bodyid[imu]).name != "torso_1":
        failures.append("la IMU no va en el torso")

    mujoco.mj_forward(model, data)
    uncovered = uncovered_mesh(model, data)
    failures += [f"{body}: {value:.1f} mm de pieza sin colisión" for body, value in uncovered.items()
                 if value > MESH_OUTSIDE_LIMIT_MM]

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    stand = data.qpos.copy()
    margins = rest_margins(model, stand)
    failures += [f"{joint}: {margin:.1f}° del límite blando en reposo" for joint, margin in margins.items()
                 if margin < MINIMUM_REST_MARGIN_DEG]

    soles = {model.geom("left_sole").id, model.geom("right_sole").id}
    contacts = [(contact.geom1, contact.geom2, contact.dist) for contact in data.contact[:data.ncon]]
    sole_contacts = sum(1 for one, two, _ in contacts if soles & {one, two})
    false_contacts = [(model.geom(one).name, model.geom(two).name) for one, two, _ in contacts if not soles & {one, two}]
    deepest = max((-1000 * dist for *_, dist in contacts), default=0.0)
    if sole_contacts < MINIMUM_SOLE_CONTACTS:
        failures.append(f"solo {sole_contacts} contactos de suela en reposo")
    if false_contacts:
        failures.append(f"choques falsos en reposo: {false_contacts}")
    if deepest > MAXIMUM_PENETRATION_MM:
        failures.append(f"la suela entra {deepest:.2f} mm en el suelo")

    left = stand[[model.jnt_qposadr[model.joint(f"left_{joint}").id] for joint in ("hip_pitch", "knee", "ankle_pitch")]]
    right = stand[[model.jnt_qposadr[model.joint(f"right_{joint}").id] for joint in ("hip_pitch", "knee", "ankle_pitch")]]
    if np.abs(left - right).max() > 1e-3:
        failures.append("la postura de reposo no es simétrica")

    result = {
        "mass_kg": mass,
        "link_contacts": int(model.npair),
        "smallest_rest_margin_deg": min(margins.values()),
        "tightest_joint": min(margins, key=margins.get),
        "sole_contacts_at_rest": sole_contacts,
        "deepest_contact_mm": deepest,
        "worst_uncovered_mesh_mm": max(uncovered.values()),
        "worst_uncovered_body": max(uncovered, key=uncovered.get),
        "failures": failures,
    }
    OUTPUT.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1, ensure_ascii=False))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
