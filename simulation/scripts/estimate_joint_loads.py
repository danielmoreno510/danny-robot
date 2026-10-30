"""
Danny - Par y velocidad que debe dar cada articulación (etapa 5.1).

Dos estimaciones para elegir motores, con el modelo actual:
  - Cuasiestática: posturas quietas sobre uno o dos pies. Se reparte el peso entre
    los pies con la solución de mínima norma y se resuelve el equilibrio. Sale del
    modelo, sin escalar nada.
  - Dinámica: pares y velocidades de una persona andando, escalados a Danny con
    número de Froude constante. Da un orden de magnitud, no una medida.

No sustituye a medir una marcha dinámica real: esa no existe todavía.

Uso:
  python simulation/scripts/estimate_joint_loads.py   # escribe results/joint_load_estimates.json
"""

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

from biped_control import Biped, Pose, center_com_over, pelvis_height
from robot_spec import ARM_PARTS, HUMANOID, JOINT_PARTS, SHIN_LENGTH, THIGH_LENGTH


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "joint_load_estimates.json"

FOOT_Y = 0.05  # m
PAYLOAD = 0.3  # kg en cada mano, del presupuesto de masa
HUMAN_LEG = 0.90  # m, cadera-suelo de una persona de 1,75 m
MARGIN = 1.5  # sobre el mayor caso: equilibrio, tropiezos y error de modelo
CONTINUOUS_RATIO = 0.5  # par eficaz / par de pico al andar, estimado

# Marcha humana a unos 1,4 m/s: par de pico [N·m/kg] y velocidad de pico [rad/s].
# Tobillo y rodilla, contrastados con la literatura en la evaluación de la fase 5.1: la
# rodilla típica ronda 0,5 y se deja en 0,9 como margen. Cadera y ejes roll y yaw son
# órdenes de magnitud sin verificar.
HUMAN_GAIT = {
    "hip_pitch": (1.0, 3.5),
    "hip_roll": (0.9, 1.5),
    "hip_yaw": (0.3, 1.5),
    "knee": (0.9, 7.0),
    "ankle_pitch": (1.6, 6.3),
    "ankle_roll": (0.4, 1.5),
}


def support_torques(model, data, feet: tuple[str, ...]) -> tuple[dict, float]:
    """Par en cada articulación para seguir quieto sobre 'feet', y fuerza vertical total."""
    jacobians = []
    for side in feet:
        position, rotation = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, position, rotation, model.site(f"{side}_sole_site").id)
        jacobians.append(np.vstack([position, rotation]))
    jacobian = np.vstack(jacobians)
    wrench = np.linalg.lstsq(jacobian[:, :6].T, data.qfrc_bias[:6], rcond=None)[0]
    generalized = data.qfrc_bias - jacobian.T @ wrench
    torques = {model.joint(j).name.removesuffix("_joint"): float(generalized[model.jnt_dofadr[j]])
               for j in range(1, model.njnt)}
    return torques, float(wrench[2::6].sum())


def stand_pose(knee_deg: float) -> Pose:
    return Pose(np.array([0.0, 0.0, pelvis_height(knee_deg)]),
                np.array([0.0, FOOT_Y, 0.0]), np.array([0.0, -FOOT_Y, 0.0]))


def leg_cases(model, data) -> dict:
    biped = Biped(model, data)
    cases = {}
    for name, knee, feet in (
        ("de pie, rodilla 30°", 30, ("left", "right")),
        ("sentadilla, rodilla 90°", 90, ("left", "right")),
        ("sentadilla profunda, rodilla 120°", 120, ("left", "right")),
        ("un pie, rodilla 30°", 30, ("left",)),
        ("un pie, rodilla 60°", 60, ("left",)),
        ("un pie, rodilla 90°", 90, ("left",)),
    ):
        pose = stand_pose(knee)
        if len(feet) == 1:
            pose = center_com_over(biped, pose, feet[0])
            pose = replace(pose, right_sole=pose.right_sole + np.array([0.0, 0.0, 0.03]))
        else:
            pose = center_com_over(biped, pose, "left", FOOT_Y)
        biped.set_pose(data, pose)
        mujoco.mj_forward(model, data)
        torques, vertical = support_torques(model, data, feet)
        cases[name] = {"pares": torques, "fuerza_vertical": vertical}
    return cases


def arm_cases(model) -> dict:
    """Brazo izquierdo extendido y cintura inclinada, con la pelvis fija: lo que cuesta sujetarlos."""
    loaded = mujoco.MjSpec.from_file(str(ROOT / "simulation" / "models" / HUMANOID.file)).compile()
    hand = loaded.body("left_hand").id
    loaded.body_mass[hand] += PAYLOAD
    data = mujoco.MjData(loaded)
    cases = {}
    for name, angles in (
        ("brazo al frente con carga", {"left_shoulder_pitch": -math.pi / 2}),
        ("brazo al lado con carga", {"left_shoulder_roll": math.pi / 2}),
        ("antebrazo al frente con carga", {"left_elbow": math.pi / 2}),
        ("cintura a 45° con carga", {"waist_pitch": math.pi / 4, "left_shoulder_pitch": -math.pi / 2}),
    ):
        data.qpos[:] = 0.0
        data.qpos[2:4] = 0.5, 1.0
        for joint, angle in angles.items():
            data.qpos[loaded.joint(f"{joint}_joint").qposadr[0]] = angle
        mujoco.mj_forward(loaded, data)
        cases[name] = {"pares": {
            loaded.joint(j).name.removesuffix("_joint"): float(data.qfrc_bias[loaded.jnt_dofadr[j]])
            for j in range(1, loaded.njnt)
            if not loaded.joint(j).name.startswith(("right_",))
            and any(key in loaded.joint(j).name for key in ("shoulder", "elbow", "wrist", "waist"))
        }}
    return cases


def froude_scaled(mass: float) -> dict:
    """A igual número de Froude: par/kg ∝ longitud y velocidad angular ∝ 1/√longitud."""
    ratio = (THIGH_LENGTH + SHIN_LENGTH + 0.04) / HUMAN_LEG
    speed_scale = 1 / math.sqrt(ratio)
    return {part: {"par_pico": torque * ratio * mass, "velocidad_pico": speed * speed_scale}
            for part, (torque, speed) in HUMAN_GAIT.items()}


def worst(cases: dict, part: str) -> float:
    return max((abs(v) for case in cases.values() for k, v in case["pares"].items()
                if k == part or k == f"left_{part}"), default=0.0)


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / HUMANOID.file))
    data = mujoco.MjData(model)
    mass = float(model.body_subtreemass[1])
    legs, arms = leg_cases(model, data), arm_cases(model)
    gait = froude_scaled(mass)

    weight = mass * -model.opt.gravity[2]
    for name, case in legs.items():
        if abs(case["fuerza_vertical"] - weight) > 0.01:
            print(f"FAIL: {name}: {case['fuerza_vertical']:.2f} N frente a {weight:.2f} N de peso")
            return 1

    requirements = {}
    print(f"{'Articulación':<16}{'estático':>10}{'marcha':>9}{'pico':>8}{'continuo':>10}{'velocidad':>11}")
    for part in (*JOINT_PARTS, "waist_pitch", "waist_roll", "waist_yaw", *ARM_PARTS[:4]):
        static = worst({**legs, **arms}, part)
        dynamic = gait.get(part, {"par_pico": 0.0, "velocidad_pico": 0.0})
        peak = MARGIN * max(static, dynamic["par_pico"])
        requirements[part] = {
            "estatico": static,
            "marcha_escalada": dynamic["par_pico"],
            "par_pico": peak,
            "par_continuo": CONTINUOUS_RATIO * peak,
            "velocidad_pico": dynamic["velocidad_pico"],
        }
        print(f"{part:<16}{static:>10.2f}{dynamic['par_pico']:>9.2f}{peak:>8.2f}"
              f"{CONTINUOUS_RATIO * peak:>10.2f}{dynamic['velocidad_pico']:>11.1f}")

    RESULTS.write_text(json.dumps({"masa": mass, "margen": MARGIN, "casos_piernas": legs,
                                   "casos_brazos": arms, "requisitos": requirements},
                                  indent=1, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
