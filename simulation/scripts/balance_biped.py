"""
H1 - Equilibrio del bípedo: de pie, sentadilla y apoyo en un pie (etapa 10).

La pelvis tiene freejoint: nada la sujeta salvo las piernas. La secuencia
se define en el espacio de tareas (dónde están la pelvis y los pies) y en
cada paso leg_ik calcula los ángulos de cada pierna:

  1. de pie (rodillas a 40°)
  2. sentadilla (rodillas a 80°) y vuelta a de pie
  3. pasar el peso a la izquierda: la pelvis se desplaza hasta que el
     centro de masas (con el pie derecho levantado) queda sobre el pie
     izquierdo
  4. levantar el pie derecho 5 cm, bajarlo
  5. volver al centro

Las fracciones de peso de StanceCompensator salen de dónde está el centro
de masas entre los dos pies; con un pie en el aire, todo va al otro.

Uso:
  python simulation/scripts/balance_biped.py           # análisis, sin ventana
  mjpython simulation/scripts/balance_biped.py --view  # además, verlo
"""

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from biped_control import (
    FOOT_HALF,
    HIP_Y,
    JOINTS,
    POSITION_SENSORS,
    SIDES,
    Biped,
    Phase,
    Pose,
    center_com_over,
    joint_targets,
    pelvis_height,
)
from leg_control import HOLD_TIME, TRANSITION_TIME, interpolate, read_sensors, total_weight


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "biped_6dof.xml"

STAND_KNEE_DEG = 40
SQUAT_KNEE_DEG = 80
LIFT_HEIGHT = 0.05  # m, altura a la que sube el pie derecho

SETTLE_TIME = 1.0  # s

# Criterios de aceptación
MAX_TILT_DEG = 5.0
MAX_PELVIS_ERROR = 0.01  # m
MAX_FOOT_SLIP = 0.005  # m
FORCE_RELATIVE = 0.05
MIN_CLEARANCE = 0.04  # m
MIN_COM_MARGIN = 0.02  # m
MAX_JOINT_ERROR_DEG = 0.5


@dataclass
class PhaseResult:
    name: str
    right_in_air: bool  # según el plan
    pelvis_error: float  # m
    max_tilt_deg: float  # durante la fase
    foot_slip: np.ndarray  # m, (izq, der); NaN si el pie está en el aire
    foot_force: np.ndarray  # N, (izq, der)
    right_clearance: float  # m, altura de la planta derecha
    com_margin: float  # m, distancia del CdM al borde del pie de apoyo
    max_joint_error_deg: float


def build_plan(biped: Biped) -> tuple[Pose, list[Phase]]:
    stand = Pose(
        np.array([0.0, 0.0, pelvis_height(STAND_KNEE_DEG)]),
        np.array([0.0, HIP_Y, 0.0]),
        np.array([0.0, -HIP_Y, 0.0]),
    )
    squat = Pose(stand.pelvis * [1, 1, 0] + [0, 0, pelvis_height(SQUAT_KNEE_DEG)],
                 stand.left_sole, stand.right_sole)
    # CdM sobre el pie izquierdo con el pie derecho ya levantado
    lifted = center_com_over(
        biped, Pose(stand.pelvis, stand.left_sole, stand.right_sole + [0, 0, LIFT_HEIGHT]), "left"
    )
    over_left = Pose(lifted.pelvis, stand.left_sole, stand.right_sole)
    shift = lifted.pelvis[1] - stand.pelvis[1]

    phases = [
        Phase(f"sentadilla {SQUAT_KNEE_DEG}°", squat, swing=None),
        Phase(f"de pie {STAND_KNEE_DEG}°", stand, swing=None),
        Phase(f"peso a la izquierda ({100 * shift:.1f} cm)", over_left, swing=None),
        Phase(f"pie derecho arriba {100 * LIFT_HEIGHT:.0f} cm", lifted, swing="right"),
        Phase("pie derecho abajo", over_left, swing="right"),
        Phase("vuelta al centro", stand, swing=None),
    ]
    return stand, phases


def run(model: mujoco.MjModel, data: mujoco.MjData, viewer=None) -> tuple[list[PhaseResult], np.ndarray]:
    biped = Biped(model, data, viewer)
    stand, phases = build_plan(biped)
    timestep = model.opt.timestep

    mujoco.mj_resetData(model, data)
    biped.set_pose(data, stand)
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, swing=None)
    soles_start = biped.soles()

    results: list[PhaseResult] = []
    previous = stand
    for phase in phases:
        max_tilt = 0.0
        for i in range(round((TRANSITION_TIME + HOLD_TIME) / timestep)):
            vector = interpolate(previous.vector(), phase.pose.vector(), i * timestep, "smooth")
            biped.step(Pose.from_vector(vector), phase.swing)
            max_tilt = max(max_tilt, biped.tilt_deg())

        soles = biped.soles()
        slip = np.linalg.norm((soles - soles_start)[:, :2], axis=1)
        right_in_air = phase.swing == "right" and phase.pose.right_sole[2] > 0
        if right_in_air:
            slip[1] = np.nan

        com = biped.com()
        com_margin = (
            float(np.min(FOOT_HALF - np.abs(com[:2] - soles[0, :2])))
            if phase.swing == "right" else math.nan
        )
        target = joint_targets(phase.pose)
        results.append(
            PhaseResult(
                name=phase.name,
                right_in_air=right_in_air,
                pelvis_error=float(np.linalg.norm(data.xpos[biped.pelvis_id] - phase.pose.pelvis)),
                max_tilt_deg=max_tilt,
                foot_slip=slip,
                foot_force=np.array([data.sensor(f"{s}_foot_force").data[0] for s in SIDES]),
                right_clearance=float(soles[1, 2]),
                com_margin=com_margin,
                max_joint_error_deg=float(
                    np.degrees(np.abs(target - read_sensors(data, POSITION_SENSORS))).max()
                ),
            )
        )
        previous = phase.pose

    return results, biped.saturated_steps / biped.total_steps


def print_report(results: list[PhaseResult], saturation: np.ndarray) -> None:
    print("\nFase                         | err pelvis [mm] | incl. máx [°] | desliz. izq/der [mm] "
          "| pie izq/der [N] | altura pie der [mm] | margen CdM [mm] | error art. [°]")
    for r in results:
        slip = " / ".join("  —  " if math.isnan(v) else f"{1000 * v:5.2f}" for v in r.foot_slip)
        margin = "   —" if math.isnan(r.com_margin) else f"{1000 * r.com_margin:5.1f}"
        print(f"  {r.name:27s}| {1000 * r.pelvis_error:6.2f}          | {r.max_tilt_deg:5.2f}         "
              f"| {slip}       | {r.foot_force[0]:5.1f} / {r.foot_force[1]:5.1f}  "
              f"| {1000 * r.right_clearance:6.1f}              | {margin}           "
              f"| {r.max_joint_error_deg:5.2f}")
    print()
    for i, joint in enumerate(JOINTS):
        if saturation[i] > 0:
            print(f"  {joint}: saturado {100 * saturation[i]:.1f} % del tiempo")
    if not np.any(saturation > 0):
        print("  Ningún motor saturado.")


def check_acceptance(results: list[PhaseResult], saturation: np.ndarray, weight: float) -> list[str]:
    failures: list[str] = []
    if np.any(saturation > 0):
        failures.append("hay saturación de torque")
    for r in results:
        if r.max_tilt_deg > MAX_TILT_DEG:
            failures.append(f"{r.name}: la pelvis se inclina {r.max_tilt_deg:.1f}°")
        if r.pelvis_error > MAX_PELVIS_ERROR:
            failures.append(f"{r.name}: pelvis a {1000 * r.pelvis_error:.1f} mm del plan")
        if np.nanmax(r.foot_slip) > MAX_FOOT_SLIP:
            failures.append(f"{r.name}: un pie desliza {1000 * np.nanmax(r.foot_slip):.1f} mm")
        if r.max_joint_error_deg > MAX_JOINT_ERROR_DEG:
            failures.append(f"{r.name}: error articular {r.max_joint_error_deg:.2f}°")

        if r.right_in_air:
            if r.foot_force[1] > 0:
                failures.append(f"{r.name}: el pie derecho toca el suelo")
            if abs(r.foot_force[0] - weight) > FORCE_RELATIVE * weight:
                failures.append(f"{r.name}: pie izquierdo {r.foot_force[0]:.1f} N != peso")
            if r.right_clearance < MIN_CLEARANCE:
                failures.append(f"{r.name}: pie derecho solo a {1000 * r.right_clearance:.0f} mm")
            if r.com_margin < MIN_COM_MARGIN:
                failures.append(f"{r.name}: CdM a {1000 * r.com_margin:.0f} mm del borde del pie")
        elif abs(r.foot_force.sum() - weight) > FORCE_RELATIVE * weight:
            failures.append(f"{r.name}: fuerza total {r.foot_force.sum():.1f} N != peso")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Equilibrio del bípedo")
    parser.add_argument("--view", action="store_true", help="verlo (requiere mjpython)")
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    weight = total_weight(model)
    print(f"Peso total: {weight:.2f} N")

    try:
        results, saturation = run(model, data)
    except RuntimeError as error:
        print(f"\n=== Criterios de aceptación ===\n  FAIL: {error}")
        return 1

    print_report(results, saturation)
    failures = check_acceptance(results, saturation, weight)
    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: no se cae, pies sin deslizar, apoyo en un pie con el CdM dentro, sin saturación")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run(model, data, viewer=viewer)
        except (KeyboardInterrupt, RuntimeError):
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
