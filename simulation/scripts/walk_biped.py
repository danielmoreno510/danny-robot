"""
H1 - Primeros pasos del bípedo: marcha estática (etapa 11).

Marcha estática: el centro de masas está siempre sobre un pie, así que si
el robot se congelara en cualquier instante no se caería. Cada paso son
cuatro fases:

  1. peso sobre el pie de apoyo (CdM sobre su centro)
  2. pie libre arriba
  3. pie libre adelante (en el aire, la pelvis se reajusta para que el CdM
     siga sobre el pie de apoyo)
  4. pie libre abajo

Empieza el pie derecho, y cada pie aterriza STEP_LENGTH por delante del
otro. Tras N_STEPS pasos, el peso vuelve al centro entre los dos pies.

--speed divide la duración de las transiciones y --hold fija la pausa tras
cada una (etapa 12: --speed 1.27 --hold 0.25 hace la marcha en la mitad de
tiempo). --dynamic usa la compensación dinámica (dinámica inversa + ZMP)
en lugar de la estática, para reproducir la comparación de la etapa 12.

Uso:
  python simulation/scripts/walk_biped.py                               # etapa 11
  python simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25      # etapa 12
  mjpython simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25 --view
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
    leg_reach,
    pelvis_height,
)
from leg_control import HOLD_TIME, TRANSITION_TIME, interpolate, read_sensors, total_weight


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "biped_6dof.xml"

STAND_KNEE_DEG = 60  # más flexión que en la etapa 10: con 50° la pierna se estiraba al 99,9 %
STEP_LENGTH = 0.08  # m, cuánto adelanta cada pie al otro
LIFT_HEIGHT = 0.05  # m
N_STEPS = 4

SETTLE_TIME = 1.0  # s
MAX_REACH = 0.98  # fracción de la pierna estirada que se permite planificar

# La aceleración pico del perfil coseno crece con distancia / duración².
# Para no pasar de la de la etapa 10 (pelvis 15,8 cm en 1,5 s), las
# transiciones largas duran más: duración ∝ √distancia.
REFERENCE_SHIFT = 0.158  # m

NAMES = {"left": "izquierdo", "right": "derecho"}

# Criterios de aceptación
MAX_TILT_DEG = 5.0
MAX_YAW_DEG = 3.0
MAX_PELVIS_ERROR = 0.01  # m
MAX_FOOT_SLIP = 0.005  # m, respecto a donde se colocó el pie
FORCE_RELATIVE = 0.05
MIN_CLEARANCE = 0.04  # m
MIN_COM_MARGIN = 0.02  # m
MAX_JOINT_ERROR_DEG = 0.5


@dataclass
class PhaseResult:
    name: str
    swing_in_air: str | None  # pie en el aire según el plan
    pelvis_error: float  # m
    max_tilt_deg: float  # durante la fase
    yaw_deg: float
    foot_slip: np.ndarray  # m, (izq, der) respecto al plan; NaN si está en el aire
    foot_force: np.ndarray  # N, (izq, der)
    clearance: float  # m, altura de la planta del pie en el aire; NaN si no hay
    com_margin: float  # m, del CdM al borde del pie de apoyo; NaN en doble apoyo
    max_joint_error_deg: float


def transition_time(start: Pose, end: Pose) -> float:
    """Duración [s] para que ni la pelvis ni los pies aceleren más que en la etapa 10."""
    distance = float(np.max(np.linalg.norm((end.vector() - start.vector()).reshape(3, 3), axis=1)))
    return TRANSITION_TIME * max(1.0, math.sqrt(distance / REFERENCE_SHIFT))


def with_sole(pose: Pose, side: str, sole: np.ndarray) -> Pose:
    if side == "left":
        return Pose(pose.pelvis, sole, pose.right_sole)
    return Pose(pose.pelvis, pose.left_sole, sole)


def build_plan(biped: Biped) -> tuple[Pose, list[Phase]]:
    stand = Pose(
        np.array([0.0, 0.0, pelvis_height(STAND_KNEE_DEG)]),
        np.array([0.0, HIP_Y, 0.0]),
        np.array([0.0, -HIP_Y, 0.0]),
    )
    phases: list[Phase] = []
    pose = stand
    swing = "right"
    for step in range(1, N_STEPS + 1):
        stance = "left" if swing == "right" else "right"
        sole = pose.sole(swing)
        up = sole + [0.0, 0.0, LIFT_HEIGHT]
        forward = np.array([pose.sole(stance)[0] + STEP_LENGTH, sole[1], LIFT_HEIGHT])
        landed = forward * [1, 1, 0]

        lifted = center_com_over(biped, with_sole(pose, swing, up), stance)
        moved = center_com_over(biped, with_sole(lifted, swing, forward), stance)

        label = f"paso {step}: pie {NAMES[swing]}"
        phases += [
            Phase(f"paso {step}: peso al {NAMES[stance]}", with_sole(lifted, swing, sole), None),
            Phase(f"{label} arriba", lifted, swing),
            Phase(f"{label} adelante", moved, swing),
            Phase(f"{label} abajo", with_sole(moved, swing, landed), swing),
        ]
        pose = with_sole(moved, swing, landed)
        swing = stance

    middle = 0.5 * (pose.left_sole + pose.right_sole)
    final = Pose(np.array([middle[0], 0.0, stand.pelvis[2]]), pose.left_sole, pose.right_sole)
    phases.append(Phase("peso al centro", final, None))

    for phase in phases:
        reach = leg_reach(phase.pose)
        if reach > MAX_REACH:
            raise ValueError(
                f"'{phase.name}' pide estirar la pierna al {100 * reach:.1f} % "
                f"(máximo {100 * MAX_REACH:.0f} %)"
            )
    return stand, phases


def run(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer=None,
    speed: float = 1.0,
    hold: float = HOLD_TIME,
    dynamic: bool = False,
) -> tuple[list[PhaseResult], np.ndarray]:
    """speed divide las transiciones; hold es la pausa tras cada una [s]."""
    biped = Biped(model, data, viewer, dynamic=dynamic)
    stand, phases = build_plan(biped)
    timestep = model.opt.timestep

    mujoco.mj_resetData(model, data)
    biped.set_pose(data, stand)
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, swing=None)

    results: list[PhaseResult] = []
    previous = stand
    for phase in phases:
        max_tilt = 0.0
        duration = transition_time(previous, phase.pose) / speed
        for i in range(round((duration + hold) / timestep)):
            vector = interpolate(previous.vector(), phase.pose.vector(), i * timestep, "smooth", duration)
            biped.step(Pose.from_vector(vector), phase.swing)
            max_tilt = max(max_tilt, biped.tilt_deg())

        soles = biped.soles()
        planned = np.array([phase.pose.left_sole, phase.pose.right_sole])
        on_ground = planned[:, 2] <= 0
        slip = np.where(on_ground, np.linalg.norm((soles - planned)[:, :2], axis=1), np.nan)
        swing_in_air = phase.swing if phase.swing and not on_ground[SIDES.index(phase.swing)] else None

        com = biped.com()
        com_margin = math.nan
        clearance = math.nan
        if phase.swing is not None:
            stance_sole = soles[1 - SIDES.index(phase.swing)]
            com_margin = float(np.min(FOOT_HALF - np.abs(com[:2] - stance_sole[:2])))
        if swing_in_air is not None:
            clearance = float(soles[SIDES.index(swing_in_air), 2])

        target = joint_targets(phase.pose)
        results.append(
            PhaseResult(
                name=phase.name,
                swing_in_air=swing_in_air,
                pelvis_error=float(np.linalg.norm(data.xpos[biped.pelvis_id] - phase.pose.pelvis)),
                max_tilt_deg=max_tilt,
                yaw_deg=biped.yaw_deg(),
                foot_slip=slip,
                foot_force=np.array([data.sensor(f"{s}_foot_force").data[0] for s in SIDES]),
                clearance=clearance,
                com_margin=com_margin,
                max_joint_error_deg=float(
                    np.degrees(np.abs(target - read_sensors(data, POSITION_SENSORS))).max()
                ),
            )
        )
        previous = phase.pose

    return results, biped.saturated_steps / biped.total_steps


def print_report(results: list[PhaseResult], saturation: np.ndarray, final_x: float) -> None:
    def mm(value: float) -> str:
        return "   — " if math.isnan(value) else f"{1000 * value:5.1f}"

    print("\nFase                              | err pelvis | incl. máx | yaw   | desliz. izq/der "
          "| pie izq/der [N] | altura pie libre | margen CdM | error art.")
    print("                                  | [mm]       | [°]       | [°]   | [mm]            "
          "|                 | [mm]             | [mm]       | [°]")
    for r in results:
        print(f"  {r.name:32s}| {1000 * r.pelvis_error:5.2f}      | {r.max_tilt_deg:5.2f}     "
              f"| {r.yaw_deg:5.2f} | {mm(r.foot_slip[0])} / {mm(r.foot_slip[1])}  "
              f"| {r.foot_force[0]:5.1f} / {r.foot_force[1]:5.1f}   | {mm(r.clearance)}            "
              f"| {mm(r.com_margin)}      | {r.max_joint_error_deg:5.2f}")
    print(f"\n  Avance de la pelvis: {100 * final_x:.1f} cm en {N_STEPS} pasos")
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
        if abs(r.yaw_deg) > MAX_YAW_DEG:
            failures.append(f"{r.name}: la pelvis gira {r.yaw_deg:.1f}°")
        if r.pelvis_error > MAX_PELVIS_ERROR:
            failures.append(f"{r.name}: pelvis a {1000 * r.pelvis_error:.1f} mm del plan")
        if np.nanmax(r.foot_slip) > MAX_FOOT_SLIP:
            failures.append(f"{r.name}: un pie está a {1000 * np.nanmax(r.foot_slip):.1f} mm de su sitio")
        if r.max_joint_error_deg > MAX_JOINT_ERROR_DEG:
            failures.append(f"{r.name}: error articular {r.max_joint_error_deg:.2f}°")

        if r.swing_in_air is not None:
            swing = SIDES.index(r.swing_in_air)
            if r.foot_force[swing] > 0:
                failures.append(f"{r.name}: el pie {NAMES[r.swing_in_air]} toca el suelo")
            if abs(r.foot_force[1 - swing] - weight) > FORCE_RELATIVE * weight:
                failures.append(f"{r.name}: el pie de apoyo carga {r.foot_force[1 - swing]:.1f} N != peso")
            if r.clearance < MIN_CLEARANCE:
                failures.append(f"{r.name}: pie libre solo a {1000 * r.clearance:.0f} mm")
        elif abs(r.foot_force.sum() - weight) > FORCE_RELATIVE * weight:
            failures.append(f"{r.name}: fuerza total {r.foot_force.sum():.1f} N != peso")

        if not math.isnan(r.com_margin) and r.com_margin < MIN_COM_MARGIN:
            failures.append(f"{r.name}: CdM a {1000 * r.com_margin:.0f} mm del borde del pie")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Marcha estática del bípedo")
    parser.add_argument("--view", action="store_true", help="verlo (requiere mjpython)")
    parser.add_argument("--speed", type=float, default=1.0, help="divide la duración de las transiciones")
    parser.add_argument("--hold", type=float, default=HOLD_TIME, help="pausa tras cada transición [s]")
    parser.add_argument("--dynamic", action="store_true", help="compensación dinámica (comparación de la etapa 12)")
    args = parser.parse_args()
    dynamic = args.dynamic

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    weight = total_weight(model)
    print(f"Peso total: {weight:.2f} N | {N_STEPS} pasos de {100 * STEP_LENGTH:.0f} cm | "
          f"transiciones x{args.speed:g}, pausas {args.hold:g} s | "
          f"compensación {'dinámica' if dynamic else 'estática'}")

    try:
        results, saturation = run(model, data, speed=args.speed, hold=args.hold, dynamic=dynamic)
    except (RuntimeError, ValueError) as error:
        print(f"\n=== Criterios de aceptación ===\n  FAIL: {error}")
        return 1

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    print_report(results, saturation, float(data.xpos[pelvis_id, 0]))
    print(f"  Duración total: {data.time:.1f} s")
    failures = check_acceptance(results, saturation, weight)
    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print(f"  PASS: {N_STEPS} pasos sin caerse, pies en su sitio, CdM siempre sobre el pie "
              "de apoyo, sin saturación")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run(model, data, viewer=viewer, speed=args.speed, hold=args.hold, dynamic=dynamic)
        except (KeyboardInterrupt, RuntimeError):
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
