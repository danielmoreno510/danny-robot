"""
Danny - Primeros pasos del bípedo: marcha estática (experimento 11).

Marcha estática: el centro de masas está sobre el pie de apoyo mientras el
otro está en el aire, y dentro del polígono de los dos pies al pasar el peso.
Si el robot se congelara en cualquier instante, no se caería. Cada paso son
cuatro fases:

  1. peso sobre el pie de apoyo (CdM sobre su centro)
  2. pie libre arriba
  3. pie libre adelante (en el aire, la pelvis se reajusta para que el CdM
     siga sobre el pie de apoyo)
  4. pie libre abajo

Empieza el pie derecho, y cada pie aterriza gait.step por delante del
otro. Tras N_STEPS pasos, el peso vuelve al centro entre los dos pies.

--speed divide la duración de las transiciones y --hold fija la pausa tras
cada una (experimento 12: --speed 1.27 --hold 0.25 hace la marcha en la mitad de
tiempo).

--knee, --step, --width, --inset y --turn cambian la forma de andar (experimento 13):
postura, largo del paso, separación de los pies, cuánto se queda el CdM hacia
dentro del pie de apoyo y giro final a la izquierda.

Uso:
  python simulation/scripts/walk_biped.py                               # experimento 11
  python simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25      # experimento 12
  mjpython simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25 --view
"""

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from biped_control import (
    FOOT_HALF,
    HIP_Y,
    JOINTS,
    SIDES,
    Biped,
    Phase,
    Pose,
    center_com_over,
    joint_targets,
    leg_reach,
    pelvis_height,
    rotation_z,
)
from leg_control import HOLD_TIME, TRANSITION_TIME, interpolate, read_sensors, total_weight
from robot_spec import POSITION_SENSORS, ROBOTS


ROOT = Path(__file__).resolve().parents[1]

LIFT_HEIGHT = 0.03  # m
N_STEPS = 4
TURN_INCREMENT_DEG = 15.0  # giro máximo por paso

SETTLE_TIME = 1.0  # s
MAX_REACH = 0.98  # fracción de la pierna estirada que se permite planificar

# La aceleración pico del perfil coseno crece con distancia / duración².
# Para no pasar de la del experimento 10 (pelvis 10 cm en 1,5 s), las
# transiciones largas duran más: duración ∝ √distancia.
REFERENCE_SHIFT = 0.10  # m

NAMES = {"left": "izquierdo", "right": "derecho"}

# Criterios de aceptación
MAX_TILT_DEG = 5.0
MAX_YAW_DEG = 3.0
MAX_PELVIS_ERROR = 0.01  # m
MAX_FOOT_SLIP = 0.005  # m, respecto a donde se colocó el pie
FORCE_RELATIVE = 0.05
MIN_CLEARANCE = 0.025  # m
MIN_COM_MARGIN = 0.02  # m
MAX_JOINT_ERROR_DEG = 0.5
MAX_TRACKING_ERROR_DEG = 1.0  # durante el movimiento
MAX_TRACKING_PELVIS = 0.02  # m, durante el movimiento


@dataclass
class Gait:
    """Forma de andar. Los valores por defecto son los de los experimentos 11 y 12."""

    knee_deg: float = 60.0  # postura; con 50° la pierna se estiraba al 99,9 %
    step: float = 0.05  # m, cuánto adelanta cada pie al otro
    width: float = HIP_Y  # m, distancia de cada pie al centro
    inset: float = 0.0  # m, CdM hacia dentro del centro del pie de apoyo
    turn_deg: float = 0.0  # giro final a la izquierda


@dataclass
class PhaseResult:
    name: str
    swing_in_air: str | None  # pie en el aire según el plan
    pelvis_error: float  # m
    max_tilt_deg: float  # durante la fase
    yaw_error_deg: float  # pelvis respecto al plan
    foot_slip: np.ndarray  # m, (izq, der) respecto al plan; NaN si está en el aire
    foot_yaw_error_deg: float  # el mayor de los pies apoyados
    tracking_error_deg: float  # error articular máximo durante toda la fase
    tracking_pelvis: float  # m, error de pelvis máximo durante toda la fase
    light_contact_fraction: float  # tiempo con un pie cargado apoyado en 2 puntos o menos
    foot_force: np.ndarray  # N, (izq, der)
    clearance: float  # m, altura de la planta del pie en el aire; NaN si no hay
    com_margin: float  # m, del CdM al borde del pie de apoyo; NaN en doble apoyo
    max_joint_error_deg: float


def transition_time(start: Pose, end: Pose) -> float:
    """Duración [s] para que ni la pelvis ni los pies aceleren más que en el experimento 10."""
    change = end.vector() - start.vector()
    distance = float(np.max(np.linalg.norm(change[:9].reshape(3, 3), axis=1)))
    return TRANSITION_TIME * max(1.0, math.sqrt(distance / REFERENCE_SHIFT))


def footsteps(gait: Gait) -> list[tuple[str, np.ndarray, float]]:
    """
    Lista de pisadas (pie, posición, giro). Cada pisada se coloca en los ejes
    del nuevo rumbo: 'avance' por delante del pie de apoyo y a 2 * width de él.
    Tras N_STEPS pasos rectos, el rumbo sube TURN_INCREMENT_DEG por paso hasta
    gait.turn_deg, y un último paso deja los dos pies paralelos.
    """
    feet = {
        "left": (np.array([0.0, gait.width, 0.0]), 0.0),
        "right": (np.array([0.0, -gait.width, 0.0]), 0.0),
    }
    target_yaw = math.radians(gait.turn_deg)
    increment = math.radians(TURN_INCREMENT_DEG)
    result = []
    swing = "right"
    count = 0
    while count < N_STEPS or feet["left"][1] < target_yaw or feet["right"][1] < target_yaw:
        turning = count >= N_STEPS
        stance = "left" if swing == "right" else "right"
        stance_position, stance_yaw = feet[stance]
        yaw = min(stance_yaw + increment, target_yaw) if turning else stance_yaw
        sideways = 2 * gait.width if swing == "left" else -2 * gait.width
        advance = 0.5 * gait.step if turning else gait.step
        position = stance_position + rotation_z(yaw) @ np.array([advance, sideways, 0.0])
        feet[swing] = (position, yaw)
        result.append((swing, position, yaw))
        swing = stance
        count += 1
    return result


def build_plan(biped: Biped, gait: Gait) -> tuple[Pose, list[Phase]]:
    stand = Pose(
        np.array([0.0, 0.0, pelvis_height(gait.knee_deg)]),
        np.array([0.0, gait.width, 0.0]),
        np.array([0.0, -gait.width, 0.0]),
    )
    phases: list[Phase] = []
    pose = stand
    for step, (swing, position, yaw) in enumerate(footsteps(gait), start=1):
        stance = "left" if swing == "right" else "right"
        lift = np.array([0.0, 0.0, LIFT_HEIGHT])
        sole = pose.sole(swing)

        lifted = center_com_over(biped, pose.with_foot(swing, sole + lift), stance, gait.inset)
        ahead = replace(
            lifted.with_foot(swing, position + lift, yaw),
            pelvis_yaw=0.5 * (pose.yaw(stance) + yaw),
        )
        moved = center_com_over(biped, ahead, stance, gait.inset)
        landed = moved.with_foot(swing, position)

        label = f"paso {step}: pie {NAMES[swing]}"
        phases += [
            Phase(f"paso {step}: peso al {NAMES[stance]}", lifted.with_foot(swing, sole), None),
            Phase(f"{label} arriba", lifted, swing),
            Phase(f"{label} adelante", moved, swing),
            Phase(f"{label} abajo", landed, swing),
        ]
        pose = landed

    middle = 0.5 * (pose.left_sole + pose.right_sole)
    final = replace(
        pose,
        pelvis=np.array([middle[0], middle[1], stand.pelvis[2]]),
        pelvis_yaw=0.5 * (pose.left_yaw + pose.right_yaw),
    )
    phases.append(Phase("peso al centro", final, None))

    for phase in phases:
        reach = leg_reach(phase.pose)
        if reach > MAX_REACH:
            raise ValueError(
                f"'{phase.name}' pide estirar la pierna al {100 * reach:.1f} % "
                f"(máximo {100 * MAX_REACH:.0f} %)"
            )
        joint_targets(phase.pose)  # lanza un error si alguna postura es imposible
    return stand, phases


def run(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer=None,
    speed: float = 1.0,
    hold: float = HOLD_TIME,
    gait: Gait | None = None,
    nominal_model: mujoco.MjModel | None = None,
) -> tuple[list[PhaseResult], np.ndarray, bool]:
    """
    speed divide las transiciones; hold es la pausa tras cada una [s].
    Devuelve resultados por fase, fracción saturada y si el robot chocó consigo mismo.
    nominal_model: el modelo que cree tener el controlador, si difiere de 'model'.
    """
    biped = Biped(model, data, viewer, nominal_model=nominal_model)
    stand, phases = build_plan(biped, Gait() if gait is None else gait)
    timestep = model.opt.timestep

    mujoco.mj_resetData(model, data)
    biped.set_pose(data, stand)
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, swing=None)

    results: list[PhaseResult] = []
    previous = stand
    for phase in phases:
        max_tilt = 0.0
        tracking_error = tracking_pelvis = 0.0
        light_contact = steps = 0
        duration = transition_time(previous, phase.pose) / speed
        for i in range(round((duration + hold) / timestep)):
            vector = interpolate(previous.vector(), phase.pose.vector(), i * timestep, "smooth", duration)
            pose = Pose.from_vector(vector)
            biped.step(pose, phase.swing)
            max_tilt = max(max_tilt, biped.tilt_deg())
            error = np.abs(joint_targets(pose) - read_sensors(data, POSITION_SENSORS))
            tracking_error = max(tracking_error, math.degrees(float(error.max())))
            tracking_pelvis = max(tracking_pelvis, float(np.linalg.norm(data.xpos[biped.pelvis_id] - pose.pelvis)))
            light_contact += biped.has_light_contact()
            steps += 1

        soles = biped.soles()
        planned = np.array([phase.pose.left_sole, phase.pose.right_sole])
        on_ground = planned[:, 2] <= 0
        slip = np.where(on_ground, np.linalg.norm((soles - planned)[:, :2], axis=1), np.nan)
        swing_in_air = phase.swing if phase.swing and not on_ground[SIDES.index(phase.swing)] else None

        com = biped.com()
        com_margin = math.nan
        clearance = math.nan
        if phase.swing is not None:
            stance = SIDES[1 - SIDES.index(phase.swing)]
            stance_sole = soles[SIDES.index(stance)]
            # En los ejes del pie: con el pie girado, su rectángulo también lo está
            in_foot = rotation_z(-phase.pose.yaw(stance)) @ (com - stance_sole)
            com_margin = float(np.min(FOOT_HALF - np.abs(in_foot[:2])))
        if swing_in_air is not None:
            clearance = float(soles[SIDES.index(swing_in_air), 2])

        foot_yaw_errors = [
            abs(biped.foot_yaw_deg(side) - math.degrees(phase.pose.yaw(side)))
            for side, down in zip(SIDES, on_ground) if down
        ]
        target = joint_targets(phase.pose)
        results.append(
            PhaseResult(
                name=phase.name,
                swing_in_air=swing_in_air,
                pelvis_error=float(np.linalg.norm(data.xpos[biped.pelvis_id] - phase.pose.pelvis)),
                max_tilt_deg=max_tilt,
                yaw_error_deg=biped.yaw_deg() - math.degrees(phase.pose.pelvis_yaw),
                foot_slip=slip,
                foot_yaw_error_deg=max(foot_yaw_errors),
                tracking_error_deg=tracking_error,
                tracking_pelvis=tracking_pelvis,
                light_contact_fraction=light_contact / steps,
                foot_force=np.array([data.sensor(f"{s}_foot_force").data[0] for s in SIDES]),
                clearance=clearance,
                com_margin=com_margin,
                max_joint_error_deg=float(
                    np.degrees(np.abs(target - read_sensors(data, POSITION_SENSORS))).max()
                ),
            )
        )
        previous = phase.pose

    return results, biped.saturated_steps / biped.total_steps, biped.self_collided


def print_report(
    results: list[PhaseResult],
    saturation: np.ndarray,
    final_position: np.ndarray,
    final_yaw_deg: float,
) -> None:
    def mm(value: float) -> str:
        return "   — " if math.isnan(value) else f"{1000 * value:5.1f}"

    print("\nFase                              | err pelvis | incl. máx | err yaw | desliz. izq/der "
          "| pie izq/der [N] | altura pie libre | margen CdM | error art.")
    print("                                  | [mm]       | [°]       | [°]     | [mm]            "
          "|                 | [mm]             | [mm]       | [°]")
    for r in results:
        print(f"  {r.name:32s}| {1000 * r.pelvis_error:5.2f}      | {r.max_tilt_deg:5.2f}     "
              f"| {r.yaw_error_deg:5.2f}   | {mm(r.foot_slip[0])} / {mm(r.foot_slip[1])}  "
              f"| {r.foot_force[0]:5.1f} / {r.foot_force[1]:5.1f}   | {mm(r.clearance)}            "
              f"| {mm(r.com_margin)}      | {r.max_joint_error_deg:5.2f}")
    steps = (len(results) - 1) // 4
    print(f"\n  Pelvis al final: ({100 * final_position[0]:.1f}, {100 * final_position[1]:.1f}) cm, "
          f"rumbo {final_yaw_deg:.1f}°, tras {steps} pasos")
    print(f"  En movimiento: error articular máx {max(r.tracking_error_deg for r in results):.2f}°, "
          f"pelvis máx {1000 * max(r.tracking_pelvis for r in results):.1f} mm, "
          f"pie cargado con <= 2 apoyos {100 * max(r.light_contact_fraction for r in results):.1f} % de la peor fase")
    for i, joint in enumerate(JOINTS):
        if saturation[i] > 0:
            print(f"  {joint}: saturado {100 * saturation[i]:.1f} % del tiempo")
    if not np.any(saturation > 0):
        print("  Ningún motor saturado.")


def check_acceptance(
    results: list[PhaseResult],
    saturation: np.ndarray,
    weight: float,
    self_collided: bool,
) -> list[str]:
    failures: list[str] = []
    if np.any(saturation > 0):
        failures.append("hay saturación de torque")
    if self_collided:
        failures.append("el robot choca consigo mismo")
    for r in results:
        if r.max_tilt_deg > MAX_TILT_DEG:
            failures.append(f"{r.name}: la pelvis se inclina {r.max_tilt_deg:.1f}°")
        if abs(r.yaw_error_deg) > MAX_YAW_DEG:
            failures.append(f"{r.name}: la pelvis gira {r.yaw_error_deg:.1f}° de más")
        if r.foot_yaw_error_deg > MAX_YAW_DEG:
            failures.append(f"{r.name}: un pie gira {r.foot_yaw_error_deg:.1f}° de más")
        if r.pelvis_error > MAX_PELVIS_ERROR:
            failures.append(f"{r.name}: pelvis a {1000 * r.pelvis_error:.1f} mm del plan")
        if np.nanmax(r.foot_slip) > MAX_FOOT_SLIP:
            failures.append(f"{r.name}: un pie está a {1000 * np.nanmax(r.foot_slip):.1f} mm de su sitio")
        if r.max_joint_error_deg > MAX_JOINT_ERROR_DEG:
            failures.append(f"{r.name}: error articular {r.max_joint_error_deg:.2f}°")
        if r.tracking_error_deg > MAX_TRACKING_ERROR_DEG:
            failures.append(f"{r.name}: error articular en movimiento {r.tracking_error_deg:.2f}°")
        if r.tracking_pelvis > MAX_TRACKING_PELVIS:
            failures.append(f"{r.name}: pelvis en movimiento a {1000 * r.tracking_pelvis:.1f} mm")

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
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="biped", help="configuración del robot")
    parser.add_argument("--speed", type=float, default=1.0, help="divide la duración de las transiciones")
    parser.add_argument("--hold", type=float, default=HOLD_TIME, help="pausa tras cada transición [s]")
    parser.add_argument("--knee", type=float, default=Gait.knee_deg, help="rodillas de pie [°]")
    parser.add_argument("--step", type=float, default=Gait.step, help="largo del paso [m]")
    parser.add_argument("--width", type=float, default=Gait.width, help="distancia de cada pie al centro [m]")
    parser.add_argument("--inset", type=float, default=Gait.inset, help="CdM hacia dentro del pie de apoyo [m]")
    parser.add_argument("--turn", type=float, default=Gait.turn_deg, help="giro final a la izquierda [°]")
    parser.add_argument("--torso-error", type=float, default=0.0,
                        help="masa real de pelvis y torso respecto a la nominal (0.1 = +10 %%)")
    parser.add_argument("--mass-error", type=float, default=0.0,
                        help="masas reales respecto a las que cree el controlador (0.1 = +10 %%)")
    parser.add_argument("--friction", type=float, default=None, help="fricción real del suelo")
    parser.add_argument("--timestep", type=float, default=None, help="paso de simulación [s]")
    args = parser.parse_args()
    gait = Gait(args.knee, args.step, args.width, args.inset, args.turn)

    model_path = ROOT / "models" / ROBOTS[args.robot].file
    print(f"MuJoCo {mujoco.__version__} | modelo: {model_path.name}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    nominal = mujoco.MjModel.from_xml_path(str(model_path))
    model.body_mass[:] *= 1.0 + args.mass_error
    model.body_inertia[:] *= 1.0 + args.mass_error
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    model.body_mass[pelvis_id] *= 1.0 + args.torso_error
    model.body_inertia[pelvis_id] *= 1.0 + args.torso_error
    mujoco.mj_setConst(model, mujoco.MjData(model))
    if args.friction is not None:
        model.geom_friction[:, 0] = args.friction
    if args.timestep is not None:
        model.opt.timestep = args.timestep
        nominal.opt.timestep = args.timestep
    data = mujoco.MjData(model)
    weight = total_weight(model)
    print(f"Peso total: {weight:.2f} N | {N_STEPS} pasos de {100 * gait.step:.0f} cm, "
          f"giro {gait.turn_deg:g}°, rodillas {gait.knee_deg:g}°, pies a ±{100 * gait.width:g} cm | "
          f"transiciones x{args.speed:g}, pausas {args.hold:g} s")

    try:
        results, saturation, self_collided = run(
            model, data, speed=args.speed, hold=args.hold, gait=gait, nominal_model=nominal
        )
    except (RuntimeError, ValueError) as error:
        print(f"\n=== Criterios de aceptación ===\n  FAIL: {error}")
        return 1

    x_axis = data.xmat[pelvis_id].reshape(3, 3)[:, 0]
    print_report(results, saturation, data.xpos[pelvis_id], math.degrees(math.atan2(x_axis[1], x_axis[0])))
    print(f"  Duración total: {data.time:.1f} s")
    failures = check_acceptance(results, saturation, weight, self_collided)
    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: sin caerse, pies en su sitio, CdM siempre sobre el pie de apoyo, sin saturación")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run(model, data, viewer=viewer, speed=args.speed, hold=args.hold,
                    gait=gait, nominal_model=nominal)
        except (KeyboardInterrupt, RuntimeError):
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
