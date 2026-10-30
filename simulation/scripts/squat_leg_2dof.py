"""
Danny - Sentadilla con la pierna 2 GDL apoyada en el suelo (experimento 8).

La pelvis solo puede subir y bajar. La pierna sostiene su peso y el de
la pelvis a través del pie. Se recorre una secuencia de profundidades de
sentadilla con el perfil coseno de analyze_leg_2dof.py, en tres modos:
  - ninguna: sin compensación.
  - gravedad (aire): el GravityCompensator del experimento 7, que supone la
    pierna colgando. Sirve para ver que en apoyo no vale.
  - apoyo: StanceCompensator, que incluye el peso que sube desde el suelo.

Mide por objetivo: altura de la pelvis, torque pico y de sostén,
fuerza en el pie, error articular y deslizamiento del pie.

Uso:
  python simulation/scripts/squat_leg_2dof.py           # análisis, sin ventana
  mjpython simulation/scripts/squat_leg_2dof.py --view  # además, ver el modo apoyo
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from leg_control import (
    LEG_2DOF_ACTUATORS as ACTUATORS,
    LEG_2DOF_JOINTS as JOINTS,
    LEG_2DOF_POSITION_SENSORS as POSITION_SENSORS,
    LEG_2DOF_TORQUE_SENSORS as TORQUE_SENSORS,
    HOLD_TIME,
    SQUAT_KNEE_DEG,
    TRANSITION_TIME,
    GravityCompensator,
    StanceCompensator,
    get_id,
    interpolate,
    read_sensors,
    total_weight,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "leg_2dof_foot.xml"

THIGH_LENGTH = 0.25  # m
SHIN_LENGTH = 0.25  # m
HIP_BELOW_PELVIS = 0.05  # m, de pelvis a cadera
FOOT_RADIUS = 0.04  # m
PELVIS_BODY_Z = 0.6  # m, pos del body pelvis; qpos del slide es relativo a esto

# quede justo bajo la cadera: cadera = -rodilla / 2.
SETTLE_TIME = 1.0  # s, quieto en la postura inicial antes de empezar

COMPENSATIONS = ("ninguna", "gravedad (aire)", "apoyo")

# Criterios de aceptación (modo apoyo)
FOOT_FORCE_RELATIVE = 0.05  # fuerza en el pie = peso total ± 5 %
MAX_FOOT_SLIP = 0.005  # m
MAX_HEIGHT_ERROR = 0.01  # m


@dataclass
class SquatResult:
    knee_deg: float
    pelvis_height: float  # m, medida
    expected_height: float  # m, cinemática con ángulos objetivo
    peak_torque: np.ndarray  # N·m
    holding_torque: np.ndarray  # N·m
    joint_error_deg: np.ndarray
    foot_force: float  # N
    foot_slip: float  # m, |x del pie - x inicial|


@dataclass
class SquatRun:
    mode: str
    targets: list[SquatResult]
    saturation_fraction: np.ndarray


def joint_targets(knee_deg: float) -> np.ndarray:
    """(cadera, rodilla) en rad con el pie bajo la cadera."""
    knee = math.radians(knee_deg)
    return np.array([-knee / 2, knee])


def pelvis_height(knee_deg: float) -> float:
    """Altura de la pelvis [m] con el pie apoyado y la postura objetivo."""
    half_knee = math.radians(knee_deg) / 2
    return (
        FOOT_RADIUS
        + (THIGH_LENGTH + SHIN_LENGTH) * math.cos(half_knee)
        + HIP_BELOW_PELVIS
    )


def run_squat(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    compensation: str,
    viewer=None,
) -> SquatRun:
    """compensation: uno de COMPENSATIONS."""
    actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
    dof_ids = [
        int(model.jnt_dofadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in JOINTS
    ]
    foot_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "foot")
    joint_qpos = [
        int(model.jnt_qposadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in ("pelvis_slide", "hip_joint", "knee_joint")
    ]
    force_limit = model.actuator_forcerange[actuator_ids, 1]
    timestep = model.opt.timestep

    compensator = {
        "ninguna": None,
        "gravedad (aire)": GravityCompensator(model, actuator_ids, dof_ids),
        "apoyo": StanceCompensator(model, actuator_ids, dof_ids, [foot_id]),
    }[compensation]

    # Postura inicial con el pie justo tocando el suelo
    mujoco.mj_resetData(model, data)
    start = joint_targets(SQUAT_KNEE_DEG[0])
    data.qpos[joint_qpos] = [pelvis_height(SQUAT_KNEE_DEG[0]) - PELVIS_BODY_Z, *start]
    mujoco.mj_forward(model, data)

    saturated_steps = np.zeros(len(ACTUATORS))
    total_steps = 0

    def step(target: np.ndarray) -> np.ndarray:
        nonlocal saturated_steps, total_steps
        step_start = time.monotonic()

        ctrl = target
        if compensator is not None:
            ctrl = ctrl + compensator.offset(data.qpos)
        data.ctrl[actuator_ids] = ctrl
        mujoco.mj_step(model, data)

        torque = np.abs(read_sensors(data, TORQUE_SENSORS))
        saturated_steps += torque >= force_limit - 1e-6
        total_steps += 1

        if viewer is not None:
            if not viewer.is_running():
                raise KeyboardInterrupt
            viewer.sync()
            remaining = timestep - (time.monotonic() - step_start)
            if remaining > 0:
                time.sleep(remaining)
        return torque

    for _ in range(round(SETTLE_TIME / timestep)):
        step(start)
    foot_x0 = data.xpos[foot_id, 0]

    results: list[SquatResult] = []
    previous = start
    for knee_deg in SQUAT_KNEE_DEG[1:]:
        target = joint_targets(knee_deg)
        peak = np.zeros(len(ACTUATORS))

        for i in range(round((TRANSITION_TIME + HOLD_TIME) / timestep)):
            peak = np.maximum(peak, step(interpolate(previous, target, i * timestep, "smooth")))

        results.append(
            SquatResult(
                knee_deg=knee_deg,
                pelvis_height=PELVIS_BODY_Z + data.sensor("pelvis_height").data[0],
                expected_height=pelvis_height(knee_deg),
                peak_torque=peak,
                holding_torque=read_sensors(data, TORQUE_SENSORS),
                joint_error_deg=np.degrees(np.abs(target - read_sensors(data, POSITION_SENSORS))),
                foot_force=data.sensor("foot_force").data[0],
                foot_slip=abs(data.xpos[foot_id, 0] - foot_x0),
            )
        )
        previous = target

    return SquatRun(
        mode=compensation,
        targets=results,
        saturation_fraction=saturated_steps / total_steps,
    )


def print_report(run: SquatRun) -> None:
    print(f"\n=== Compensación: {run.mode} ===")
    print(
        "Rodilla | altura pelvis [m]   | tau pico [N·m] | tau sostén [N·m] "
        "| error [°]   | pie [N] | desliz. [mm]"
    )
    print("        | medida   teórica    | cad    rod     | cad    rod       | cad   rod   |         |")
    for r in run.targets:
        print(
            f"  {r.knee_deg:4}° | {r.pelvis_height:.4f}   {r.expected_height:.4f}     "
            f"| {r.peak_torque[0]:5.2f}  {r.peak_torque[1]:5.2f}  "
            f"| {r.holding_torque[0]:5.2f}  {r.holding_torque[1]:5.2f}    "
            f"| {r.joint_error_deg[0]:4.2f}  {r.joint_error_deg[1]:4.2f}  "
            f"| {r.foot_force:6.2f}  | {1000 * r.foot_slip:5.2f}"
        )
    for i, joint in enumerate(JOINTS):
        print(f"  {joint:10s}: saturado {100 * run.saturation_fraction[i]:5.1f} % del tiempo")


def check_acceptance(run: SquatRun, weight: float) -> list[str]:
    """Devuelve la lista de criterios incumplidos (vacía = PASS)."""
    failures: list[str] = []
    if np.any(run.saturation_fraction > 0):
        failures.append("hay saturación de torque")
    for r in run.targets:
        if abs(r.foot_force - weight) > FOOT_FORCE_RELATIVE * weight:
            failures.append(f"fuerza en el pie {r.foot_force:.2f} N != peso en {r.knee_deg}°")
        if r.foot_slip > MAX_FOOT_SLIP:
            failures.append(f"el pie desliza {1000 * r.foot_slip:.1f} mm en {r.knee_deg}°")
        if abs(r.pelvis_height - r.expected_height) > MAX_HEIGHT_ERROR:
            failures.append(
                f"altura de pelvis fuera de {100 * MAX_HEIGHT_ERROR:.0f} cm en {r.knee_deg}°"
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Sentadilla con la pierna apoyada")
    parser.add_argument("--view", action="store_true", help="ver el modo apoyo (requiere mjpython)")
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    weight = total_weight(model)
    print(f"Peso total sobre el pie: {weight:.2f} N")

    runs = [run_squat(model, data, mode) for mode in COMPENSATIONS]
    for run in runs:
        print_report(run)

    # Los criterios se exigen al modo apoyo; los otros dos son referencia.
    stance = runs[-1]
    failures = check_acceptance(stance, weight)
    print(f"\n=== Criterios de aceptación (compensación: {stance.mode}) ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print(
            "  PASS: sin saturación, pie = peso, sin deslizar, "
            f"altura de pelvis ± {100 * MAX_HEIGHT_ERROR:.0f} cm"
        )

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_squat(model, data, "apoyo", viewer=viewer)
        except KeyboardInterrupt:
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
