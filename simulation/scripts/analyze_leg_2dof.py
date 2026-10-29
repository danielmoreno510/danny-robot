"""
H1 - Análisis de esfuerzos de la pierna 2 GDL.

Ejecuta la misma secuencia de posiciones en tres modos:
  - step:   el objetivo salta de golpe (como la primera versión de la pierna).
  - smooth: el objetivo se interpola con un perfil coseno.
  - smooth + gravedad: igual, sumando compensación de gravedad.

Mide por articulación: torque pico, % de tiempo saturado,
torque de sostén, torque gravitatorio y error final.

Uso:
  python simulation/scripts/analyze_leg_2dof.py           # análisis, sin ventana
  mjpython simulation/scripts/analyze_leg_2dof.py --view  # además, ver el modo smooth
"""

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from leg_control import (
    HOLD_TIME,
    TRANSITION_TIME,
    GravityCompensator,
    get_id,
    interpolate,
    read_sensors,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "leg_2dof.xml"

JOINTS = ("hip_joint", "knee_joint")
ACTUATORS = ("hip_motor", "knee_motor")
POSITION_SENSORS = ("hip_pos", "knee_pos")
TORQUE_SENSORS = ("hip_torque", "knee_torque")

# (cadera, rodilla) en grados, solo para legibilidad
MOVEMENTS_DEG = [
    (0, 0),
    (30, 45),
    (-20, 90),
    (45, 60),
    (0, 120),
    (0, 0),
]

# Criterios de aceptación (modos smooth)
MAX_FINAL_ERROR_DEG = 2.0
MAX_FINAL_ERROR_GRAVITY_COMP_DEG = 0.2
HOLDING_TORQUE_TOLERANCE = 0.05  # N·m absolutos ...
HOLDING_TORQUE_RELATIVE = 0.05  # ... más un 5 % del valor


@dataclass
class TargetResult:
    target_deg: tuple[float, float]
    peak_torque: np.ndarray  # N·m, |tau| máximo durante transición + sostén
    holding_torque: np.ndarray  # N·m, torque del motor al final del sostén
    gravity_torque: np.ndarray  # N·m, qfrc_bias al final del sostén
    final_error_deg: np.ndarray  # grados


@dataclass
class SequenceResult:
    mode: str
    targets: list[TargetResult]
    peak_torque: np.ndarray
    saturation_fraction: np.ndarray
    force_limit: np.ndarray


def run_sequence(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mode: str,
    gravity_comp: bool = False,
    viewer=None,
) -> SequenceResult:
    actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
    dof_ids = [
        int(model.jnt_dofadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in JOINTS
    ]
    force_limit = model.actuator_forcerange[actuator_ids, 1].copy()
    compensator = GravityCompensator(model, actuator_ids, dof_ids) if gravity_comp else None

    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    targets_rad = np.radians(np.array(MOVEMENTS_DEG, dtype=float))
    timestep = model.opt.timestep
    steps_per_target = round((TRANSITION_TIME + HOLD_TIME) / timestep)

    results: list[TargetResult] = []
    global_peak = np.zeros(len(JOINTS))
    saturated_steps = np.zeros(len(JOINTS))
    total_steps = 0

    previous = targets_rad[0]
    for target_deg, target in zip(MOVEMENTS_DEG[1:], targets_rad[1:]):
        peak = np.zeros(len(JOINTS))

        for step in range(steps_per_target):
            step_start = time.monotonic()

            ctrl = interpolate(previous, target, step * timestep, mode)
            if compensator is not None:
                ctrl = ctrl + compensator.offset(data.qpos)
            data.ctrl[actuator_ids] = ctrl
            mujoco.mj_step(model, data)

            torque = np.abs(read_sensors(data, TORQUE_SENSORS))
            peak = np.maximum(peak, torque)
            saturated_steps += torque >= force_limit - 1e-6
            total_steps += 1

            if viewer is not None:
                if not viewer.is_running():
                    raise KeyboardInterrupt
                viewer.sync()
                remaining = timestep - (time.monotonic() - step_start)
                if remaining > 0:
                    time.sleep(remaining)

        position = read_sensors(data, POSITION_SENSORS)
        results.append(
            TargetResult(
                target_deg=target_deg,
                peak_torque=peak,
                holding_torque=read_sensors(data, TORQUE_SENSORS),
                gravity_torque=data.qfrc_bias[dof_ids].copy(),
                final_error_deg=np.degrees(np.abs(target - position)),
            )
        )
        global_peak = np.maximum(global_peak, peak)
        previous = target

    return SequenceResult(
        mode=f"{mode} + gravedad" if gravity_comp else mode,
        targets=results,
        peak_torque=global_peak,
        saturation_fraction=saturated_steps / total_steps,
        force_limit=force_limit,
    )


def print_report(result: SequenceResult) -> None:
    print(f"\n=== Modo: {result.mode} ===")
    print(
        "Objetivo (cad, rod) | tau pico [N·m]  | tau sostén [N·m] "
        "| tau gravedad [N·m] | error final [°]"
    )
    for item in result.targets:
        hip, knee = item.target_deg
        print(
            f"  ({hip:4}, {knee:4})     "
            f"| {item.peak_torque[0]:6.2f} {item.peak_torque[1]:6.2f}   "
            f"| {item.holding_torque[0]:6.2f} {item.holding_torque[1]:6.2f}    "
            f"| {item.gravity_torque[0]:6.2f} {item.gravity_torque[1]:6.2f}      "
            f"| {item.final_error_deg[0]:5.2f} {item.final_error_deg[1]:5.2f}"
        )
    for i, joint in enumerate(JOINTS):
        print(
            f"  {joint:10s}: pico {result.peak_torque[i]:6.2f} N·m "
            f"(límite {result.force_limit[i]:.0f}), "
            f"saturado {100 * result.saturation_fraction[i]:5.1f} % del tiempo"
        )


def check_acceptance(result: SequenceResult, max_error_deg: float) -> list[str]:
    """Devuelve la lista de criterios incumplidos (vacía = PASS)."""
    failures: list[str] = []

    if np.any(result.saturation_fraction > 0):
        failures.append("hay saturación de torque")

    for item in result.targets:
        if np.any(item.final_error_deg > max_error_deg):
            failures.append(f"error final > {max_error_deg}° en {item.target_deg}")

        tolerance = HOLDING_TORQUE_TOLERANCE + HOLDING_TORQUE_RELATIVE * np.abs(item.gravity_torque)
        if np.any(np.abs(item.holding_torque - item.gravity_torque) > tolerance):
            failures.append(f"torque de sostén != gravedad en {item.target_deg}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Análisis de esfuerzos de la pierna 2 GDL")
    parser.add_argument(
        "--view",
        action="store_true",
        help="ver el modo smooth + gravedad (requiere mjpython)",
    )
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    step_result = run_sequence(model, data, "step")
    smooth_result = run_sequence(model, data, "smooth")
    gravity_result = run_sequence(model, data, "smooth", gravity_comp=True)

    print_report(step_result)
    print_report(smooth_result)
    print_report(gravity_result)

    any_failure = False
    for result, max_error in (
        (smooth_result, MAX_FINAL_ERROR_DEG),
        (gravity_result, MAX_FINAL_ERROR_GRAVITY_COMP_DEG),
    ):
        failures = check_acceptance(result, max_error)
        print(f"\n=== Criterios de aceptación (modo {result.mode}) ===")
        if failures:
            any_failure = True
            for failure in failures:
                print(f"  FAIL: {failure}")
        else:
            print(f"  PASS: sin saturación, error final < {max_error}°, sostén = gravedad")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_sequence(model, data, "smooth", gravity_comp=True, viewer=viewer)
        except KeyboardInterrupt:
            pass

    return 1 if any_failure else 0


if __name__ == "__main__":
    sys.exit(main())
