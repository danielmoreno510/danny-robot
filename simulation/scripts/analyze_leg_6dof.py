"""
Danny - Análisis de la pierna de 6 GDL con pie plano (experimento 9).

Dos pruebas con el mismo modelo:
  - aire:  la pelvis se sujeta alta (restricción hold_pelvis) y se mueve
           cada articulación por separado, con GravityCompensator.
           Comprueba que cada eje gira en el sentido documentado.
  - apoyo: la pelvis se suelta, el pie plano se apoya y se hace la misma
           sentadilla que en el experimento 8, con StanceCompensator.
           Comprueba además que el pie se queda plano.

Uso:
  python simulation/scripts/analyze_leg_6dof.py           # análisis, sin ventana
  mjpython simulation/scripts/analyze_leg_6dof.py --view  # además, verlo
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import mujoco
import mujoco.viewer
import numpy as np

from leg_control import (
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
MODEL_PATH = ROOT / "models" / "leg_6dof.xml"

JOINT_PARTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
JOINTS = tuple(f"{p}_joint" for p in JOINT_PARTS)
ACTUATORS = tuple(f"{p}_motor" for p in JOINT_PARTS)
POSITION_SENSORS = tuple(f"{p}_pos" for p in JOINT_PARTS)
TORQUE_SENSORS = tuple(f"{p}_torque" for p in JOINT_PARTS)

LEG_LENGTH = 0.5  # m, muslo + tibia
HIP_BELOW_PELVIS = 0.05  # m
SOLE_BELOW_ANKLE = 0.04  # m
PELVIS_BODY_Z = 0.6  # m, pos del body pelvis; qpos del slide es relativo a esto
HOLD_PELVIS_SLIDE = 0.2  # m, el polycoef de hold_pelvis

SETTLE_TIME = 1.0  # s

# Prueba en el aire: (articulación, objetivos en grados). Cada articulación
# va a +a, a -a y vuelve a 0 con las demás a cero.
AIR_MOVES_DEG = [
    ("hip_yaw_joint", (30, -30, 0)),
    ("hip_roll_joint", (25, -25, 0)),
    ("hip_pitch_joint", (45, -45, 0)),
    ("knee_joint", (60, 120, 0)),
    ("ankle_pitch_joint", (30, -45, 0)),
    ("ankle_roll_joint", (25, -25, 0)),
]

# Sentido esperado del primer objetivo (positivo) de cada articulación.
# p = posición del centro de la planta, R = orientación del pie (columnas =
# ejes x, y, z del pie en el mundo). Con todo a cero, p = (0, 0, z) y R = I.
SignCheck = Callable[[np.ndarray, np.ndarray], bool]
SIGN_CHECKS: dict[str, tuple[str, SignCheck]] = {
    "hip_yaw_joint": ("la punta del pie gira a +y", lambda p, R: R[1, 0] > 0),
    "hip_roll_joint": ("el pie se va a +y", lambda p, R: p[1] > 0),
    "hip_pitch_joint": ("el pie se va a -x", lambda p, R: p[0] < 0),
    "knee_joint": ("el pie se va a -x", lambda p, R: p[0] < 0),
    "ankle_pitch_joint": ("la punta baja", lambda p, R: R[2, 0] < 0),
    "ankle_roll_joint": ("el borde +y del pie sube", lambda p, R: R[2, 1] > 0),
}

# Criterios de aceptación
MAX_AIR_ERROR_DEG = 0.2
FOOT_FORCE_RELATIVE = 0.05
MAX_FOOT_SLIP = 0.005  # m
MAX_HEIGHT_ERROR = 0.01  # m
MAX_FOOT_TILT_DEG = 1.0


@dataclass
class AirResult:
    joint: str
    target_deg: float
    peak_torque: float  # N·m, de la articulación que se mueve
    max_error_deg: float  # de todas las articulaciones, al final del sostén
    sign_ok: bool | None  # None si no es el primer objetivo


@dataclass
class StanceResult:
    knee_deg: float
    pelvis_height: float
    expected_height: float
    peak_torque: np.ndarray  # N·m, las 6 articulaciones
    max_error_deg: float
    foot_force: float
    foot_slip: float  # m, desplazamiento horizontal de la planta
    foot_tilt_deg: float


@dataclass
class Run:
    air: list[AirResult]
    stance: list[StanceResult]
    saturation_fraction: np.ndarray  # por articulación, las dos pruebas


def squat_targets(knee_deg: float) -> np.ndarray:
    """Las 6 articulaciones con el pie plano y bajo la cadera."""
    knee = math.radians(knee_deg)
    return np.array([0.0, 0.0, -knee / 2, knee, -knee / 2, 0.0])


def pelvis_height(knee_deg: float) -> float:
    half_knee = math.radians(knee_deg) / 2
    return SOLE_BELOW_ANKLE + LEG_LENGTH * math.cos(half_knee) + HIP_BELOW_PELVIS


class Simulation:
    """Modelo + estado + paso con compensación, compartido por las dos pruebas."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, viewer=None):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
        self.dof_ids = [
            int(model.jnt_dofadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in JOINTS
        ]
        self.joint_qpos = [
            int(model.jnt_qposadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in JOINTS
        ]
        self.slide_qpos = int(
            model.jnt_qposadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, "pelvis_slide")]
        )
        self.hold_id = get_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_pelvis")
        self.sole_id = get_id(model, mujoco.mjtObj.mjOBJ_SITE, "sole_site")
        self.force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        self.compensator = None
        self.saturated_steps = np.zeros(len(JOINTS))
        self.total_steps = 0

    def reset(self, pose: np.ndarray, slide: float, hold_pelvis: bool, compensator) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.eq_active[self.hold_id] = hold_pelvis
        self.data.qpos[self.slide_qpos] = slide
        self.data.qpos[self.joint_qpos] = pose
        mujoco.mj_forward(self.model, self.data)
        self.compensator = compensator

    def step(self, target: np.ndarray) -> np.ndarray:
        step_start = time.monotonic()
        ctrl = target + self.compensator.offset(self.data.qpos)
        self.data.ctrl[self.actuator_ids] = ctrl
        mujoco.mj_step(self.model, self.data)

        torque = np.abs(read_sensors(self.data, TORQUE_SENSORS))
        self.saturated_steps += torque >= self.force_limit - 1e-6
        self.total_steps += 1

        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()
            remaining = self.model.opt.timestep - (time.monotonic() - step_start)
            if remaining > 0:
                time.sleep(remaining)
        return torque

    def move(self, previous: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Transición suave + sostén. Devuelve el torque pico por articulación."""
        timestep = self.model.opt.timestep
        peak = np.zeros(len(JOINTS))
        for i in range(round((TRANSITION_TIME + HOLD_TIME) / timestep)):
            peak = np.maximum(peak, self.step(interpolate(previous, target, i * timestep, "smooth")))
        return peak

    def hold(self, target: np.ndarray, duration: float) -> None:
        for _ in range(round(duration / self.model.opt.timestep)):
            self.step(target)

    def joint_error_deg(self, target: np.ndarray) -> float:
        return float(np.degrees(np.abs(target - read_sensors(self.data, POSITION_SENSORS))).max())

    def sole(self) -> tuple[np.ndarray, np.ndarray]:
        return (
            self.data.site_xpos[self.sole_id].copy(),
            self.data.site_xmat[self.sole_id].reshape(3, 3).copy(),
        )


def run_air(sim: Simulation) -> list[AirResult]:
    zero = np.zeros(len(JOINTS))
    sim.reset(zero, HOLD_PELVIS_SLIDE, hold_pelvis=True,
              compensator=GravityCompensator(sim.model, sim.actuator_ids, sim.dof_ids))
    sim.hold(zero, SETTLE_TIME)
    sole_zero, _ = sim.sole()

    results: list[AirResult] = []
    previous = zero
    for joint, targets_deg in AIR_MOVES_DEG:
        index = JOINTS.index(joint)
        for n, target_deg in enumerate(targets_deg):
            target = zero.copy()
            target[index] = math.radians(target_deg)
            peak = sim.move(previous, target)

            sign_ok = None
            if n == 0:
                position, orientation = sim.sole()
                _, check = SIGN_CHECKS[joint]
                sign_ok = check(position - sole_zero, orientation)

            results.append(
                AirResult(
                    joint=joint,
                    target_deg=target_deg,
                    peak_torque=float(peak[index]),
                    max_error_deg=sim.joint_error_deg(target),
                    sign_ok=sign_ok,
                )
            )
            previous = target
    return results


def run_stance(sim: Simulation) -> list[StanceResult]:
    foot_id = get_id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "foot")
    start = squat_targets(SQUAT_KNEE_DEG[0])
    sim.reset(start, pelvis_height(SQUAT_KNEE_DEG[0]) - PELVIS_BODY_Z, hold_pelvis=False,
              compensator=StanceCompensator(sim.model, sim.actuator_ids, sim.dof_ids, [foot_id]))
    sim.hold(start, SETTLE_TIME)
    sole_start, _ = sim.sole()

    results: list[StanceResult] = []
    previous = start
    for knee_deg in SQUAT_KNEE_DEG[1:]:
        target = squat_targets(knee_deg)
        peak = sim.move(previous, target)
        position, orientation = sim.sole()
        results.append(
            StanceResult(
                knee_deg=knee_deg,
                pelvis_height=PELVIS_BODY_Z + sim.data.sensor("pelvis_height").data[0],
                expected_height=pelvis_height(knee_deg),
                peak_torque=peak,
                max_error_deg=sim.joint_error_deg(target),
                foot_force=sim.data.sensor("foot_force").data[0],
                foot_slip=float(np.linalg.norm((position - sole_start)[:2])),
                foot_tilt_deg=math.degrees(math.acos(min(1.0, orientation[2, 2]))),
            )
        )
        previous = target
    return results


def run_all(model: mujoco.MjModel, data: mujoco.MjData, viewer=None) -> Run:
    sim = Simulation(model, data, viewer)
    air = run_air(sim)
    stance = run_stance(sim)
    return Run(air=air, stance=stance, saturation_fraction=sim.saturated_steps / sim.total_steps)


def print_report(run: Run) -> None:
    print("\n=== Aire: cada articulación por separado (GravityCompensator) ===")
    print("Articulación        objetivo | tau pico [N·m] | error máx [°] | sentido")
    for r in run.air:
        sign = "" if r.sign_ok is None else ("OK" if r.sign_ok else "MAL")
        if r.sign_ok is not None:
            sign += f" ({SIGN_CHECKS[r.joint][0]})"
        print(f"  {r.joint:18s} {r.target_deg:5}° | {r.peak_torque:6.2f}         "
              f"| {r.max_error_deg:6.3f}        | {sign}")

    print("\n=== Apoyo: sentadilla con pie plano (StanceCompensator) ===")
    print("Rodilla | altura pelvis [m] | tau pico cad.pitch rod tob.pitch [N·m] "
          "| error [°] | pie [N] | desliz. [mm] | incl. pie [°]")
    for r in run.stance:
        print(f"  {r.knee_deg:4}° | {r.pelvis_height:.4f} / {r.expected_height:.4f} "
              f"| {r.peak_torque[2]:5.2f}  {r.peak_torque[3]:5.2f}  {r.peak_torque[4]:5.2f}"
              f"                 | {r.max_error_deg:6.3f}    | {r.foot_force:6.2f}  "
              f"| {1000 * r.foot_slip:5.2f}        | {r.foot_tilt_deg:5.3f}")

    print()
    for i, joint in enumerate(JOINTS):
        print(f"  {joint:18s}: saturado {100 * run.saturation_fraction[i]:5.1f} % del tiempo")


def check_acceptance(run: Run, weight: float) -> list[str]:
    failures: list[str] = []
    if np.any(run.saturation_fraction > 0):
        failures.append("hay saturación de torque")

    for r in run.air:
        if r.max_error_deg > MAX_AIR_ERROR_DEG:
            failures.append(f"aire: error > {MAX_AIR_ERROR_DEG}° con {r.joint} en {r.target_deg}°")
        if r.sign_ok is False:
            failures.append(f"aire: {r.joint} no gira como dice el modelo")

    for r in run.stance:
        if abs(r.foot_force - weight) > FOOT_FORCE_RELATIVE * weight:
            failures.append(f"apoyo: fuerza en el pie {r.foot_force:.2f} N != peso en {r.knee_deg}°")
        if r.foot_slip > MAX_FOOT_SLIP:
            failures.append(f"apoyo: el pie desliza {1000 * r.foot_slip:.1f} mm en {r.knee_deg}°")
        if abs(r.pelvis_height - r.expected_height) > MAX_HEIGHT_ERROR:
            failures.append(f"apoyo: altura de pelvis fuera de ±1 cm en {r.knee_deg}°")
        if r.foot_tilt_deg > MAX_FOOT_TILT_DEG:
            failures.append(f"apoyo: pie inclinado {r.foot_tilt_deg:.2f}° en {r.knee_deg}°")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Pierna de 6 GDL: aire y apoyo")
    parser.add_argument("--view", action="store_true", help="verlo (requiere mjpython)")
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    weight = total_weight(model)
    print(f"Peso total sobre el pie: {weight:.2f} N")

    run = run_all(model, data)
    print_report(run)

    failures = check_acceptance(run, weight)
    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: aire (error < 0,2°, sentidos correctos) y apoyo "
              "(pie = peso, sin deslizar, pie plano, altura ± 1 cm), sin saturación")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_all(model, data, viewer=viewer)
        except KeyboardInterrupt:
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
