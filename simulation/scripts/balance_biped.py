"""
H1 - Equilibrio del bípedo: de pie, sentadilla y apoyo en un pie (etapa 10).

La pelvis tiene freejoint: nada la sujeta salvo las piernas. La secuencia
se define en el espacio de tareas (dónde están la pelvis y los pies) y en
cada paso leg_ik calcula los ángulos de cada pierna:

  1. de pie (rodillas a 40°)
  2. sentadilla (rodillas a 80°) y vuelta a de pie
  3. pasar el peso a la izquierda: la pelvis se desplaza en y hasta que el
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
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from leg_control import (
    HOLD_TIME,
    TRANSITION_TIME,
    StanceCompensator,
    get_id,
    interpolate,
    leg_ik,
    read_sensors,
    total_weight,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "biped_6dof.xml"

SIDES = ("left", "right")
JOINT_PARTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
JOINTS = tuple(f"{side}_{p}_joint" for side in SIDES for p in JOINT_PARTS)
ACTUATORS = tuple(f"{side}_{p}_motor" for side in SIDES for p in JOINT_PARTS)
POSITION_SENSORS = tuple(f"{side}_{p}_pos" for side in SIDES for p in JOINT_PARTS)
TORQUE_SENSORS = tuple(f"{side}_{p}_torque" for side in SIDES for p in JOINT_PARTS)

LEG_LENGTH = 0.5  # m, muslo + tibia
HIP_Y = 0.1  # m, caderas en y = ±0.1
HIP_BELOW_PELVIS = 0.05  # m
SOLE_BELOW_ANKLE = 0.04  # m
FOOT_HALF = np.array([0.10, 0.05])  # m, medio largo y medio ancho de la planta

STAND_KNEE_DEG = 40
SQUAT_KNEE_DEG = 80
LIFT_HEIGHT = 0.05  # m, altura a la que sube el pie derecho

SETTLE_TIME = 1.0  # s
FALL_TILT_DEG = 30.0  # se considera caído

# Criterios de aceptación
MAX_TILT_DEG = 5.0
MAX_PELVIS_ERROR = 0.01  # m
MAX_FOOT_SLIP = 0.005  # m
FORCE_RELATIVE = 0.05
MIN_CLEARANCE = 0.04  # m
MIN_COM_MARGIN = 0.02  # m
MAX_JOINT_ERROR_DEG = 0.5


@dataclass
class Pose:
    """Pose en el espacio de tareas. Posiciones en el mundo [m]."""

    pelvis: np.ndarray
    left_sole: np.ndarray
    right_sole: np.ndarray

    def vector(self) -> np.ndarray:
        return np.concatenate([self.pelvis, self.left_sole, self.right_sole])

    @staticmethod
    def from_vector(v: np.ndarray) -> "Pose":
        return Pose(v[0:3], v[3:6], v[6:9])


@dataclass
class Phase:
    name: str
    pose: Pose
    right_swing: bool  # True si el pie derecho no carga peso


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


def pelvis_height(knee_deg: float) -> float:
    half_knee = math.radians(knee_deg) / 2
    return SOLE_BELOW_ANKLE + LEG_LENGTH * math.cos(half_knee) + HIP_BELOW_PELVIS


def joint_targets(pose: Pose) -> np.ndarray:
    """IK de las dos piernas, con la pelvis horizontal y mirando a +x."""
    targets = []
    for side, sole, hip_y in (("left", pose.left_sole, HIP_Y), ("right", pose.right_sole, -HIP_Y)):
        hip = pose.pelvis + np.array([0.0, hip_y, -HIP_BELOW_PELVIS])
        ankle = sole + np.array([0.0, 0.0, SOLE_BELOW_ANKLE])
        targets.append(leg_ik(ankle - hip))
    return np.concatenate(targets)


class Biped:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, viewer=None):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
        joint_ids = [get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        self.dof_ids = [int(model.jnt_dofadr[j]) for j in joint_ids]
        self.joint_qpos = [int(model.jnt_qposadr[j]) for j in joint_ids]
        self.pelvis_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.foot_ids = [get_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot") for s in SIDES]
        self.sole_ids = [get_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{s}_sole_site") for s in SIDES]
        self.force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        self.compensator = StanceCompensator(model, self.actuator_ids, self.dof_ids, self.foot_ids)
        self.saturated_steps = np.zeros(len(JOINTS))
        self.total_steps = 0

    # --- estado ---------------------------------------------------------------

    def set_pose(self, data: mujoco.MjData, pose: Pose) -> None:
        """Pone qpos en la pose (pelvis horizontal) y calcula la cinemática."""
        data.qpos[0:3] = pose.pelvis
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        data.qpos[self.joint_qpos] = joint_targets(pose)
        mujoco.mj_forward(self.model, data)

    def com(self, data: mujoco.MjData | None = None) -> np.ndarray:
        data = self.data if data is None else data
        return data.subtree_com[self.pelvis_id].copy()

    def soles(self) -> np.ndarray:
        return self.data.site_xpos[self.sole_ids].copy()

    def tilt_deg(self) -> float:
        z_axis_z = self.data.xmat[self.pelvis_id].reshape(3, 3)[2, 2]
        return math.degrees(math.acos(min(1.0, z_axis_z)))

    def support(self, right_swing: bool) -> list[float]:
        """Fracción del peso en cada pie según dónde está el CdM."""
        if right_swing:
            return [1.0, 0.0]
        left_y, right_y = self.soles()[:, 1]
        fraction = (self.com()[1] - right_y) / (left_y - right_y)
        fraction = min(1.0, max(0.0, fraction))
        return [fraction, 1.0 - fraction]

    # --- pasos ----------------------------------------------------------------

    def pressure_points(self) -> list[np.ndarray]:
        """Donde actúa la fuerza de cada pie: bajo el CdM en x, en su pie en y."""
        com_x = self.com()[0]
        return [np.array([com_x, sole[1], sole[2]]) for sole in self.soles()]

    def step(self, pose: Pose, right_swing: bool) -> None:
        step_start = time.monotonic()
        ctrl = joint_targets(pose) + self.compensator.offset(
            self.data.qpos, self.support(right_swing), self.pressure_points()
        )
        self.data.ctrl[self.actuator_ids] = ctrl
        mujoco.mj_step(self.model, self.data)

        torque = np.abs(read_sensors(self.data, TORQUE_SENSORS))
        self.saturated_steps += torque >= self.force_limit - 1e-6
        self.total_steps += 1

        if self.tilt_deg() > FALL_TILT_DEG:
            raise RuntimeError(f"el robot se ha caído en t = {self.data.time:.2f} s")

        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()
            remaining = self.model.opt.timestep - (time.monotonic() - step_start)
            if remaining > 0:
                time.sleep(remaining)


def weight_shift_y(biped: Biped, stand: Pose) -> float:
    """
    Desplazamiento lateral de la pelvis [m] que pone el CdM sobre el centro
    del pie izquierdo, con el pie derecho ya levantado. Bisección sobre una
    copia de los datos.
    """
    scratch = mujoco.MjData(biped.model)
    target_y = stand.left_sole[1]

    def com_error(shift: float) -> float:
        pose = Pose(
            stand.pelvis + [0.0, shift, 0.0],
            stand.left_sole,
            stand.right_sole + [0.0, 0.0, LIFT_HEIGHT],
        )
        biped.set_pose(scratch, pose)
        return biped.com(scratch)[1] - target_y

    low, high = 0.0, 2 * HIP_Y
    for _ in range(40):
        middle = 0.5 * (low + high)
        if com_error(middle) < 0:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def build_plan(biped: Biped) -> tuple[Pose, list[Phase]]:
    stand = Pose(
        np.array([0.0, 0.0, pelvis_height(STAND_KNEE_DEG)]),
        np.array([0.0, HIP_Y, 0.0]),
        np.array([0.0, -HIP_Y, 0.0]),
    )
    squat = Pose(stand.pelvis * [1, 1, 0] + [0, 0, pelvis_height(SQUAT_KNEE_DEG)],
                 stand.left_sole, stand.right_sole)
    shift = weight_shift_y(biped, stand)
    over_left = Pose(stand.pelvis + [0.0, shift, 0.0], stand.left_sole, stand.right_sole)
    lifted = Pose(over_left.pelvis, stand.left_sole, stand.right_sole + [0.0, 0.0, LIFT_HEIGHT])

    phases = [
        Phase(f"sentadilla {SQUAT_KNEE_DEG}°", squat, right_swing=False),
        Phase(f"de pie {STAND_KNEE_DEG}°", stand, right_swing=False),
        Phase(f"peso a la izquierda ({100 * shift:.1f} cm)", over_left, right_swing=False),
        Phase(f"pie derecho arriba {100 * LIFT_HEIGHT:.0f} cm", lifted, right_swing=True),
        Phase("pie derecho abajo", over_left, right_swing=True),
        Phase("vuelta al centro", stand, right_swing=False),
    ]
    return stand, phases


def run(model: mujoco.MjModel, data: mujoco.MjData, viewer=None) -> tuple[list[PhaseResult], np.ndarray]:
    biped = Biped(model, data, viewer)
    stand, phases = build_plan(biped)
    timestep = model.opt.timestep

    mujoco.mj_resetData(model, data)
    biped.set_pose(data, stand)
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, right_swing=False)
    soles_start = biped.soles()

    results: list[PhaseResult] = []
    previous = stand
    for phase in phases:
        max_tilt = 0.0
        for i in range(round((TRANSITION_TIME + HOLD_TIME) / timestep)):
            vector = interpolate(previous.vector(), phase.pose.vector(), i * timestep, "smooth")
            biped.step(Pose.from_vector(vector), phase.right_swing)
            max_tilt = max(max_tilt, biped.tilt_deg())

        soles = biped.soles()
        slip = np.linalg.norm((soles - soles_start)[:, :2], axis=1)
        right_in_air = phase.right_swing and phase.pose.right_sole[2] > 0
        if right_in_air:
            slip[1] = np.nan

        com = biped.com()
        support_sole = soles[0] if phase.right_swing else None
        com_margin = (
            float(np.min(FOOT_HALF - np.abs(com[:2] - support_sole[:2])))
            if support_sole is not None else math.nan
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
