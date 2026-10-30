"""
H1 - Recuperar el equilibrio con un paso (experimento 14).

Un empujón en la pelvis mueve el punto de captura. Si sale de lo que los pies
pueden frenar, el robot da un paso rápido: predice dónde estará el punto de
captura al aterrizar, pone ahí el pie y lleva la pelvis al centro de los
nuevos apoyos. Todo se planifica desde el estado medido en ese instante.

Busca por bisección el mayor empujón que aguanta en cada dirección.

Uso:
  python simulation/scripts/recover_biped.py           # análisis, sin ventana
  mjpython simulation/scripts/recover_biped.py --view  # además, ver un empujón
"""

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from biped_control import FOOT_HALF, SIDES, Biped, Pose, center_com_over, pelvis_height


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "biped_6dof.xml"

STAND_KNEE_DEG = 56.0
FOOT_Y = 0.07  # m, distancia de cada pie al centro

PUSH_TIME = 0.1  # s
SETTLE_TIME = 1.0  # s, antes del empujón
AFTER_TIME = 4.0  # s, tras el empujón, para juzgar si se ha recuperado

TRIGGER_MARGIN = 0.02  # m, el paso se dispara a esta distancia del borde de los apoyos
REPLAN_FRACTION = 0.7  # hasta esta fracción del vuelo se recalcula el aterrizaje
SWING_TIME = 0.18  # s, duración del paso: el error crece como exp(omega * T)
LIFT_HEIGHT = 0.04  # m
MAX_STEP = np.array([0.26, 0.38])  # m, en x e y, respecto al pie de apoyo
RECENTER_TIME = 0.8  # s, pelvis hasta el centro de los nuevos apoyos

# Criterios de aceptación: más que el límite físico sin dar un paso
REQUIRED_FORCE = {"adelante": 30.0, "atrás": 30.0, "izquierda": 36.0, "derecha": 36.0}
DIRECTIONS = {
    "adelante": np.array([1.0, 0.0, 0.0]),
    "atrás": np.array([-1.0, 0.0, 0.0]),
    "izquierda": np.array([0.0, 1.0, 0.0]),
    "derecha": np.array([0.0, -1.0, 0.0]),
}
MAX_FINAL_TILT_DEG = 2.0


@dataclass
class Trial:
    survived: bool
    peak_tilt_deg: float
    step_vector: np.ndarray  # m, cuánto se movió el pie que más se movió


def hermite(start: np.ndarray, velocity: np.ndarray, end: np.ndarray, duration: float, t: float) -> np.ndarray:
    """De (start, velocity) a (end, velocidad cero) en 'duration' segundos."""
    s = min(1.0, t / duration)
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    return h00 * start + h10 * duration * velocity + h01 * end


class RecoveryStep:
    """Plan del paso, desde el estado medido al dispararse."""

    def __init__(self, biped: Biped, pose: Pose):
        self.pose = pose
        centre = 0.5 * (pose.left_sole[:2] + pose.right_sole[:2])
        away = biped.capture_point() - centre
        self.swing = "left" if away[1] > 0 else "right"
        self.stance = SIDES[1 - SIDES.index(self.swing)]
        self.start_sole = pose.sole(self.swing).copy()
        self.pelvis_start = biped.data.xpos[biped.pelvis_id].copy()
        self.pelvis_start[2] = pose.pelvis[2]
        self.pelvis_velocity = biped.com_velocity()
        self.pelvis_velocity[2] = 0.0
        self.replan(biped, 0.0)

    def replan(self, biped: Biped, t: float) -> None:
        """Aterrizaje = punto de captura previsto al final del vuelo."""
        stance_xy = self.pose.sole(self.stance)[:2]
        remaining = max(0.0, SWING_TIME - t)
        predicted = stance_xy + (biped.capture_point() - stance_xy) * math.exp(biped.omega() * remaining)
        step = np.clip(predicted - stance_xy, -MAX_STEP, MAX_STEP)
        side = 1.0 if self.swing == "left" else -1.0
        step[1] = side * max(side * step[1], 2 * FOOT_Y)  # el pie libre no cruza al de apoyo
        self.landing = np.array([*(stance_xy + step), 0.0])
        self.pelvis_end = np.array([*(0.5 * (stance_xy + self.landing[:2])), self.pose.pelvis[2]])

    def at(self, t: float) -> tuple[Pose, str | None]:
        pelvis = hermite(self.pelvis_start, self.pelvis_velocity, self.pelvis_end,
                         SWING_TIME + RECENTER_TIME, t)
        s = min(1.0, t / SWING_TIME)
        blend = 0.5 * (1.0 - math.cos(math.pi * s))
        sole = self.start_sole + (self.landing - self.start_sole) * blend
        sole[2] = LIFT_HEIGHT * math.sin(math.pi * s)
        pose = replace(self.pose, pelvis=pelvis).with_foot(self.swing, sole)
        return pose, (self.swing if t < SWING_TIME else None)


def standing_pose(biped: Biped) -> Pose:
    """De pie con el CdM centrado entre los pies: los muslos adelantan masa."""
    pose = Pose(
        np.array([0.0, 0.0, pelvis_height(STAND_KNEE_DEG)]),
        np.array([0.0, FOOT_Y, 0.0]),
        np.array([0.0, -FOOT_Y, 0.0]),
    )
    return center_com_over(biped, pose, "left", inset=FOOT_Y)


def outside_support(point: np.ndarray, centre: np.ndarray) -> bool:
    """¿Está el punto cerca del borde de los dos pies, o fuera?"""
    half = FOOT_HALF + [0.0, FOOT_Y]
    return bool(np.any(np.abs(point - centre) > half - TRIGGER_MARGIN))


def run_push(model, data, direction: np.ndarray, force: float, step_enabled=True, viewer=None) -> Trial:
    mujoco.mj_resetData(model, data)
    biped = Biped(model, data, viewer, measured_swing=True)
    stand = standing_pose(biped)
    biped.set_pose(data, stand)
    timestep = model.opt.timestep
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, None)

    centre = 0.5 * (stand.left_sole[:2] + stand.right_sole[:2])
    recovery = None
    trigger_time = 0.0
    peak = 0.0
    start = biped.soles()
    try:
        for i in range(round((PUSH_TIME + AFTER_TIME) / timestep)):
            t = i * timestep
            data.xfrc_applied[biped.pelvis_id, :3] = direction * force if t < PUSH_TIME else 0.0

            if step_enabled and recovery is None:
                if outside_support(biped.capture_point(), centre):
                    recovery = RecoveryStep(biped, stand)
                    trigger_time = t
                    biped.history = []

            if recovery is None:
                biped.step(stand, None)
            else:
                if t - trigger_time < REPLAN_FRACTION * SWING_TIME:
                    recovery.replan(biped, t - trigger_time)
                pose, swing = recovery.at(t - trigger_time)
                biped.step(pose, swing)
            peak = max(peak, biped.tilt_deg())
    except RuntimeError:
        return Trial(False, peak, np.zeros(2))

    moved = (biped.soles() - start)[:, :2]
    step_vector = moved[np.argmax(np.linalg.norm(moved, axis=1))]
    survived = biped.tilt_deg() < MAX_FINAL_TILT_DEG
    return Trial(survived, peak, step_vector)


def strongest_push(model, data, direction: np.ndarray, step_enabled=True) -> tuple[float, Trial]:
    """
    Mayor fuerza que aguanta subiendo de 2 en 2 N desde cero. No se usa
    bisección: con paso, aguantar no es monótono con la fuerza.
    """
    best = 0.0
    last = run_push(model, data, direction, 0.0, step_enabled)
    for force in np.arange(2.0, 82.0, 2.0):
        trial = run_push(model, data, direction, float(force), step_enabled)
        if not trial.survived:
            break
        best, last = float(force), trial
    return best, last


def main() -> int:
    parser = argparse.ArgumentParser(description="Recuperar el equilibrio con un paso")
    parser.add_argument("--view", action="store_true", help="verlo (requiere mjpython)")
    parser.add_argument("--force", type=float, default=30.0, help="empujón a ver con --view [N]")
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name} | empujón de {PUSH_TIME} s")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print("\nDirección  | sin paso | con paso | exigido | paso dado [cm]  | incl. pico")
    failures = []
    for name, direction in DIRECTIONS.items():
        without, _ = strongest_push(model, data, direction, step_enabled=False)
        best, trial = strongest_push(model, data, direction)
        print(f"  {name:9s}| {without:5.1f} N  | {best:5.1f} N  | {REQUIRED_FORCE[name]:4.0f} N  "
              f"| ({100 * trial.step_vector[0]:5.1f}, {100 * trial.step_vector[1]:5.1f})  "
              f"| {trial.peak_tilt_deg:5.1f}°")
        if best < REQUIRED_FORCE[name]:
            failures.append(f"{name}: aguanta {best:.0f} N, se exigen {REQUIRED_FORCE[name]:.0f} N")

    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: aguanta más que el límite físico sin dar un paso, en las cuatro direcciones")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_push(model, data, DIRECTIONS["adelante"], args.force, viewer=viewer)
        except KeyboardInterrupt:
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
