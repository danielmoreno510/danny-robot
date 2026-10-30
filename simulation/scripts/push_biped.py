"""
Danny - Respuesta a empujones sin dar un paso (experimento 14, etapa 19).

El robot está de pie con el CdM centrado entre los pies y recibe un empujón
de 0,1 s en la pelvis. Los motores tienen curva de par-velocidad. Sube la
fuerza de 1 en 1 N hasta que deja de recuperarse, y comprueba que aguanta
los límites definidos en cada dirección.

Uso:
  python simulation/scripts/push_biped.py                     # análisis
  mjpython simulation/scripts/push_biped.py --force 20 --view  # ver un empujón
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from biped_control import Biped, Pose, center_com_over, pelvis_height


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "biped_6dof.xml"

STAND_KNEE_DEG = 56.0
FOOT_Y = 0.07  # m, distancia de cada pie al centro

PUSH_TIME = 0.1  # s
SETTLE_TIME = 1.0  # s, antes del empujón
AFTER_TIME = 4.0  # s, tras el empujón, para juzgar si se ha recuperado
MAX_JOINT_SPEED = 20.0  # rad/s, velocidad de vacío provisional de los motores

DIRECTIONS = {
    "adelante": np.array([1.0, 0.0, 0.0]),
    "atrás": np.array([-1.0, 0.0, 0.0]),
    "izquierda": np.array([0.0, 1.0, 0.0]),
    "derecha": np.array([0.0, -1.0, 0.0]),
}

# Límites definidos [N]: lo medido, redondeado a la baja con 2 N de margen.
# El máximo físico sin dar un paso es de unos 29 N en x y 35 N en y.
REQUIRED_FORCE = {"adelante": 23.0, "atrás": 24.0, "izquierda": 25.0, "derecha": 25.0}

# Se considera recuperado si acaba así
MAX_FINAL_TILT_DEG = 2.0
MAX_FOOT_DRIFT = 0.02  # m


@dataclass
class Trial:
    recovered: bool
    peak_tilt_deg: float
    foot_drift: float  # m, lo que se movió el pie que más se movió


def standing_pose(biped: Biped) -> Pose:
    """De pie con el CdM centrado entre los pies: los muslos adelantan masa."""
    pose = Pose(
        np.array([0.0, 0.0, pelvis_height(STAND_KNEE_DEG)]),
        np.array([0.0, FOOT_Y, 0.0]),
        np.array([0.0, -FOOT_Y, 0.0]),
    )
    return center_com_over(biped, pose, "left", inset=FOOT_Y)


def run_push(model, data, direction: np.ndarray, force: float, viewer=None) -> Trial:
    mujoco.mj_resetData(model, data)
    biped = Biped(model, data, viewer, max_joint_speed=MAX_JOINT_SPEED)
    stand = standing_pose(biped)
    biped.set_pose(data, stand)
    timestep = model.opt.timestep
    for _ in range(round(SETTLE_TIME / timestep)):
        biped.step(stand, None)

    start = biped.soles()
    peak = 0.0
    try:
        for i in range(round((PUSH_TIME + AFTER_TIME) / timestep)):
            data.xfrc_applied[biped.pelvis_id, :3] = direction * force if i * timestep < PUSH_TIME else 0.0
            biped.step(stand, None)
            peak = max(peak, biped.tilt_deg())
    except RuntimeError:
        return Trial(False, peak, float("nan"))

    drift = float(np.max(np.linalg.norm((biped.soles() - start)[:, :2], axis=1)))
    return Trial(biped.tilt_deg() < MAX_FINAL_TILT_DEG and drift < MAX_FOOT_DRIFT, peak, drift)


def strongest_push(model, data, direction: np.ndarray) -> tuple[float, Trial]:
    best, last = 0.0, run_push(model, data, direction, 0.0)
    for force in np.arange(1.0, 61.0):
        trial = run_push(model, data, direction, float(force))
        if not trial.recovered:
            break
        best, last = float(force), trial
    return best, last


def main() -> int:
    parser = argparse.ArgumentParser(description="Respuesta a empujones sin dar un paso")
    parser.add_argument("--view", action="store_true", help="verlo (requiere mjpython)")
    parser.add_argument("--force", type=float, default=20.0, help="empujón a ver con --view [N]")
    args = parser.parse_args()

    print(f"MuJoCo {mujoco.__version__} | modelo: {MODEL_PATH.name} | empujón de {PUSH_TIME} s")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print("\nDirección  | máximo medido | límite definido | incl. pico | pies movidos")
    failures = []
    for name, direction in DIRECTIONS.items():
        best, trial = strongest_push(model, data, direction)
        print(f"  {name:9s}| {best:5.1f} N       | {REQUIRED_FORCE[name]:5.1f} N         "
              f"| {trial.peak_tilt_deg:5.1f}°     | {1000 * trial.foot_drift:5.1f} mm")
        if best < REQUIRED_FORCE[name]:
            failures.append(f"{name}: aguanta {best:.0f} N, límite {REQUIRED_FORCE[name]:.0f} N")

    print("\n=== Criterios de aceptación ===")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}")
    else:
        print("  PASS: se recupera de los empujones definidos en las cuatro direcciones, sin dar un paso")

    if args.view:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_push(model, data, DIRECTIONS["adelante"], args.force, viewer=viewer)
        except KeyboardInterrupt:
            pass

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
