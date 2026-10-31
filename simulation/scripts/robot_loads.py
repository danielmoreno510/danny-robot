"""
Danny - Cargas sobre las articulaciones, compartidas por los estudios de tamaño.

  - support_torques: par para seguir quieto sobre uno o dos pies.
  - froude_scaled: marcha humana escalada con número de Froude constante.
  - fastest_walk: la marcha más rápida que permite una velocidad de motor.
"""

import math

import mujoco
import numpy as np


HUMAN_LEG = 0.90  # m, cadera-suelo de una persona de 1,75 m
HUMAN_SPEED = 1.4  # m/s, de la marcha de HUMAN_GAIT
GRAVITY = 9.81  # m/s²

# Marcha humana a unos 1,4 m/s: par de pico [N·m/kg] y velocidad de pico [rad/s].
# Tobillo y rodilla, contrastados con la literatura en el anexo de componentes: la
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


def froude_scaled(mass: float, leg: float) -> dict:
    """A igual número de Froude: par/kg ∝ longitud y velocidad angular ∝ 1/√longitud."""
    ratio = leg / HUMAN_LEG
    speed_scale = 1 / math.sqrt(ratio)
    return {part: {"par_pico": torque * ratio * mass, "velocidad_pico": speed * speed_scale}
            for part, (torque, speed) in HUMAN_GAIT.items()}


def fastest_walk(motor_speed: float, leg: float) -> dict:
    """
    Marcha que no pide a la rodilla más velocidad que 'motor_speed'. Con la misma forma de
    movimiento, la velocidad angular crece con la velocidad de marcha.
    """
    needed = froude_scaled(1.0, leg)["knee"]["velocidad_pico"]
    factor = min(1.0, motor_speed / needed)
    froude = factor ** 2 * HUMAN_SPEED ** 2 / (GRAVITY * HUMAN_LEG)
    return {"velocidad": math.sqrt(froude * GRAVITY * leg), "froude": froude,
            "humano_equivalente": math.sqrt(froude * GRAVITY * HUMAN_LEG)}
