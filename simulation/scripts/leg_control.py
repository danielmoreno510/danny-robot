"""
H1 - Utilidades de control compartidas por los scripts de la pierna.

  - get_id, read_sensors: buscar elementos del modelo por nombre y leer sensores.
  - interpolate: perfil coseno entre dos objetivos articulares (etapa 5).
  - GravityCompensator: feedforward de gravedad con la pierna en el aire (etapa 7).
  - StanceCompensator: feedforward con el pie apoyado en el suelo (etapa 8).
  - SQUAT_KNEE_DEG: la sentadilla de las etapas 8 y 9.
"""

import math

import mujoco
import numpy as np


TRANSITION_TIME = 1.5  # s, duración de la interpolación en modo smooth
HOLD_TIME = 1.5  # s, tiempo quieto en cada objetivo antes de medir

# Flexión de rodilla en grados para la sentadilla (etapas 8 y 9).
SQUAT_KNEE_DEG = [20, 60, 90, 120, 20]


def get_id(model: mujoco.MjModel, obj: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, obj, name)
    if object_id < 0:
        raise ValueError(f"No existe '{name}' en el modelo")
    return object_id


def read_sensors(data: mujoco.MjData, names: tuple[str, ...]) -> np.ndarray:
    return np.array([data.sensor(name).data[0] for name in names])


class GravityCompensator:
    """
    Feedforward de gravedad para motores de posición.

    Un motor de posición da tau = kp * (ctrl - q). Para sostener la pierna
    necesita tau = tau_g(q), así que sin ayuda se queda tau_g / kp por debajo
    del objetivo. Si se adelanta el objetivo justo esa cantidad, el motor da
    tau_g con error cero:

        ctrl = objetivo + tau_g(q) / kp

    tau_g se calcula con el modelo en la postura medida (lo que daría el
    encoder) y velocidad cero, para no meter Coriolis ni centrífuga.
    """

    def __init__(self, model: mujoco.MjModel, actuator_ids: list[int], dof_ids: list[int]):
        self.model = model
        self.scratch = mujoco.MjData(model)
        self.dof_ids = dof_ids
        self.kp = model.actuator_gainprm[actuator_ids, 0].copy()

    def gravity_torque(self, qpos: np.ndarray) -> np.ndarray:
        self.scratch.qpos[:] = qpos
        self.scratch.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.scratch)
        return self.scratch.qfrc_bias[self.dof_ids].copy()

    def offset(self, qpos: np.ndarray) -> np.ndarray:
        """Desplazamiento del objetivo [rad] por articulación."""
        return self.gravity_torque(qpos) / self.kp


def interpolate(
    start: np.ndarray,
    end: np.ndarray,
    elapsed: float,
    mode: str,
) -> np.ndarray:
    """Objetivo articular en el instante 'elapsed' de la transición."""
    if mode == "step" or elapsed >= TRANSITION_TIME:
        return end

    # Perfil coseno: velocidad cero al inicio y al final,
    # evita el pico de torque kp * error del salto brusco.
    progress = elapsed / TRANSITION_TIME
    blend = 0.5 * (1.0 - math.cos(math.pi * progress))
    return start + (end - start) * blend


def total_weight(model: mujoco.MjModel) -> float:
    """Peso de todo lo que se apoya en el pie [N]."""
    pelvis_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    return float(model.body_subtreemass[pelvis_id]) * -model.opt.gravity[2]


class StanceCompensator:
    """
    Feedforward para la pierna apoyada en el suelo.

    En apoyo el motor no solo sostiene la pierna: el suelo empuja el pie
    hacia arriba con todo el peso, y ese empuje hay que llevarlo hasta la
    pelvis. En equilibrio estático:

        tau = tau_g(q) - J_pie(q)^T * F_suelo,   F_suelo = (0, 0, peso)

    J_pie es el jacobiano de la posición del pie: traduce una fuerza en el
    pie a torques en cada articulación. Se pide que F_suelo sea vertical,
    sin componente horizontal, para que el pie no empuje de lado.

    Como en GravityCompensator, el objetivo se adelanta tau / kp.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        actuator_ids: list[int],
        dof_ids: list[int],
        foot_id: int,
    ):
        self.model = model
        self.scratch = mujoco.MjData(model)
        self.dof_ids = dof_ids
        self.foot_id = foot_id
        self.kp = model.actuator_gainprm[actuator_ids, 0].copy()
        self.ground_force = np.array([0.0, 0.0, total_weight(model)])
        self.jacobian = np.zeros((3, model.nv))

    def torque(self, qpos: np.ndarray) -> np.ndarray:
        self.scratch.qpos[:] = qpos
        self.scratch.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.scratch)
        mujoco.mj_jacBody(self.model, self.scratch, self.jacobian, None, self.foot_id)
        tau = self.scratch.qfrc_bias - self.jacobian.T @ self.ground_force
        return tau[self.dof_ids]

    def offset(self, qpos: np.ndarray) -> np.ndarray:
        """Desplazamiento del objetivo [rad] por articulación."""
        return self.torque(qpos) / self.kp
