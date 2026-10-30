"""
Danny - Utilidades de control compartidas por los scripts de la pierna.

  - get_id, read_sensors: buscar elementos del modelo por nombre y leer sensores.
  - interpolate: perfil coseno entre dos objetivos articulares (experimento 5).
  - GravityCompensator: feedforward de gravedad con la pierna en el aire (experimento 7).
  - StanceCompensator: feedforward con uno o dos pies apoyados (experimentos 8-10).
  - leg_ik: cinemática inversa de una pierna de 6 GDL con el pie plano (experimento 10).
  - check_joint_limits: informa de las articulaciones fuera de rango.
  - SQUAT_KNEE_DEG: la sentadilla de los experimentos 8 y 9.
"""

import math

import mujoco
import numpy as np


TRANSITION_TIME = 1.5  # s, duración de la interpolación en modo smooth
HOLD_TIME = 1.5  # s, tiempo quieto en cada objetivo antes de medir

LENGTH_TOLERANCE = 1e-9  # m
ANGLE_TOLERANCE = 1e-9  # rad

# Nombres de la pierna de 2 GDL (experimentos 4 a 8)
LEG_2DOF_JOINTS = ("hip_joint", "knee_joint")
LEG_2DOF_ACTUATORS = ("hip_motor", "knee_motor")
LEG_2DOF_POSITION_SENSORS = ("hip_pos", "knee_pos")
LEG_2DOF_TORQUE_SENSORS = ("hip_torque", "knee_torque")

# (cadera, rodilla) en grados, solo para legibilidad
LEG_2DOF_MOVEMENTS_DEG = [
    (0, 0),
    (30, 45),
    (-20, 90),
    (45, 60),
    (0, 120),
    (0, 0),
]

# Flexión de rodilla en grados para la sentadilla (experimentos 8 y 9).
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
    duration: float = TRANSITION_TIME,
) -> np.ndarray:
    """Objetivo en el instante 'elapsed' de una transición de 'duration' s."""
    if mode == "step" or elapsed >= duration:
        return end

    # Perfil coseno: velocidad cero al inicio y al final,
    # evita el pico de torque kp * error del salto brusco.
    # Aceleración pico = (pi^2 / 2) * distancia / duration^2.
    progress = elapsed / duration
    blend = 0.5 * (1.0 - math.cos(math.pi * progress))
    return start + (end - start) * blend


def total_weight(model: mujoco.MjModel) -> float:
    """Peso de todo lo que se apoya en el pie [N]."""
    pelvis_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    return float(model.body_subtreemass[pelvis_id]) * -model.opt.gravity[2]


class StanceCompensator:
    """
    Feedforward para las piernas apoyadas en el suelo.

    En apoyo el motor no solo sostiene la pierna: el suelo empuja cada pie
    hacia arriba con una parte del peso, y ese empuje hay que llevarlo
    hasta la pelvis. En equilibrio estático:

        tau = tau_g(q) - sum_i J_pie_i(q)^T * F_i,   F_i = (0, 0, f_i * peso)

    J_pie_i es el jacobiano de la posición del pie i: traduce una fuerza en
    el pie a torques en cada articulación. f_i es la fracción del peso que
    carga ese pie (suman 1). Se pide F_i vertical, sin componente
    horizontal, para que los pies no empujen de lado.

    Con un solo pie (experimentos 8 y 9), f = 1. Con dos (experimento 10), el script
    reparte el peso según dónde está el centro de masas.

    Dónde se aplica F_i importa. Por defecto, en el tobillo (origen del body
    del pie): vale si el centro de masas está encima. Si no (experimento 10: en
    sentadilla el CdM queda delante de los tobillos), hay que aplicarla en
    el centro de presiones, bajo el CdM; si no, falta el torque de tobillo
    que evita que el cuerpo vuelque y el tobillo cede.

    Como en GravityCompensator, el objetivo se adelanta tau / kp.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        actuator_ids: list[int],
        dof_ids: list[int],
        foot_ids: list[int],
    ):
        self.model = model
        self.scratch = mujoco.MjData(model)
        self.dof_ids = dof_ids
        self.foot_ids = foot_ids
        self.kp = model.actuator_gainprm[actuator_ids, 0].copy()
        self.weight = total_weight(model)
        self.jacobian = np.zeros((3, model.nv))

    def torque(
        self,
        qpos: np.ndarray,
        support: list[float] | None = None,
        points: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        """
        support: fracción del peso en cada pie; None = a partes iguales.
        points: punto del mundo donde actúa la fuerza de cada pie; None = el tobillo.
        """
        if support is None:
            support = [1.0 / len(self.foot_ids)] * len(self.foot_ids)
        self.scratch.qpos[:] = qpos
        self.scratch.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.scratch)
        if points is None:
            points = [self.scratch.xpos[foot_id] for foot_id in self.foot_ids]
        tau = self.scratch.qfrc_bias.copy()
        for foot_id, fraction, point in zip(self.foot_ids, support, points):
            mujoco.mj_jac(self.model, self.scratch, self.jacobian, None, point, foot_id)
            tau -= self.jacobian.T @ np.array([0.0, 0.0, fraction * self.weight])
        return tau[self.dof_ids]

    def offset(
        self,
        qpos: np.ndarray,
        support: list[float] | None = None,
        points: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        """Desplazamiento del objetivo [rad] por articulación."""
        return self.torque(qpos, support, points) / self.kp


class UnreachableTargetError(ValueError):
    """El tobillo pedido está más lejos de lo que alcanza la pierna."""


class JointLimitError(ValueError):
    """La postura pedida saca alguna articulación de su rango mecánico."""


def check_joint_limits(angles: np.ndarray, limits: np.ndarray, names: tuple[str, ...]) -> None:
    """Lanza JointLimitError con todas las articulaciones fuera de rango."""
    outside = [
        f"{name} = {math.degrees(angle):.1f}° (rango {math.degrees(low):.1f}° a {math.degrees(high):.1f}°)"
        for name, angle, (low, high) in zip(names, angles, limits)
        if not low - ANGLE_TOLERANCE <= angle <= high + ANGLE_TOLERANCE
    ]
    if outside:
        raise JointLimitError("; ".join(outside))


def leg_ik(
    ankle_in_hip: np.ndarray,
    thigh_length: float = 0.25,
    shin_length: float = 0.25,
) -> np.ndarray:
    """
    Cinemática inversa de una pierna de 6 GDL con el pie plano y recto.

    ankle_in_hip: posición del tobillo respecto a la cadera [m], en los ejes
    de la pelvis (que se supone horizontal). Devuelve los 6 ángulos [rad] en
    el orden hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll.

      1. hip_roll inclina el plano de la pierna hasta que contiene el tobillo.
      2. En ese plano, cadera + rodilla son un brazo de 2 eslabones
         (ley del coseno).
      3. El tobillo deshace los giros para que la planta quede horizontal:
         ankle_pitch = -(hip_pitch + knee), ankle_roll = -hip_roll.
    El yaw se deja a 0 (pie mirando adelante).

    Lanza UnreachableTargetError si el tobillo está fuera de alcance: nunca
    recorta en silencio. Los rangos articulares los comprueba quien conoce
    el robot, con check_joint_limits.
    """
    x, y, z = ankle_in_hip
    hip_roll = math.atan2(y, -z)
    down = math.hypot(y, z)  # distancia hacia "abajo" dentro del plano de la pierna

    a, b = thigh_length, shin_length
    reach = math.hypot(x, down)
    if reach > a + b + LENGTH_TOLERANCE:
        raise UnreachableTargetError(
            f"tobillo a {100 * reach:.2f} cm de la cadera; la pierna mide {100 * (a + b):.2f} cm"
        )
    reach = min(reach, a + b)  # solo redondeo: lo que sobra es menor que la tolerancia
    cos_inner = (a * a + b * b - reach * reach) / (2 * a * b)
    knee = math.pi - math.acos(max(-1.0, min(1.0, cos_inner)))

    # Ángulo hacia el tobillo menos lo que aporta la rodilla doblada.
    # Signos: hip_pitch + lleva el pie atrás (-x), por eso atan2(-x, ...).
    hip_pitch = math.atan2(-x, down) - math.atan2(b * math.sin(knee), a + b * math.cos(knee))

    return np.array([0.0, hip_roll, hip_pitch, knee, -(hip_pitch + knee), -hip_roll])
