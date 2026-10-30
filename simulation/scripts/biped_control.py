"""
Danny - Utilidades del bípedo (danny_biped.xml), compartidas por los experimentos 10 y 11.

  - Pose: dónde están la pelvis y las plantas de los pies (espacio de tareas).
  - joint_targets: IK de las dos piernas para una Pose.
  - leg_reach: cuánto se estira cada pierna en una Pose (1 = recta).
  - Biped: estado del robot, reparto del peso y un paso de simulación con
    StanceCompensator y feedforward de velocidad.
  - center_com_over: mueve la pelvis hasta que el CdM queda sobre un pie.
  - rotation_z: rotación alrededor de la vertical.
"""

import math
import time
from dataclasses import dataclass, replace

import mujoco
import numpy as np

import robot_spec as spec
from leg_control import StanceCompensator, check_joint_limits, get_id, leg_ik, read_sensors
from robot_spec import (
    ACTUATORS,
    HIP_BELOW_PELVIS,
    HIP_Y,
    JOINT_LIMITS,
    JOINTS,
    LEG_LENGTH,
    SHIN_LENGTH,
    SIDES,
    SOLE_BELOW_ANKLE,
    THIGH_LENGTH,
    TORQUE_SENSORS,
)


FOOT_HALF = np.array(spec.FOOT_HALF[:2])  # m, medio largo y medio ancho de la planta
LOADED_FOOT_FORCE = 10.0  # N, a partir de aquí el pie se considera cargado
FALL_TILT_DEG = 30.0  # a partir de aquí se considera caído


@dataclass
class Pose:
    """
    Pose en el espacio de tareas. Posiciones en el mundo [m], giros alrededor
    de la vertical [rad]. La pelvis y los pies están siempre horizontales.
    """

    pelvis: np.ndarray
    left_sole: np.ndarray
    right_sole: np.ndarray
    pelvis_yaw: float = 0.0
    left_yaw: float = 0.0
    right_yaw: float = 0.0

    def vector(self) -> np.ndarray:
        return np.concatenate(
            [self.pelvis, self.left_sole, self.right_sole,
             [self.pelvis_yaw, self.left_yaw, self.right_yaw]]
        )

    @staticmethod
    def from_vector(v: np.ndarray) -> "Pose":
        return Pose(v[0:3], v[3:6], v[6:9], float(v[9]), float(v[10]), float(v[11]))

    def sole(self, side: str) -> np.ndarray:
        return self.left_sole if side == "left" else self.right_sole

    def yaw(self, side: str) -> float:
        return self.left_yaw if side == "left" else self.right_yaw

    def with_foot(self, side: str, sole: np.ndarray, yaw: float | None = None) -> "Pose":
        """La misma pose con el pie 'side' en otro sitio (y otro giro, si se da)."""
        yaw = self.yaw(side) if yaw is None else yaw
        if side == "left":
            return replace(self, left_sole=sole, left_yaw=yaw)
        return replace(self, right_sole=sole, right_yaw=yaw)


@dataclass
class Phase:
    name: str
    pose: Pose
    swing: str | None  # pie que no carga peso ("left" / "right"), o None


def pelvis_height(knee_deg: float) -> float:
    """Altura de la pelvis con los pies bajo las caderas y la rodilla así."""
    half_knee = math.radians(knee_deg) / 2
    return SOLE_BELOW_ANKLE + LEG_LENGTH * math.cos(half_knee) + HIP_BELOW_PELVIS


def rotation_z(yaw: float) -> np.ndarray:
    cos, sin = math.cos(yaw), math.sin(yaw)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def _ankles_in_hips(pose: Pose) -> list[np.ndarray]:
    """Vector cadera -> tobillo de cada pierna, en ejes del mundo."""
    result = []
    for sole, hip_y in ((pose.left_sole, HIP_Y), (pose.right_sole, -HIP_Y)):
        hip = pose.pelvis + rotation_z(pose.pelvis_yaw) @ np.array([0.0, hip_y, -HIP_BELOW_PELVIS])
        ankle = sole + np.array([0.0, 0.0, SOLE_BELOW_ANKLE])
        result.append(ankle - hip)
    return result


def joint_targets(pose: Pose) -> np.ndarray:
    """
    IK de las dos piernas, con la pelvis y los pies horizontales. hip_yaw es
    lo que gira el pie respecto a la pelvis; el resto de la pierna trabaja en
    los ejes del pie, y ahí vale leg_ik. Lanza UnreachableTargetError o
    JointLimitError en vez de pedir una postura imposible.
    """
    targets = []
    for vector, foot_yaw in zip(_ankles_in_hips(pose), (pose.left_yaw, pose.right_yaw)):
        angles = leg_ik(rotation_z(-foot_yaw) @ vector, THIGH_LENGTH, SHIN_LENGTH)
        angles[0] = foot_yaw - pose.pelvis_yaw
        targets.append(angles)
    result = np.concatenate(targets)
    check_joint_limits(result, JOINT_LIMITS, JOINTS)
    return result


def leg_reach(pose: Pose) -> float:
    """Distancia cadera-tobillo / longitud de la pierna, la mayor de las dos."""
    return max(float(np.linalg.norm(v)) for v in _ankles_in_hips(pose)) / LEG_LENGTH


class Biped:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        viewer=None,
        max_joint_speed: float | None = None,
        nominal_model: mujoco.MjModel | None = None,
    ):
        """
        max_joint_speed: velocidad de vacío de los motores [rad/s]; None = sin
        curva de par-velocidad.
        nominal_model: el modelo que cree tener el controlador, si difiere del
        que se simula (auditoría de robustez); None = el mismo.
        """
        self.model = model
        self.data = data
        self.viewer = viewer
        self.max_joint_speed = max_joint_speed
        self.actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
        joint_ids = [get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        self.dof_ids = [int(model.jnt_dofadr[j]) for j in joint_ids]
        self.joint_qpos = [int(model.jnt_qposadr[j]) for j in joint_ids]
        self.pelvis_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.foot_ids = [get_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot") for s in SIDES]
        self.foot_geoms = {get_id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_foot_geom") for s in SIDES}
        self.floor_geom = get_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.self_collided = False
        self.sole_ids = [get_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{s}_sole_site") for s in SIDES]
        self.force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        self.plan_model = model if nominal_model is None else nominal_model
        self.compensator = StanceCompensator(self.plan_model, self.actuator_ids, self.dof_ids, self.foot_ids)
        # Motor de posición: tau = kp (ctrl - q) - kv q'. biasprm = [0, -kp, -kv].
        self.kp = model.actuator_gainprm[self.actuator_ids, 0].copy()
        self.kv = -model.actuator_biasprm[self.actuator_ids, 2].copy()
        self.previous_target: np.ndarray | None = None
        self.saturated_steps = np.zeros(len(JOINTS))
        self.total_steps = 0

    # --- estado ---------------------------------------------------------------

    def set_pose(self, data: mujoco.MjData, pose: Pose) -> None:
        """Pone qpos en la pose (pelvis horizontal) y calcula la cinemática."""
        if data is self.data:
            self.previous_target = None
        data.qpos[0:3] = pose.pelvis
        half_yaw = 0.5 * pose.pelvis_yaw
        data.qpos[3:7] = [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]
        data.qpos[self.joint_qpos] = joint_targets(pose)
        mujoco.mj_forward(self.model if data is self.data else self.plan_model, data)

    def com(self, data: mujoco.MjData | None = None) -> np.ndarray:
        data = self.data if data is None else data
        return data.subtree_com[self.pelvis_id].copy()

    def soles(self) -> np.ndarray:
        """Centro de cada planta (izq, der) en el mundo."""
        return self.data.site_xpos[self.sole_ids].copy()

    def tilt_deg(self) -> float:
        """Inclinación de la pelvis respecto a la vertical."""
        z_axis_z = self.data.xmat[self.pelvis_id].reshape(3, 3)[2, 2]
        return math.degrees(math.acos(min(1.0, z_axis_z)))

    def yaw_deg(self) -> float:
        """Giro de la pelvis alrededor de la vertical."""
        x_axis = self.data.xmat[self.pelvis_id].reshape(3, 3)[:, 0]
        return math.degrees(math.atan2(x_axis[1], x_axis[0]))

    def foot_yaw_deg(self, side: str) -> float:
        """Giro de un pie alrededor de la vertical."""
        x_axis = self.data.xmat[self.foot_ids[SIDES.index(side)]].reshape(3, 3)[:, 0]
        return math.degrees(math.atan2(x_axis[1], x_axis[0]))

    # --- reparto del peso ------------------------------------------------------

    def estimated_com(self) -> np.ndarray:
        """
        CdM que cree el controlador: el del modelo nominal en la postura medida.
        Un robot real solo conoce eso; si su modelo está mal, este dato también.
        """
        estimator = self.compensator.scratch
        estimator.qpos[:] = self.data.qpos
        estimator.qvel[:] = 0.0
        mujoco.mj_forward(self.plan_model, estimator)
        return estimator.subtree_com[self.pelvis_id].copy()

    def has_light_contact(self) -> bool:
        """¿Hay un pie con carga apoyado en 2 puntos o menos? Es que se balancea."""
        counts = dict.fromkeys(self.foot_geoms, 0)
        for contact in self.data.contact[: self.data.ncon]:
            for geom in (int(contact.geom1), int(contact.geom2)):
                if geom in counts:
                    counts[geom] += 1
        for side in SIDES:
            geom = get_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_foot_geom")
            force = self.data.sensor(f"{side}_foot_force").data[0]
            if force > LOADED_FOOT_FORCE and counts[geom] <= 2:
                return True
        return False

    def support(self, swing: str | None) -> list[float]:
        """
        Fracción del peso en cada pie (izq, der). Con un pie en el aire, todo
        al otro. En doble apoyo, según dónde cae el CdM en la recta entre los
        dos pies.
        """
        if swing == "right":
            return [1.0, 0.0]
        if swing == "left":
            return [0.0, 1.0]
        left, right = self.soles()[:, :2]
        line = left - right
        fraction = float((self.estimated_com()[:2] - right) @ line / (line @ line))
        fraction = min(1.0, max(0.0, fraction))
        return [fraction, 1.0 - fraction]

    def pressure_points(self, swing: str | None) -> list[np.ndarray]:
        """
        Dónde actúa la fuerza de cada pie. Con un pie, bajo el CdM. Con dos,
        cada uno en su pie desplazado lo que el CdM se sale de la recta entre
        pies, para que el centro de presiones total quede bajo el CdM.
        """
        com = self.estimated_com()
        soles = self.soles()
        if swing is not None:
            return [np.array([com[0], com[1], sole[2]]) for sole in soles]
        left, right = soles[:, :2]
        line = left - right
        fraction = float((com[:2] - right) @ line / (line @ line))
        offset = com[:2] - (right + fraction * line)
        return [np.array([*(sole[:2] + offset), sole[2]]) for sole in soles]

    # --- pasos ----------------------------------------------------------------

    def limit_torque_speed(self, ctrl: np.ndarray) -> np.ndarray:
        """
        Curva de par-velocidad de un motor: empujando en el sentido del giro,
        tau_disponible = tau_max * (1 - |w| / w_max); a w_max ya no da par.
        Frenar sigue permitido hasta tau_max. Devuelve el ctrl que da ese par.
        """
        q = self.data.qpos[self.joint_qpos]
        speed = self.data.qvel[self.dof_ids]
        torque = self.kp * (ctrl - q) - self.kv * speed
        available = self.force_limit * np.clip(1.0 - np.abs(speed) / self.max_joint_speed, 0.0, 1.0)
        forward = np.sign(torque) == np.sign(speed)
        torque = np.where(forward, np.clip(torque, -available, available), torque)
        return q + (torque + self.kv * speed) / self.kp

    def step_targets(self, pose: Pose, swing: str | None) -> np.ndarray:
        """ctrl de los 12 motores para seguir 'pose' en este paso."""
        target = joint_targets(pose)
        velocity = np.zeros_like(target)
        if self.previous_target is not None:
            velocity = (target - self.previous_target) / self.model.opt.timestep
        self.previous_target = target

        # Feedforward de velocidad: el amortiguamiento del motor (-kv q') frena
        # hacia velocidad cero, así que en movimiento la articulación se
        # retrasa (kv / kp) q'. Sumando (kv / kp) q'_deseada al objetivo, el
        # motor da kp (q_obj - q) + kv (q'_deseada - q') y sigue sin retraso.
        return (
            target
            + self.kv / self.kp * velocity
            + self.compensator.offset(self.data.qpos, self.support(swing), self.pressure_points(swing))
        )

    def step(self, pose: Pose, swing: str | None) -> None:
        step_start = time.monotonic()
        ctrl = self.step_targets(pose, swing)
        if self.max_joint_speed is not None:
            ctrl = self.limit_torque_speed(ctrl)

        self.data.ctrl[self.actuator_ids] = ctrl
        mujoco.mj_step(self.model, self.data)

        torque = np.abs(read_sensors(self.data, TORQUE_SENSORS))
        self.saturated_steps += torque >= self.force_limit - 1e-6
        self.total_steps += 1
        for contact in self.data.contact[: self.data.ncon]:
            if self.floor_geom not in (int(contact.geom1), int(contact.geom2)):
                self.self_collided = True

        if self.tilt_deg() > FALL_TILT_DEG:
            raise RuntimeError(f"el robot se ha caído en t = {self.data.time:.2f} s")

        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()
            remaining = self.model.opt.timestep - (time.monotonic() - step_start)
            if remaining > 0:
                time.sleep(remaining)


def center_com_over(biped: Biped, pose: Pose, side: str, inset: float = 0.0) -> Pose:
    """
    La misma Pose con la pelvis movida en x e y hasta que el CdM queda sobre
    el pie 'side', a 'inset' metros de su centro hacia el otro pie. Iteración
    de punto fijo sobre una copia de los datos: cada vuelta mueve la pelvis lo
    que le falta al CdM. Converge porque el CdM se mueve algo menos que la
    pelvis (las piernas no la siguen del todo).
    """
    scratch = mujoco.MjData(biped.plan_model)
    inward = np.array([0.0, -inset if side == "left" else inset, 0.0])
    target = (pose.sole(side) + rotation_z(pose.yaw(side)) @ inward)[:2]
    pelvis = pose.pelvis.copy()
    for _ in range(100):
        biped.set_pose(scratch, replace(pose, pelvis=pelvis))
        error = target - biped.com(scratch)[:2]
        if np.linalg.norm(error) < 1e-6:
            break
        pelvis[:2] += error
    return replace(pose, pelvis=pelvis)
