"""
H1 - Utilidades del bípedo (biped_6dof.xml), compartidas por las etapas 10 y 11.

  - Pose: dónde están la pelvis y las plantas de los pies (espacio de tareas).
  - joint_targets: IK de las dos piernas para una Pose.
  - leg_reach: cuánto se estira cada pierna en una Pose (1 = recta).
  - Biped: estado del robot, reparto del peso y un paso de simulación con
    StanceCompensator (estático o dinámico) y feedforward de velocidad.
  - center_com_over: mueve la pelvis hasta que el CdM queda sobre un pie.
  - rotation_z: rotación alrededor de la vertical.
"""

import math
import time
from dataclasses import dataclass, replace

import mujoco
import numpy as np

from leg_control import StanceCompensator, get_id, leg_ik, read_sensors


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
    los ejes del pie, y ahí vale leg_ik.
    """
    targets = []
    for vector, foot_yaw in zip(_ankles_in_hips(pose), (pose.left_yaw, pose.right_yaw)):
        angles = leg_ik(rotation_z(-foot_yaw) @ vector)
        angles[0] = foot_yaw - pose.pelvis_yaw
        targets.append(angles)
    return np.concatenate(targets)


def leg_reach(pose: Pose) -> float:
    """Distancia cadera-tobillo / longitud de la pierna, la mayor de las dos."""
    return max(float(np.linalg.norm(v)) for v in _ankles_in_hips(pose)) / LEG_LENGTH


class Biped:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        viewer=None,
        dynamic: bool = False,
    ):
        """
        dynamic: compensación con dinámica inversa y ZMP exacto. En la etapa 12
        no mejoró la marcha (inclina más la pelvis), así que por defecto es la
        estática; se deja para reproducir la comparación. No tiene en cuenta
        la aceleración de giro de la pelvis: no vale para marchas con giro.
        """
        self.model = model
        self.data = data
        self.viewer = viewer
        self.dynamic = dynamic
        self.actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
        joint_ids = [get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        self.dof_ids = [int(model.jnt_dofadr[j]) for j in joint_ids]
        self.joint_qpos = [int(model.jnt_qposadr[j]) for j in joint_ids]
        self.pelvis_id = get_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.foot_ids = [get_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot") for s in SIDES]
        self.foot_geoms = {get_id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_foot_geom") for s in SIDES}
        self.feet_collided = False
        self.sole_ids = [get_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{s}_sole_site") for s in SIDES]
        self.force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        self.compensator = StanceCompensator(model, self.actuator_ids, self.dof_ids, self.foot_ids)
        # Motor de posición: tau = kp (ctrl - q) - kv q'. biasprm = [0, -kp, -kv].
        self.kp = model.actuator_gainprm[self.actuator_ids, 0].copy()
        self.kv = -model.actuator_biasprm[self.actuator_ids, 2].copy()
        # Historia del plan para derivar por diferencias: (objetivo articular,
        # pelvis) de los dos pasos anteriores. Vacía tras set_pose.
        self.history: list[tuple[np.ndarray, np.ndarray]] = []
        self.dynamics_scratch = mujoco.MjData(model)
        self.inverse = np.zeros(model.nv)
        self.point_jacobian = np.zeros((3, model.nv))
        self.saturated_steps = np.zeros(len(JOINTS))
        self.total_steps = 0

    # --- estado ---------------------------------------------------------------

    def set_pose(self, data: mujoco.MjData, pose: Pose) -> None:
        """Pone qpos en la pose (pelvis horizontal) y calcula la cinemática."""
        if data is self.data:
            self.history = []
        data.qpos[0:3] = pose.pelvis
        half_yaw = 0.5 * pose.pelvis_yaw
        data.qpos[3:7] = [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]
        data.qpos[self.joint_qpos] = joint_targets(pose)
        mujoco.mj_forward(self.model, data)

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

    def support(self, swing: str | None, reference: np.ndarray | None = None) -> list[float]:
        """
        Fracción de la fuerza del suelo en cada pie (izq, der). Con un pie en
        el aire, todo al otro. En doble apoyo, según dónde cae 'reference' en
        la recta entre los dos pies: el CdM (estático) o el ZMP (dinámico).
        """
        if swing == "right":
            return [1.0, 0.0]
        if swing == "left":
            return [0.0, 1.0]
        reference = self.com()[:2] if reference is None else reference
        left, right = self.soles()[:, :2]
        line = left - right
        fraction = float((reference - right) @ line / (line @ line))
        fraction = min(1.0, max(0.0, fraction))
        return [fraction, 1.0 - fraction]

    def pressure_points(self, swing: str | None, reference: np.ndarray | None = None) -> list[np.ndarray]:
        """
        Dónde actúa la fuerza de cada pie. Con un pie, en 'reference' (CdM o
        ZMP). Con dos, cada uno en su pie desplazado lo que 'reference' se sale
        de la recta entre pies, para que el centro de presiones total quede ahí.
        """
        reference = self.com()[:2] if reference is None else reference
        soles = self.soles()
        if swing is not None:
            return [np.array([reference[0], reference[1], sole[2]]) for sole in soles]
        left, right = soles[:, :2]
        line = left - right
        fraction = float((reference - right) @ line / (line @ line))
        offset = reference - (right + fraction * line)
        return [np.array([*(sole[:2] + offset), sole[2]]) for sole in soles]

    # --- pasos ----------------------------------------------------------------

    def plan_derivatives(self, target: np.ndarray, pelvis: np.ndarray) -> tuple[np.ndarray, ...]:
        """
        Por diferencias finitas sobre el plan: (q' articular, q'' articular,
        aceleración de la pelvis). Ceros mientras no hay historia suficiente.
        """
        self.history = (self.history + [(target, pelvis.copy())])[-3:]
        dt = self.model.opt.timestep
        if len(self.history) < 2:
            return np.zeros_like(target), np.zeros_like(target), np.zeros(3)
        (q1, _), (q2, _) = self.history[-2:]
        velocity = (q2 - q1) / dt
        if len(self.history) < 3:
            return velocity, np.zeros_like(target), np.zeros(3)
        (q0, p0), (q1, p1), (q2, p2) = self.history
        return velocity, (q2 - 2 * q1 + q0) / dt**2, (p2 - 2 * p1 + p0) / dt**2

    def required_contact(self, qacc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Fuerza total del suelo [N] y punto del suelo donde tiene que actuar (el
        ZMP), para que el robot haga la aceleración qacc.

        La dinámica inversa (mj_rne) da lo que necesita cada grado de libertad.
        Los 6 de la pelvis (freejoint) no tienen motor: solo los puede aportar
        el suelo. Las 3 filas lineales son la fuerza total del suelo. Las 3
        angulares, el momento que esa fuerza tiene que hacer: de ahí sale el
        punto (mínimos cuadrados; el momento vertical, que un punto no puede
        dar, queda para la fricción de los pies). Incluye el momento angular de
        las piernas, que la fórmula del carro y la mesa ignora.
        """
        scratch = self.dynamics_scratch
        scratch.qpos[:] = self.data.qpos
        scratch.qvel[:] = self.data.qvel
        mujoco.mj_forward(self.model, scratch)
        scratch.qacc[:] = qacc
        mujoco.mj_rne(self.model, scratch, 1, self.inverse)
        force = self.inverse[0:3].copy()

        ground_z = float(np.min(self.soles()[:, 2]))

        def base_moment(x: float, y: float) -> np.ndarray:
            point = np.array([x, y, ground_z])
            mujoco.mj_jac(self.model, scratch, self.point_jacobian, None, point, self.pelvis_id)
            return self.point_jacobian[:, 3:6].T @ force

        origin = base_moment(0.0, 0.0)
        columns = np.column_stack([base_moment(1.0, 0.0) - origin, base_moment(0.0, 1.0) - origin])
        point, *_ = np.linalg.lstsq(columns, self.inverse[3:6] - origin, rcond=None)
        return force, point

    def step_targets(self, pose: Pose, swing: str | None) -> np.ndarray:
        """ctrl de los 12 motores para seguir 'pose' en este paso."""
        target = joint_targets(pose)
        velocity, acceleration, pelvis_acc = self.plan_derivatives(target, pose.pelvis)

        # Feedforward de velocidad: el amortiguamiento del motor (-kv q') frena
        # hacia velocidad cero, así que en movimiento la articulación se
        # retrasa (kv / kp) q'. Sumando (kv / kp) q'_deseada al objetivo, el
        # motor da kp (q_obj - q) + kv (q'_deseada - q') y sigue sin retraso.
        ctrl = target + self.kv / self.kp * velocity

        if not self.dynamic:
            return ctrl + self.compensator.offset(
                self.data.qpos, self.support(swing), self.pressure_points(swing)
            )

        qacc = np.zeros(self.model.nv)
        qacc[0:3] = pelvis_acc  # freejoint: lineal en ejes del mundo; angular = 0
        qacc[self.dof_ids] = acceleration
        ground_force, zmp = self.required_contact(qacc)
        return ctrl + self.compensator.offset(
            self.data.qpos,
            self.support(swing, zmp),
            self.pressure_points(swing, zmp),
            qvel=self.data.qvel,
            qacc=qacc,
            ground_force=ground_force,
        )

    def step(self, pose: Pose, swing: str | None) -> None:
        step_start = time.monotonic()
        ctrl = self.step_targets(pose, swing)

        self.data.ctrl[self.actuator_ids] = ctrl
        mujoco.mj_step(self.model, self.data)

        torque = np.abs(read_sensors(self.data, TORQUE_SENSORS))
        self.saturated_steps += torque >= self.force_limit - 1e-6
        self.total_steps += 1
        for contact in self.data.contact[: self.data.ncon]:
            if {int(contact.geom1), int(contact.geom2)} == self.foot_geoms:
                self.feet_collided = True

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
    scratch = mujoco.MjData(biped.model)
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
