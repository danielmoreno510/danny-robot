"""
H1 - Utilidades del bípedo (biped_6dof.xml), compartidas por las etapas 10 y 11.

  - Pose: dónde están la pelvis y las plantas de los pies (espacio de tareas).
  - joint_targets: IK de las dos piernas para una Pose.
  - leg_reach: cuánto se estira cada pierna en una Pose (1 = recta).
  - Biped: estado del robot, reparto del peso y un paso de simulación con
    StanceCompensator y feedforward de velocidad.
  - center_com_over: mueve la pelvis hasta que el CdM queda sobre un pie.
"""

import math
import time
from dataclasses import dataclass

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
    """Pose en el espacio de tareas. Posiciones en el mundo [m]."""

    pelvis: np.ndarray
    left_sole: np.ndarray
    right_sole: np.ndarray

    def vector(self) -> np.ndarray:
        return np.concatenate([self.pelvis, self.left_sole, self.right_sole])

    @staticmethod
    def from_vector(v: np.ndarray) -> "Pose":
        return Pose(v[0:3], v[3:6], v[6:9])

    def sole(self, side: str) -> np.ndarray:
        return self.left_sole if side == "left" else self.right_sole


@dataclass
class Phase:
    name: str
    pose: Pose
    swing: str | None  # pie que no carga peso ("left" / "right"), o None


def pelvis_height(knee_deg: float) -> float:
    """Altura de la pelvis con los pies bajo las caderas y la rodilla así."""
    half_knee = math.radians(knee_deg) / 2
    return SOLE_BELOW_ANKLE + LEG_LENGTH * math.cos(half_knee) + HIP_BELOW_PELVIS


def _ankles_in_hips(pose: Pose) -> list[np.ndarray]:
    result = []
    for sole, hip_y in ((pose.left_sole, HIP_Y), (pose.right_sole, -HIP_Y)):
        hip = pose.pelvis + np.array([0.0, hip_y, -HIP_BELOW_PELVIS])
        ankle = sole + np.array([0.0, 0.0, SOLE_BELOW_ANKLE])
        result.append(ankle - hip)
    return result


def joint_targets(pose: Pose) -> np.ndarray:
    """IK de las dos piernas, con la pelvis horizontal y mirando a +x."""
    return np.concatenate([leg_ik(v) for v in _ankles_in_hips(pose)])


def leg_reach(pose: Pose) -> float:
    """Distancia cadera-tobillo / longitud de la pierna, la mayor de las dos."""
    return max(float(np.linalg.norm(v)) for v in _ankles_in_hips(pose)) / LEG_LENGTH


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
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
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

    # --- reparto del peso ------------------------------------------------------

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
        fraction = float((self.com()[:2] - right) @ line / (line @ line))
        fraction = min(1.0, max(0.0, fraction))
        return [fraction, 1.0 - fraction]

    def pressure_points(self, swing: str | None) -> list[np.ndarray]:
        """
        Dónde actúa la fuerza de cada pie. Con un pie, bajo el CdM. Con dos,
        cada uno en su pie desplazado lo que el CdM se sale de la recta entre
        pies, para que el centro de presiones total quede bajo el CdM.
        """
        soles = self.soles()
        com = self.com()
        if swing is not None:
            return [np.array([com[0], com[1], sole[2]]) for sole in soles]
        left, right = soles[:, :2]
        line = left - right
        fraction = float((com[:2] - right) @ line / (line @ line))
        offset = com[:2] - (right + fraction * line)
        return [np.array([*(sole[:2] + offset), sole[2]]) for sole in soles]

    # --- pasos ----------------------------------------------------------------

    def velocity_feedforward(self, target: np.ndarray) -> np.ndarray:
        """
        El amortiguamiento del motor (-kv q') frena hacia velocidad cero, así
        que en movimiento la articulación se retrasa (kv / kp) q'. Sumando
        (kv / kp) q'_deseada al objetivo, el motor da
        kp (q_obj - q) + kv (q'_deseada - q') y sigue al objetivo sin retraso.
        q'_deseada se estima por diferencias entre pasos.
        """
        if self.previous_target is None:
            velocity = np.zeros_like(target)
        else:
            velocity = (target - self.previous_target) / self.model.opt.timestep
        self.previous_target = target
        return self.kv / self.kp * velocity

    def step(self, pose: Pose, swing: str | None) -> None:
        step_start = time.monotonic()
        target = joint_targets(pose)
        ctrl = (
            target
            + self.velocity_feedforward(target)
            + self.compensator.offset(self.data.qpos, self.support(swing), self.pressure_points(swing))
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


def center_com_over(biped: Biped, pose: Pose, side: str) -> Pose:
    """
    La misma Pose con la pelvis movida en x e y hasta que el CdM queda sobre
    el centro del pie 'side'. Iteración de punto fijo sobre una copia de los
    datos: cada vuelta mueve la pelvis lo que le falta al CdM. Converge
    porque el CdM se mueve algo menos que la pelvis (las piernas no la
    siguen del todo).
    """
    scratch = mujoco.MjData(biped.model)
    target = pose.sole(side)[:2]
    pelvis = pose.pelvis.copy()
    for _ in range(100):
        candidate = Pose(pelvis, pose.left_sole, pose.right_sole)
        biped.set_pose(scratch, candidate)
        error = target - biped.com(scratch)[:2]
        if np.linalg.norm(error) < 1e-6:
            break
        pelvis[:2] += error
    return Pose(pelvis, pose.left_sole, pose.right_sole)
