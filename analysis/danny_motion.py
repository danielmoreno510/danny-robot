"""
Danny - Trayectorias de medida y cinemática inversa, etapa 3.

Módulo compartido. Cada tarea es una función del tiempo que devuelve objetivos, centro de
masas, postura y contactos. Son herramientas de medida, no animaciones.
"""

import mujoco
import numpy as np

import danny_sim

RATE_HZ = 200.0

UPRIGHT = np.eye(3)
ARMS_DOWN = {"left_shoulder_roll": 0.05, "right_shoulder_roll": -0.05}
BENT_LEGS = {
    f"{side}_{joint}": angle
    for side in ("left", "right")
    for joint, angle in (("hip_pitch", -0.3), ("knee", 0.6), ("ankle_pitch", -0.3))
}
STAND_DROP = 0.015
WALK_SPEED = 0.25
STEP_PERIOD = 0.30
STEP_HEIGHT = 0.02
SWING_TOE_DOWN = 0.25  # rad: el pie en el aire no va plano, y evita el tope del tobillo
SQUAT_LEAN = 0.35
COM_HEIGHT = 0.21
DOUBLE_SUPPORT = 0.12  # fracción de cada paso, a cada lado del cambio de pie
SQUAT_DROP = 0.045  # más abajo, con el pie plano, el tobillo llega a su tope


def pitched(angle):
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, 0, sin], [0, 1, 0], [-sin, 0, cos]])


def smooth(value):
    value = np.clip(value, 0, 1)
    return value * value * (3 - 2 * value)


class Solver:
    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.hinges = np.where(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)[0]
        self.names = danny_sim.hinge_names(model)
        self.addresses = model.jnt_qposadr[self.hinges]
        self.seed = model.qpos0.copy()
        for name, angle in BENT_LEGS.items():
            self.seed[7 + self.names.index(name)] = angle
        self.seed[2] = -0.015
        self.q = self.seed.copy()

    def site_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)

    def solve(self, targets, com, posture, iterations=40):
        """targets: {site: (posición, rotación)}. Cualquiera, o un eje de la posición, puede ser None."""
        model, data = self.model, self.data
        reference = self.seed.copy()
        for name, value in posture.items():
            reference[7 + self.names.index(name)] = value
        q = self.q.copy()
        worst = 0.0
        for _ in range(iterations):
            data.qpos[:] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            rows, errors = [], []
            worst = 0.0
            for name, (position, rotation) in targets.items():
                site = self.site_id(name)
                jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
                mujoco.mj_jacSite(model, data, jacp, jacr, site)
                if position is not None:
                    axes = [axis for axis in range(3) if position[axis] is not None]
                    error = np.array([position[axis] - data.site_xpos[site][axis] for axis in axes])
                    worst = max(worst, float(np.linalg.norm(error)))
                    rows.append(jacp[axes])
                    errors.append(error)
                if rotation is not None:
                    current = data.site_xmat[site].reshape(3, 3)
                    error = 0.5 * sum(np.cross(current[:, i], rotation[:, i]) for i in range(3))
                    rows.append(0.3 * jacr)
                    errors.append(0.3 * error)
            if com is not None:
                jac = np.zeros((3, model.nv))
                mujoco.mj_jacSubtreeCom(model, data, jac, 1)
                error = np.asarray(com) - data.subtree_com[1][:2]
                worst = max(worst, float(np.linalg.norm(error)))
                rows.append(jac[:2])
                errors.append(error)
            jacobian = np.vstack(rows)
            error = np.concatenate(errors)
            posture_error = np.zeros(model.nv)
            posture_error[6:] = reference[7:] - q[7:]
            damping = 1e-4
            step = np.linalg.solve(
                jacobian.T @ jacobian + damping * np.eye(model.nv),
                jacobian.T @ error + damping * 0.3 * posture_error)
            step *= min(1.0, 0.2 / max(np.abs(step).max(), 1e-9))
            mujoco.mj_integratePos(model, q, step, 1.0)
            for hinge, address in zip(self.hinges, self.addresses):
                q[address] = np.clip(q[address], *model.jnt_range[hinge])
        self.q = q
        return q.copy(), worst


def feet_targets(left=(0, 0), right=(0, 0), left_z=0.0, right_z=0.0):
    return {
        "left_sole": (
            (danny_sim.FOOT_CENTER_X + left[0], danny_sim.foot_center_y(1) + left[1], left_z), UPRIGHT),
        "right_sole": (
            (danny_sim.FOOT_CENTER_X + right[0], danny_sim.foot_center_y(-1) + right[1], right_z), UPRIGHT),
    }


BOTH_FEET = [("left_sole", True), ("right_sole", True)]
TORSO_ZERO_Z = 0.3


def squat(time):
    seconds = 3.0
    phase = smooth(time / 1.5) if time < 1.5 else smooth((seconds - time) / 1.5)
    targets = feet_targets()
    targets["torso"] = (None, pitched(SQUAT_LEAN * phase))
    targets["pelvis_bottom"] = ((None, None, 0.241 - STAND_DROP - SQUAT_DROP * phase), None)
    return targets, (danny_sim.FOOT_CENTER_X, 0.0), ARMS_DOWN, BOTH_FEET


def single_support(time):
    shift = smooth(time / 1.5)
    lift = smooth((time - 1.5) / 0.5)
    targets = feet_targets(right_z=0.03 * lift)
    targets["right_sole"] = (targets["right_sole"][0], pitched(SWING_TOE_DOWN * lift))
    targets["torso"] = (None, UPRIGHT)
    targets["pelvis_bottom"] = ((None, None, 0.241 - STAND_DROP), None)
    com = (danny_sim.FOOT_CENTER_X, shift * danny_sim.foot_center_y(1))
    contacts = BOTH_FEET if time < 1.5 else [("left_sole", True)]
    return targets, com, ARMS_DOWN, contacts


def walk(time):
    step_length = WALK_SPEED * STEP_PERIOD
    index = int(time / STEP_PERIOD)
    phase = time / STEP_PERIOD - index
    stance_is_left = index % 2 == 0
    stance_x = index * step_length
    swing_from = (index - 1) * step_length
    swing_to = (index + 1) * step_length
    swing_x = swing_from + (swing_to - swing_from) * smooth(phase)
    swing_z = STEP_HEIGHT * np.sin(np.pi * phase) ** 2
    swing_rotation = pitched(SWING_TOE_DOWN * np.sin(np.pi * phase) ** 2)
    frequency = np.pi / STEP_PERIOD
    amplitude = danny_sim.foot_center_y(1) / (1 + frequency ** 2 * COM_HEIGHT / 9.81)
    com_y = amplitude * np.sin(frequency * time + 0) * 1.0
    com_x = danny_sim.FOOT_CENTER_X + WALK_SPEED * (time - STEP_PERIOD / 2)
    if stance_is_left:
        targets = feet_targets(left=(stance_x, 0), right=(swing_x, 0), right_z=swing_z)
        targets["right_sole"] = (targets["right_sole"][0], swing_rotation)
        contacts = [("left_sole", True)]
    else:
        targets = feet_targets(left=(swing_x, 0), right=(stance_x, 0), left_z=swing_z)
        targets["left_sole"] = (targets["left_sole"][0], swing_rotation)
        contacts = [("right_sole", True)]
    if phase < DOUBLE_SUPPORT or phase > 1 - DOUBLE_SUPPORT:
        contacts = BOTH_FEET
    targets["torso"] = (None, UPRIGHT)
    targets["pelvis_bottom"] = ((None, None, 0.241 - STAND_DROP), None)
    swing = 0.25 * np.sin(frequency * time)
    posture = {
        **ARMS_DOWN,
        "left_shoulder_pitch": -swing,
        "right_shoulder_pitch": swing,
        "waist_yaw": 0.5 * swing,
    }
    return targets, (com_x, com_y), posture, contacts


def rise_from_crouch(time):
    """De cuclillas a de pie, sin manos, con el tronco inclinado para llevar el peso sobre los pies."""
    rise = smooth(time / 2.0)
    targets = feet_targets()
    targets["torso"] = (None, pitched(SQUAT_LEAN * (1 - rise)))
    targets["pelvis_bottom"] = ((None, None, 0.241 - STAND_DROP - SQUAT_DROP * (1 - rise)), None)
    posture = {**ARMS_DOWN, "left_shoulder_pitch": -1.2 * (1 - rise), "right_shoulder_pitch": -1.2 * (1 - rise)}
    return targets, (danny_sim.FOOT_CENTER_X, 0.0), posture, BOTH_FEET


def push_up_from_prone(time):
    """Boca abajo: las manos elevan el tronco, con las puntas de los pies apoyadas."""
    rise = smooth(time / 2.0)
    pitch = 1.45 - 0.45 * rise
    targets = {
        "left_toe": ((0.0, 0.032, 0.0), None),
        "right_toe": ((0.0, -0.032, 0.0), None),
        "left_hand": ((0.3, 0.075, 0.0), None),
        "right_hand": ((0.3, -0.075, 0.0), None),
        "torso": (None, pitched(pitch)),
    }
    posture = {"left_shoulder_pitch": -1.4, "right_shoulder_pitch": -1.4, "left_elbow": -0.9 * (1 - rise),
               "right_elbow": -0.9 * (1 - rise), "left_shoulder_roll": 0.1, "right_shoulder_roll": -0.1}
    contacts = [(name, False) for name in ("left_toe", "right_toe", "left_hand", "right_hand")]
    return targets, None, posture, contacts


def sit_up_from_back(time):
    """Boca arriba: la cadera levanta el tronco, con pelvis y talones en el suelo."""
    rise = smooth(time / 2.0)
    targets = {
        "pelvis_back": ((0.0, 0.0, 0.0), None),
        "left_heel": ((None, 0.032, 0.0), None),
        "right_heel": ((None, -0.032, 0.0), None),
        "torso": (None, pitched(-1.5 + 0.85 * rise)),
    }
    contacts = [("pelvis_back", False), ("left_heel", False), ("right_heel", False)]
    straight = {f"{side}_{joint}": 0.0 for side in ("left", "right") for joint in ("knee", "ankle_pitch")}
    straight.update({"left_hip_pitch": -rise, "right_hip_pitch": -rise})
    return targets, None, {**ARMS_DOWN, **straight}, contacts


TASKS = {
    "squat": (squat, 3.0),
    "single_support": (single_support, 3.0),
    "walk": (walk, 3.0),
    "rise_from_crouch": (rise_from_crouch, 2.0),
    "push_up_from_prone": (push_up_from_prone, 2.0),
    "sit_up_from_back": (sit_up_from_back, 2.0),
}
# Inclinación de partida de la base, en rad: tumbado no se llega bien desde de pie
START_PITCH = {
    "sit_up_from_back": -1.5,
}


def task_trajectory(model, task, solver=None):
    """Devuelve los qpos de la tarea, a RATE_HZ."""
    solver = solver or Solver(model)
    generator, seconds = TASKS[task]
    solver.q = solver.seed.copy()
    if task in START_PITCH:
        solver.q = model.qpos0.copy()
        mujoco.mju_mat2Quat(solver.q[3:7], pitched(START_PITCH[task]).flatten())
    solver.solve(*generator(0.0)[:3], iterations=600)
    return np.array([
        solver.solve(*generator(float(time))[:3], iterations=30)[0]
        for time in np.arange(0.0, seconds, 1 / RATE_HZ)
    ])
