"""Ejecuta las pruebas existentes con un Biped que registra el estado real."""

import collections

import mujoco
import numpy as np

import push_biped
import walk_biped
from biped_control import joint_targets
from build_model import build_robot
from conftest import RecordingBiped, load_model
from robot_spec import BIPED

E13 = walk_biped.Gait(56, 0.08, 0.05, 0.012, 45)
STRAIGHT = walk_biped.Gait(56, 0.08, 0.05, 0.012, 0)
E11 = walk_biped.Gait()


class RealisticBiped(RecordingBiped):
    """Añade lo que un robot real tendría: periodo de control, retardo y ruido."""

    control_period = 1  # pasos de simulación por ciclo de control
    delay_steps = 0
    encoder_noise = 0.0  # rad, desviación
    seed = 2026

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.queue = collections.deque()
        self.held = None
        self.last_target = None
        self.counter = 0
        self.rng = np.random.default_rng(self.seed)

    def set_pose(self, data, pose):
        super().set_pose(data, pose)
        if data is self.data and self.model.na:
            data.act[:] = np.concatenate([joint_targets(pose), self.held_target])  # el filtro arranca en la postura

    def step_targets(self, pose, swing):
        if self.held is None or self.counter % self.control_period == 0:
            target = np.concatenate([joint_targets(pose), self.held_target])
            velocity = np.zeros_like(target)
            if self.last_target is not None:
                velocity = (target - self.last_target) / (self.control_period * self.model.opt.timestep)
            self.last_target = target
            ctrl = (target + self.kv / self.kp * velocity
                    + self.compensator.offset(self.data.qpos, self.support(swing), self.pressure_points(swing)))
            if self.encoder_noise:
                ctrl = ctrl + self.rng.normal(0.0, self.encoder_noise, ctrl.shape)
            self.queue.append(ctrl)
            if len(self.queue) > self.delay_steps:
                self.held = self.queue.popleft()
            elif self.held is None:
                self.held = ctrl
        self.counter += 1
        return self.held


def run_walk(monkeypatch, gait=E13, speed=1.27, hold=0.25, biped_class=RecordingBiped, model=None, nominal=None):
    """Devuelve (biped o None si cae, resultados, saturación, chocan, modelo, mensaje)."""
    instances = []

    class Factory(biped_class):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            instances.append(self)

    monkeypatch.setattr(walk_biped, "Biped", Factory)
    model = load_model() if model is None else model
    data = mujoco.MjData(model)
    try:
        results, saturation, collided = walk_biped.run(model, data, speed=speed, hold=hold, gait=gait, nominal_model=nominal)
    except (RuntimeError, ValueError) as error:
        return instances[-1] if instances else None, None, None, None, model, str(error)
    return instances[-1], results, saturation, collided, model, ""


def run_push(monkeypatch, direction, force, biped_class=RecordingBiped):
    instances = []

    class Factory(biped_class):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            instances.append(self)

    monkeypatch.setattr(push_biped, "Biped", Factory)
    model = load_model()
    data = mujoco.MjData(model)
    trial = push_biped.run_push(model, data, direction, force)
    return instances[-1], trial, model


def model_with_motor_lag(time_constant: float, robot=BIPED) -> mujoco.MjModel:
    spec = build_robot(robot)
    for motor in spec.actuators:
        motor.dyntype = mujoco.mjtDyn.mjDYN_FILTEREXACT
        motor.dynprm[0] = time_constant
    return spec.compile()
