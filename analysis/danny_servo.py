"""
Danny - Servo HD-1910 con el modelo BAM en MuJoCo por CPU.

Sigue a bam.mjlab.BamActuator de xgoduck_rl (Apache-2.0): el firmware convierte el error de
posición en ciclo de trabajo, y el rozamiento crece con la carga. Es lo que pasa en el
entrenamiento, y lo que deben usar las pruebas de la etapa 3.
"""

import mujoco
import numpy as np

import danny_sim

SERVO = danny_sim.SERVO
FRICTION_CONSTRAINT = int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)
STIFF_SOLREF = [-5e4, -2e2]
STIFF_SOLIMP = [0.99, 0.9999, 0.001, 0.5, 2]


def friction_budget(external, motor, speed):
    """Rozamiento seco en la salida, en N·m, según los pares externo y de motor y la velocidad."""
    stribeck = np.exp(-(np.abs(speed / SERVO["dtheta_stribeck"]) ** SERVO["alpha"]))
    friction = SERVO["friction_base"] + stribeck * SERVO["friction_stribeck"]
    friction = friction + np.abs(external * SERVO["load_friction_external"] - motor * SERVO["load_friction_motor"])
    friction = friction + stribeck * np.abs(
        external * SERVO["load_friction_external_stribeck"] - motor * SERVO["load_friction_motor_stribeck"])
    external, motor = np.abs(external), np.abs(motor)
    quadratic = np.where(motor > external, SERVO["load_friction_external_quad"] * external ** 2,
                         SERVO["load_friction_motor_quad"] * motor ** 2)
    return friction + stribeck * quadratic


def required_motor_torque(torque, speed):
    """Par de motor para dar ese par en la salida, venciendo el rozamiento bajo esa misma carga."""
    return np.abs(torque) + friction_budget(torque, torque, speed)


class Servos:
    """Convierte los actuadores del modelo en servos BAM. Modifica el modelo que recibe."""

    def __init__(self, model, volts, firmware_gain=SERVO["firmware_kp"]):
        self.model = model
        self.volts = volts
        self.gain = firmware_gain * SERVO["error_gain"] * SERVO["error_gain_ratio"]
        limit = volts * SERVO["kt"] / SERVO["R"]
        model.actuator_gainprm[:, 0] = 1
        model.actuator_biasprm[:, :3] = 0
        model.actuator_forcerange[:] = [-limit, limit]
        model.dof_damping[6:] = SERVO["friction_viscous"]
        model.dof_frictionloss[6:] = SERVO["friction_base"]
        model.dof_solref[6:] = STIFF_SOLREF
        model.dof_solimp[6:] = STIFF_SOLIMP

    def reset(self, data):
        """Vuelve al inicio: mj_resetData no deshace el rozamiento que `control` escribe en el modelo."""
        mujoco.mj_resetData(self.model, data)
        self.model.dof_frictionloss[6:] = SERVO["friction_base"]

    def stiffness(self):
        """Par por radián de error, sin contar el rozamiento."""
        return self.gain * self.volts * SERVO["kt"] / SERVO["R"]

    def control(self, data, target):
        """Pone en data.ctrl el par de motor hacia las posiciones objetivo, y lo devuelve."""
        position, speed = data.qpos[7:], data.qvel[6:]
        duty = np.clip((target - position) * self.gain, -SERVO["max_pwm"], SERVO["max_pwm"])
        torque = SERVO["kt"] * (self.volts * duty - SERVO["kt"] * speed) / SERVO["R"]
        friction_force = np.zeros(self.model.nv)
        for index in range(data.nefc):
            if data.efc_type[index] == FRICTION_CONSTRAINT:
                friction_force[data.efc_id[index]] += data.efc_force[index]
        external = (data.qfrc_constraint - friction_force - data.qfrc_bias)[6:]
        self.model.dof_frictionloss[6:] = friction_budget(external, data.qfrc_actuator[6:], speed)
        data.ctrl[:] = torque
        return torque

    def step(self, data, target):
        """Avanza un paso de física, y devuelve el par de motor."""
        torque = self.control(data, target)
        mujoco.mj_step(self.model, data)
        return torque


def gravity_targets(model, data, pose, servos):
    """Objetivos que sostienen esa pose: el servo es blando y necesita error para hacer par."""
    data.qpos[:] = pose
    data.qvel[:] = 0
    data.qacc[:] = 0
    mujoco.mj_forward(model, data)
    data.qacc[:] = 0
    mujoco.mj_inverse(model, data)
    return pose[7:] + data.qfrc_inverse[6:] / servos.stiffness()


def trajectory_targets(model, data, trajectory, servos):
    """Objetivos para seguir una trayectoria de qpos, compensando la carga de cada postura."""
    return np.array([gravity_targets(model, data, pose, servos) for pose in trajectory])
