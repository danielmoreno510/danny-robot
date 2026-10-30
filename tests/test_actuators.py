"""
Actuadores y control (auditoría 3). Banco de pruebas: danny.xml con la pelvis fija
en el aire (sin freejoint) y GravityCompensator activo. Es la respuesta de cada motor.
"""

import math

import mujoco
import numpy as np
import pytest

from biped_control import ACTUATORS, JOINT_PARTS, JOINTS, SIDES
from conftest import MODEL_PATH, load_model, record
from leg_control import GravityCompensator

STEP_DEG = 10.0
MIDPOINT = {"hip_yaw": 0, "hip_roll": 0, "hip_pitch": 0, "knee": 40, "ankle_pitch": 0, "ankle_roll": 0}


class Rig:
    def __init__(self):
        text = MODEL_PATH.read_text().replace('<freejoint name="pelvis_free"/>', "")
        text = text.replace('<body name="pelvis" pos="0 0 0.4">', '<body name="pelvis" pos="0 0 0.8">')
        self.model = mujoco.MjModel.from_xml_string(text)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.act = [m.actuator(n).id for n in ACTUATORS]
        self.dof = [int(m.joint(n).dofadr[0]) for n in JOINTS]
        self.adr = [int(m.joint(n).qposadr[0]) for n in JOINTS]
        self.comp = GravityCompensator(m, self.act, self.dof)
        self.kp = m.actuator_gainprm[self.act, 0]
        self.kv = -m.actuator_biasprm[self.act, 2]

    def step(self, target, velocity=None):
        d = self.data
        velocity = np.zeros(12) if velocity is None else velocity
        d.ctrl[self.act] = target + self.comp.offset(d.qpos) + self.kv / self.kp * velocity
        mujoco.mj_step(self.model, d)
        return d.qpos[self.adr].copy()


def step_response(index, step=STEP_DEG):
    rig = Rig()
    start = np.radians([MIDPOINT[p] for _ in SIDES for p in JOINT_PARTS])
    rig.data.qpos[rig.adr] = start
    dt = rig.model.opt.timestep
    for _ in range(500):
        rig.step(start)
    target = start.copy()
    target[index] += math.radians(step)
    trace, torque = [], []
    for _ in range(1500):
        trace.append(rig.step(target)[index])
        torque.append(rig.data.actuator_force[rig.act[index]])
    trace = np.degrees(np.array(trace) - start[index]) * math.copysign(1, step)
    step = abs(step)
    above = np.where(np.abs(trace - step) > 0.05 * step)[0]
    settle = (above[-1] + 1) * dt if len(above) else 0.0
    rise = np.argmax(trace >= 0.9 * step) * dt
    overshoot = 100 * (trace.max() - step) / step
    crossings = int(np.sum(np.diff(np.sign(trace[int(0.05 / dt):] - trace[-1])) != 0))
    return {"rise": rise, "settle": settle, "overshoot": overshoot, "final": abs(step - trace[-1]),
            "peak": float(np.abs(torque).max()), "crossings": crossings}


@pytest.mark.parametrize("name,index", list(zip(JOINTS, range(12))))
def test_step_response_each_joint(name, index):
    """Escalón de 10°: sin oscilar, asentado en < 0,5 s y sin error final."""
    # Hacia fuera: hacia dentro, el pie choca con el otro pie (se comprueba abajo)
    r = step_response(index, -STEP_DEG if name == "right_hip_roll_joint" else STEP_DEG)
    record(f"actuador.{name}.subida", r["rise"], "informativa", "s")
    record(f"actuador.{name}.asentamiento", r["settle"], "< 0,5", "s")
    record(f"actuador.{name}.sobrepaso", r["overshoot"], "< 20", "%")
    record(f"actuador.{name}.error_final", r["final"], "< 0,1", "°")
    record(f"actuador.{name}.par_pico", r["peak"], "<= 20", "N·m")
    assert r["settle"] < 0.5
    assert r["overshoot"] < 20
    assert r["final"] < 0.1
    assert r["peak"] <= 20.0 + 1e-6


@pytest.mark.parametrize("frequency", [0.5, 1.0, 2.0, 4.0])
def test_tracking_of_changing_targets(frequency):
    """Seno de ±10° en rodilla y cadera con feedforward de velocidad."""
    rig = Rig()
    dt = rig.model.opt.timestep
    knee, hip = JOINTS.index("left_knee_joint"), JOINTS.index("left_hip_pitch_joint")
    base = np.zeros(12)
    base[knee] = math.radians(40)
    rig.data.qpos[rig.adr] = base
    for _ in range(500):
        rig.step(base)
    errors = []
    amplitude = math.radians(10)
    for i in range(round(2 / dt)):
        t = i * dt
        phase = 2 * math.pi * frequency * t
        target, velocity = base.copy(), np.zeros(12)
        for joint in (knee, hip):
            target[joint] += amplitude * math.sin(phase)
            velocity[joint] = amplitude * 2 * math.pi * frequency * math.cos(phase)
        q = rig.step(target, velocity)
        if t > 0.5:
            errors.append(np.degrees(target - q)[[knee, hip]])
    worst = float(np.abs(errors).max())
    record(f"actuador.seguimiento_{frequency:g}hz", worst, "< 2" if frequency <= 2 else "informativa", "°")
    if frequency <= 2:
        assert worst < 2.0


def test_feet_collide_when_a_leg_crosses():
    """Causa de la asimetría de hip_roll: hacia dentro, los pies se tocan."""
    r = step_response(JOINTS.index("right_hip_roll_joint"), STEP_DEG)
    record("actuador.hip_roll_hacia_dentro.error_final", r["final"], "> 0: choca con el otro pie", "°")
    assert r["final"] > 0.05


def test_torque_saturation_is_enforced():
    rig = Rig()
    target = np.zeros(12)
    target[JOINTS.index("left_knee_joint")] = math.radians(120)
    peak = 0.0
    for _ in range(300):
        rig.step(target)
        peak = max(peak, float(np.abs(rig.data.actuator_force[rig.act]).max()))
    record("actuador.par_saturado", peak, "= 20", "N·m")
    assert abs(peak - 20.0) < 1e-6


def test_control_runs_at_simulation_rate():
    """Documenta una idealización: el control va a 500 Hz, con estado sin retardo."""
    model = load_model()
    record("actuador.frecuencia_control", 1 / model.opt.timestep, "informativa: idealizada", "Hz")
    assert model.opt.timestep == 0.002
