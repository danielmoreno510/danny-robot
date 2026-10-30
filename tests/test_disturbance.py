"""
Perturbaciones (auditoría 6). Distingue la respuesta pasiva, por rigidez de los
servos, de una recuperación con realimentación, que hoy no existe.
"""

import mujoco
import numpy as np
import pytest

import push_biped
import walk_biped
from biped_control import Pose, center_com_over, joint_targets, pelvis_height
from conftest import RecordingBiped, load_model, record
from helpers import STRAIGHT, run_push, run_walk

DIRECTIONS = push_biped.DIRECTIONS


@pytest.mark.parametrize("name", list(DIRECTIONS))
def test_response_at_defined_limit(name, monkeypatch):
    force = push_biped.REQUIRED_FORCE[name]
    biped, trial, model = run_push(monkeypatch, DIRECTIONS[name], force)
    a = biped.arrays()
    start = round(push_biped.SETTLE_TIME / model.opt.timestep)
    after = a["tilt"][start:]
    settled = np.where(after > 0.5)[0]
    recovery = (settled[-1] + 1) * model.opt.timestep - push_biped.PUSH_TIME if len(settled) else 0.0
    displacement = np.linalg.norm(a["com"][start:, :2] - a["com"][start, :2], axis=1).max()
    targets = joint_targets(biped.log["target"][start])
    tracking = np.degrees(np.abs(a["q"][start:] - targets).max())
    air = float(((a["left_force"][start:] < 1) & (a["right_force"][start:] < 1)).sum() * model.opt.timestep)
    prefix = f"empujon.{name}"
    record(f"{prefix}.fuerza", force, "límite definido", "N")
    record(f"{prefix}.tiempo_recuperacion", recovery, "< 2", "s")
    record(f"{prefix}.incl_max", a["tilt"][start:].max(), "< 15", "°")
    record(f"{prefix}.desplazamiento_cdm", 1000 * displacement, "informativa", "mm")
    record(f"{prefix}.error_articular", tracking, "informativa", "°")
    record(f"{prefix}.pies_movidos", 1000 * trial.foot_drift, "< 20", "mm")
    record(f"{prefix}.tiempo_sin_contacto", 1000 * air, "informativa", "ms")
    assert trial.recovered
    assert recovery < 2.0
    assert a["tilt"][start:].max() < 15.0


def test_measured_maximum_push():
    model = load_model()
    data = mujoco.MjData(model)
    for name, direction in DIRECTIONS.items():
        best, _ = push_biped.strongest_push(model, data, direction)
        record(f"empujon.{name}.maximo", best, f">= {push_biped.REQUIRED_FORCE[name]:g}", "N")
        assert best >= push_biped.REQUIRED_FORCE[name]


def test_response_is_passive_not_feedback(monkeypatch):
    """El plan no cambia con el empujón: no hay recuperación con realimentación."""
    biped, *_ = run_push(monkeypatch, DIRECTIONS["adelante"], 20.0)
    vectors = np.array([p.vector() for p in biped.log["target"]])
    change = float(np.abs(vectors - vectors[0]).max())
    record("empujon.cambio_del_plan", change, "0 = sin realimentación: equilibrio activo NO IMPLEMENTADO")
    assert change == 0.0


def one_foot(side, force, direction):
    model = load_model()
    data = mujoco.MjData(model)
    biped = RecordingBiped(model, data)
    swing = "right" if side == "left" else "left"
    width = 0.05
    stand = Pose(np.array([0, 0, pelvis_height(56)]), np.array([0, width, 0.0]), np.array([0, -width, 0.0]))
    pose = center_com_over(biped, stand.with_foot(swing, stand.sole(swing) + [0, 0, 0.03]), side, 0.012)
    biped.set_pose(data, pose)
    biped.previous_target = None
    dt = model.opt.timestep
    try:
        for _ in range(500):
            biped.step(pose, swing)
        for i in range(round(3.1 / dt)):
            data.xfrc_applied[biped.pelvis_id, :3] = direction * force if i * dt < 0.1 else 0.0
            biped.step(pose, swing)
    except RuntimeError:
        return False
    return biped.tilt_deg() < 2.0


@pytest.mark.parametrize("side", ["left", "right"])
def test_push_on_one_foot(side):
    best = {}
    for name, direction in DIRECTIONS.items():
        value = 0
        for force in range(1, 31):
            if not one_foot(side, float(force), direction):
                break
            value = force
        best[name] = value
        record(f"empujon.un_pie_{side}.{name}", value, "informativa", "N")
    assert min(best.values()) >= 1


class PushedWalk(RecordingBiped):
    push_force = 0.0
    push_start = 0.0

    def step(self, pose, swing):
        t = self.data.time
        active = self.push_start <= t < self.push_start + 0.1
        self.data.xfrc_applied[self.pelvis_id, :3] = [self.push_force if active else 0.0, 0.0, 0.0]
        super().step(pose, swing)


@pytest.mark.parametrize("force", [2.0, 5.0, 10.0, 15.0])
def test_push_while_walking(force, monkeypatch):
    class Biped(PushedWalk):
        push_force = force
        push_start = 6.0

    _, results, saturation, collided, model, error = run_walk(monkeypatch, STRAIGHT, 1.27, 0.25, biped_class=Biped)
    if error:
        record(f"empujon.andando_{force:g}n", None, "cae")
        return
    failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
    record(f"empujon.andando_{force:g}n", len(failures), "fallos de criterio; informativa")
