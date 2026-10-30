"""
Utilidades de la suite de validación de la fase 3.

Mide estados simulados, nunca comandos: RecordingBiped guarda en cada paso lo
que de verdad hace el robot. record() deja cada métrica, con su límite, en
results/phase3_metrics.json: el informe de validación cita ese archivo.
"""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "scripts"))

from biped_control import FOOT_HALF, JOINTS, SIDES, Biped, rotation_z  # noqa: E402
from robot_spec import POSITION_SENSORS  # noqa: E402

MODEL_PATH = ROOT / "simulation" / "models" / "danny_biped.xml"
METRICS_PATH = ROOT / "results" / "phase3_metrics.json"
_metrics: dict[str, dict] = {}


def record(name: str, value, limit: str = "informativa", unit: str = "") -> None:
    """Guarda una métrica medida. 'limit' es el criterio, como texto."""
    value = None if value is None else (float(value) if np.isscalar(value) else value)
    _metrics[name] = {"value": value, "limit": limit, "unit": unit}


def pytest_sessionfinish(session):
    METRICS_PATH.parent.mkdir(exist_ok=True)
    old = json.loads(METRICS_PATH.read_text()) if METRICS_PATH.exists() else {}
    old.update(_metrics)
    METRICS_PATH.write_text(json.dumps(dict(sorted(old.items())), indent=1, ensure_ascii=False) + "\n")


def load_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MODEL_PATH))


@pytest.fixture
def model():
    return load_model()


def contact_summary(biped: Biped) -> dict:
    """Por pie: fuerza normal, centro de presiones y número de contactos."""
    model, data = biped.model, biped.data
    geoms = {model.geom(f"{s}_foot_geom").id: s for s in SIDES}
    out = {s: {"force": 0.0, "moment": np.zeros(2), "count": 0} for s in SIDES}
    wrench = np.zeros(6)
    for i in range(data.ncon):
        contact = data.contact[i]
        side = geoms.get(int(contact.geom1), geoms.get(int(contact.geom2)))
        if side is None:
            continue
        mujoco.mj_contactForce(model, data, i, wrench)
        normal = wrench[0]
        out[side]["force"] += normal
        out[side]["moment"] += normal * contact.pos[:2]
        out[side]["count"] += 1
    return out


def foot_corners(center: np.ndarray, yaw: float) -> np.ndarray:
    half = FOOT_HALF
    corners = np.array([[half[0], half[1]], [half[0], -half[1]], [-half[0], -half[1]], [-half[0], half[1]]])
    rotation = rotation_z(yaw)[:2, :2]
    return center[:2] + corners @ rotation.T


def convex_hull(points: np.ndarray) -> np.ndarray:
    pts = sorted(map(tuple, points))

    def half(chain):
        out = []
        for p in chain:
            while len(out) >= 2:
                u, v = np.subtract(out[-1], out[-2]), np.subtract(p, out[-2])
                if u[0] * v[1] - u[1] * v[0] > 0:
                    break
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def polygon_margin(point: np.ndarray, polygon: np.ndarray) -> float:
    """Distancia del punto al borde del polígono convexo: negativa si está fuera."""
    hull = convex_hull(polygon)
    margin = np.inf
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b - a
        normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
        margin = min(margin, float((point - a) @ normal))
    return margin


class RecordingBiped(Biped):
    """Biped que guarda en cada paso el estado real del simulador."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.log: dict[str, list] = {key: [] for key in (
            "t", "q", "qd", "torque", "soles", "com", "tilt", "yaw", "pelvis", "contacts",
            "left_force", "right_force", "left_count", "right_count", "target", "cop",
            "foot_tilt", "ctrl",
        )}

    def step(self, pose, swing):
        super().step(pose, swing)
        data, log = self.data, self.log
        contacts = contact_summary(self)
        total = contacts["left"]["force"] + contacts["right"]["force"]
        log["t"].append(data.time)
        log["q"].append(data.qpos[self.joint_qpos].copy())
        log["qd"].append(data.qvel[self.dof_ids].copy())
        log["torque"].append(data.actuator_force[self.actuator_ids].copy())
        log["ctrl"].append(data.ctrl[self.actuator_ids].copy())
        log["soles"].append(self.soles())
        log["com"].append(self.com())
        log["tilt"].append(self.tilt_deg())
        log["yaw"].append(self.yaw_deg())
        log["pelvis"].append(data.xpos[self.pelvis_id].copy())
        log["contacts"].append(data.ncon)
        log["left_force"].append(contacts["left"]["force"])
        log["right_force"].append(contacts["right"]["force"])
        log["left_count"].append(contacts["left"]["count"])
        log["right_count"].append(contacts["right"]["count"])
        log["target"].append(pose)
        cop = (contacts["left"]["moment"] + contacts["right"]["moment"]) / total if total > 1.0 else np.full(2, np.nan)
        log["cop"].append(cop)
        tilts = [np.degrees(np.arccos(min(1.0, data.xmat[f].reshape(3, 3)[2, 2]))) for f in self.foot_ids]
        log["foot_tilt"].append(tilts)

    def arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.log.items() if k != "target"}


__all__ = ["JOINTS", "POSITION_SENSORS"]
