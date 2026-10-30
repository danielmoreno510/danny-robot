"""Marcha (auditoría 5): se juzga el movimiento real, paso a paso."""

import numpy as np
import pytest

import walk_biped
from biped_control import SIDES
from helpers import E11, E13, STRAIGHT, run_walk
from conftest import foot_corners, polygon_margin, record

MAX_SLIDE = 0.005  # m por apoyo
MIN_CLEARANCE = 0.02  # m con el pie libre arriba
MAX_TILT = 5.0
MAX_TRACKING = 1.0
MAX_IMPACT = 2.5  # veces el peso, al aterrizar
MAX_PLACEMENT = 0.005


def analyse(biped):
    a = biped.arrays()
    targets = biped.log["target"]
    plan_left = np.array([p.left_sole for p in targets])
    plan_right = np.array([p.right_sole for p in targets])
    plan = np.stack([plan_left, plan_right], axis=1)
    out = {}
    # Deslizamiento: recorrido horizontal mientras el pie carga
    forces = np.stack([a["left_force"], a["right_force"]], axis=1)
    slide = 0.0
    for foot in range(2):
        loaded = forces[:, foot] > 10.0
        moved = np.linalg.norm(np.diff(a["soles"][:, foot, :2], axis=0), axis=1)
        run = 0.0
        for flag, delta in zip(loaded[1:], moved):
            run = run + delta if flag else 0.0
            slide = max(slide, run)
    out["slide"] = slide
    # Holgura del pie libre
    high = plan[:, :, 2] > 0.8 * plan[:, :, 2].max()
    out["clearance"] = float(a["soles"][:, :, 2][high].min()) if high.any() else 0.0
    # Seguimiento de la trayectoria del pie libre
    swinging = plan[:, :, 2] > 0.001
    out["swing_error"] = float(np.linalg.norm((a["soles"] - plan)[swinging], axis=1).max()) if swinging.any() else 0.0
    # Impacto al aterrizar y contactos fantasma
    out["impact"] = float(forces.max() / (a["left_force"] + a["right_force"]).mean())
    in_air = plan[:, :, 2] > 0.01
    out["contact_in_air"] = float(forces[in_air].max()) if in_air.any() else 0.0
    # Rangos y torso
    model = biped.model
    ranges = np.array([model.joint(j).range for j in walk_biped.JOINTS])
    out["range_margin"] = float(np.degrees(min((a["q"] - ranges[:, 0]).min(), (ranges[:, 1] - a["q"]).min())))
    out["tilt"] = float(a["tilt"].max())
    out["torque"] = float(np.abs(a["torque"]).max())
    out["speed"] = float(np.abs(a["qd"]).max())
    out["foot_tilt"] = float(a["foot_tilt"].max())
    return out, a


CASES = {
    "e11": (E11, 1.0, 1.5),
    "e12": (E11, 1.27, 0.25),
    "e13_giro": (E13, 1.27, 0.25),
    "recta_x2": (STRAIGHT, 2.0, 0.25),
}


@pytest.mark.parametrize("name", list(CASES))
def test_walk_measured_states(name, monkeypatch):
    gait, speed, hold = CASES[name]
    biped, results, saturation, collided, model, error = run_walk(monkeypatch, gait, speed, hold)
    assert not error, error
    stats, a = analyse(biped)
    prefix = f"marcha.{name}"
    record(f"{prefix}.pasos", (len(results) - 1) // 4, ">= 4")
    record(f"{prefix}.deslizamiento", 1000 * stats["slide"], f"< {1000 * MAX_SLIDE:g}", "mm")
    record(f"{prefix}.pie_libre_min", 1000 * stats["clearance"], f"> {1000 * MIN_CLEARANCE:g}", "mm")
    record(f"{prefix}.error_pie_libre", 1000 * stats["swing_error"], "< 15", "mm")
    record(f"{prefix}.impacto", stats["impact"], f"< {MAX_IMPACT} × peso medio")
    record(f"{prefix}.margen_rangos", stats["range_margin"], "> 0", "°")
    record(f"{prefix}.incl_max", stats["tilt"], f"< {MAX_TILT}", "°")
    record(f"{prefix}.par_max", stats["torque"], "< 20", "N·m")
    record(f"{prefix}.vel_max", stats["speed"], "informativa", "rad/s")
    record(f"{prefix}.incl_pie_max", stats["foot_tilt"], "informativa", "°")
    record(f"{prefix}.error_movimiento", max(r.tracking_error_deg for r in results), f"< {MAX_TRACKING}", "°")
    record(f"{prefix}.duracion", a["t"][-1], "informativa", "s")
    assert (len(results) - 1) // 4 >= 4
    assert stats["slide"] < MAX_SLIDE
    assert stats["clearance"] > MIN_CLEARANCE
    assert stats["swing_error"] < 0.015
    assert stats["impact"] < MAX_IMPACT
    assert stats["range_margin"] > 0
    assert stats["tilt"] < MAX_TILT
    assert stats["torque"] < 20.0
    assert stats["contact_in_air"] == 0.0
    assert not collided
    assert not np.any(saturation > 0)


@pytest.mark.parametrize("name", ["e12", "e13_giro"])
def test_step_length_and_placement(name, monkeypatch):
    gait, speed, hold = CASES[name]
    biped, results, _, _, _, error = run_walk(monkeypatch, gait, speed, hold)
    assert not error, error
    worst = max(np.nanmax(r.foot_slip) for r in results)
    record(f"marcha.{name}.colocacion_pie", 1000 * worst, f"< {1000 * MAX_PLACEMENT:g}", "mm")
    assert worst < MAX_PLACEMENT


def test_contact_transitions_alternate(monkeypatch):
    """Secuencia de apoyos real: doble, derecho al aire, doble, izquierdo al aire..."""
    biped, results, *_ , error = run_walk(monkeypatch, E11, 1.27, 0.25)
    assert not error, error
    a = biped.arrays()
    left, right = a["left_force"] > 2.0, a["right_force"] > 2.0
    state = np.where(left & right, 0, np.where(left, 1, np.where(right, 2, 3)))
    changes = state[np.r_[True, state[1:] != state[:-1]]]
    single = [s for s in changes if s in (1, 2)]
    alternating = sum(single[i] != single[i + 1] for i in range(len(single) - 1))
    air = int((state == 3).sum())
    record("marcha.transiciones.pasos_simples", len(single), "informativa")
    record("marcha.transiciones.alternan", alternating, ">= 3")
    record("marcha.transiciones.pasos_sin_contacto", air, "= 0")
    assert alternating >= 3
    assert air == 0


@pytest.mark.parametrize("speed", [0.7, 1.0, 1.27, 1.6, 2.0, 2.3, 2.5, 3.0])
def test_speed_sweep(speed, monkeypatch):
    """Barrido de velocidad, recta. Mide; solo exige hasta ×2."""
    biped, results, saturation, collided, model, error = run_walk(monkeypatch, STRAIGHT, speed, 0.25)
    if error:
        record(f"marcha.barrido_x{speed}.resultado", None, "cae: " + error[:50])
        ok = False
    else:
        failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
        record(f"marcha.barrido_x{speed}.fallos", len(failures), "= 0 hasta ×2")
        record(f"marcha.barrido_x{speed}.duracion", biped.log["t"][-1], "informativa", "s")
        ok = not failures
    if speed <= 2.0:
        assert ok, error


def test_walk_support_margin(monkeypatch):
    """CdM y centro de presiones reales frente al polígono real, en toda la marcha."""
    biped, *_, error = run_walk(monkeypatch, STRAIGHT, 1.27, 0.25)
    assert not error, error
    a = biped.arrays()
    targets = biped.log["target"]
    com_margin, cop_margin = np.inf, np.inf
    for i in range(0, len(targets), 5):
        corners = []
        for foot, side in enumerate(SIDES):
            if a[f"{side}_force"][i] > 5.0:
                corners.append(foot_corners(a["soles"][i, foot], 0))
        if not corners:
            continue
        polygon = np.vstack(corners)
        com_margin = min(com_margin, polygon_margin(a["com"][i, :2], polygon))
        if not np.isnan(a["cop"][i]).any():
            cop_margin = min(cop_margin, polygon_margin(a["cop"][i], polygon))
    record("marcha.margen_cdm_estatico", 1000 * com_margin, "> 0", "mm")
    record("marcha.margen_cop_dinamico", 1000 * cop_margin, "> 0", "mm")
    assert com_margin > 0
    assert cop_margin > 0


def test_deterministic(monkeypatch):
    first = run_walk(monkeypatch, STRAIGHT, 1.27, 0.25)[0].arrays()
    second = run_walk(monkeypatch, STRAIGHT, 1.27, 0.25)[0].arrays()
    difference = float(np.abs(first["q"] - second["q"]).max())
    record("software.determinismo", difference, "= 0", "rad")
    assert difference == 0.0
