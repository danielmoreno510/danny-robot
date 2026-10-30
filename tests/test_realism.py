"""
Sensibilidad a lo que un robot real tendrá (auditoría 9). La marcha del E13 se
repite con cada efecto, en el bípedo y en el humanoide. Son medidas: no se exige
pasar, se registra qué rompe.
"""

import mujoco
import numpy as np
import pytest

import robot_spec as spec
import walk_biped
from conftest import MODEL_PATH, ROOT, load_model, record
from helpers import RealisticBiped, model_with_motor_lag, run_walk


@pytest.fixture(params=["biped", "humanoid"])
def robot(request):
    return request.param


def load(robot):
    if robot == "biped":
        return load_model()
    return mujoco.MjModel.from_xml_path(str(ROOT / "simulation" / "models" / spec.HUMANOID.file))


def outcome(result):
    biped, results, saturation, collided, model, error = result
    if error:
        return "cae", None
    failures = walk_biped.check_acceptance(results, saturation, walk_biped.total_weight(model), collided)
    slip = max(np.nanmax(r.foot_slip) for r in results)
    return ("pasa" if not failures else f"{len(failures)} fallos"), 1000 * slip


def note(robot, name, result):
    verdict, slip = outcome(result)
    prefix = "realismo" if robot == "biped" else "humanoide.realismo"
    record(f"{prefix}.{name}", slip, verdict, "mm (pie fuera de sitio)")


@pytest.mark.parametrize("period", [1, 2, 4, 5, 10, 20])
def test_control_rate(period, robot, monkeypatch):
    class Biped(RealisticBiped):
        control_period = period

    note(robot, f"control_{500 // period}hz", run_walk(monkeypatch, biped_class=Biped, model=load(robot)))


@pytest.mark.parametrize("steps", [1, 2, 3, 5, 10])
def test_delay(steps, robot, monkeypatch):
    class Biped(RealisticBiped):
        delay_steps = steps

    note(robot, f"retardo_{2 * steps}ms", run_walk(monkeypatch, biped_class=Biped, model=load(robot)))


@pytest.mark.parametrize("sigma", [0.05, 0.1, 0.5, 1.0])
def test_encoder_noise(sigma, robot, monkeypatch):
    class Biped(RealisticBiped):
        encoder_noise = np.radians(sigma)

    note(robot, f"ruido_encoder_{sigma}deg", run_walk(monkeypatch, biped_class=Biped, model=load(robot)))


@pytest.mark.parametrize("friction", [0.05, 0.2, 0.5, 1.0])
def test_joint_friction(friction, robot, monkeypatch):
    model = load(robot)
    leg_dofs = np.arange(6, model.nv)
    model.dof_frictionloss[leg_dofs] = friction
    note(robot, f"friccion_articular_{friction}nm", run_walk(monkeypatch, model=model, nominal=load(robot)))


@pytest.mark.parametrize("tau", [0.002, 0.005, 0.01, 0.02])
def test_motor_lag(tau, robot, monkeypatch):
    note(robot, f"lag_motor_{1000 * tau:g}ms",
         run_walk(monkeypatch, model=model_with_motor_lag(tau, spec.ROBOTS[robot]), nominal=load(robot), biped_class=RealisticBiped))


@pytest.mark.parametrize("mu", [0.3, 0.6, 1.5])
def test_ground_friction(mu, robot, monkeypatch):
    model = load(robot)
    model.geom_friction[:, 0] = mu
    note(robot, f"friccion_suelo_{mu}", run_walk(monkeypatch, model=model, nominal=load(robot)))


@pytest.mark.parametrize("scale", [0.5, 2.0, 5.0])
def test_contact_softness(scale, robot, monkeypatch):
    model = load(robot)
    model.geom_solref[:, 0] *= scale
    note(robot, f"contacto_blando_x{scale}", run_walk(monkeypatch, model=model, nominal=load(robot)))


@pytest.mark.parametrize("shift", [0.005, 0.01, 0.02])
def test_mass_distribution(shift, robot, monkeypatch):
    model = load(robot)
    for body in ("pelvis", "torso"):  # el bípedo no tiene torso propio
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body) >= 0:
            model.body_ipos[model.body(body).id][0] += shift
    mujoco.mj_setConst(model, mujoco.MjData(model))
    note(robot, f"cdm_torso_{1000 * shift:g}mm", run_walk(monkeypatch, model=model, nominal=load(robot)))


@pytest.mark.parametrize("degrees", [0.1, 0.25, 0.5, 1.0])
def test_floor_tilt(degrees, robot, monkeypatch):
    model = load(robot)
    angle = np.radians(degrees)
    model.opt.gravity[:] = (-9.81 * np.sin(angle), 0.0, -9.81 * np.cos(angle))
    note(robot, f"suelo_{degrees}deg", run_walk(monkeypatch, model=model, nominal=load(robot)))


def test_backlash_is_not_tested():
    """El modelo no tiene holgura mecánica: se registra como NOT TESTED."""
    record("realismo.backlash", None, "NOT TESTED: MuJoCo no lo modela directamente")
    assert MODEL_PATH.exists()
