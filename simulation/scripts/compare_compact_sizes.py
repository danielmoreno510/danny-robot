"""
Danny - Compara el humanoide compacto a 35, 40 y 45 cm (fase 5).

Masa pieza a pieza, posturas quietas del modelo y marcha humana escalada. Comprueba
además si el modelo se sostiene de pie con los servos, en simulación.

Uso:
  python simulation/scripts/compare_compact_sizes.py          # escribe results/compact_sizes.json
  mjpython simulation/scripts/compare_compact_sizes.py --view  # ver el modelo de 40 cm de pie
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

import compact_spec as compact
from robot_loads import fastest_walk, froude_scaled, support_torques


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "compact_sizes.json"
HEIGHTS = (0.35, 0.40, 0.45)
HOLD = 3.0  # s
PARTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
CASES = (("dos pies, rodilla 30°", 30, ("left", "right"), 0.0),
         ("dos pies, rodilla 60°", 60, ("left", "right"), 0.0),
         ("dos pies, rodilla 90°", 90, ("left", "right"), 0.0),
         ("un pie, rodilla 30°", 30, ("left",), 0.3),
         ("un pie, rodilla 60°", 60, ("left",), 0.3))


def statics(model, data) -> dict:
    cases = {}
    for name, knee, feet, lift in CASES:
        compact.balance_on(model, data, math.radians(knee), feet, lift)
        torques, _ = support_torques(model, data, feet)
        cases[name] = {part: abs(torques[f"left_{part}"]) for part in PARTS}
    return cases


def stand_test(model, data, knee: float) -> dict:
    """Mantiene la pose de pie solo con los servos: mide inclinación, bajada y par."""
    compact.balance_on(model, data, knee, ("left", "right"))
    start = data.qpos.copy()
    joints = model.actuator_trnid[:, 0]
    target = start[model.jnt_qposadr[joints]]
    data.ctrl[:] = target
    height = data.qpos[2]
    peak, tilt = 0.0, 0.0
    for _ in range(round(HOLD / model.opt.timestep)):
        mujoco.mj_step(model, data)
        peak = max(peak, float(np.abs(data.actuator_force).max()))
        tilt = max(tilt, math.degrees(math.acos(min(1.0, abs(data.body("pelvis").xmat[8])))))
    return {"bajada_mm": 1000 * (height - data.qpos[2]), "inclinacion_max": tilt, "par_max": peak}


def study(height: float) -> dict:
    design = compact.Design(height)
    model = compact.build(design).compile()
    data = mujoco.MjData(model)
    assert abs(model.body_subtreemass[1] - design.total_mass()) < 1e-6
    mass = float(model.body_subtreemass[1])
    cases = statics(model, data)
    worst = {part: max(case[part] for case in cases.values()) for part in PARTS}
    gait = froude_scaled(mass, design.leg)
    compact.set_stand(model, data, 0.0)
    top = float(data.geom("head_geom").xpos[2] + design.head_radius)
    com = float(data.subtree_com[1][2])
    return {
        "altura": top,
        "pierna": design.leg,
        "masa": mass,
        "masa_sin_jetson": mass - compact.JETSON_MASS + compact.PI_ZERO_MASS,
        "cdm_altura": com,
        "torso_mm": [2000 * v for v in design.torso_half],
        "pares_quietos": worst,
        "casos": cases,
        "marcha_froude": gait,
        "marcha_c018": fastest_walk(compact.STS3215_C018.no_load_speed, design.leg),
        "marcha_c044": fastest_walk(compact.STS3215_C044.no_load_speed, design.leg),
        "de_pie": stand_test(model, mujoco.MjData(model), math.radians(30)),
    }


def view() -> int:
    model = compact.build(compact.Design(0.40)).compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--view", action="store_true", help="abre el modelo de 40 cm")
    if parser.parse_args().view:
        return view()
    results = {f"{round(h * 100)}": study(h) for h in HEIGHTS}
    RESULTS.write_text(json.dumps(results, indent=1, ensure_ascii=False, sort_keys=True))
    for name, item in results.items():
        print(f"\n{name} cm: altura {item['altura']:.3f} m, masa {item['masa']:.2f} kg "
              f"({item['masa_sin_jetson']:.2f} sin Jetson), CdM {item['cdm_altura']:.3f} m, pecho {item['torso_mm']}")
        print("  quieto:", {k: round(v, 2) for k, v in item["pares_quietos"].items()})
        print("  Froude:", {k: (round(v['par_pico'], 2), round(v['velocidad_pico'], 1)) for k, v in item["marcha_froude"].items()})
        print("  marcha C018:", {k: round(v, 3) for k, v in item["marcha_c018"].items()},
              "C044:", {k: round(v, 3) for k, v in item["marcha_c044"].items()})
        print("  de pie:", {k: round(v, 2) for k, v in item["de_pie"].items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
