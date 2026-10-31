"""
Danny - Comprueba que el robot se sostiene de pie, etapa 3.1.

Simula 3 s con los actuadores en la postura elegida y mide inclinación y altura, no solo si
cae. Escribe results/standing_check.json y sale con código 1 si no cumple.

Uso: .venv/bin/python analysis/check_standing.py
"""

import json
import sys

import mujoco
import numpy as np

import danny_sim

OUTPUT = danny_sim.ROOT / "results" / "standing_check.json"
SECONDS = 3.0
MAXIMUM_TILT_DEG = 2.0
MAXIMUM_DROP_MM = 5.0


def tilt_degrees(data):
    up = np.zeros(3)
    mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), data.qpos[3:7])
    return float(np.degrees(np.arccos(np.clip(up[2], -1, 1))))


def main():
    model = danny_sim.build_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    start_height = float(data.subtree_com[0][2])

    worst_tilt = 0.0
    for _ in range(int(SECONDS / model.opt.timestep)):
        mujoco.mj_step(model, data)
        worst_tilt = max(worst_tilt, tilt_degrees(data))

    drop = 1000 * (start_height - float(data.subtree_com[0][2]))
    peak = float(np.abs(data.actuator_force).max())
    failures = []
    if worst_tilt > MAXIMUM_TILT_DEG:
        failures.append(f"inclinación de {worst_tilt:.2f}°")
    if drop > MAXIMUM_DROP_MM:
        failures.append(f"el centro de masas baja {drop:.1f} mm")
    result = {
        "seconds": SECONDS,
        "worst_tilt_deg": worst_tilt,
        "center_of_mass_drop_mm": drop,
        "final_peak_torque_nm": peak,
        "failures": failures,
    }
    OUTPUT.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1, ensure_ascii=False))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
