"""Calcula cuánto premia el entrenamiento caminar frente a quedarse quieto, y la rigidez del servo.

Las fórmulas de recompensa son las de mjlab, y el modelo del servo el de BAM en xgoduck_rl.
"""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent

# Firmware Feetech según BAM: duty = kp * error_gain * error_gain_ratio * error de posición.
FIRMWARE_KP = 5.0
ERROR_GAIN = 0.166
ERROR_GAIN_RATIO = 1.321738701681577
MAX_DUTY = 0.97
VOLTAGES = (6.0, 7.4, 8.4)

# Órdenes de velocidad (m/s) y std de track_linear_velocity (m/s).
CASES = {
    "danny_entrenado": {"lin_vel_x": 0.25, "lin_vel_y": 0.10, "std": 0.5},
    "xgoduck_rl": {"lin_vel_x": 0.40, "lin_vel_y": 0.30, "std": 0.25},
    "danny_propuesto": {"lin_vel_x": 0.25, "lin_vel_y": 0.10, "std": 0.125},
}
SAMPLES = 2_000_000


def standing_still(lin_vel_x, lin_vel_y, std, rng):
    """Error y recompensa de seguimiento de un robot quieto, de 0 a 1, con órdenes uniformes."""
    x = rng.uniform(-lin_vel_x, lin_vel_x, SAMPLES)
    y = rng.uniform(-lin_vel_y, lin_vel_y, SAMPLES)
    error = x**2 + y**2
    return float(np.sqrt(error.mean())), float(np.exp(-error / std**2).mean())


def servo_stiffness(kt, resistance):
    """Par por radián de error, a tensión de bloqueo y sin contar el rozamiento."""
    gain = FIRMWARE_KP * ERROR_GAIN * ERROR_GAIN_RATIO
    return {
        "saturation_error_rad": MAX_DUTY / gain,
        "newton_meters_per_rad": {str(volts): gain * volts * kt / resistance for volts in VOLTAGES},
    }


def main():
    rng = np.random.default_rng(0)
    servo = json.loads((ROOT / "analysis/hd1910_bam_parameters.json").read_text())
    results = {"standing_still": {}, "servo": servo_stiffness(servo["kt"], servo["R"])}
    for name, case in CASES.items():
        error, reward = standing_still(case["lin_vel_x"], case["lin_vel_y"], case["std"], rng)
        results["standing_still"][name] = {**case, "error_m_s": error, "tracking_reward": reward}
        print(f"{name}: quieto da {reward:.0%} de la recompensa, con {error:.3f} m/s de error")
    stiffness = results["servo"]["newton_meters_per_rad"]["7.4"]
    print(f"Servo a 7,4 V: {stiffness:.2f} N·m/rad, y satura con {results['servo']['saturation_error_rad']:.2f} rad")
    (ROOT / "results/walk_incentive.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
