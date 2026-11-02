"""
Danny - Comprueba que el robot se sostiene con el servo BAM, etapa 3.1.

Parte de la postura `stand` y simula 3 s a varias tensiones, sin política. Luego sigue las
tareas lentas en simulación directa. Compensa la carga de cada postura, porque el servo es
blando y necesita error para hacer par.
Escribe results/standing_check.json y sale con código 1 si no cumple.

Uso: .venv/bin/python analysis/check_standing.py
"""

import json
import sys

import mujoco
import numpy as np

import danny_motion
import danny_servo
import danny_sim

OUTPUT = danny_sim.ROOT / "results" / "standing_check.json"
SECONDS = 3.0
MAXIMUM_TILT_DEG = 2.0
MAXIMUM_DROP_MM = 5.0
VOLTS = (8.4, 7.4, 6.0)
TASKS = ("squat", "single_support", "rise_from_crouch")
MAXIMUM_TASK_TILT_DEG = 45.0  # las cuclillas inclinan la pelvis a propósito: esto mide caídas
FIRMWARE_GAINS = (5.0, 10.0)  # la del XGO-Duck y el doble: se decide al medir el servo, en la 5.1
TASK_VOLTS = (7.4, 6.0)


def tilt_degrees(data):
    up = np.zeros(3)
    mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), data.qpos[3:7])
    return float(np.degrees(np.arccos(np.clip(up[2], -1, 1))))


def simulate(pose, volts):
    model = danny_sim.build_model()
    data = mujoco.MjData(model)
    servos = danny_servo.Servos(model, volts)
    target = danny_servo.gravity_targets(model, data, pose, servos)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = pose
    mujoco.mj_forward(model, data)
    start_height = float(data.subtree_com[0][2])
    worst_tilt, peak = 0.0, 0.0
    for _ in range(int(SECONDS / model.opt.timestep)):
        torque = servos.step(data, target)
        worst_tilt = max(worst_tilt, tilt_degrees(data))
        peak = float(np.abs(torque).max())
    return {
        "stiffness_nm_per_rad": servos.stiffness(),
        "worst_tilt_deg": worst_tilt,
        "center_of_mass_drop_mm": 1000 * (start_height - float(data.subtree_com[0][2])),
        "final_peak_torque_nm": peak,
    }


def follow_task(trajectory, volts, firmware_gain):
    """Peor inclinación, en grados, al seguir la tarea y aguantar la última postura 1 s."""
    model = danny_sim.build_model()
    data = mujoco.MjData(model)
    servos = danny_servo.Servos(model, volts, firmware_gain)
    targets = danny_servo.trajectory_targets(model, data, trajectory, servos)
    mujoco.mj_resetData(model, data)
    data.qpos[:] = trajectory[0]
    mujoco.mj_forward(model, data)
    worst_tilt = 0.0
    for step in range(int((len(targets) / danny_motion.RATE_HZ + 1.0) / model.opt.timestep)):
        sample = min(int(step * model.opt.timestep * danny_motion.RATE_HZ), len(targets) - 1)
        servos.step(data, targets[sample])
        worst_tilt = max(worst_tilt, tilt_degrees(data))
    return worst_tilt


def main():
    rigid = danny_sim.build_model(actuated=False)
    pose = danny_motion.stand_pose(rigid)
    trajectories = {task: danny_motion.task_trajectory(rigid, task) for task in TASKS}
    result = {
        "seconds": SECONDS,
        "volts": {str(volts): simulate(pose, volts) for volts in VOLTS},
        "tasks": {
            task: {str(gain): {str(volts): follow_task(trajectory, volts, gain) for volts in TASK_VOLTS}
                   for gain in FIRMWARE_GAINS}
            for task, trajectory in trajectories.items()},
        "failures": [],
    }
    for volts, run in result["volts"].items():
        if run["worst_tilt_deg"] > MAXIMUM_TILT_DEG:
            result["failures"].append(f"{volts} V: inclinación de {run['worst_tilt_deg']:.2f}°")
        if run["center_of_mass_drop_mm"] > MAXIMUM_DROP_MM:
            result["failures"].append(f"{volts} V: el centro de masas baja {run['center_of_mass_drop_mm']:.1f} mm")
    for task, gains in result["tasks"].items():
        for gain, runs in gains.items():
            for volts, tilt in runs.items():
                if tilt > MAXIMUM_TASK_TILT_DEG:
                    result["failures"].append(f"{task}, ganancia {gain} a {volts} V: inclinación de {tilt:.1f}°")
    OUTPUT.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1, ensure_ascii=False))
    return 1 if result["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
