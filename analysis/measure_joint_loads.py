"""
Danny - Mide par y velocidad por articulación en las tareas de medida, etapa 3.2.

Aún no hay políticas: usa trayectorias paramétricas con cinemática inversa y dinámica inversa
con el reparto de fuerzas de contacto de menor par. Son herramientas de medida, no animaciones.
Con contactos unilaterales sin imponer: por eso informa de fuerza normal y centro de presiones.
Escribe results/joint_loads.json y sale con código 1 si alguna tarea supera los límites.

Uso: .venv/bin/python analysis/measure_joint_loads.py
"""

import json
import sys

import mujoco
import numpy as np

import danny_motion
import danny_servo
import danny_sim

OUTPUT = danny_sim.ROOT / "results" / "joint_loads.json"
RATE_HZ = danny_motion.RATE_HZ
# Criterio de la etapa 3.3, en N·m: bloqueo del modelo BAM entre 1,5, provisional
TORQUE_LIMIT_NM = {7.4: 0.61, 6.0: 0.49}
MAXIMUM_IK_ERROR_MM = 5.0
FOOT_MARGIN_MM = 0.0


def contact_forces(model, data, contacts, demand):
    """Reparte la demanda: cumple la base y minimiza el par. contacts: [(site, con_momento)]."""
    rows = []
    for name, with_moment in contacts:
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, site)
        rows.append(jacp)
        if with_moment:
            rows.append(jacr)
    jacobian = np.vstack(rows)
    base, joints = jacobian[:, :6].T, jacobian[:, 6:].T
    count = jacobian.shape[0]
    system = np.zeros((count + 6, count + 6))
    system[:count, :count] = joints.T @ joints + 1e-8 * np.eye(count)
    system[:count, count:] = base.T
    system[count:, :count] = base
    right = np.concatenate([joints.T @ demand[6:], demand[:6]])
    forces = np.linalg.lstsq(system, right, rcond=None)[0][:count]
    return forces, demand[6:] - joints @ forces


def foot_margin(force, moment):
    """Distancia del centro de presiones al borde del pie, en mm. Negativa: fuera."""
    if force[2] <= 0:
        return -1e3
    cop_x, cop_y = -moment[1] / force[2], moment[0] / force[2]
    half_x, half_y = danny_sim.FOOT_HALF
    return 1000 * min(half_x - abs(cop_x), half_y - abs(cop_y))


def measure_task(model, solver, generator, seconds, start_pitch=None):
    data = mujoco.MjData(model)
    solver.q = solver.seed.copy()
    if start_pitch is not None:
        solver.q = model.qpos0.copy()
        mujoco.mju_mat2Quat(solver.q[3:7], danny_motion.pitched(start_pitch).flatten())
    solver.solve(*generator(0.0)[:3], iterations=600)
    times = np.arange(0.0, seconds, 1 / RATE_HZ)
    poses, ik_errors, contact_sets = [], [], []
    for time in times:
        targets, com, posture, contacts = generator(float(time))
        q, error = solver.solve(targets, com, posture, iterations=30)
        poses.append(q)
        ik_errors.append(error)
        contact_sets.append(contacts)

    step = 1 / RATE_HZ
    velocities = np.zeros((len(poses), model.nv))
    for index in range(len(poses) - 1):
        mujoco.mj_differentiatePos(model, velocities[index], step, poses[index], poses[index + 1])
    accelerations = np.gradient(velocities, step, axis=0)

    torques, margins, normals = [], [], []
    for index in range(2, len(poses) - 2):
        data.qpos[:] = poses[index]
        data.qvel[:] = velocities[index]
        data.qacc[:] = accelerations[index]
        mujoco.mj_fwdPosition(model, data)
        mujoco.mj_fwdVelocity(model, data)
        inertial = np.zeros(model.nv)
        mujoco.mj_mulM(model, data, inertial, accelerations[index])
        demand = inertial + data.qfrc_bias
        forces, torque = contact_forces(model, data, contact_sets[index], demand)
        torques.append(torque)
        cursor = 0
        step_margin, step_normal = [], []
        for name, with_moment in contact_sets[index]:
            force = forces[cursor:cursor + 3]
            step_normal.append(float(force[2]))
            # Con dos pies, el reparto de menor par no es único: solo vale el apoyo simple
            if with_moment and len(contact_sets[index]) == 1:
                step_margin.append(foot_margin(force, forces[cursor + 3:cursor + 6]))
            cursor += 6 if with_moment else 3
        if step_margin:
            margins.append(min(step_margin))
        normals.append(min(step_normal))

    torques = np.array(torques)
    speeds = np.abs(velocities[2:-2, 6:])
    return torques, speeds, np.array(ik_errors), np.array(margins), np.array(normals)


def minimum_volts(torques, speeds):
    """Tensión más baja a la que la curva del modelo da todo lo que pide la tarea."""
    torque = danny_servo.required_motor_torque(torques, speeds)
    for volts in np.arange(4.0, 8.41, 0.1):
        if (torque <= danny_sim.available_torque(float(volts), speeds)).all():
            return round(float(volts), 1)
    return None


def main():
    model = danny_sim.build_model(actuated=False)
    names = danny_sim.hinge_names(model)
    solver = danny_motion.Solver(model)
    report = {"limits_nm": {str(k): v for k, v in TORQUE_LIMIT_NM.items()},
              "model_stall_nm": {str(v): danny_sim.stall_torque(v) for v in danny_sim.BATTERY_VOLTS},
              "tasks": {}}
    failures = []
    for task, (generator, seconds) in danny_motion.TASKS.items():
        torques, speeds, ik_errors, margins, normals = measure_task(
            model, solver, generator, seconds, danny_motion.START_PITCH.get(task))
        joints = {}
        for index, name in enumerate(names):
            speed = speeds[:, index]
            torque = danny_servo.required_motor_torque(torques[:, index], speed)
            usage = {
                str(volts): float((torque / np.maximum(danny_sim.available_torque(volts, speed), 1e-3)).max())
                for volts in danny_sim.BATTERY_VOLTS
            }
            joints[name] = {
                "peak_torque_nm": float(torque.max()),
                "rms_torque_nm": float(np.sqrt((torque ** 2).mean())),
                "peak_speed_rad_s": float(speed.max()),
                "usage_of_model_curve": usage,
            }
        peak = max(joints, key=lambda name: joints[name]["peak_torque_nm"])
        fastest = max(joints, key=lambda name: joints[name]["peak_speed_rad_s"])
        summary = {
            "worst_ik_error_mm": float(1000 * ik_errors.max()),
            "minimum_foot_margin_mm": float(margins.min()) if len(margins) else None,
            "minimum_normal_force_n": float(normals.min()),
            "peak_joint": peak,
            "peak_torque_nm": joints[peak]["peak_torque_nm"],
            "fastest_joint": fastest,
            "peak_speed_rad_s": joints[fastest]["peak_speed_rad_s"],
            "minimum_volts": minimum_volts(torques, speeds),
            "peak_usage_of_curve": {
                str(volts): max(j["usage_of_model_curve"][str(volts)] for j in joints.values())
                for volts in danny_sim.BATTERY_VOLTS
            },
            "joints": joints,
        }
        report["tasks"][task] = summary
        for volts, limit in TORQUE_LIMIT_NM.items():
            if summary["peak_torque_nm"] > limit:
                failures.append(f"{task}: {peak} {summary['peak_torque_nm']:.2f} N·m a {volts} V, límite {limit}")
        if summary["peak_speed_rad_s"] > danny_sim.no_load_speed(6.0):
            failures.append(f"{task}: {fastest} a {summary['peak_speed_rad_s']:.1f} rad/s")
        if summary["worst_ik_error_mm"] > MAXIMUM_IK_ERROR_MM:
            failures.append(f"{task}: postura no alcanzada, {summary['worst_ik_error_mm']:.1f} mm")
        if len(margins) and margins.min() < FOOT_MARGIN_MM:
            failures.append(f"{task}: centro de presiones fuera del pie, {margins.min():.1f} mm")
        if normals.min() < -0.01:
            failures.append(f"{task}: un contacto tendría que tirar del suelo, {normals.min():.2f} N")
        print(f"{task:20s} par {summary['peak_torque_nm']:.3f} ({peak}), vel {summary['peak_speed_rad_s']:.2f} ({fastest}), "
              f"ik {summary['worst_ik_error_mm']:.2f} mm, cop {summary['minimum_foot_margin_mm']}, "
              f"{summary['minimum_volts']} V, "
              f"normal {summary['minimum_normal_force_n']:.2f}")
    report["failures"] = failures
    report["warnings"] = [
        f"{task}: pide el {100 * usage:.0f} % de la curva del modelo a {volts} V"
        for task, summary in report["tasks"].items()
        for volts, usage in summary["peak_usage_of_curve"].items() if usage > 1
    ]
    print(json.dumps(report["warnings"], indent=1, ensure_ascii=False))
    OUTPUT.write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(failures, indent=1, ensure_ascii=False))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
