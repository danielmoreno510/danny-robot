"""
Danny - Comprueba que el modelo permite dar pasos alternos que avanzan con servos BAM.

Sigue la marcha calculada de danny_motion, con su par previsto y una realimentación mínima de
inclinación, y busca solo esas ganancias. Usa la física del entrenamiento: 5 ms, 10 iteraciones, 50 Hz
y 20 ms de retardo. No es una marcha ni pretende sostenerse: si da zancadas sin deslizar, lo que falta es
aprendizaje, no robot. Escribe results/walk_feasibility.json y sale con código 1 si no las da.

Uso: .venv/bin/python analysis/check_walk_feasibility.py [--firmware-gain 5]
"""

import argparse
import json
import multiprocessing
import sys

import mujoco
import numpy as np

import danny_motion
import danny_servo
import danny_sim

OUTPUT = danny_sim.ROOT / "results" / "walk_feasibility.json"
SECONDS = 6.0
VOLTS = 7.4
CONTROL_HZ = 50
TIMESTEP = 0.005
SOLVER_ITERATIONS = 10
DELAY_CONTROLS = 1
FALL_TILT_DEG = 40.0
MEASURE_TILT_DEG = 25.0  # más inclinado ya es caer, no avanzar
MINIMUM_AIR_SECONDS = 0.06
GENERATIONS, POPULATION, ELITE = 20, 200, 20
# Mínimos y máximos: inclinación y giro sobre tobillos y cadera, y fracción del par previsto
LOW = np.array([-8.0, -1.0, -8.0, -1.0, -6.0, 0.5])
HIGH = np.array([8.0, 1.0, 8.0, 1.0, 6.0, 1.5])
# Fijado antes de esta búsqueda: tres apoyos alternos con zancada real y el pie quieto
MINIMUM_FORWARD_M = 0.15
MINIMUM_ALTERNATING_STEPS = 3
MINIMUM_STRIDE_M = 0.05
MAXIMUM_SLIP_M = 0.02


def walk_plan():
    """Posturas, velocidades y par de la marcha, a RATE_HZ."""
    model = danny_sim.build_model(actuated=False)
    data = mujoco.MjData(model)
    solver = danny_motion.Solver(model)
    solver.q = solver.seed.copy()
    solver.solve(*danny_motion.walk(0.0)[:3], iterations=600)
    step = 1 / danny_motion.RATE_HZ
    poses, contacts = [], []
    for time in np.arange(0.0, SECONDS, step):
        targets, com, posture, contact = danny_motion.walk(float(time))
        poses.append(solver.solve(targets, com, posture, iterations=30)[0].copy())
        contacts.append(contact)
    velocities = np.zeros((len(poses), model.nv))
    for index in range(len(poses) - 1):
        mujoco.mj_differentiatePos(model, velocities[index], step, poses[index], poses[index + 1])
    accelerations = np.gradient(velocities, step, axis=0)
    torques = np.zeros((len(poses), model.nv - 6))
    for index in range(2, len(poses) - 2):
        data.qpos[:], data.qvel[:] = poses[index], velocities[index]
        mujoco.mj_fwdPosition(model, data)
        mujoco.mj_fwdVelocity(model, data)
        inertial = np.zeros(model.nv)
        mujoco.mj_mulM(model, data, inertial, accelerations[index])
        torques[index] = danny_motion.contact_forces(model, data, contacts[index], inertial + data.qfrc_bias)[1]
    return np.array(poses), velocities, torques


class Runner:
    def __init__(self, plan, firmware_gain):
        self.poses, self.velocities, self.torques = plan
        self.model = danny_sim.build_model()
        self.model.opt.timestep = TIMESTEP
        self.model.opt.iterations = SOLVER_ITERATIONS
        self.data = mujoco.MjData(self.model)
        self.servos = danny_servo.Servos(self.model, VOLTS, firmware_gain)
        self.index = {name: i for i, name in enumerate(danny_sim.hinge_names(self.model))}
        self.soles = {side: self.model.geom(f"{side}_sole").id for side in ("left", "right")}
        self.floor = next(geom for geom in range(self.model.ngeom) if self.model.geom_bodyid[geom] == 0)

    def on_ground(self, side):
        pair = {self.soles[side], self.floor}
        return any({contact.geom1, contact.geom2} == pair for contact in self.data.contact[:self.data.ncon])

    def run(self, gains):
        pitch, pitch_rate, roll, roll_rate, hip_roll, load = gains
        model, data = self.model, self.data
        back_emf = danny_sim.SERVO["kt"] ** 2 / danny_sim.SERVO["R"]
        samples = int(danny_motion.RATE_HZ / CONTROL_HZ)
        substeps = int(round(1 / CONTROL_HZ / TIMESTEP))
        self.servos.reset(data)
        data.qpos[:], data.qvel[:] = self.poses[2], self.velocities[2]
        mujoco.mj_forward(model, data)
        start = float(data.subtree_com[1, 0])
        target = self.poses[2, 7:] + load * self.torques[2] / self.servos.stiffness()
        delayed = [target.copy() for _ in range(DELAY_CONTROLS)]
        air = {"left": 0.0, "right": 0.0}
        planted = {}
        strides, slips, forward = [], [0.0], 0.0
        last_foot, alternating, survived = None, 0, 0.0
        for index in range(2, len(self.poses) - 2, samples):
            up = data.xmat[1].reshape(3, 3)[:, 2]
            reference = np.zeros(9)
            mujoco.mju_quat2Mat(reference, self.poses[index, 3:7])
            wanted = reference.reshape(3, 3)[:, 2]
            target = self.poses[index, 7:] + load * (
                self.torques[index] + back_emf * self.velocities[index, 6:]) / self.servos.stiffness()
            for side in ("left", "right"):
                target[self.index[f"{side}_ankle_pitch"]] += pitch * (up[0] - wanted[0]) + pitch_rate * data.qvel[4]
                target[self.index[f"{side}_ankle_roll"]] += roll * (up[1] - wanted[1]) + roll_rate * data.qvel[3]
                target[self.index[f"{side}_hip_roll"]] += hip_roll * (up[1] - wanted[1])
            delayed.append(target)
            for _ in range(substeps):
                self.servos.step(data, delayed[0])
            delayed.pop(0)
            survived = index / danny_motion.RATE_HZ
            if up[2] < np.cos(np.radians(FALL_TILT_DEG)):
                break
            upright = up[2] >= np.cos(np.radians(MEASURE_TILT_DEG))
            if upright:
                forward = float(data.subtree_com[1, 0]) - start
            for side in air:
                position = float(data.site(f"{side}_sole").xpos[0])
                if self.on_ground(side):
                    if air[side] >= MINIMUM_AIR_SECONDS and upright:
                        if side in planted:
                            strides.append(abs(position - planted[side]))
                        alternating += last_foot not in (None, side)
                        last_foot = side
                        planted[side] = position
                    elif side in planted:
                        slips.append(abs(position - planted[side]))
                    air[side] = 0.0
                else:
                    air[side] += 1 / CONTROL_HZ
        return {
            "seconds": survived,
            "forward_m": forward,
            "alternating_steps": int(alternating),
            "mean_stride_m": float(np.mean(strides)) if strides else 0.0,
            "worst_slip_m": max(slips),
        }


runner = None


def start_worker(plan, firmware_gain):
    global runner
    runner = Runner(plan, firmware_gain)


def fitness(gains):
    result = runner.run(gains)
    slip_cost = 20 * max(0.0, result["worst_slip_m"] - MAXIMUM_SLIP_M)
    return result["seconds"] + 15 * min(result["forward_m"], 0.8) + 0.5 * result["alternating_steps"] + 10 * result["mean_stride_m"] - slip_cost


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--firmware-gain", type=float, default=5.0)
    args = parser.parse_args()

    plan = walk_plan()
    generator = np.random.default_rng(0)
    mean, deviation = (LOW + HIGH) / 2, (HIGH - LOW) / 3
    best, best_score = mean, -np.inf
    with multiprocessing.Pool(initializer=start_worker, initargs=(plan, args.firmware_gain)) as pool:
        for _ in range(GENERATIONS):
            samples = np.clip(generator.normal(mean, deviation, (POPULATION, len(LOW))), LOW, HIGH)
            scores = np.array(pool.map(fitness, samples))
            elite = samples[np.argsort(scores)[-ELITE:]]
            mean, deviation = elite.mean(0), elite.std(0) + 0.01 * (HIGH - LOW)
            if scores.max() > best_score:
                best_score, best = float(scores.max()), samples[scores.argmax()].copy()
    result = Runner(plan, args.firmware_gain).run(best)
    report = {"firmware_gain": args.firmware_gain, "volts": VOLTS, "gains": np.round(best, 3).tolist(), **result}
    report["passed"] = bool(
        result["forward_m"] >= MINIMUM_FORWARD_M and result["alternating_steps"] >= MINIMUM_ALTERNATING_STEPS
        and result["mean_stride_m"] >= MINIMUM_STRIDE_M and result["worst_slip_m"] <= MAXIMUM_SLIP_M)
    OUTPUT.write_text(json.dumps(report, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=1, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
