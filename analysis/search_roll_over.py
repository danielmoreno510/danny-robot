"""
Danny - Busca una secuencia para pasar de boca arriba a boca abajo, para R2.

Levantarse se resuelve boca abajo: desde boca arriba, la cadera no deja llevar el peso sobre los
pies. Esta búsqueda usa entropía cruzada con los dos lados libres, en simulación con par limitado.
Escribe results/roll_over.json y sale con código 1 si no se da la vuelta.

Uso: .venv/bin/python analysis/search_roll_over.py --generations 30
"""

import argparse
import json
import multiprocessing
import sys

import mujoco
import numpy as np

import danny_sim

KEYFRAMES = 4
JOINTS = (
    "hip_pitch", "hip_roll", "knee", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
)
SIDES = ("left", "right")
PARAMETERS = KEYFRAMES * (len(JOINTS) * 2 + 2)
START_PITCH = -1.5
START_HEIGHT = 0.03
SETTLE_SECONDS = 0.8
FINISH_SECONDS = 1.0
SEGMENT_SECONDS = (0.2, 1.5)
POSITION_GAIN = 20.0
POSITION_DAMPING = 0.5
CONTROL_STEPS = 10

model = None
joint_map = None


def setup():
    global model, joint_map
    model = danny_sim.build_model()
    model.actuator_gainprm[:, 0] = POSITION_GAIN
    model.actuator_biasprm[:, 1] = -POSITION_GAIN
    model.actuator_biasprm[:, 2] = -POSITION_DAMPING
    joint_map = {name: index for index, name in enumerate(danny_sim.hinge_names(model))}


def lying_down():
    data = mujoco.MjData(model)
    angle = START_PITCH
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    mujoco.mju_mat2Quat(data.qpos[3:7], rotation.flatten())
    data.qpos[2] = START_HEIGHT
    for _ in range(int(SETTLE_SECONDS / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return data


def chest_down(data):
    """Componente vertical del frente del robot: -1 boca abajo, +1 boca arriba."""
    front = np.zeros(3)
    mujoco.mju_rotVecQuat(front, np.array([1.0, 0.0, 0.0]), data.qpos[3:7])
    return float(front[2])


def targets_of(chunk):
    target = np.zeros(model.nu)
    for side_index, side in enumerate(SIDES):
        for joint_index, joint in enumerate(JOINTS):
            index = joint_map[f"{side}_{joint}"]
            value = chunk[side_index * len(JOINTS) + joint_index]
            target[index] = np.clip(value, *model.jnt_range[index + 1])
    target[joint_map["waist_yaw"]] = np.clip(chunk[len(JOINTS) * 2], *model.jnt_range[joint_map["waist_yaw"] + 1])
    return target


def rollout(vector, record=False):
    data = lying_down()
    width = len(JOINTS) * 2 + 2
    previous = data.qpos[7:].copy()
    trace = []
    for index in range(KEYFRAMES):
        chunk = vector[index * width:(index + 1) * width]
        target = targets_of(chunk)
        steps = max(1, int(np.clip(chunk[-1], *SEGMENT_SECONDS) / (model.opt.timestep * CONTROL_STEPS)))
        for step in range(steps):
            data.ctrl[:] = previous + (target - previous) * (step + 1) / steps
            for _ in range(CONTROL_STEPS):
                mujoco.mj_step(model, data)
            if record:
                trace.append(data.qpos.copy())
        previous = target
    for step in range(int(FINISH_SECONDS / (model.opt.timestep * CONTROL_STEPS))):
        data.ctrl[:] = 0
        for _ in range(CONTROL_STEPS):
            mujoco.mj_step(model, data)
        if record:
            trace.append(data.qpos.copy())
    front = chest_down(data)
    if record:
        return front, trace
    return -front


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--population", type=int, default=200)
    args = parser.parse_args()
    setup()
    generator = np.random.default_rng(0)
    width = len(JOINTS) * 2 + 2
    # Gira hacia su izquierda: la pierna y el brazo derechos cruzan por encima
    guess = np.zeros(width)
    guess[JOINTS.index("shoulder_roll")] = 1.0
    guess[len(JOINTS) + JOINTS.index("shoulder_pitch")] = -1.6
    guess[len(JOINTS) + JOINTS.index("hip_pitch")] = -1.2
    guess[len(JOINTS) + JOINTS.index("knee")] = 1.0
    guess[-1] = 0.6
    mean = np.tile(guess, KEYFRAMES)
    sigma = np.tile(np.array([0.6] * (width - 1) + [0.3]), KEYFRAMES)
    best = (-1e9, None)
    with multiprocessing.Pool(initializer=setup) as pool:
        for generation in range(args.generations):
            samples = mean + sigma * generator.standard_normal((args.population, PARAMETERS))
            scores = np.array(pool.map(rollout, list(samples), chunksize=4))
            order = np.argsort(scores)[::-1]
            elite = samples[order[:max(4, args.population // 10)]]
            mean = 0.7 * elite.mean(axis=0) + 0.3 * mean
            sigma = 0.7 * elite.std(axis=0) + 0.3 * sigma + 0.01
            if scores[order[0]] > best[0]:
                best = (float(scores[order[0]]), samples[order[0]].copy())
            print(f"gen {generation} mejor {best[0]:.3f} media {scores.mean():.3f}", flush=True)
    front, trace = rollout(best[1], record=True)
    success = front < -0.8
    report = {"success": success, "chest_vertical": front, "vector": best[1].tolist(),
              "poses_hz": 1 / (model.opt.timestep * CONTROL_STEPS),
              "poses": np.round(np.array(trace), 4).tolist()}
    output = danny_sim.ROOT / "results" / "roll_over.json"
    output.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print({"success": success, "chest_vertical": front})
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
