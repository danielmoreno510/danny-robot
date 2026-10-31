"""
Danny - Busca una secuencia para levantarse del suelo, para el riesgo de R2.

Usa entropía cruzada sobre posturas simétricas en una simulación con par limitado. No es la
política final: sirve para saber si la mecánica permite levantarse antes de entrenar. Escribe
results/get_up_<inicio>.json y sale con código 1 si no encuentra una secuencia válida.

Uso: .venv/bin/python analysis/search_get_up.py prone --generations 45 --population 240 --seeds 3
Con --verify no busca: repite el mejor resultado con cambios de batería, suelo y masa.
"""

import argparse
import json
import multiprocessing
import sys

import mujoco
import numpy as np

import danny_sim

KEYFRAMES = 8
DIMENSIONS = ("hip_pitch", "knee", "ankle_pitch", "shoulder_pitch", "shoulder_roll", "elbow", "hip_roll")
PARAMETERS = KEYFRAMES * (len(DIMENSIONS) + 1)
FINAL_POSE = {"hip_pitch": -0.3, "knee": 0.6, "ankle_pitch": -0.3}
SETTLE_SECONDS = 0.8
HOLD_SECONDS = 1.5
SEGMENT_SECONDS = (0.2, 2.0)
POSITION_GAIN = 20.0
POSITION_DAMPING = 0.5
STANDING_COM_Z = 0.19
MAXIMUM_TILT = 0.3
START_PITCH = {"prone": 1.5, "supine": -1.5}
START_HEIGHT = {"prone": 0.045, "supine": 0.03}
CONTROL_STEPS = 10  # 50 Hz con paso de 2 ms
# Punto de partida boca abajo. El hombro no pasa por detrás, y con las manos a la altura de la cadera
# el robot se voltea sobre la cabeza: los brazos barren por el costado hasta delante de la cabeza,
# empujan, y las rodillas se recogen. Orden de DIMENSIONS más la duración del tramo.
PRONE_GUESS = [
    [0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, -2.9, 1.5, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, -2.9, 0.1, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, -2.0, 0.1, 0.0, 0.0, 1.0],
    [-0.8, 1.2, 0.0, -2.0, 0.1, 0.0, 0.0, 1.0],
    [-1.7, 2.2, 0.0, -2.0, 0.1, 0.0, 0.0, 1.0],
    [-1.7, 2.2, -0.2, -1.2, 0.1, 0.0, 0.0, 1.0],
    [-1.2, 1.8, -0.5, -0.6, 0.1, 0.0, 0.0, 1.0],
]
FROZEN_KEYFRAMES = 3  # el barrido de brazos ya funciona: apenas se varía

# Condiciones de la búsqueda robusta. Las de --verify son más duras y distintas
SEARCH_CASES = [
    {},
    {"limit": danny_sim.stall_torque(6.6)},
    {"friction": 0.6},
    {"mass": 1.07},
]

model = None
joint_map = None


def setup():
    global model, joint_map
    model = danny_sim.build_model()
    model.actuator_gainprm[:, 0] = POSITION_GAIN
    model.actuator_biasprm[:, 1] = -POSITION_GAIN
    model.actuator_biasprm[:, 2] = -POSITION_DAMPING
    names = danny_sim.hinge_names(model)
    joint_map = {name: index for index, name in enumerate(names)}


def full_targets(pose):
    target = np.zeros(model.nu)
    for name, value in pose.items():
        for side, sign in (("left", 1), ("right", -1)):
            mirrored = sign if "roll" in name else 1
            index = joint_map[f"{side}_{name}"]
            low, high = model.jnt_range[index + 1]
            target[index] = np.clip(mirrored * value, low, high)
    return target


def decode(vector):
    keyframes, durations = [], []
    width = len(DIMENSIONS) + 1
    for index in range(KEYFRAMES):
        chunk = vector[index * width:(index + 1) * width]
        keyframes.append(dict(zip(DIMENSIONS, chunk[:-1])))
        durations.append(float(np.clip(chunk[-1], *SEGMENT_SECONDS)))
    return keyframes, durations


def lying_state(start):
    data = mujoco.MjData(model)
    rotation = np.array(
        [[np.cos(START_PITCH[start]), 0, np.sin(START_PITCH[start])], [0, 1, 0],
         [-np.sin(START_PITCH[start]), 0, np.cos(START_PITCH[start])]])
    mujoco.mju_mat2Quat(data.qpos[3:7], rotation.flatten())
    # El origen del modelo está en el suelo de la pose cero: lo lleva hasta la altura del tumbado
    data.qpos[2] = START_HEIGHT[start]
    data.ctrl[:] = 0
    for _ in range(int(SETTLE_SECONDS / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return data


def apply_case(case, nominal):
    for field, value in nominal.items():
        getattr(model, field)[:] = value
    if "limit" in case:
        model.actuator_forcerange[:] = [-case["limit"], case["limit"]]
    if "friction" in case:
        model.geom_friction[:, 0] = case["friction"]
    if "mass" in case:
        model.body_mass[:] *= case["mass"]
        model.body_inertia[:] *= case["mass"]


def nominal_values():
    return {name: getattr(model, name).copy() for name in ("actuator_forcerange", "geom_friction", "body_mass", "body_inertia")}


def tilt_of(data):
    up = np.zeros(3)
    mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), data.qpos[3:7])
    return float(np.arccos(np.clip(up[2], -1, 1)))


def rollout(arguments, record=False):
    vector, start = arguments
    keyframes, durations = decode(vector)
    data = lying_state(start)
    targets = [data.qpos[7:].copy()] + [full_targets(pose) for pose in keyframes] + [full_targets(FINAL_POSE)]
    durations = durations + [0.8]
    trace = []
    best_height = 0.0
    best_tilt = 3.2
    for segment, seconds in enumerate(durations):
        steps = max(1, int(seconds / (model.opt.timestep * CONTROL_STEPS)))
        for step in range(steps):
            data.ctrl[:] = targets[segment] + (targets[segment + 1] - targets[segment]) * (step + 1) / steps
            for _ in range(CONTROL_STEPS):
                mujoco.mj_step(model, data)
            if record:
                trace.append(data.qpos.copy())
            best_height = max(best_height, float(data.subtree_com[1][2]))
            best_tilt = min(best_tilt, tilt_of(data))
    data.ctrl[:] = targets[-1]
    best_tilt = min(best_tilt, tilt_of(data))
    heights, tilts = [], []
    for step in range(int(HOLD_SECONDS / model.opt.timestep)):
        mujoco.mj_step(model, data)
        heights.append(float(data.subtree_com[1][2]))
        tilts.append(tilt_of(data))
        if record and step % CONTROL_STEPS == 0:
            trace.append(data.qpos.copy())
    height = float(np.mean(heights[-250:]))
    tilt = float(np.mean(tilts[-250:]))
    score = 6 * height - 0.6 * tilt + best_height - 0.3 * best_tilt
    if record:
        return score, height, tilt, trace
    return score


def robust_rollout(arguments):
    """Puntuación media en varias condiciones: premia lo que no depende del ajuste exacto."""
    nominal = nominal_values()
    scores = []
    for case in SEARCH_CASES:
        apply_case(case, nominal)
        scores.append(rollout(arguments))
    apply_case({}, nominal)
    return float(np.mean(scores))


def search(start, generations, population, seed, pool, initial=None):
    generator = np.random.default_rng(seed)
    mean = np.array(PRONE_GUESS).flatten()
    sigma = np.tile(np.array([0.35, 0.35, 0.3, 0.5, 0.2, 0.4, 0.15, 0.3]), KEYFRAMES)
    sigma[:FROZEN_KEYFRAMES * (len(DIMENSIONS) + 1)] *= 0.1
    if initial is not None:
        mean = initial.copy()
        sigma = 0.3 * sigma + 0.02
    best = (-1e9, None)
    history = []
    for generation in range(generations):
        samples = mean + sigma * generator.standard_normal((population, PARAMETERS))
        evaluate = robust_rollout if initial is not None else rollout
        scores = np.array(pool.map(evaluate, [(sample, start) for sample in samples], chunksize=4))
        order = np.argsort(scores)[::-1]
        elite = samples[order[:max(4, population // 10)]]
        mean = 0.7 * elite.mean(axis=0) + 0.3 * mean
        sigma = 0.7 * elite.std(axis=0) + 0.3 * sigma + 0.01
        if scores[order[0]] > best[0]:
            best = (float(scores[order[0]]), samples[order[0]].copy())
        history.append(float(scores[order[0]]))
        print(f"{start} {seed} gen {generation} mejor {best[0]:.3f} media {scores.mean():.3f}", flush=True)
    return best, history


def verify(start):
    """Prueba la mejor secuencia, sin retocarla, en condiciones distintas de las de la búsqueda."""
    path = danny_sim.ROOT / "results" / f"get_up_{start}.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    vector = np.array(report["best"]["vector"])
    nominal = nominal_values()
    cases = {
        "nominal": {},
        "battery_6v": {"limit": danny_sim.stall_torque(6.0)},
        "floor_friction_0.5": {"friction": 0.5},
        "floor_friction_1.2": {"friction": 1.2},
        "mass_plus_10_percent": {"mass": 1.1},
        "mass_minus_10_percent": {"mass": 0.9},
    }
    results = {}
    for name, case in cases.items():
        apply_case(case, nominal)
        score, height, tilt, trace = rollout((vector, start), record=True)
        results[name] = {
            "height_m": height, "tilt_rad": tilt,
            "stands": height >= STANDING_COM_Z and tilt <= MAXIMUM_TILT,
        }
        if name == "nominal":
            report["best"].update({
                "poses_hz": 1 / (model.opt.timestep * CONTROL_STEPS),
                "poses": np.round(np.array(trace), 4).tolist(),
            })
        print(name, results[name])
    report["robustness"] = results
    path.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    return 0 if all(item["stands"] for item in results.values()) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("start", choices=sorted(START_PITCH))
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--population", type=int, default=160)
    parser.add_argument("--seeds", type=int, default=2)
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--robust", action="store_true", help="parte del mejor resultado y puntúa varias condiciones")
    args = parser.parse_args()

    setup()
    if args.verify:
        return verify(args.start)
    results = []
    initial = None
    output = danny_sim.ROOT / "results" / f"get_up_{args.start}.json"
    if args.robust:
        initial = np.array(json.loads(output.read_text(encoding="utf-8"))["best"]["vector"])
    with multiprocessing.Pool(initializer=setup) as pool:
        for seed in range(args.seeds):
            (score, vector), history = search(args.start, args.generations, args.population, seed, pool, initial)
            score, height, tilt, trace = rollout((vector, args.start), record=True)
            results.append({"seed": seed, "score": score, "height_m": height, "tilt_rad": tilt,
                            "vector": vector.tolist(), "history": history,
                            "poses_hz": 1 / (model.opt.timestep * CONTROL_STEPS),
                            "poses": np.round(np.array(trace), 4).tolist()})
    best = max(results, key=lambda item: item["score"])
    success = best["height_m"] >= STANDING_COM_Z and best["tilt_rad"] <= MAXIMUM_TILT
    report = {"start": args.start, "success": success, "best": best, "all": [
        {k: item[k] for k in ("seed", "score", "height_m", "tilt_rad")} for item in results]}
    output.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "best"}, indent=1))
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
