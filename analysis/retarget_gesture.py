"""
Danny - Adapta un gesto de pie de CMU (AMC con su ASF en la misma carpeta) y mide el criterio de la fase 6.

Criterio aprobado por el dueño el 10 de noviembre (PROJECT_HANDOFF.md). El script mide los puntos 2 a 6; el 1, el vídeo junto a la
persona, lo aprueba el dueño con record_gesture_video.py.

Escribe la trayectoria a 50 Hz en results/retargeted/cmu_<clip>.csv, con la convención de mjlab para imitar movimiento: por fila,
posición de la base (x, y, z), orientación (qx, qy, qz, qw) y las 23 articulaciones en el orden del modelo. El informe va a
results/gesture_cmu_<clip>.json.

Es solo cinemática: no comprueba equilibrio ni par.

Con --slowdown el gesto va más lento que el de la persona (1,75 = 1,75 veces más largo), para no pasar de la velocidad del servo.

Con --both-hands los dos brazos copian el derecho de la persona, en espejo, y con --amplitude se reduce la agitación rápida de los brazos (0,5 = la
mitad), igual de rápida, sin cambiar cuánto suben las manos: agitarse menos necesita menos velocidad del servo.

Uso: .venv/bin/python analysis/retarget_gesture.py external/cmu/141_16.amc [--slowdown 1.8] [--both-hands] [--amplitude 0.5]
"""

import argparse
import json
import pathlib

import mujoco
import numpy as np

import danny_gestures
import danny_sim

ELBOW_HAND_MEDIAN_M = 0.01
ELBOW_HAND_P95_M = 0.02
AT_LIMIT_FRACTION = 0.05


def quantiles(values):
    return {"median": round(float(np.median(values)), 4), "p95": round(float(np.percentile(values, 95)), 4),
            "max": round(float(np.max(values)), 4)}


def reduced_targets(model, raw):
    """Objetivos de codo y mano iguales a la posición del brazo reducido (el error del punto 2 es entonces cero por construcción)."""
    data = mujoco.MjData(model)
    out = []
    for q in raw:
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        site = lambda n: data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)].copy()
        out.append({side: (site(f"{side}_elbow_site"), site(f"{side}_hand")) for side in ("left", "right")})
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("amc", type=pathlib.Path)
    parser.add_argument("--slowdown", type=float, default=1.0, help="cuántas veces más lento que la persona")
    parser.add_argument("--both-hands", action="store_true", help="los dos brazos copian el derecho de la persona, en espejo")
    parser.add_argument("--amplitude", type=float, default=1.0, help="fracción de la agitación rápida de los brazos (1, entera)")
    parser.add_argument("--continuity", type=float, default=danny_gestures.CONTINUITY_WEIGHT, help="peso hacia la postura anterior (ver danny_gestures)")
    parser.add_argument("--body-margin", type=float, default=danny_gestures.BODY_MARGIN_M, help="margen en m entre brazo y cuerpo")
    parser.add_argument("--speed-cap", action="store_true", help="ninguna articulación del brazo va más deprisa que el 80 %% del servo")
    parser.add_argument("--smooth", type=float, default=0.0, help="sigma en s del suavizado final de brazos, cintura y cuello (0, sin suavizar)")
    args = parser.parse_args()
    name = danny_gestures.clip_name(args.amc, args.slowdown, args.both_hands, args.amplitude)

    model = danny_gestures.gesture_model()
    motion = danny_gestures.CmuMotion(args.amc)
    retarget = danny_gestures.GestureRetarget(model, args.both_hands, args.continuity, args.body_margin,
                                              danny_gestures.SPEED_LIMIT_RAD_S * 0.97 if args.speed_cap else None)
    retarget.frame_dt = motion.frame_time * args.slowdown
    raw, targets = retarget.run(motion)
    if args.amplitude != 1.0:
        # Los objetivos de codo y mano se recalculan con el brazo reducido: el punto 2 mide la distancia a ese brazo reducido.
        raw = danny_gestures.shrink_arms(raw, model, args.amplitude, motion.frame_time)
        targets = reduced_targets(model, raw)

    qpos = danny_gestures.resample(raw, motion.frame_time * args.slowdown, 1.0 / danny_gestures.RATE_HZ)
    if args.smooth > 0:
        qpos = danny_gestures.smooth_arms(qpos, model, args.smooth, 1.0 / danny_gestures.RATE_HZ)

    # Punto 2: error de codo y mano de la trayectoria que se entrega (50 Hz, tras remuestrear y suavizar), frente al objetivo del
    # fotograma de CMU más cercano en el tiempo.
    data = mujoco.MjData(model)
    errors = {f"{side}_{part}": [] for side in ("left", "right") for part in ("elbow", "hand")}
    for i, q in enumerate(qpos):
        frame = min(int(round(i / danny_gestures.RATE_HZ / (motion.frame_time * args.slowdown))), len(targets) - 1)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        for side in ("left", "right"):
            for part, site, target in (("elbow", f"{side}_elbow_site", targets[frame][side][0]), ("hand", f"{side}_hand", targets[frame][side][1])):
                position = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)]
                errors[f"{side}_{part}"].append(np.linalg.norm(position - target))

    joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, model.njnt)]
    angles = qpos[:, 7:]
    ranges = model.jnt_range[1:]
    speeds = np.abs(np.gradient(angles, 1.0 / danny_gestures.RATE_HZ, axis=0))
    at_limit = (angles <= ranges[:, 0] + danny_gestures.AT_LIMIT_RAD) | (angles >= ranges[:, 1] - danny_gestures.AT_LIMIT_RAD)
    arm = [j for j, n in enumerate(joints) if any(n.endswith(a) for a in danny_gestures.ARM_JOINTS)]

    # Punto 5: holgura entre brazo y cuerpo en cada fotograma de 50 Hz.
    clearance = []
    for q in qpos:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        clearance.append(danny_gestures.arm_clearance(model, data))
    clearance = np.array(clearance)
    # Punto 6: piernas y pelvis en la pose de pie.
    legs = slice(0, joints.index("waist_yaw") + 7)
    legs_offset = float(np.abs(qpos[:, legs] - retarget.stand[legs]).max())

    all_errors = np.concatenate([np.array(e) for e in errors.values()])
    criteria = {
        "elbow_hand_error_median_m": {"value": round(float(np.median(all_errors)), 4), "limit": ELBOW_HAND_MEDIAN_M},
        "elbow_hand_error_p95_m": {"value": round(float(np.percentile(all_errors, 95)), 4), "limit": ELBOW_HAND_P95_M},
        "worst_arm_joint_fraction_at_limit": {"value": round(float(at_limit[:, arm].mean(axis=0).max()), 4), "limit": AT_LIMIT_FRACTION},
        "max_joint_speed_rad_s": {"value": round(float(speeds.max()), 2), "limit": round(danny_gestures.SPEED_LIMIT_RAD_S, 2)},
        "min_arm_body_clearance_m": {"value": round(float(clearance.min()), 4), "limit": 0.0},
        "legs_and_pelvis_max_offset_from_stand": {"value": round(legs_offset, 6), "limit": 0.0},
    }
    passes = {
        "elbow_hand_error_median_m": criteria["elbow_hand_error_median_m"]["value"] <= ELBOW_HAND_MEDIAN_M,
        "elbow_hand_error_p95_m": criteria["elbow_hand_error_p95_m"]["value"] <= ELBOW_HAND_P95_M,
        "worst_arm_joint_fraction_at_limit": criteria["worst_arm_joint_fraction_at_limit"]["value"] <= AT_LIMIT_FRACTION,
        "max_joint_speed_rad_s": criteria["max_joint_speed_rad_s"]["value"] <= danny_gestures.SPEED_LIMIT_RAD_S,
        "min_arm_body_clearance_m": criteria["min_arm_body_clearance_m"]["value"] >= 0.0,
        "legs_and_pelvis_max_offset_from_stand": legs_offset <= 1e-9,
    }
    for key in criteria:
        criteria[key]["passes"] = passes[key]

    out_dir = danny_sim.ROOT / "results" / "retargeted"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = np.column_stack([qpos[:, :3], qpos[:, 4:7], qpos[:, 3:4], angles])
    np.savetxt(out_dir / f"{name}.csv", rows, delimiter=",", fmt="%.6f")
    report = {
        "source": f"CMU {args.amc.stem} (mocap.cs.cmu.edu; creada con fondos de NSF EIA-0196217)",
        "slowdown": args.slowdown,
        "both_hands": args.both_hands,
        "amplitude": args.amplitude,
        "continuity": args.continuity,
        "body_margin_m": args.body_margin,
        "speed_cap": args.speed_cap,
        "smooth_s": args.smooth,
        "seconds": round(len(qpos) / danny_gestures.RATE_HZ, 2),
        "rate_hz": danny_gestures.RATE_HZ,
        "joint_order": joints,
        "criteria_2_to_6": criteria,
        "passes_2_to_6": all(passes.values()),
        "criterion_1": "vídeo junto a la persona, aprobado por el dueño (record_gesture_video.py)",
        "errors_m": {k: quantiles(v) for k, v in errors.items()},
        "arm_joints": {joints[j]: {"used_deg": [round(float(np.degrees(angles[:, j].min())), 1), round(float(np.degrees(angles[:, j].max())), 1)],
                                   "range_deg": [round(float(v), 1) for v in np.degrees(ranges[j])],
                                   "fraction_at_limit": round(float(at_limit[:, j].mean()), 4),
                                   "max_speed_rad_s": round(float(speeds[:, j].max()), 2)} for j in arm},
        "max_arm_jitter_deg": round(float(np.degrees(np.abs(angles[2:, arm] - 2 * angles[1:-1, arm] + angles[:-2, arm])).max() / 2), 2),
        "waist_and_neck_deg": {n: [round(float(np.degrees(angles[:, joints.index(n)].min())), 1), round(float(np.degrees(angles[:, joints.index(n)].max())), 1)]
                               for n in ("waist_yaw", "neck_yaw", "neck_pitch")},
        "not_checked": "equilibrio y par; las colisiones se miden con la envolvente convexa de las mallas del CAD, y las cajas de colisión del entrenamiento pueden tocarse aunque las piezas no",
    }
    path = danny_sim.ROOT / "results" / f"gesture_{name}.json"
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    for key, value in criteria.items():
        print(f"{'pasa' if value['passes'] else 'NO PASA'}  {key}: {value['value']} (límite {value['limit']})")
    print(f"Escrito {out_dir / (name + '.csv')} y {path}")


if __name__ == "__main__":
    main()
