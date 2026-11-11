"""
Danny - Graba el vídeo del punto 1 del criterio de gestos (fase 6): la persona de CMU, a escala, junto a Danny, vistos de frente.

Danny va a la izquierda de la imagen, con la trayectoria de results/retargeted/cmu_<clip>[_lento<factor>].csv, sin física. La persona
va a la derecha, a la izquierda de Danny, como un esqueleto de varillas (azul su lado izquierdo y rojo el derecho), con su pelvis quieta
y girada hacia la cámara como Danny, y con el mismo ritmo que la trayectoria. El vídeo se escribe en results/videos/ con el mismo nombre,
con encode_video.swift (AVFoundation de macOS).

La persona se dibuja tal como se grabó, aunque Danny use las dos manos o un gesto más pequeño. Con --walk, la cámara sigue a Danny hacia
delante y mira de tres cuartos, y la persona avanza a su par.

Uso: .venv/bin/python analysis/record_gesture_video.py external/cmu/141_16.amc [--slowdown 1.8] [--both-hands] [--amplitude 0.5]
"""

import argparse
import pathlib
import subprocess
import tempfile

import mujoco
import numpy as np

import danny_gestures
import danny_sim

WIDTH, HEIGHT = 960, 540
BONES = [("root", "lowerback"), ("lowerback", "thorax"), ("thorax", "lowerneck"), ("lowerneck", "upperneck"), ("upperneck", "head")]
for _p in ("l", "r"):
    BONES += [("thorax", f"{_p}clavicle"), (f"{_p}clavicle", f"{_p}humerus"), (f"{_p}humerus", f"{_p}radius"), (f"{_p}radius", f"{_p}hand"),
              ("root", f"{_p}hipjoint"), (f"{_p}hipjoint", f"{_p}femur"), (f"{_p}femur", f"{_p}tibia"), (f"{_p}tibia", f"{_p}foot")]
LEFT_COLOR = np.array([0.2, 0.55, 1.0, 1.0], dtype=np.float32)
RIGHT_COLOR = np.array([1.0, 0.3, 0.2, 1.0], dtype=np.float32)
SIDE_GAP_M = 0.32  # separación entre la persona y Danny
WALK_AZIMUTH_DEG = 150.0  # al andar, vista de tres cuartos


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("amc", type=pathlib.Path)
    parser.add_argument("--slowdown", type=float, default=1.0, help="el mismo que en retarget_gesture.py")
    parser.add_argument("--both-hands", action="store_true", help="el mismo que en retarget_gesture.py")
    parser.add_argument("--amplitude", type=float, default=1.0, help="el mismo que en retarget_gesture.py")
    parser.add_argument("--end", type=float, default=None, help="corta el clip de la persona en ese segundo")
    parser.add_argument("--legs", action="store_true", help="el mismo que en retarget_gesture.py")
    parser.add_argument("--walk", action="store_true", help="el mismo que en retarget_gesture.py")
    args = parser.parse_args()
    args.legs = args.legs or args.walk
    name = danny_gestures.clip_name(args.amc, args.slowdown, args.both_hands, args.amplitude, args.legs, args.end)

    model = danny_gestures.gesture_model()
    data = mujoco.MjData(model)
    motion = danny_gestures.CmuMotion(args.amc, args.end)
    rows = np.loadtxt(danny_sim.ROOT / "results" / "retargeted" / f"{name}.csv", delimiter=",")
    qpos = np.column_stack([rows[:, :3], rows[:, 6:7], rows[:, 3:6], rows[:, 7:]])

    # Persona a la altura de Danny: de los pies a la cabeza en el primer fotograma.
    data.qpos[:] = qpos[0]
    mujoco.mj_forward(model, data)
    danny_height = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")][2] + 0.08
    human_height = motion.end["head"][0][2] - min(motion.end["lfoot"][0][2], motion.end["rfoot"][0][2])
    scale = danny_height / human_height
    yaw = danny_gestures.pelvis_yaw(motion)

    model.vis.global_.offwidth, model.vis.global_.offheight = WIDTH, HEIGHT
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth, camera.elevation, camera.distance = 180.0, -5.0, 0.95
    camera.lookat[:] = [0.0, SIDE_GAP_M / 2, 0.2]

    times = np.arange(len(qpos)) / danny_gestures.RATE_HZ
    with tempfile.NamedTemporaryFile(suffix=".rgb") as frames:
        for t, q in zip(times, qpos):
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            if args.walk:
                # Al andar, la cámara sigue a la pelvis hacia delante (Danny avanza unos 0,7 m) y mira de tres cuartos, para ver las rodillas.
                camera.lookat[0] = q[0]
                camera.azimuth = WALK_AZIMUTH_DEG
            renderer.update_scene(data, camera)
            scene = renderer.scene
            frame = min(int(round(t / (motion.frame_time * args.slowdown))), motion.count - 1)
            turn = np.array([[np.cos(-yaw[frame]), -np.sin(-yaw[frame]), 0], [np.sin(-yaw[frame]), np.cos(-yaw[frame]), 0], [0, 0, 1]])
            hips = motion.end["root"][frame]
            floor = min(motion.end["lfoot"][frame][2], motion.end["rfoot"][frame][2])

            def place(point):
                local = turn @ (point - hips) * scale
                return local + np.array([q[0] if args.walk else 0.0, SIDE_GAP_M, (hips[2] - floor) * scale])  # al andar, a la par de Danny

            for a, b in BONES:
                if scene.ngeom >= scene.maxgeom:
                    break
                geom = scene.geoms[scene.ngeom]
                color = RIGHT_COLOR if b.startswith("r") and b != "root" else LEFT_COLOR
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).flatten(), color)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.005, place(motion.end[a][frame]), place(motion.end[b][frame]))
                scene.ngeom += 1
            frames.write(renderer.render().tobytes())
        frames.flush()
        out = danny_sim.ROOT / "results" / "videos" / f"gesture_{name}.mp4"
        out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["swift", str(danny_sim.ROOT / "analysis" / "encode_video.swift"), frames.name, str(WIDTH), str(HEIGHT),
                        str(int(danny_gestures.RATE_HZ)), str(out)], check=True)


if __name__ == "__main__":
    main()
