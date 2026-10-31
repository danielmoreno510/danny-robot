"""
Danny - Escribe el modelo listo para entrenar en cad/export/danny/danny_training.xml.

Es el MJCF de Fusion con lo que añade analysis/danny_sim.py: base libre, suelo, colisiones,
actuadores de posición y parámetros del servo, más la postura de pie `stand`. Después carga
el archivo escrito y comprueba que se mantiene. No se edita a mano: se regenera.

Uso: .venv/bin/python analysis/export_training_model.py
"""

import sys

import mujoco
import numpy as np

import danny_motion
import danny_sim

OUTPUT = danny_sim.MODEL.parent / "danny_training.xml"


def main():
    spec = danny_sim.build_spec()
    model = spec.compile()
    solver = danny_motion.Solver(model)
    stand, error = solver.solve(*danny_motion.TASKS["squat"][0](0.0)[:3], iterations=600)
    key = spec.add_key()
    key.name = "stand"
    key.qpos = stand.tolist()
    key.ctrl = stand[7:].tolist()
    spec.compile()
    OUTPUT.write_text(spec.to_xml(), encoding="utf-8")

    written = mujoco.MjModel.from_xml_path(str(OUTPUT))
    data = mujoco.MjData(written)
    mujoco.mj_resetDataKeyframe(written, data, 0)
    start = float(mujoco.mj_name2id(written, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    for _ in range(1500):
        mujoco.mj_step(written, data)
    up = np.zeros(3)
    mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), data.qpos[3:7])
    print(f"{OUTPUT.name}: {written.nu} actuadores, {written.ngeom} geoms, {written.body_mass.sum():.4f} kg, "
          f"clave {start:.0f}, error de postura {1000 * error:.2f} mm, inclinación tras 3 s "
          f"{np.degrees(np.arccos(np.clip(up[2], -1, 1))):.2f}°, altura del centro de masas "
          f"{data.subtree_com[1][2]:.3f} m")
    return 0 if up[2] > 0.99 else 1


if __name__ == "__main__":
    sys.exit(main())
