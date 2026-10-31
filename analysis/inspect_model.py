"""
Danny - Comprueba un modelo MJCF exportado desde Fusion, sin abrir ventana.

Carga el modelo y muestra masa total, masa por cuerpo y eje de cada articulación,
para compararlos con el CAD. Sale con código 1 si el modelo no carga, o si
--expected-mass difiere más de un 1 %.

Uso: .venv/bin/python analysis/inspect_model.py cad/export/modelo.xml --expected-mass 0.25
"""

import argparse
import sys

import mujoco
import numpy as np

MASS_TOLERANCE = 0.01


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model")
    parser.add_argument("--expected-mass", type=float, help="masa total del CAD, en kg")
    args = parser.parse_args()

    try:
        model = mujoco.MjModel.from_xml_path(args.model)
    except (ValueError, OSError) as error:
        print(f"El modelo no carga: {error}")
        return 1

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    for body in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        print(f"Cuerpo {name}: {model.body_mass[body]:.4f} kg")
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        axis = np.round(data.xaxis[joint], 4)
        limits = np.degrees(model.jnt_range[joint])
        print(f"Articulación {name}: eje {axis}, tope {limits[0]:.1f}° a {limits[1]:.1f}°")

    total = float(model.body_mass.sum())
    print(f"Masa total: {total:.4f} kg")
    if args.expected_mass is not None:
        error = abs(total - args.expected_mass) / args.expected_mass
        print(f"Diferencia con el CAD: {100 * error:.2f} %")
        if error > MASS_TOLERANCE:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
