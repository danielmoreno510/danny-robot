from pathlib import Path
import time

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "falling_box.xml"


def main():
    print("🤖 Starting H1 Simulation...")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print("Model loaded successfully!")
    print(f"Bodies: {model.nbody}")
    print(f"Joints: {model.njnt}")
    print(f"Simulation timestep: {model.opt.timestep}")

    print("Launching passive viewer...")

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            step_start = time.time()

            # Avanzar la simulación física
            mujoco.mj_step(model, data)

            # Actualizar la visualización
            viewer.sync()

            # Mantener la velocidad de simulación
            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed

            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()