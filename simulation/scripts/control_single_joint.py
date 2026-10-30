import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = ROOT / "models" / "single_joint.xml"


def degrees_to_radians(degrees):
    return math.radians(degrees)


def main():
    print("🤖 Danny - Joint Control")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "joint_1",
    )

    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "joint_1_motor",
    )

    qpos_address = model.jnt_qposadr[joint_id]

    # Posiciones objetivo en grados
    target_angles = [
        0,
        30,
        60,
        0,
        -45,
        -90,
        0,
    ]

    target_index = 0
    target_duration = 2.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.monotonic()
        last_target_change = start_time

        while viewer.is_running():
            step_start = time.monotonic()

            if step_start - last_target_change >= target_duration:
                target_index = (target_index + 1) % len(target_angles)
                last_target_change = step_start

            target_angle = target_angles[target_index]

            data.ctrl[actuator_id] = degrees_to_radians(
                target_angle
            )

            mujoco.mj_step(model, data)

            current_angle = math.degrees(
                data.qpos[qpos_address]
            )

            if int(data.time * 10) % 10 == 0:
                print(
                    f"Target: {target_angle:>4}° | "
                    f"Current: {current_angle:>7.2f}°"
                )

            viewer.sync()

            elapsed = time.monotonic() - step_start
            remaining = model.opt.timestep - elapsed

            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()