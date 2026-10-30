"""
Danny - Control de la pierna 2 GDL colgando (leg_2dof.xml).

Recorre la secuencia de posiciones de analyze_leg_2dof.py con el perfil
coseno (modo smooth), así los motores no saturan en los cambios, y con
compensación de gravedad, así la pierna llega exactamente al objetivo.

Uso:
  mjpython simulation/scripts/control_leg_2dof.py
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from leg_control import (
    LEG_2DOF_ACTUATORS as ACTUATORS,
    LEG_2DOF_JOINTS as JOINTS,
    LEG_2DOF_MOVEMENTS_DEG as MOVEMENTS_DEG,
    LEG_2DOF_POSITION_SENSORS as POSITION_SENSORS,
    LEG_2DOF_TORQUE_SENSORS as TORQUE_SENSORS,
    HOLD_TIME,
    TRANSITION_TIME,
    GravityCompensator,
    get_id,
    interpolate,
    read_sensors,
)


ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = ROOT / "models" / "leg_2dof.xml"

MOVEMENT_DURATION = TRANSITION_TIME + HOLD_TIME  # s por objetivo
LOG_INTERVAL = 0.5  # s de simulación


def main():
    print("🤖 Danny - Leg 2DOF Control (smooth + gravedad)")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    actuator_ids = [get_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS]
    dof_ids = [
        int(model.jnt_dofadr[get_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)])
        for n in JOINTS
    ]
    compensator = GravityCompensator(model, actuator_ids, dof_ids)
    targets_rad = np.radians(np.array(MOVEMENTS_DEG, dtype=float))
    timestep = model.opt.timestep

    movement_index = 0
    movement_start = 0.0
    last_log_time = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.monotonic()

            # Tiempo de simulación, no de reloj: el perfil no depende
            # de lo rápido que vaya el ordenador.
            if data.time - movement_start >= MOVEMENT_DURATION:
                movement_index = (movement_index + 1) % len(targets_rad)
                movement_start = data.time

            previous = targets_rad[movement_index - 1]
            target = targets_rad[movement_index]

            data.ctrl[actuator_ids] = interpolate(
                previous,
                target,
                data.time - movement_start,
                "smooth",
            ) + compensator.offset(data.qpos)

            mujoco.mj_step(model, data)

            if data.time - last_log_time >= LOG_INTERVAL:
                hip_position, knee_position = np.degrees(read_sensors(data, POSITION_SENSORS))
                hip_torque, knee_torque = read_sensors(data, TORQUE_SENSORS)
                hip_target, knee_target = MOVEMENTS_DEG[movement_index]

                print(
                    f"Hip: {hip_position:7.2f}° (target {hip_target:4}°, "
                    f"{hip_torque:6.2f} N·m) | "
                    f"Knee: {knee_position:7.2f}° (target {knee_target:4}°, "
                    f"{knee_torque:6.2f} N·m)"
                )

                last_log_time = data.time

            viewer.sync()

            remaining = timestep - (time.monotonic() - step_start)

            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
