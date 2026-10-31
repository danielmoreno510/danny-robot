"""
Danny - Abre el visor con el modelo de simulación: suelo, gravedad y motores con par limitado.

Sin argumentos, el robot arranca de pie y se sostiene solo: empújalo con doble clic en una
pieza y Ctrl + arrastrar. Con una tarea, la repite en bucle siguiendo sus posturas, sin
realimentación de equilibrio. Si cae, el botón Reset o la tecla Retroceso lo devuelven al inicio: `squat`, `single_support` y `rise_from_crouch` se sostienen y
`walk` se cae, porque eso necesita la política de la etapa 4. `get_up_prone` y `roll_over`
son grabaciones de results/: se ven tal como ocurrieron, sin volver a simular, porque son
tan sensibles que repetirlas con un mínimo redondeo acaba en otro resultado.

Uso: .venv/bin/python analysis/view_simulation.py [tarea]
"""

import argparse
import json

import mujoco
import mujoco.viewer
import numpy as np

import danny_motion
import danny_sim

HOLD_SECONDS = 2.0
SEARCHED_MOTIONS = ("get_up_prone", "roll_over")


def set_gains(model, gain, damping):
    model.actuator_gainprm[:, 0] = gain
    model.actuator_biasprm[:, 1] = -gain
    model.actuator_biasprm[:, 2] = -damping


def play_recording(model, task):
    report = json.loads((danny_sim.ROOT / "results" / f"{task}.json").read_text(encoding="utf-8"))
    best = report.get("best", report)
    poses, rate = np.array(best["poses"]), best["poses_hz"]
    model.opt.gravity[:] = 0
    duration = len(poses) / rate + HOLD_SECONDS

    def follow(_, step_data):
        # No se reinicia nada aquí: tocar el estado dentro del paso de MuJoCo lo deja atascado
        step_data.qpos[:] = poses[min(int((step_data.time % duration) * rate), len(poses) - 1)]
        step_data.qvel[:] = 0

    return poses[0], follow


def follow_task(model, task):
    set_gains(model, 20.0, 0.5)
    trajectory = danny_motion.task_trajectory(model, task)
    controls, rate = trajectory[:, 7:], danny_motion.RATE_HZ
    duration = len(controls) / rate + HOLD_SECONDS

    def follow(_, step_data):
        step_data.ctrl[:] = controls[min(int((step_data.time % duration) * rate), len(controls) - 1)]

    return trajectory[0], follow


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("task", nargs="?", choices=[*danny_motion.TASKS, *SEARCHED_MOTIONS])
    args = parser.parse_args()

    model = danny_sim.build_model()
    data = mujoco.MjData(model)
    if args.task:
        loader = play_recording if args.task in SEARCHED_MOTIONS else follow_task
        start, follow = loader(model, args.task)
        model.qpos0[:] = start  # el botón Reset del visor vuelve aquí
        data.qpos[:] = start
        mujoco.set_mjcb_control(follow)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
