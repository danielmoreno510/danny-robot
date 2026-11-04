# Entrenamiento de Danny (etapa 4)

Entorno de entrenamiento con mjlab para Robot Danny. Ver [documentation/training-guide.md](../documentation/training-guide.md).

Base de código: [xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl) (Apache 2.0), adaptada a Danny. Las piezas 3D de
XGO-Duck no se copian. BAM se usa desde `vendor/bam` (Rhoban, Apache 2.0).

Requisitos: GPU NVIDIA con CUDA, Python 3.12 y uv.

    cd training
    uv sync

Entrenar (Windows: `PYTHONUTF8=1`; el registro por defecto, wandb, falla con `start_method`):

    uv run train Mjlab-Velocity-Flat-Danny --env.scene.num-envs 4096 --agent.max-iterations 3000 --agent.run-name <nombre> --agent.logger tensorboard

Medir un punto de control, sin empujones. Cada script escribe su JSON en `results/`:

    uv run python scripts/evaluate_walk.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name <nombre>
    uv run python scripts/measure_gait.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name <nombre>
    uv run python scripts/measure_foot_peak_height.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name <nombre>
    uv run python scripts/measure_actuator_torque.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --forward-speed 0.25 --name <nombre>

- `evaluate_walk.py`: supervivencia y error de velocidad con órdenes aleatorias, medida en el centro de masas de `base_link`.
- `measure_gait.py`: orden fija de 0,25 m/s, o la de `--forward` y `--turn`; avance, giro, apoyo a un pie, apoyos por segundo, si alternan y reparto entre pies.
- `measure_yaw_oscillation.py`: separa el error de giro en oscilación del paso y giro medio corto (error por paso, promediado a 0,8 s, de la media por robot, amplitud y frecuencias). Uso: `uv run python scripts/measure_yaw_oscillation.py --name <nombre> <punto de control> [...]`.
- `measure_foot_peak_height.py`: altura máxima de suela en el episodio (no por paso).
- `measure_actuator_torque.py`: par de cada actuador frente al de parada a 6 V, con orden fija si se pasa `--forward-speed`.

Comprobar el modelo y la aleatorización (masa, centro de masas, colisiones; sale con código 1 si falla):

    uv run python scripts/check_training_setup.py

Los scripts de medida aceptan `--vin 7.4` para fijar la tensión de los servos (sin ella se sortea entre 6 y 8,4 V).

Comprobar el reloj de marcha y su recompensa sin simular (sale con código 1 si falla):

    uv run python scripts/check_gait_clock.py

La ganancia del firmware del servo es `KP_FW` en `src/danny_training/robot.py`.

Empujones en `measure_gait.py` y `measure_actuator_torque.py`: `--pushes` los deja como en la etapa 3 (±0,20 m/s) y `--push M_S` fija su fuerza (±M_S m/s en XY cada 3 a 6 s; la guía usa `--push 0.12`).

Diagnóstico con `measure_gait.py`: `--nominal-mass` quita la aleatorización de masa y centro de masas, y `--drop-link-contacts REGEX` quita los contactos entre eslabones cuyo nombre encaje (`.` los quita todos).

Ver las curvas del entrenamiento en directo (solo CPU; http://localhost:6006, filtra las corridas por nombre):

    uv run tensorboard --logdir logs/rsl_rl/danny_walk

Verlo en el navegador (http://localhost:8080):

    uv run play Mjlab-Velocity-Flat-Danny --checkpoint-file logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --viewer viser --num-envs 4
