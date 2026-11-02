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
    uv run python scripts/measure_actuator_torque.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --action-scale 1.0 --name <nombre>

- `evaluate_walk.py`: supervivencia y error de velocidad con órdenes aleatorias.
- `measure_gait.py`: orden fija de 0,25 m/s; avance, apoyo a un pie, apoyos por segundo y si alternan.
- `measure_foot_peak_height.py`: altura máxima de suela en el episodio (no por paso).
- `measure_actuator_torque.py`: par de cada actuador frente al de parada a 6 V.

Verlo en el navegador (http://localhost:8080):

    uv run play Mjlab-Velocity-Flat-Danny --checkpoint-file logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --viewer viser --num-envs 4
