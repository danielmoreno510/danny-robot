# Entrenamiento de Danny (etapa 4)

Entorno de entrenamiento con mjlab para Robot Danny. Ver [documentation/training-guide.md](../documentation/training-guide.md).

Base de código: [xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl) (Apache 2.0), adaptada a Danny. Las piezas 3D de
XGO-Duck no se copian. BAM se usa desde `vendor/bam` (Rhoban, Apache 2.0).

Requisitos: GPU NVIDIA con CUDA, Python 3.12 y uv.

    cd training
    uv sync
