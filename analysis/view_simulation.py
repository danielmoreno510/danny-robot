"""
Danny - Abre el visor con el modelo de simulación: suelo, gravedad y actuadores en postura.

El robot arranca de pie y se sostiene solo. En el visor, empújalo con doble clic en una pieza
y Ctrl + arrastrar, o mueve los actuadores en el panel Control de la derecha.

Uso: .venv/bin/python analysis/view_simulation.py
"""

import mujoco
import mujoco.viewer

import danny_sim


def main():
    model = danny_sim.build_model()
    mujoco.viewer.launch(model, mujoco.MjData(model))


if __name__ == "__main__":
    main()
