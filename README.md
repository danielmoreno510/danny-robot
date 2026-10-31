# Robot Danny

Humanoide de 40 cm, autónomo y con aspecto de Tesla Optimus, impreso en 3D, con servos de bus y movimiento
natural aprendido en simulación. Toma del XGO-Duck el servo y el método de entrenamiento, no la forma.

**Estado:** etapas 2 y 3, diseño en Fusion y simulación. El robot completo ya está en CAD y en MuJoCo, con una primera capa de paneles. Nada se ha impreso, y
no hay piezas físicas. Para unirte al proyecto, empieza por
[PROJECT_HANDOFF.md](PROJECT_HANDOFF.md) y sigue con el [handbook](documentation/HANDBOOK.md). Para entrenar, usa la
[guía de entrenamiento](documentation/training-guide.md).

## Plan

1. Investigación y definición.
2. Diseño en Autodesk Fusion.
3. Modelo en MuJoCo.
4. Aprendizaje en mjlab.
5. Robot físico.

Detalle en [ROADMAP.md](ROADMAP.md). Requisitos y fuentes en
[documentation/research-and-requirements.md](documentation/research-and-requirements.md). Piezas, medidas
y precios en [documentation/architecture-and-bom.md](documentation/architecture-and-bom.md).

## Puesta en marcha

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
.venv/bin/python analysis/estimate_robot_budget.py
```

## Proyecto anterior

El trabajo con el robot de 70 cm y la propuesta de 40 cm está en la etiqueta de git
`archivo-danny-70cm`:

```bash
git switch --detach archivo-danny-70cm
```
