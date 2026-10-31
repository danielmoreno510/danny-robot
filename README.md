# Robot Danny

Humanoide pequeño, autónomo y económico, de 32-36 cm, inspirado en el XGO-Duck: impreso en 3D, con
servos de bus y movimiento natural aprendido en simulación. El XGO-Duck es una referencia, no una
plantilla.

**Estado:** etapa 1, investigación y definición. Aún no hay diseño ni piezas.

## Plan

1. Investigación y definición.
2. Diseño en Autodesk Fusion.
3. Modelo en MuJoCo.
4. Aprendizaje en mjlab.
5. Robot físico.

Detalle en [ROADMAP.md](ROADMAP.md). Requisitos y fuentes en
[documentation/research-and-requirements.md](documentation/research-and-requirements.md).

## Puesta en marcha

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
```

## Proyecto anterior

El trabajo con el robot de 70 cm y la propuesta de 40 cm está en la etiqueta de git
`archivo-danny-70cm`:

```bash
git switch --detach archivo-danny-70cm
```
