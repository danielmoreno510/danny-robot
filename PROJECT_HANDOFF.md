# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.

**Fecha:** 31 de octubre de 2026. **Etapa actual:** 1, investigación y definición.
**Siguiente paso:** aprobar la compra del banco de pruebas de la etapa 1.2.

## Cómo empezar

Lee en este orden:

1. Este archivo.
2. [CLAUDE.md](CLAUDE.md): reglas de trabajo.
3. [ROADMAP.md](ROADMAP.md): plan y estado.
4. [documentation/research-and-requirements.md](documentation/research-and-requirements.md):
   decisiones, requisitos, referencias y actuadores.

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
```

## Situación

El 31 de octubre de 2026 el usuario reinició el proyecto: los modelos de 70 cm y la propuesta de 40 cm con
STS3215 quedan en la etiqueta `archivo-danny-70cm`. Sirve de consulta, no como base. Ya no son válidos
sus requisitos, sus 31 GDL ni Isaac Lab.

Nueva dirección: humanoide de 32-36 cm, hasta 1 500 €, inspirado en el XGO-Duck sin copiarlo.
Prioridad: caminar natural y levantarse. Saltar, correr y manipular se intentan si el diseño lo
permite. Entrenamiento con mjlab en la RTX 3080 Ti.

**Candidato a validar:** todo HD-1910, porque el STS3215 pesa más y es más lento. Hay que medir antes su
calor, porque los picos superan el par nominal.

## Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que las cifras de `results/actuator_comparison.json` sean de diseño: son de primer orden.

## Pendiente de decidir por el usuario

- Compra del banco de pruebas.
- Licencia del repositorio, que es público.

## Última verificación

`analysis/compare_actuators.py` y pyflakes, el 31 de octubre de 2026. No hay simulador, pruebas ni CAD
en esta nueva etapa.
