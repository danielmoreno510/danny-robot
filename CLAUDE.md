# Robot Danny — instrucciones para Claude

Robot humanoide autónomo de unos 70 cm, hoy solo en simulación con MuJoCo. El
usuario escribe en español: responde en español. **Empieza por
[PROJECT_HANDOFF.md](PROJECT_HANDOFF.md).** Lee [README.md](README.md) para
la visión general y [ROADMAP.md](ROADMAP.md) para el plan y el estado de cada
etapa. Ese estado es la fuente de verdad: mantenlo al día.

**Objetivo de movimiento (prioritario a largo plazo):** Danny debe moverse de forma natural,
con todo el cuerpo coordinado. No sacrifiques física, estabilidad ni viabilidad de motores
para que se vea humano. No inviertas en animaciones a mano que se vayan a sustituir. La
cabeza es simple, sin cara ni actuadores. Ver
[la revisión](documentation/whole-body-motion-review.md).

## Puesta en marcha

Si no existe `.venv` (por ejemplo, tras clonar en otra máquina):

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulation/scripts/run_checks.py
```

Las 27 comprobaciones deben pasar antes de empezar a trabajar. Si no pasan, dilo.

## Reglas del usuario

1. **Documentación siempre al día.** Cada cambio actualiza lo que corresponda:
   `README.md`, `ROADMAP.md`, `docs/HANDBOOK.md` y este archivo. Sin menciones
   obsoletas.
2. **Decir cómo verlo.** Tras cada cambio, da el comando exacto para verlo en el
   simulador (`.venv/bin/mjpython simulation/scripts/<script>.py --view`).
3. **Quitar lo que no se usa.** Archivos y código muerto fuera. Git guarda el
   historial.
4. **Nombres descriptivos.** Sin versiones como `_v2` ni `first_`. Modelos por lo
   que contienen (`leg_2dof_foot.xml`), scripts verbo + objeto
   (`analyze_leg_6dof.py`). El código compartido va en módulos (`leg_control.py`,
   `biped_control.py`): un script de análisis nunca importa de otro.
5. **Python 3.12**, siempre con el `.venv` del proyecto, también en scripts
   auxiliares.
6. **Commit y push solo cuando el usuario lo pida.**
7. **Avisar al terminar cada fase.** Una fase necesita implementación, validación
   reproducible y documentación.
8. **Dejar el relevo escrito al terminar cada fase.** Actualiza `PROJECT_HANDOFF.md` y la
   sección «Estado actual» de este archivo. El usuario abre un chat nuevo entre fases: debe
   poder seguirse desde otro chat u otra computadora sin nada más.

## Cómo se trabaja

- **Etapas** (1 a 76): las del roadmap. **Experimentos** (E1, E2…): el trabajo
  concreto en el handbook. No las mezcles.
- **Fijar los criterios de aceptación antes de ejecutar**, y calcular antes si son
  físicamente posibles. Si se cambian, decirlo y documentarlo.
- **Medir antes de cambiar.** Ante un fallo, primero mira qué pasa instante a
  instante. Si un arreglo no explica el fallo, no se deja.
- **Decir lo que no se ha comprobado**, y documentar también lo que falla.
- Unidades SI; ejes REP-103; detalle en el handbook, sección 1.
- **`danny_biped.xml` y `danny_humanoid.xml` no se editan a mano.** Cambia `robot_spec.py` y ejecuta
  `build_model.py`. Unas pruebas comprueban que coinciden.

Las cifras de la documentación salen siempre de `results/`, nunca de memoria.

Para añadir un experimento:

1. Script con criterios y código de salida 1 si falla, con `--view` si se puede ver.
2. Añadirlo a `CHECKS` en `simulation/scripts/run_checks.py`.
3. Ejecutar `run_checks.py`, `pytest tests`, pyflakes y una prueba del camino del visor.
4. Documentarlo en el handbook, y actualizar README y ROADMAP.

El visor de macOS necesita `mjpython`. Un agente no puede verlo: la parte de
`--view` se prueba sustituyendo la ventana por un objeto falso. Di siempre que la
ventana no se ha abierto.

## Estado actual

Fases 3 y 3.5 en **CONDITIONAL PASS**, revisadas y aceptadas por el usuario. Limitaciones en
`documentation/known-limitations.md`. **Actual: fase 5, diseño mecánico, antes que la fase 4.**
Etapa 5.1 entregada, en `documentation/phase-5.1-mechanical-architecture.md`; falta ejecutarla en
Fusion y la revisión del usuario.
La propuesta aprobada está en `documentation/phase-3.5-proposal.md`, y la revisión
`documentation/whole-body-motion-review.md` la amplió a 31 GDL. Mide 70 cm. El
objetivo de masa es 11 kg, con el reparto de `documentation/phase-3.5-mass-budget.md`.

Hecha la etapa 3.5.8: 31 GDL, cada articulación con su motor en `robot_spec.py`, IMU, velocidades
y postura `stand`. Armature y amortiguamiento son estimaciones.
Validación en `documentation/phase-3.5-validation.md`. **Antes de entrenar:** GPU para Isaac Lab y licencias.

**Se avanza por etapas, con revisión: no implementes todo a la vez.** Tras la fase 5 se actualiza MuJoCo, y después viene la fase 4, con Isaac Lab, cuyo entorno debe
cumplir la lista del roadmap. **No compres nada sin aprobación.** Di siempre que un script de Fusion
no se ha ejecutado si no hay `skeleton_report.json`. No marques como implementados la compensación dinámica ni el
equilibrio activo.

Comprobaciones: `run_checks.py` (27, unos 3 minutos) y `python -m pytest tests` (255, unos
19 minutos).
Las pruebas de realismo miden sin exigir: que pasen no significa que el robot aguante
ese efecto. Mira `results/phase3_metrics.json`.

**Pendiente de decidir por el usuario:**

- Licencia: el repositorio es público y no tiene.
- Hardware de entrenamiento: la RTX 3080 Ti está por debajo del mínimo de Isaac Sim.
- Prototipo de una articulación y presupuesto (opciones A, B y C).

**Decisiones ya tomadas:**

- El bípedo (`danny_biped.xml`) mide 70 cm y pesa 8 kg, con torso provisional. El
  humanoide (`danny_humanoid.xml`) pesa 11 kg, provisionales hasta la etapa 31.
- La marcha rápida del humanoide (E12) usa una espera de 0,35 s, no 0,25 s: con menos,
  incumple por muy poco el error en reposo. Ver E18 y E22.
- El paso de recuperación no funcionó con control clásico: el robot salta en vez
  de dar un paso. Se pasó a la etapa 26. El código está en el commit `6c083cc`.
- Extensión de cadera del humanoide: 30°. La flexión dorsal se penalizará al entrenar.
- La marcha usa compensación estática. La dinámica se probó (E15) y se eliminó: no
  aporta con el cuerpo de Danny. El código está en el commit `f24ed39`.
- La auditoría de la etapa 20 fue una revisión independiente del código, que
  encontró fallos que las pruebas no veían: repítela al cerrar cada fase.
