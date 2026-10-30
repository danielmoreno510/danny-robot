# Robot Danny — instrucciones para Claude

Robot humanoide autónomo de unos 70 cm, hoy solo en simulación con MuJoCo. El
usuario escribe en español: responde en español. Lee [README.md](README.md) para
la visión general y [ROADMAP.md](ROADMAP.md) para el plan y el estado de cada
etapa. Ese estado es la fuente de verdad: mantenlo al día.

## Puesta en marcha

Si no existe `.venv` (por ejemplo, tras clonar en otra máquina):

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulation/scripts/run_checks.py
```

Las 16 comprobaciones deben pasar antes de empezar a trabajar. Si no pasan, dilo.

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
7. **Avisar al terminar cada fase.** En particular, avisa cuando termine la fase 3.
   Una fase necesita implementación, validación reproducible y documentación.

## Cómo se trabaja

- **Etapas** (1 a 76): las del roadmap. **Experimentos** (E1, E2…): el trabajo
  concreto en el handbook. No las mezcles.
- **Fijar los criterios de aceptación antes de ejecutar**, y calcular antes si son
  físicamente posibles. Si se cambian, decirlo y documentarlo.
- **Medir antes de cambiar.** Ante un fallo, primero mira qué pasa instante a
  instante. Si un arreglo no explica el fallo, no se deja.
- **Decir lo que no se ha comprobado**, y documentar también lo que falla.
- Unidades SI; ejes REP-103; detalle en el handbook, sección 1.

Para añadir un experimento:

1. Script con criterios y código de salida 1 si falla, con `--view` si se puede ver.
2. Añadirlo a `CHECKS` en `simulation/scripts/run_checks.py`.
3. Ejecutar `run_checks.py`, pyflakes y una prueba del camino del visor.
4. Documentarlo en el handbook, y actualizar README y ROADMAP.

El visor de macOS necesita `mjpython`. Un agente no puede verlo: la parte de
`--view` se prueba sustituyendo la ventana por un objeto falso. Di siempre que la
ventana no se ha abierto.

## Estado actual

Fases 1, 2 y 3 terminadas. Experimentos E1 a E16 hechos. Las 16 comprobaciones
pasan. **Siguiente: fase 4, aprendizaje por refuerzo** (etapas 21 a 28).

**Límites conocidos de la fase 3**, a tener en cuenta en la fase 4: el control sigue
su plan sin corregir con la IMU. Un error de 1 cm en el CdM o un suelo inclinado
0,25° incumplen los criterios. Detalle en el handbook, E16.

**Pendiente de decidir por el usuario:**

- Licencia: el repositorio es público y no tiene.
- Propuestas del final de `ROADMAP.md`: límites de motor realistas y ruido de
  sensores.

**Decisiones ya tomadas:**

- Danny mide 70 cm y pesa 8 kg, con torso provisional (`danny.xml`), hasta la
  etapa 31.
- El paso de recuperación no funcionó con control clásico: el robot salta en vez
  de dar un paso. Se pasó a la etapa 26. El código está en el commit `6c083cc`.
- La marcha usa compensación estática. La dinámica se probó (E15) y se eliminó: no
  aporta con el cuerpo de Danny. El código está en el commit `f24ed39`.
- La auditoría de la etapa 20 fue una revisión independiente del código, que
  encontró fallos que las pruebas no veían: repítela al cerrar cada fase.
