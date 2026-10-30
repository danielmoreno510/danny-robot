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

Las 9 comprobaciones deben pasar antes de empezar a trabajar. Si no pasan, dilo.

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

Fase 3, simulación física y control del bípedo. Experimentos E1 a E14 hechos. Las
9 comprobaciones pasan.

**Falta para cerrar la fase 3:**

1. **Etapa 18, compensación dinámica.** Existe (`walk_biped.py --dynamic`,
   `Biped.required_contact`), pero inclina más la pelvis que la estática. No es por
   el momento angular, ni por el perfil de trayectoria, ni por la inercia de los
   eslabones. Hay que averiguarlo o decidir dejarla.
2. **Etapa 13.** Masas, inercias y contactos son didácticos.
3. **Etapa 20.** Ajustar el modelo a los 70 cm (hoy mide unos 60 cm, sin torso, 6,2
   kg) y hacer la auditoría técnica de toda la simulación.

**Pendiente de decidir por el usuario:**

- Licencia: el repositorio es público y no tiene.
- Propuestas del final de `ROADMAP.md`: torso provisional, límites de motor
  realistas y ruido de sensores.

**Decisiones ya tomadas:**

- El paso de recuperación no funcionó con control clásico: el robot salta en vez
  de dar un paso. Se pasó a la etapa 26. El código está en el commit `6c083cc`.
- La marcha usa compensación estática. Lo que permitió ir más rápido fue recortar
  pausas, no un control más complejo.
