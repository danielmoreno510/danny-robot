# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.
Se actualiza al terminar cada fase y cada vez que cambia el estado.

**Fecha:** 30 de octubre de 2026. **Fase actual:** 5, diseño mecánico, antes que la fase 4.
**Siguiente paso:** ejecutar el esqueleto en Fusion y revisar la etapa 5.1.

## Cómo empezar

Si eres Claude en un chat nuevo, lee en este orden:

1. Este archivo.
2. [CLAUDE.md](CLAUDE.md): reglas de trabajo del usuario.
3. [ROADMAP.md](ROADMAP.md): plan y estado de cada etapa. Es la fuente de verdad.
4. [documentation/phase-5.1-mechanical-architecture.md](documentation/phase-5.1-mechanical-architecture.md):
   arquitectura mecánica, motores, coste y guía de Fusion.
5. [documentation/phase-3.5-validation.md](documentation/phase-3.5-validation.md): resultado de
   la fase, comparación con el bípedo y condiciones.
6. [documentation/phase-4-readiness-audit.md](documentation/phase-4-readiness-audit.md): qué
   falta para entrenar.

Prepara el proyecto, con Python 3.12:

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulation/scripts/run_checks.py
```

Debe terminar con `27 de 27 comprobaciones pasan`, en unos 3 minutos. La suite completa
tarda unos 19:

```bash
.venv/bin/python -m pytest tests
```

Si `.venv` ya existe, basta con las dos comprobaciones. Si algo no pasa, díselo al usuario
antes de seguir. El usuario escribe en español. Commit y push, solo cuando los pida.

## Qué es Danny hoy

Un humanoide de 70 cm, solo en simulación con MuJoCo. Hay dos modelos, generados desde
`simulation/scripts/robot_spec.py` con `build_model.py`. **No se editan a mano.**

| Modelo | Masa | GDL | Uso |
|---|---|---|---|
| `danny_biped.xml` | 8 kg | 12 | Referencia de la fase 3, no cambia |
| `danny_humanoid.xml` | 11 kg | 31 | El robot en desarrollo |

Los 31 GDL: piernas 6 × 2, cintura 3 (yaw, roll, pitch), brazos 7 × 2 y cuello 2. Las manos
son bloques rígidos y la cabeza es una esfera simple, sin cara, a petición del usuario.

## Qué está hecho

| Etapa | Resultado | Experimento |
|---|---|---|
| 3.5.1 | Presupuesto de masa: 11 kg | Documento propio |
| 3.5.2 | Modelo generado desde una especificación | E17 |
| 3.5.3 | Torso y cintura | E18 |
| 3.5.4 | Brazos de 7 GDL | E19 |
| 3.5.5 | Manos rígidas | E20 |
| 3.5.6 | Cuello y cabeza | E21 |
| Revisión | Motor por articulación y roll de cintura | E22 |
| 3.5.7 | Integración: cuerpo en movimiento, IMU, postura `stand` y sensibilidad | E23 |
| 3.5.8 | Validación y auditoría independiente del código | Documento propio |
| Auditoría | Preparación para la fase 4: **todavía no estamos listos** | Documento propio |

El humanoide, con motores de catálogo y brazos fijos, pasa E10 a E13 con los límites del
bípedo, salvo un cambio de plan: E12 espera 0,35 s entre pasos. Aguanta empujones de 48 N
adelante, 49 N atrás y 59 N de lado. E13 cumple con todo el cuerpo moviéndose a la vez. Su
sensibilidad es casi igual que la del bípedo. Detalle en el [handbook](docs/HANDBOOK.md), de E18
a E23.

## Qué sigue

**Cambio de plan del usuario:** primero el robot real en Autodesk Fusion, después MuJoCo con esas
medidas, y después Isaac Sim e Isaac Lab. **No empieces a entrenar.** No compres nada sin aprobación.

**Etapa 5.1:** evaluación entregada y script en `cad/fusion/DannyMasterSkeleton`. **No se ha ejecutado
en Fusion.** Falta que el usuario lo ejecute, comprobar el informe con
`export_cad_parameters.py --check`, y su revisión. Después: diseño detallado de pelvis y piernas.

**Pendiente de decisión del usuario:** puntos de seguimiento e IMU de cabeza, y el `kv` de los
brazos: con ζ 6,2 no oscilan libremente, y hay que decidirlo antes de buscar un balanceo natural.

**Antes de la fase 4 faltan también decisiones del usuario:** cómo entrenar, porque la RTX 3080 Ti
está por debajo del mínimo de Isaac Sim, y la licencia del repositorio. Hay que comprobar las licencias de AMASS y
LAFAN1. Isaac Lab ya está decidido, así que MJX deja de ser un requisito. La lista completa está en la sección 8 de la auditoría.

## Objetivo que guía las decisiones

Danny debe moverse de forma natural, con todo el cuerpo coordinado, sin sacrificar física,
estabilidad ni viabilidad de motores. La fase 4 combinará aprendizaje por refuerzo e imitación
de movimiento humano: primero seguir un clip adaptado, y después estilo adversarial con órdenes
de velocidad. Ver la [revisión](documentation/whole-body-motion-review.md).

## Decisiones ya tomadas

- 11 kg de objetivo; el margen de masa del torso está agotado.
- 7 GDL por brazo, cintura de 3 ejes y cabeza simple.
- `kp` por regla; `armature` y amortiguamiento son estimaciones que la fase 4 debe variar.
- Brazos separados 15° en la postura fija. Con ±20° de roll de cintura hay contactos reales, y
  se aceptan.
- No gastar tiempo en animaciones a mano que se vayan a sustituir.
- La auditoría automática del humanoide usa masa -5 %: con -10 % incumple por 0,1 mm.
- Extensión de cadera del humanoide: 30°. La flexión dorsal se queda y se penalizará al
  entrenar, porque la marcha clásica llega a 50°.

## Lo que no se debe afirmar

- Que exista equilibrio activo o compensación dinámica: no existen.
- Que la marcha clásica sea natural: va a 0,009 m/s, y solo sirve como comprobación.
- Que el exceso de los empujones sobre la estimación esté explicado: hay una causa probable, sin
  prueba en el repositorio.
- Que las cifras de ruido de encoder sean realistas: el ruido se suma al mando, no a la medida.
- Que la ventana del visor se haya abierto: nadie la ha comprobado, solo con una falsa.
- Que las 73 pruebas de realismo demuestren que el robot aguanta: solo miden.

## Última verificación

Del 30 de octubre de 2026, en macOS con Apple Silicon, Python 3.12 y MuJoCo 3.17:

- `pytest tests`: **236 de 236** en la última ejecución completa (19 min 3 s), más las 19 de CAD, que
  pasaron por separado.
- `run_checks.py`: **27 de 27**.
- Las cifras no cambian tras la auditoría, salvo el sobrepaso de tope, que ahora mide bien.
- pyflakes sin avisos.
- Camino del visor probado con una ventana falsa: la real no se ha abierto.

Las cifras vienen de `results/`. Vuelve a ejecutarlas antes de citarlas.
