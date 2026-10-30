# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.
Se actualiza al terminar cada fase y cada vez que cambia el estado.

**Fecha:** 30 de octubre de 2026. **Fase actual:** 3.5, humanoide completo.
**Siguiente etapa:** 3.5.7, integración.

## Cómo empezar

Si eres Claude en un chat nuevo, lee en este orden:

1. Este archivo.
2. [CLAUDE.md](CLAUDE.md): reglas de trabajo del usuario.
3. [ROADMAP.md](ROADMAP.md): plan y estado de cada etapa. Es la fuente de verdad.
4. [documentation/phase-4-readiness-audit.md](documentation/phase-4-readiness-audit.md): qué
   falta para entrenar.

Prepara el proyecto, con Python 3.12:

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulation/scripts/run_checks.py
```

Debe terminar con `21 de 21 comprobaciones pasan`, en unos 2 minutos. La suite completa
tarda unos 8:

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
| Auditoría | Preparación para la fase 4: **todavía no estamos listos** | Documento propio |

El humanoide, con motores de catálogo y brazos fijos, pasa E10 a E13 con los límites del
bípedo, salvo un cambio de plan: E12 espera 0,35 s entre pasos. Aguanta empujones de 48 N
adelante, 49 N atrás y 59 N de lado. Detalle en el
[handbook](docs/HANDBOOK.md), de E18 a E22.

## Qué sigue

**Etapa 3.5.7, integración:**

- Repetir en el humanoide las medidas de sensibilidad de `tests/test_realism.py`: suelo
  inclinado, errores de masa, ruido y retardos. Nunca se han hecho con este modelo.
- Añadir giróscopo, acelerómetro, sensores de velocidad articular y una postura inicial
  guardada. Hoy el modelo no tiene ninguno.
- Traer al usuario una propuesta para los rangos de cadera y tobillo: 90° de extensión y 65°
  de flexión dorsal superan lo humano. Estrechar o penalizar.
- Proponerle puntos de seguimiento e IMU de cabeza: no están aprobados.

**Etapa 3.5.8, validación:** informe de la fase, comparación con el bípedo y auditoría
independiente del código, que se repite al cerrar cada fase. Después, avisar al usuario y
actualizar este archivo. **El usuario abrirá un chat nuevo para la fase 4.** No empieces la fase
4 sin su revisión.

**Antes de la fase 4 faltan también decisiones del usuario:** GPU local o en la nube, y la
licencia del repositorio. Hay que comprobar que el modelo funciona en MJX y las licencias de
AMASS y LAFAN1. La lista completa está en la sección 8 de la auditoría.

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

## Lo que no se debe afirmar

- Que exista equilibrio activo o compensación dinámica: no existen.
- Que la marcha clásica sea natural: va a 0,009 m/s, y solo sirve como comprobación.
- Que el exceso de los empujones sobre la estimación esté explicado: no lo está.
- Que la ventana del visor se haya abierto: nadie la ha comprobado, solo con una falsa.
- Que las 37 pruebas de realismo demuestren que el robot aguanta: solo miden.

## Última verificación

Del 30 de octubre de 2026, en macOS con Apple Silicon, Python 3.12 y MuJoCo 3.17:

- `pytest tests`: **189 pruebas pasan**, en 8 min 33 s.
- `run_checks.py`: **21 de 21**.
- Las 277 métricas del bípedo no cambian desde el commit `69fbdd1`.
- pyflakes sin avisos.
- Camino del visor probado con una ventana falsa: la real no se ha abierto.

Las cifras vienen de `results/`. Vuelve a ejecutarlas antes de citarlas.
