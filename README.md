# Robot Danny

**Danny** es un robot humanoide autónomo de unos 70 cm que se está construyendo
desde cero, paso a paso y en abierto. El objetivo final es que camine, mantenga
el equilibrio, manipule objetos, entienda instrucciones con inteligencia
artificial y se comunique a través de una aplicación móvil. Debe moverse de forma
fluida y parecida a una persona, sin renunciar a lo que la física y sus motores
permiten.

Hoy Danny existe solo en simulación, con [MuJoCo](https://mujoco.org). Hay dos modelos:
el bípedo de la fase 3 (70 cm, 8 kg, 12 grados de libertad) y el humanoide completo en
validación (70 cm, 11 kg, 31 grados de libertad).

> **Importante:** las dimensiones, masas y motores del modelo son **provisionales**
> hasta elegir los motores (etapa 31). No deben usarse para fabricar piezas.

## Qué hace hoy

- Se mantiene de pie y sobre cualquiera de los dos pies.
- Transfiere el peso de una pierna a otra.
- Camina en marcha estática: pasos de 8 cm y giros, con cada pie a menos de 1 mm
  de donde se planificó.
- Aguanta empujones de 24 a 32 N durante 0,1 s sin dar un paso, de forma pasiva.
- El humanoide (11 kg y 31 GDL: cintura de 3 ejes, brazos de 7 GDL, manos rígidas, cuello y cabeza
  simple) repite el equilibrio y la marcha, con los brazos en postura fija y también
  con la cintura moviéndose.
- Sigue caminando aunque sus masas difieran un ±10 % de las que cree tener.

Qué no hace todavía: marcha dinámica, recuperarse dando un paso, y nada con
hardware real.

**Límite importante:** el control sigue su plan sin corregir con la IMU. Un error
de 1 cm en el centro de masas, o un suelo inclinado 0,25°, bastan para incumplir
los criterios. Los detalles están en el
[handbook](docs/HANDBOOK.md#experimento-16--el-cuerpo-de-danny-70-cm-y-8-kg).

## Dónde estamos

| Fase | Objetivo | Estado |
|---|---|---|
| 1 | Entorno y fundamentos | ✅ |
| 2 | Cinemática y articulaciones | ✅ |
| 3 | Simulación física y control del bípedo | 🟡 [CONDITIONAL PASS](documentation/phase-3-validation.md) |
| **3.5** | **Humanoide completo en simulación** | 🟡 **CONDITIONAL PASS** |
| 4 | Aprendizaje por refuerzo | ⏳ |
| 5–8 | Diseño mecánico, fabricación, electrónica y paso al robot real | ⏳ |
| 9–12 | Torso y brazos, visión e IA, manipulación, aplicación móvil | ⏳ |

El detalle está en el [roadmap](ROADMAP.md).

## Instalación

Probado en macOS sobre Apple Silicon con Python 3.12 y MuJoCo 3.17. En otros
sistemas debería funcionar con `python` en lugar de `mjpython`, pero no se ha
probado.

Instala Python 3.12 (en macOS, con Homebrew):

```bash
brew install python@3.12
```

Descarga el proyecto y crea el entorno:

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Comprueba que todo funciona (tarda unos 3 minutos y no abre ventanas):

```bash
.venv/bin/python simulation/scripts/run_checks.py
```

Debe terminar con `27 de 27 comprobaciones pasan`. La salida completa de cada
prueba queda en [results/](results/).

La suite de validación, más completa, tarda unos 17 minutos:

```bash
.venv/bin/python -m pytest tests
```

## Verlo en el simulador

En macOS, el visor solo funciona lanzado con `mjpython`, que se instala con
MuJoCo. Desde la carpeta del proyecto:

| Qué se ve | Comando |
|---|---|
| Un cubo que cae | `.venv/bin/mjpython simulation/scripts/view_falling_box.py` |
| Una articulación con motor | `.venv/bin/mjpython simulation/scripts/control_single_joint.py` |
| Una pierna colgando | `.venv/bin/mjpython simulation/scripts/control_leg_2dof.py` |
| Sentadilla con una pierna | `.venv/bin/mjpython simulation/scripts/squat_leg_2dof.py --view` |
| Pierna de 6 GDL | `.venv/bin/mjpython simulation/scripts/analyze_leg_6dof.py --view` |
| Equilibrio sobre un pie | `.venv/bin/mjpython simulation/scripts/balance_biped.py --view` |
| Marcha | `.venv/bin/mjpython simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25 --view` |
| Marcha con giro | `.venv/bin/mjpython simulation/scripts/walk_biped.py --knee 56 --step 0.08 --width 0.05 --inset 0.012 --turn 45 --speed 1.27 --hold 0.25 --view` |
| Empujón | `.venv/bin/mjpython simulation/scripts/push_biped.py --force 20 --view` |
| Humanoide: equilibrio | `.venv/bin/mjpython simulation/scripts/balance_biped.py --robot humanoid --view` |
| Humanoide: marcha con giro | `.venv/bin/mjpython simulation/scripts/walk_biped.py --robot humanoid --knee 56 --step 0.08 --width 0.05 --inset 0.012 --turn 45 --speed 1.27 --hold 0.25 --view` |
| Humanoide: empujón | `.venv/bin/mjpython simulation/scripts/push_biped.py --robot humanoid --force 40 --view` |

Los scripts con `--view` imprimen primero el análisis y después abren la ventana.

## Documentación

| Documento | Para qué |
|---|---|
| [ROADMAP.md](ROADMAP.md) | El plan: 12 fases y 76 etapas, hasta el robot físico. Dónde vamos |
| [docs/HANDBOOK.md](docs/HANDBOOK.md) | Lo hecho: cada experimento, resultados, errores y lecciones. Para aprender o reproducirlo |
| [documentation/phase-3-validation.md](documentation/phase-3-validation.md) | Auditoría de la fase 3: qué funciona, qué no y qué falta |
| [documentation/known-limitations.md](documentation/known-limitations.md) | Limitaciones actuales del robot |
| [documentation/phase-3.5-proposal.md](documentation/phase-3.5-proposal.md) | Propuesta aprobada del humanoide completo |
| [documentation/phase-3.5-mass-budget.md](documentation/phase-3.5-mass-budget.md) | Presupuesto de masa con motores de catálogo |
| [documentation/phase-3.5-validation.md](documentation/phase-3.5-validation.md) | Validación de la fase 3.5, comparación con el bípedo y auditoría |
| [documentation/whole-body-motion-review.md](documentation/whole-body-motion-review.md) | Revisión del cuerpo para movimiento natural y plan de la fase 4 |
| [documentation/phase-4-readiness-audit.md](documentation/phase-4-readiness-audit.md) | Auditoría de preparación: qué falta para entrenar |
| [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md) | Relevo: cómo seguir en otro chat o en otra computadora |
| [CLAUDE.md](CLAUDE.md) | Reglas de trabajo y estado actual, para continuar el proyecto con Claude Code |

**Etapas y experimentos.** Las **etapas** (1 a 76) son los pasos del roadmap. Los
**experimentos** (E1, E2…) son el trabajo concreto hecho para cumplirlas.

## Estructura

```
danny-robot/
├── README.md           este archivo
├── ROADMAP.md          plan del proyecto
├── CLAUDE.md           reglas y estado, para Claude Code
├── PROJECT_HANDOFF.md  relevo para seguir en otro chat u otra computadora
├── requirements.txt    dependencias fijadas
├── docs/
│   └── HANDBOOK.md     detalle técnico de cada experimento
├── documentation/
│   ├── phase-3-validation.md   auditoría de la fase 3
│   ├── known-limitations.md    limitaciones actuales
│   ├── phase-3.5-proposal.md   propuesta del humanoide completo
│   ├── phase-3.5-mass-budget.md   presupuesto de masa
│   ├── phase-3.5-validation.md   validación de la fase 3.5
│   ├── whole-body-motion-review.md   revisión para movimiento natural
│   └── phase-4-readiness-audit.md   auditoría de preparación
├── tests/              suite de validación automática
├── results/            salida de la última ejecución de run_checks.py
└── simulation/
    ├── models/                     modelos MJCF (XML de MuJoCo)
    │   ├── falling_box.xml         E1: cubo con freejoint
    │   ├── single_joint.xml        E2: un segmento con motor
    │   ├── leg_2dof.xml            E4-E7: pierna colgando
    │   ├── leg_2dof_foot.xml       E8: pierna con pie
    │   ├── leg_6dof.xml            E9: pierna didáctica de 6 GDL con pie plano
    │   ├── danny_biped.xml         E10 en adelante: generado, no editar a mano
    │   └── danny_humanoid.xml      E18 a E23: humanoide de 31 GDL; generado, no editar a mano
    └── scripts/
        ├── robot_spec.py           datos del robot: medidas, masas, articulaciones
        ├── build_model.py          genera los modelos desde robot_spec.py
        ├── leg_control.py          módulo: compensadores, IK, perfil, sensores
        ├── biped_control.py        módulo: poses, reparto del peso, paso de control
        ├── run_checks.py           ejecuta todas las comprobaciones
        ├── check_kinematics.py     IK contra MuJoCo
        ├── view_falling_box.py     E1
        ├── control_single_joint.py E2
        ├── analyze_leg_2dof.py     E5, E7: análisis de esfuerzos
        ├── control_leg_2dof.py     E6, E7: ver la pierna colgando
        ├── squat_leg_2dof.py       E8: sentadilla
        ├── analyze_leg_6dof.py     E9: pierna de 6 GDL
        ├── balance_biped.py        E10: equilibrio sobre un pie
        ├── walk_biped.py           E11-E13: marcha y giro
        └── push_biped.py           E14: empujones
```

## Licencia

Todavía no tiene licencia: el código se puede ver, pero aún no reutilizar.
