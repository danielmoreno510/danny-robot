# Robot Danny

**Danny** es un robot humanoide autónomo de unos 70 cm que se está construyendo
desde cero, paso a paso y en abierto. El objetivo final es que camine, mantenga
el equilibrio, manipule objetos, entienda instrucciones con inteligencia
artificial y se comunique a través de una aplicación móvil.

Hoy Danny existe solo en simulación, con [MuJoCo](https://mujoco.org). Es un
bípedo de dos piernas de 6 grados de libertad, sin torso ni brazos.

> **Importante:** las dimensiones, masas y límites de motor de los modelos son
> **didácticos y provisionales**. No deben usarse para dimensionar piezas reales.

## Qué hace hoy

- Se mantiene de pie y sobre cualquiera de los dos pies.
- Transfiere el peso de una pierna a otra.
- Camina en marcha estática: pasos de 12 cm y giros, con cada pie a menos de 2 mm
  de donde se planificó.
- Aguanta empujones de 23 a 25 N durante 0,1 s sin dar un paso.

Qué no hace todavía: marcha dinámica, recuperarse dando un paso, y nada con
hardware real. Hoy el control usa datos exactos del simulador, sin ruido de
sensores.

## Dónde estamos

| Fase | Objetivo | Estado |
|---|---|---|
| 1 | Entorno y fundamentos | ✅ |
| 2 | Cinemática y articulaciones | ✅ |
| **3** | **Simulación física y control del bípedo** | 🔄 **Actual** |
| 4 | Aprendizaje por refuerzo | ⏳ |
| 5–8 | Diseño mecánico, fabricación, electrónica y paso al robot real | ⏳ |
| 9–12 | Torso y brazos, visión e IA, manipulación, aplicación móvil | ⏳ |

Faltan por cerrar en la fase 3 las etapas 13 y 20. El detalle está en el
[roadmap](ROADMAP.md).

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

Comprueba que todo funciona (tarda menos de un minuto y no abre ventanas):

```bash
.venv/bin/python simulation/scripts/run_checks.py
```

Debe terminar con `10 de 10 comprobaciones pasan`. La salida completa de cada
prueba queda en [results/](results/).

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
| Marcha con giro | `.venv/bin/mjpython simulation/scripts/walk_biped.py --knee 56 --step 0.12 --width 0.07 --inset 0.02 --turn 45 --speed 1.27 --hold 0.25 --view` |
| Empujón | `.venv/bin/mjpython simulation/scripts/push_biped.py --force 20 --view` |

Los scripts con `--view` imprimen primero el análisis y después abren la ventana.

## Documentación

| Documento | Para qué |
|---|---|
| [ROADMAP.md](ROADMAP.md) | El plan: 12 fases y 76 etapas, hasta el robot físico. Dónde vamos |
| [docs/HANDBOOK.md](docs/HANDBOOK.md) | Lo hecho: cada experimento, resultados, errores y lecciones. Para aprender o reproducirlo |
| [CLAUDE.md](CLAUDE.md) | Reglas de trabajo y estado actual, para continuar el proyecto con Claude Code |

**Etapas y experimentos.** Las **etapas** (1 a 76) son los pasos del roadmap. Los
**experimentos** (E1, E2…) son el trabajo concreto hecho para cumplirlas.

## Estructura

```
danny-robot/
├── README.md           este archivo
├── ROADMAP.md          plan del proyecto
├── CLAUDE.md           reglas y estado, para Claude Code
├── requirements.txt    dependencias fijadas
├── docs/
│   └── HANDBOOK.md     detalle técnico de cada experimento
├── results/            salida de la última ejecución de run_checks.py
└── simulation/
    ├── models/                     modelos MJCF (XML de MuJoCo)
    │   ├── falling_box.xml         E1: cubo con freejoint
    │   ├── single_joint.xml        E2: un segmento con motor
    │   ├── leg_2dof.xml            E4-E7: pierna colgando
    │   ├── leg_2dof_foot.xml       E8: pierna con pie
    │   ├── leg_6dof.xml            E9: pierna de 6 GDL con pie plano
    │   └── biped_6dof.xml          E10-E14: Danny, dos piernas y pelvis libre
    └── scripts/
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
        ├── walk_biped.py           E11-E13, E15: marcha, giro y compensación dinámica
        └── push_biped.py           E14: empujones
```

## Licencia

Todavía no tiene licencia: el código se puede ver, pero aún no reutilizar.
