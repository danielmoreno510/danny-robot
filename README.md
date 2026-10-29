# H1 — Humanoide desde cero en MuJoCo

Proyecto de aprendizaje para construir, paso a paso, la simulación de un robot
humanoide (H1) con [MuJoCo](https://mujoco.org). Empezamos por un cubo que cae
y vamos añadiendo articulaciones, motores, sensores y análisis, con un criterio
claro en cada etapa antes de pasar a la siguiente.

Este documento es el handbook del proyecto: explica qué hay, por qué está
hecho así y cómo reproducirlo. Debería bastar para que otra persona continúe
el trabajo o lo implemente por su cuenta.

> **Importante:** las dimensiones, masas y límites de los modelos son
> **didácticos**. No son los del robot H1 real, y no deben usarse para
> dimensionar motores reales.

---

## Índice

1. [Estado actual](#1-estado-actual)
2. [Instalación](#2-instalación)
3. [Estructura del repositorio](#3-estructura-del-repositorio)
4. [Convenciones](#4-convenciones)
5. [Recorrido por etapas](#5-recorrido-por-etapas)
6. [La pierna de 2 GDL en detalle](#6-la-pierna-de-2-gdl-en-detalle)
7. [Análisis de esfuerzos y resultados](#7-análisis-de-esfuerzos-y-resultados)
8. [Lecciones aprendidas](#8-lecciones-aprendidas)
9. [Hoja de ruta](#9-hoja-de-ruta)
10. [Glosario](#10-glosario)

---

## 1. Estado actual

| Etapa | Qué es | Estado |
|---|---|---|
| 1 | Cuerpo libre que cae (`falling_box.xml`) | ✅ |
| 2 | Una articulación con motor de posición (`single_joint.xml`) | ✅ |
| 3 | Primera pierna de 2 GDL, cadera + rodilla | ✅ sustituida en la etapa 4 (archivo eliminado) |
| 4 | Pierna de 2 GDL: unidades SI, límite de torque, sensores (`leg_2dof.xml`) | ✅ |
| 5 | Análisis de esfuerzos con criterios de aceptación (`analyze_leg_2dof.py`) | ✅ PASS en modo suave |
| 6 | Control con trayectorias suaves (`control_leg_2dof.py`) | ✅ |
| 7 | Compensación de gravedad (`GravityCompensator`) | ✅ PASS, error < 0,2° |
| 8 | Pie, contacto y sentadilla (`leg_2dof_foot.xml`, `squat_leg_2dof.py`) | ✅ PASS con compensación de apoyo |
| 9 | Pierna de 6 GDL con pie plano (`leg_6dof.xml`, `analyze_leg_6dof.py`) | ✅ PASS en aire y en apoyo |

La pierna de 2 GDL recorre su secuencia de posiciones con un torque pico de
**5,3 N·m** (límite: 20 N·m), sin saturar, y llega **exactamente** a cada
objetivo gracias a la compensación de gravedad.

Con el pie apoyado, la pierna sostiene la pelvis en sentadilla hasta 120° de
rodilla con un torque de rodilla de **7,2 N·m**, sin que el pie deslice.

La pierna completa de 6 GDL (cadera de 3 ejes, rodilla, tobillo de 2 ejes y pie
plano) mueve cada eje en el sentido documentado y hace la misma sentadilla con
el pie plano a menos de 0,03°.

---

## 2. Instalación

Probado con:

| | Versión |
|---|---|
| macOS | 27.0, Apple Silicon (arm64) |
| Python | 3.12 |
| MuJoCo | 3.17.0 |
| NumPy | 2.5.4 |

El proyecto usa **Python 3.12**. En macOS, instálalo con Homebrew y haz que
sea el `python3` por defecto añadiendo su carpeta al PATH en `~/.zshrc`:

```bash
brew install python@3.12
echo 'export PATH="/opt/homebrew/opt/python@3.12/libexec/bin:$PATH"' >> ~/.zshrc
```

Abre una terminal nueva y comprueba que `python3`, `python` y `pip3` son 3.12:

```bash
python3 --version && python --version && pip3 --version
```

```bash
git clone <url-del-repo> humanoid-h1   # o copia la carpeta
cd humanoid-h1
python3 -m venv .venv     # python3 = 3.12
source .venv/bin/activate
pip install -r requirements.txt
```

Comprueba que funciona (no abre ventana):

```bash
python simulation/scripts/analyze_leg_2dof.py
```

Debe terminar con dos `PASS`: uno para el modo `smooth` (error < 2°) y otro
para `smooth + gravedad` (error < 0,2°). Y para la etapa 8:

```bash
python simulation/scripts/squat_leg_2dof.py
```

que debe terminar con `PASS` para la compensación de apoyo. Y para la etapa 9:

```bash
python simulation/scripts/analyze_leg_6dof.py
```

### Visor en macOS: `mjpython`

En macOS el visor interactivo (`mujoco.viewer.launch_passive`) **solo funciona
lanzado con `mjpython`**, que se instala junto con el paquete `mujoco`. Con
`python` normal el script falla al abrir la ventana. En Linux y Windows basta
con `python`.

```bash
mjpython simulation/scripts/control_leg_2dof.py
```

Los scripts de análisis sin ventana funcionan con `python` en cualquier sistema.

### Ver cada etapa en el simulador

Desde la carpeta del proyecto, sin necesidad de activar `.venv`:

| Etapa | Comando |
|---|---|
| 1 — cubo que cae | `.venv/bin/mjpython simulation/scripts/view_falling_box.py` |
| 2 — una articulación | `.venv/bin/mjpython simulation/scripts/control_single_joint.py` |
| 5 — análisis (suave + gravedad) | `.venv/bin/mjpython simulation/scripts/analyze_leg_2dof.py --view` |
| 6–7 — pierna colgando | `.venv/bin/mjpython simulation/scripts/control_leg_2dof.py` |
| 8 — sentadilla con pie | `.venv/bin/mjpython simulation/scripts/squat_leg_2dof.py --view` |
| 9 — pierna de 6 GDL | `.venv/bin/mjpython simulation/scripts/analyze_leg_6dof.py --view` |

Los scripts con `--view` primero imprimen el análisis y después abren la
ventana. Cierra la ventana para terminar.

---

## 3. Estructura del repositorio

```
humanoid-h1/
├── README.md              este handbook
├── requirements.txt       dependencias fijadas
├── .gitignore             excluye .venv/, cachés y archivos de macOS
└── simulation/
    ├── models/            modelos MJCF (XML de MuJoCo)
    │   ├── falling_box.xml        etapa 1: cubo con freejoint
    │   ├── single_joint.xml       etapa 2: un segmento + motor
    │   ├── leg_2dof.xml           etapas 4-7: pierna 2 GDL colgando
    │   ├── leg_2dof_foot.xml      etapa 8: pierna 2 GDL con pie, pelvis en vertical
    │   └── leg_6dof.xml           etapa 9: pierna de 6 GDL con pie plano
    └── scripts/
        ├── leg_control.py         utilidades compartidas: compensadores, perfil, sensores
        ├── view_falling_box.py    ver caer el cubo
        ├── control_single_joint.py  mover una articulación
        ├── analyze_leg_2dof.py    análisis de esfuerzos + compensación de gravedad
        ├── control_leg_2dof.py    ver la pierna colgando (suave + gravedad)
        ├── squat_leg_2dof.py      sentadilla con el pie apoyado
        └── analyze_leg_6dof.py    pierna de 6 GDL: aire y apoyo
```

**Nombres.** Los modelos se llaman por lo que contienen (`leg_2dof_foot` =
pierna de 2 GDL con pie). Los scripts, verbo + modelo (`analyze_leg_2dof`,
`squat_leg_2dof`). Lo que usan varios scripts está en `leg_control.py`.

Todos los scripts localizan los modelos con rutas relativas al propio script,
así que se pueden lanzar desde cualquier carpeta.

---

## 4. Convenciones

Estas reglas valen para todos los modelos nuevos. Las etapas 1 y 2 son
anteriores y no siempre las cumplen (ver [Lecciones aprendidas](#8-lecciones-aprendidas)).

**Unidades SI en todo.** Metros, kilogramos, segundos, radianes, newtons y N·m.
Los modelos declaran `<compiler angle="radian"/>`. Los grados solo aparecen en
el código Python para que sea legible, y se convierten con `np.radians` justo
antes de mandarlos al motor.

**Ejes como en ROS (REP-103).** `x` hacia delante, `y` a la izquierda,
`z` hacia arriba.

**Signos de las articulaciones de la pierna.** Ambas giran alrededor de `+y`.
Una rotación positiva lleva el pie hacia `-x` (atrás):

- Cadera positiva = **extensión** (pierna hacia atrás).
- Rodilla positiva = **flexión**. La rodilla va de 0° (recta) a 130°.

En la pierna de 6 GDL (etapa 9), con la regla de la mano derecha y el resto
de articulaciones a cero:

| Articulación | Eje | Positivo significa |
|---|---|---|
| `hip_yaw` | z | la punta del pie gira a la izquierda (+y) |
| `hip_roll` | x | el pie se va a la izquierda (+y) |
| `hip_pitch` | y | extensión: el pie se va atrás (−x) |
| `knee` | y | flexión: el pie se va atrás (−x) |
| `ankle_pitch` | y | la punta del pie baja (flexión plantar) |
| `ankle_roll` | x | el borde izquierdo (+y) del pie sube |

`analyze_leg_6dof.py` comprueba cada fila de esta tabla en simulación.

**Paso de integración.** `timestep = 0.002` s (500 Hz) en todos los modelos.
Desde la etapa 9, integrador `implicitfast` (ver
[Lecciones aprendidas](#8-lecciones-aprendidas)).

**Nombres.** `<parte>_joint`, `<parte>_motor`, y `<parte>_pos` / `_vel` /
`_torque` para los sensores. Los scripts buscan los elementos por nombre, nunca
por índice.

**Tiempo de simulación, no de reloj.** La lógica de control (cuándo cambiar de
objetivo, cuándo imprimir) usa `data.time`. El reloj del ordenador solo sirve
para dormir y que el visor vaya a velocidad real.

---

## 5. Recorrido por etapas

Cada etapa añade una sola idea nueva. Para aprender, lo mejor es recorrerlas en
orden.

### Etapa 1 — Un cuerpo que cae

```bash
mjpython simulation/scripts/view_falling_box.py
```

`falling_box.xml`: un suelo, una luz y una caja de 1 kg con `<freejoint/>`
(6 grados de libertad, sin restricciones) a 1 m de altura. Cae y choca con el
suelo.

**Qué se aprende:** la estructura de un MJCF (`option`, `worldbody`, `body`,
`geom`), el bucle `mj_step` + `viewer.sync()`, y cómo sincronizar con el tiempo
real.

### Etapa 2 — Una articulación con motor

```bash
mjpython simulation/scripts/control_single_joint.py
```

`single_joint.xml`: un soporte fijo con un segmento colgado de una bisagra
(`hinge`) sobre el eje `y`, límites de ±90° y un actuador de posición
(`<position kp="5" kv="5">`). El script recorre los objetivos
0°, 30°, 60°, 0°, −45°, −90° cada 2 s.

**Qué se aprende:** un actuador `position` es un controlador PD dentro de
MuJoCo: `τ = kp·(objetivo − q) − kv·q̇`. Se manda el objetivo en
`data.ctrl` y se lee el ángulo de `data.qpos`.

**Ojo:** con `kp = 5` el motor es demasiado débil contra la gravedad y el
segmento se queda corto: de 21,5° a 43,8° cuando se le pide 30° a 60°, y −67,9°
cuando se le pide −90°. Sirve para ver el **error en régimen permanente**
(sección 6).

### Etapa 3 — Primera pierna de 2 GDL

Primera pierna: pelvis fija, muslo y tibia de 0,25 m, cadera y rodilla con
motores `kp = 100`, `kv = 10`, y objetivos que **saltan de golpe** cada 3 s.
La etapa 4 la sustituyó y su archivo se eliminó. Los saltos se pueden reproducir
con el modo `step` de `analyze_leg_2dof.py`.

**Qué se aprende:** cadenas cinemáticas (un `body` dentro de otro), `<default>`
para no repetir atributos, y que un salto brusco del objetivo pide un torque
enorme (`kp × error`). Esa observación llevó a la etapa 4.

### Etapa 4 — Pierna de 2 GDL

Ver la [sección 6](#6-la-pierna-de-2-gdl-en-detalle).

### Etapa 5 — Análisis de esfuerzos

```bash
python simulation/scripts/analyze_leg_2dof.py           # informe en la terminal
mjpython simulation/scripts/analyze_leg_2dof.py --view  # además, ver suave + gravedad
```

Ver la [sección 7](#7-análisis-de-esfuerzos-y-resultados). El script devuelve
el código de salida `0` si pasa los criterios y `1` si no, así que se puede
usar como test automático.

### Etapa 6 — Control con trayectorias suaves

```bash
mjpython simulation/scripts/control_leg_2dof.py
```

Mueve la pierna de 2 GDL por la misma secuencia que el análisis, con el mismo perfil
coseno y con compensación de gravedad (etapa 7). Cada 0,5 s imprime ángulo,
objetivo y torque de cada articulación. Usa la secuencia de `analyze_leg_2dof.py` y los tiempos, `interpolate()` y
`GravityCompensator` de `leg_control.py`, así que lo que se ve en el visor es
exactamente lo que se ha analizado.

La pierna espera quieta 3 s al arrancar, porque la secuencia empieza en
(0°, 0°).

### Etapa 7 — Compensación de gravedad

Clase `GravityCompensator` en `leg_control.py`. Elimina el error permanente
que deja la gravedad (sección 6) sin tocar `kp`.

**Idea.** El motor de posición da `τ = kp·(ctrl − q)`. Para sostener la pierna
necesita `τ = τ_g(q)`. Si en vez del objetivo se le manda

```
ctrl = objetivo + τ_g(q) / kp
```

da justo `τ_g` cuando `q = objetivo`, es decir, con error cero. Es un
*feedforward*: el controlador se adelanta a la gravedad en lugar de esperar a
que aparezca el error.

**Cómo se calcula `τ_g`.** Con el propio modelo: se copia la postura medida
(lo que daría el encoder) en un `MjData` auxiliar, se pone la velocidad a cero
y se llama a `mj_forward`. Entonces `qfrc_bias` contiene solo la gravedad, sin
Coriolis ni centrífuga. `kp` se lee del modelo (`actuator_gainprm[:, 0]`), no
está repetido en el código.

```python
compensator = GravityCompensator(model, actuator_ids, dof_ids)
data.ctrl[actuator_ids] = objetivo + compensator.offset(data.qpos)
```

**Resultado.** El error final baja de hasta 1,80° a ~3·10⁻⁹°, y el torque pico
apenas cambia (de 5,16 a 5,31 N·m). Ver la sección 7.

**Ojo, el modelo nunca es perfecto.** Aquí el error es prácticamente cero
porque el compensador usa *el mismo* modelo que simula la pierna. Si se le da
un modelo con las masas un 10 % mal (más o menos), el error máximo queda en
**0,18°**: la compensación quita el 90 % del error, pero el 10 % de error del
modelo se queda. En un robot real, la calidad del modelo (masas, centros de
masa) limita lo bien que funciona.

### Etapa 8 — Pie, contacto y sentadilla

```bash
python simulation/scripts/squat_leg_2dof.py           # informe en la terminal
mjpython simulation/scripts/squat_leg_2dof.py --view  # además, verlo
```

Primera vez que la pierna **aguanta peso**. Modelo `leg_2dof_foot.xml`:

- La pelvis ya no está fija: se desliza **solo en vertical** (`slide` en `z`).
  No puede avanzar ni girar, así que todavía no hay que mantener el equilibrio.
- **Pie puntual:** una esfera de 4 cm y 0,2 kg al final de la tibia.
- **Tobillo pasivo** (sin motor) entre tibia y esfera. Más abajo se explica
  por qué hace falta.
- Solo chocan pie y suelo (`contype` / `conaffinity`), para que nada más
  interfiera.
- Sensores nuevos: altura de la pelvis (`pelvis_height`) y fuerza normal en
  el pie (`foot_force`, un sensor `touch`).

Peso total que soporta el pie: 4 kg → **39,24 N**.

**La sentadilla.** Para que el pie quede justo debajo de la cadera (muslo y
tibia miden lo mismo), `cadera = −rodilla / 2`. La altura de la pelvis es
entonces

```
z_pelvis = r_pie + (L_muslo + L_tibia) · cos(rodilla / 2) + 0,05
```

La secuencia de rodilla es 20° → 60° → 90° → 120° → 20°, con el perfil coseno
y 1,5 s de transición + 1,5 s quieto, como en la etapa 5.

**Criterios de aceptación**, fijados *antes* de la primera ejecución:

1. Ninguna saturación de torque.
2. Fuerza en el pie = peso total ± 5 %.
3. El pie no se desplaza más de 5 mm.
4. Altura de la pelvis = teórica ± 1 cm.

**Por qué no vale la compensación de la etapa 7.** `GravityCompensator`
supone la pierna colgando: la rodilla solo sostiene la tibia. Apoyada, el
suelo empuja el pie hacia arriba con *todo* el peso, y la rodilla tiene que
llevar ese empuje hasta la pelvis. El torque cambia de signo y se multiplica.

**Compensación de apoyo (`StanceCompensator`).** En equilibrio estático:

```
τ = τ_g(q) − J_pieᵀ · F_suelo,     F_suelo = (0, 0, peso)
```

`J_pie` es el **jacobiano** de la posición del pie (`mj_jacBody`): traduce una
fuerza en el pie en torques en cada articulación. Se pide `F_suelo` vertical,
sin componente horizontal, para que el pie no empuje de lado. Igual que en la
etapa 7, el objetivo se adelanta `τ / kp`.

Comprobación a mano en 120° de rodilla, aislando tibia + pie: el suelo empuja
39,24 N a 0,2165 m (`L·sin 60°`) de la rodilla; el peso de la tibia (0,8 kg a
0,108 m) y del pie (0,2 kg a 0,2165 m) restan.
`39,24·0,2165 − 7,85·0,108 − 1,96·0,2165 ≈` **7,2 N·m**, igual que el sensor.

**Lo que pasó por el camino** (ver resultados en la sección 7):

1. *Sin compensación* la rodilla cede 4,2° en 120° (`7,3 / 100` rad), la
   pelvis queda 1,7 cm baja y el pie se desplaza hasta 28 mm. FAIL.
2. *Con compensación de apoyo* la altura sale bien (0,2 mm), pero el pie sigue
   moviéndose 5–13 mm. FAIL.
3. **El pie no patinaba, rodaba.** La fuerza horizontal nunca pasó del 30 % de
   la normal (μ = 1), así que no era deslizamiento. La esfera iba soldada a
   la tibia: al inclinarse la tibia la esfera tiene que girar, y como la
   pelvis fija el centro de la esfera, o gira contra el suelo o rueda y
   arrastra las articulaciones. Rodadura esperada: `r·Δθ = 0,04 m × 50° ≈
   35 mm` al volver de 120° a 20°; se midieron 19,5 mm.
4. *Con tobillo pasivo* la esfera gira libre y el contacto actúa como un
   pasador. Error articular 0,00°, desplazamiento 0,00 mm. **PASS**.

Quedan 0,7 mm de diferencia de altura: el pie se hunde 0,72 mm en el suelo,
porque MuJoCo usa **contacto blando**. Es normal y está dentro del criterio.

### Etapa 9 — Pierna de 6 GDL con pie plano

```bash
python simulation/scripts/analyze_leg_6dof.py           # informe en la terminal
mjpython simulation/scripts/analyze_leg_6dof.py --view  # además, verlo (~70 s)
```

La pierna de un humanoide real. Modelo `leg_6dof.xml`:

- **Cadera de 3 ejes:** yaw (z), roll (x) y pitch (y), en ese orden. Los tres
  ejes se cruzan en un punto: son tres `joint` en el mismo `body`.
  Simplificación habitual: en un robot real cada motor va en su eslabón.
- **Rodilla** (pitch).
- **Tobillo de 2 ejes con motor:** pitch (y) y roll (x).
- **Pie plano:** caja de 20 × 10 × 3 cm, 0,3 kg, con la planta 4 cm bajo el
  tobillo y centrada bajo él.
- Pelvis en vertical como en la etapa 8, más una **restricción de igualdad**
  `hold_pelvis`, desactivada por defecto. El script la activa
  (`data.eq_active`) para sujetar la pelvis a 0,8 m y mover la pierna en el aire.
- Seis motores de posición, `kp = 100`, límite ±20 N·m.

Peso total sobre el pie: 4,1 kg → **40,22 N**.

**Ganancias a partir de la inercia real.** En lugar de calcular las inercias a
mano, se miden con MuJoCo: `mj_fullM` da la matriz de inercia articular, y su
diagonal es la inercia que "ve" cada motor (pierna recta). Con
`kv = 2 · 0,8 · √(kp · I)`:

| Eje | I [kg·m²] | kv |
|---|---|---|
| hip_yaw | 0,0023 | 0,8 |
| hip_roll | 0,2249 | 7,6 |
| hip_pitch | 0,2257 | 7,6 |
| knee | 0,0424 | 3,3 |
| ankle_pitch | 0,0012 | 0,6 |
| ankle_roll | 0,0005 | 0,3 |

La cadera en roll y pitch mueve toda la pierna (inercia grande). En yaw, la
pierna solo gira sobre su eje (inercia de un cilindro fino).

**Prueba 1, en el aire.** Pelvis sujeta y `GravityCompensator`. Cada
articulación va sola a +a, a −a y vuelve a 0 (rodilla: 60°, 120°, 0°).
En el primer objetivo se comprueba que el pie se mueve en el sentido de la
tabla de [Convenciones](#4-convenciones). Ese es el error más típico al montar
un robot: un eje al revés.

**Prueba 2, en apoyo.** Se suelta la pelvis y se hace la sentadilla de la
etapa 8 con `StanceCompensator`, que no hubo que cambiar: con el pie centrado
bajo el tobillo, la fuerza vertical pasa por el tobillo. Para que el pie quede
plano y bajo la cadera:

```
hip_pitch = −rodilla / 2,    ankle_pitch = −rodilla / 2
```

**Criterios de aceptación**, fijados antes de la primera ejecución:

1. Ninguna saturación de torque, en ninguna de las dos pruebas.
2. Aire: error final < 0,2° en todas las articulaciones, y sentido correcto en
   los 6 ejes.
3. Apoyo: los cuatro criterios de la etapa 8, más el **pie plano** (inclinación
   < 1°).

**Lo que pasó por el camino:**

1. *Primera ejecución:* los seis sentidos eran correctos y el apoyo estaba bien.
   Pero hip_roll estaba saturado el 6 % del tiempo y hip_pitch el 1,8 %, en
   movimientos donde esos ejes debían estar quietos. FAIL.
2. *Era inestabilidad numérica.* hip_roll alternaba +20 / −20 N·m en cada paso
   mientras debía sostener 0°. Con la cadera a 45° de pitch, el eje largo de la
   pierna queda en el plano de los ejes yaw y roll. Una combinación yaw + roll
   hace girar la pierna sobre sí misma, un modo con inercia diminuta
   (~0,002 kg·m²). El `kv = 7,6` de roll, pensado para una inercia de 0,22,
   aplicado a ese modo pasa el límite de estabilidad de Euler (`kv·dt/I ≈ 2`).
   Con roll a −25° pasa lo mismo con pitch + yaw.
3. *Solución:* `integrator="implicitfast"`, que integra el amortiguamiento de
   los motores de forma implícita y es estable para cualquier `kv`. Bajar `kv`
   habría quitado amortiguamiento. Saturación 0 %.
4. *Quedaba* un error de 0,72° en la rodilla con la cadera a −45°: la
   compensación pedía un objetivo de −0,72° y `ctrlrange="0 …"` lo recortaba a
   0. La rodilla trabaja justo en su tope mecánico (0°, recta), así que su
   mando necesita margen. Se amplió a −5°; el tope (`range`) sigue en 0°.
   **PASS**.

---

## 6. La pierna de 2 GDL en detalle

Archivo: `simulation/models/leg_2dof.xml`.

### Geometría y masas

| Cuerpo | Forma | Longitud | Masa |
|---|---|---|---|
| pelvis (fija a 1 m) | caja 0,24 × 0,16 × 0,10 m | — | 2 kg |
| muslo | cápsula r = 0,035 m | 0,25 m | 1 kg |
| tibia | cápsula r = 0,035 m | 0,25 m | 0,8 kg |

La pelvis **no tiene `freejoint`**: está anclada al mundo. Todavía no hay pie
ni contacto con el suelo.

### Articulaciones

| Nombre | Eje | Rango | Amortiguamiento |
|---|---|---|---|
| `hip_joint` | `y` | −90° … 90° (±1,5708 rad) | 0,5 N·m·s/rad |
| `knee_joint` | `y` | 0° … 130° (0 … 2,2689 rad) | 0,5 N·m·s/rad |

### Actuadores

| Nombre | kp [N·m/rad] | kv [N·m·s/rad] | Límite de torque |
|---|---|---|---|
| `hip_motor` | 100 | 6 | ±20 N·m |
| `knee_motor` | 100 | 2 | ±20 N·m |

`forcerange` simula que un motor real **satura**: no puede dar más torque del
que tiene. Los 20 N·m son provisionales y sirven para detectar saturación. No
son todavía un requisito de motor.

### Cómo se eligieron las ganancias

Cada articulación con su motor de posición se comporta como un oscilador
masa-muelle-amortiguador, con inercia `I`, rigidez `kp` y amortiguamiento `kv`.
Para un amortiguamiento relativo `ζ`:

```
kv = 2 · ζ · √(kp · I)
```

Con `ζ ≈ 0,8` (casi crítico: llega rápido y sin oscilar) y `kp = 100`:

| Articulación | Inercia I (aprox.) | kv |
|---|---|---|
| Cadera (pierna extendida) | 0,14 kg·m² | 2 · 0,8 · √(100 · 0,14) ≈ **6** |
| Rodilla | 0,018 kg·m² | 2 · 0,8 · √(100 · 0,018) ≈ **2** |

La inercia de la cadera se calcula para todo lo que cuelga de ella: el muslo
como varilla girando por su extremo (`m·L²/3 = 0,021`) más la tibia trasladada
(`m·(L²/12 + d²) = 0,117`, con `d = 0,375` m), en total ≈ 0,14 kg·m².

En la primera pierna se usaba `kv = 10` en las dos articulaciones: la rodilla quedaba
sobreamortiguada y lenta.

### Error en régimen permanente

Un motor de posición puro solo da torque si hay error. Para sostener la pierna
contra la gravedad tiene que quedarse un poco por debajo del objetivo:

```
error ≈ τ_gravedad / kp
```

Ejemplo: en (45°, 60°) la cadera soporta 3,14 N·m → 3,14 / 100 = 0,031 rad
≈ **1,8°**, que es justo lo que mide el análisis. Se elimina con compensación
de gravedad (etapa 7).

### Sensores

Lo que tendría el controlador de un robot real:

| Sensor | Tipo | Unidad |
|---|---|---|
| `hip_pos`, `knee_pos` | `jointpos` (encoder) | rad |
| `hip_vel`, `knee_vel` | `jointvel` | rad/s |
| `hip_torque`, `knee_torque` | `actuatorfrc` (torque del motor) | N·m |

Se leen con `data.sensor("hip_pos").data[0]`.

---

## 7. Análisis de esfuerzos y resultados

`analyze_leg_2dof.py` ejecuta la secuencia (cadera, rodilla) en grados

```
(0, 0) → (30, 45) → (-20, 90) → (45, 60) → (0, 120) → (0, 0)
```

en tres modos:

- **step**: el objetivo salta de golpe al siguiente valor.
- **smooth**: el objetivo pasa de uno a otro en 1,5 s con un **perfil coseno**,
  `blend = ½·(1 − cos(π·t/T))`. Tiene velocidad cero al empezar y al acabar,
  así que no hay tirón.
- **smooth + gravedad**: como smooth, con compensación de gravedad (etapa 7).

En todos los modos, después de cada transición la pierna se queda 1,5 s quieta en
el objetivo antes de medir.

### Qué se mide en cada objetivo

- **τ pico**: torque máximo del motor durante transición y sostén.
- **τ sostén**: torque del motor al final del sostén.
- **τ gravedad**: torque que la gravedad ejerce en esa postura (`data.qfrc_bias`).
- **Error final**: diferencia entre objetivo y posición medida.
- **% saturado**: fracción del tiempo con el motor en su límite.

### Criterios de aceptación (modos smooth)

1. Ninguna saturación de torque.
2. Error final ≤ 2° en todas las articulaciones y objetivos (≤ 0,2° con
   compensación de gravedad).
3. Torque de sostén = torque de gravedad, con tolerancia de 0,05 N·m + 5 %.
   Comprueba que la pierna está quieta de verdad y que el modelo es coherente.

### Resultados (MuJoCo 3.17.0)

**Modo step**: cada cambio de objetivo satura ambos motores (20 N·m). La cadera
está saturada el 2,0 % del tiempo y la rodilla el 0,8 %.

**Modo smooth**: **PASS**.

| Objetivo (cad, rod) | τ pico [N·m] | τ sostén [N·m] | Error final [°] |
|---|---|---|---|
| (30, 45) | 3,13 · 1,11 | 2,46 · 0,94 | 1,41 · 0,54 |
| (−20, 90) | 2,46 · 1,37 | −0,17 · 0,92 | 0,10 · 0,53 |
| (45, 60) | 5,16 · 0,96 | 3,14 · 0,96 | 1,80 · 0,55 |
| (0, 120) | 3,14 · 1,46 | 0,83 · 0,86 | 0,48 · 0,49 |
| (0, 0) | 0,83 · 0,86 | 0,00 · 0,00 | 0,00 · 0,00 |

Pico global: cadera 5,16 N·m, rodilla 1,46 N·m. Una trayectoria suave reduce el
torque pico **al menos 4 veces** respecto al salto (en modo step el motor se
queda en el límite de 20 N·m, así que pediría todavía más), sin cambiar el motor.

**Modo smooth + gravedad**: **PASS**, error final < 0,2°.

| Objetivo (cad, rod) | τ pico [N·m] | τ sostén [N·m] | Error final [°] |
|---|---|---|---|
| (30, 45) | 3,25 · 1,13 | 2,54 · 0,95 | 0,00 · 0,00 |
| (−20, 90) | 2,54 · 1,37 | −0,17 · 0,92 | 0,00 · 0,00 |
| (45, 60) | 5,31 · 0,95 | 3,20 · 0,95 | 0,00 · 0,00 |
| (0, 120) | 3,20 · 1,45 | 0,85 · 0,85 | 0,00 · 0,00 |
| (0, 0) | 0,87 · 0,85 | 0,00 · 0,00 | 0,00 · 0,00 |

El torque de sostén sube un poco respecto al modo smooth (2,54 frente a
2,46 N·m en (30, 45)): ahora la pierna está exactamente en 30° y no en 28,6°,
así que la gravedad tira un poco más.

### Etapa 8: sentadilla (`squat_leg_2dof.py`)

Valores al final de cada sostén. Desplazamiento = cuánto se ha movido el pie
en `x` desde el inicio.

| Rodilla | Sin compensación | Gravedad (aire) | **Apoyo** |
|---|---|---|---|
| | error rod. · desplaz. · altura | error rod. · desplaz. · altura | error rod. · desplaz. · altura |
| 60° | 2,64° · 9,9 mm · −6,9 mm | 3,09° · 8,0 mm · −7,8 mm | **0,00° · 0,0 mm · −0,7 mm** |
| 90° | 3,65° · 11,2 mm · −12,7 mm | 4,28° · 8,8 mm · −14,5 mm | **0,00° · 0,0 mm · −0,8 mm** |
| 120° | 4,35° · 7,4 mm · −17,8 mm | 5,11° · 5,5 mm · −20,5 mm | **0,00° · 0,0 mm · −0,7 mm** |
| 20° | 0,93° · 0,0 mm · −1,5 mm | 1,09° · 0,0 mm · −1,6 mm | **0,00° · 0,0 mm · −0,7 mm** |

(Con el tobillo pasivo ya puesto.) La compensación del aire hace la rodilla
**peor** que no compensar: empuja en el sentido contrario al que hace falta.

Torques con compensación de apoyo:

| Rodilla | τ pico cad · rod [N·m] | τ sostén cad · rod [N·m] |
|---|---|---|
| 60° | 1,28 · 4,26 | −1,10 · −4,17 |
| 90° | 1,69 · 6,02 | −1,56 · −5,90 |
| 120° | 2,05 · 7,39 | −1,91 · −7,22 |
| 20° | 2,00 · 8,11 | −0,38 · −1,45 |

El pico de rodilla (8,1 N·m) aparece al **subir** de 120° a 20°: además de
sostener el peso hay que acelerar la pelvis hacia arriba. Torque de rodilla
negativo = el motor empuja hacia la extensión (estirar la pierna).

### Etapa 9: pierna de 6 GDL (`analyze_leg_6dof.py`)

**Aire** (`GravityCompensator`): error final 0,000° en todos los objetivos
salvo ankle_pitch a −45° (0,029°). Los 6 sentidos son correctos. Torque pico
de la articulación que se mueve:

| Articulación | Objetivos | τ pico [N·m] |
|---|---|---|
| hip_yaw | ±30° | 0,55 |
| hip_roll | ±25° | 2,42 |
| hip_pitch | ±45° | 4,05 |
| knee | 60°, 120° | 2,34 |
| ankle_pitch | +30°, −45° | 0,70 |
| ankle_roll | ±25° | 0,46 |

**Apoyo** (`StanceCompensator`, pie plano):

| Rodilla | Altura pelvis medida / teórica [m] | τ pico cad. pitch · rod. · tob. pitch [N·m] | Error [°] | Desplaz. pie [mm] | Inclin. pie [°] |
|---|---|---|---|---|---|
| 60° | 0,5226 / 0,5230 | 1,51 · 4,20 · 1,02 | 0,105 | 0,80 | 0,008 |
| 90° | 0,4431 / 0,4436 | 2,01 · 5,96 · 0,57 | 0,082 | 0,51 | 0,007 |
| 120° | 0,3396 / 0,3400 | 2,36 · 7,30 · 0,53 | 0,031 | 0,14 | 0,003 |
| 20° | 0,5820 / 0,5824 | 3,48 · 7,89 · 2,00 | 0,380 | 3,32 | 0,030 |

Fuerza en el pie = 40,22 N en todos los objetivos. Ninguna saturación.

Al volver a 20° el pie se desplaza 3,3 mm: pasa el criterio (5 mm), pero es lo
más justo de la prueba. Subir rápido desde 120° pide aceleración, y los
compensadores solo cubren la parte estática. Ver [Hoja de ruta](#9-hoja-de-ruta).

---

## 8. Lecciones aprendidas

**Unidades mezcladas en el mismo archivo.** Con `<compiler angle="degree"/>`,
MuJoCo convierte a radianes `range` de las articulaciones, pero **no**
`ctrlrange` de los actuadores, que siempre está en radianes. En `single_joint.xml`
(y en la primera pierna) funciona, pero el mismo archivo tiene `range="-90 90"` y
`ctrlrange="-1.5708 1.5708"`, y es fácil equivocarse. Desde la etapa 4 todo va en
radianes.

**Sin límite de torque, la simulación miente.** Sin `forcerange`, un salto de
objetivo da torques arbitrariamente grandes y todo parece funcionar. Un motor
real no puede hacerlo.

**El perfil de la trayectoria importa tanto como el motor.** Pasar de saltos a
perfil coseno bajó el pico de 20 N·m (saturado) a 5,2 N·m.

**kp bajo = el brazo se cae.** En `single_joint.xml` con `kp = 5`, la gravedad
gana: el segmento se queda hasta 22° por debajo del objetivo.

**Imports dentro de funciones en Python.** Un `import mujoco.viewer` dentro de
`main()` convierte `mujoco` en variable local de toda la función, y el primer
uso de `mujoco` falla con `UnboundLocalError`. Los imports van arriba del todo.

**Más kp no es la única forma de ganar precisión.** Subir `kp` reduce el error
de gravedad, pero hace el motor más rígido y pide más torque en cada
perturbación. La compensación de gravedad quita el error sin tocar `kp`, a
cambio de depender de lo bueno que sea el modelo.

**Una compensación vale para un tipo de contacto.** La de gravedad de la
etapa 7 supone la pierna en el aire; apoyada, empeora el error. El controlador
tiene que saber si el pie está en el suelo.

**Mirar los números antes de cambiar nada.** El pie "deslizaba", pero la
fuerza horizontal era el 30 % de la que hace falta para deslizar. El problema
era la geometría (la esfera soldada a la tibia), no la fricción. Subir la
fricción no lo habría arreglado.

**Fijar los criterios antes de ejecutar.** En la etapa 8 los criterios se
escribieron antes de la primera ejecución y no se relajaron cuando fallaron:
eso obligó a encontrar la causa real.

**Las ganancias de cada eje por separado no bastan.** El `kv` se calculó con
la inercia de cada eje, pero los ejes se acoplan: en ciertas posturas aparece
un modo de inercia mucho menor que cualquier diagonal. Con un integrador
explícito, eso basta para que la simulación se vuelva inestable. Integrador
`implicitfast` desde la etapa 9.

**`range` no es `ctrlrange`.** `range` es el tope mecánico de la articulación;
`ctrlrange` es hasta dónde puede llegar el mando. Una articulación que trabaja
en su tope (la rodilla recta) necesita margen en el mando para que la
compensación no se recorte.

**Comprobar los sentidos de los ejes en simulación.** Un eje al revés es el
error más común al montar un robot, y en el modelo es igual de fácil
equivocarse. La etapa 9 lo comprueba en cada ejecución.

**Medir con el tiempo de simulación.** Si la lógica usa el reloj del ordenador,
el movimiento cambia según la carga de la máquina. Con `data.time` es
reproducible.

---

## 9. Hoja de ruta

Hecho: compensación de gravedad (etapa 7), pie y contacto (etapa 8), pierna
de 6 GDL (etapa 9).

Siguientes pasos propuestos, en orden. Cada uno debería tener su modelo o
script y un criterio de aceptación comprobable, como en la etapa 5.

1. **Dos piernas y `freejoint`.** La pelvis queda libre: hay que mantener el
   equilibrio de pie. Se puede empezar con un compensador dinámico (que incluya
   aceleraciones, no solo estática) para reducir el desplazamiento del pie al
   subir rápido.
2. **Dimensiones reales.** Sustituir medidas y masas didácticas por las del
   robot de referencia y convertir los torques medidos en requisitos de motor.
3. **Caminar.** Generador de pasos y control de equilibrio (ZMP o similar).

---

## 10. Glosario

- **GDL**: grados de libertad. Cada ángulo independiente que puede moverse.
- **MJCF**: formato XML de MuJoCo para describir modelos.
- **Actuador de posición**: motor que recibe un ángulo objetivo y aplica
  `τ = kp·(objetivo − q) − kv·q̇` (un PD).
- **kp / kv**: rigidez y amortiguamiento del controlador PD.
- **ζ (zeta)**: amortiguamiento relativo. < 1 oscila, = 1 es crítico, > 1 es lento.
- **Saturación**: el motor pide más torque del que puede dar y se queda en el límite.
- **Jacobiano (del pie)**: matriz que relaciona velocidades articulares con la
  velocidad del pie. Su traspuesta convierte una fuerza en el pie en torques
  articulares.
- **Contacto blando**: MuJoCo deja que los cuerpos se hundan un poco (aquí
  0,7 mm) en vez de imponer un contacto perfectamente rígido.
- **Integrador implícito (`implicitfast`)**: calcula el amortiguamiento con la
  velocidad del final del paso, no la del principio. Es estable aunque el
  amortiguamiento sea grande comparado con la inercia.
- **`range` / `ctrlrange`**: tope mecánico de la articulación / límite del mando
  que se manda al motor.
- **Restricción de igualdad**: condición que MuJoCo impone entre elementos del
  modelo (aquí, fijar la altura de la pelvis). Se activa y desactiva con
  `data.eq_active`.
- **Feedforward**: parte del control que se calcula de antemano con el modelo
  (aquí, el torque de gravedad), en lugar de reaccionar al error medido.
- **qfrc_bias**: fuerzas de gravedad, Coriolis y centrífugas que calcula MuJoCo
  en cada paso. Quieta, la pierna solo nota la gravedad.
- **REP-103**: convención de ejes y unidades de ROS.
