# Danny — Handbook técnico

Detalle de cada experimento: qué se hizo, por qué, qué falló y qué se aprendió.
Para empezar, lee el [README](../README.md). El plan está en el
[ROADMAP](../ROADMAP.md).

> **Importante:** las dimensiones, masas y límites de los modelos son
> **didácticos**. No son los definitivos de Danny, y no deben usarse para
> dimensionar motores reales.

## Índice

1. [Convenciones](#1-convenciones)
2. [Recorrido por experimentos](#2-recorrido-por-experimentos)
3. [La pierna de 2 GDL en detalle](#3-la-pierna-de-2-gdl-en-detalle)
4. [Análisis de esfuerzos y resultados](#4-análisis-de-esfuerzos-y-resultados)
5. [Lecciones aprendidas](#5-lecciones-aprendidas)
6. [Glosario](#6-glosario)

---

## 1. Convenciones

Estas reglas valen para todos los modelos nuevos. Los experimentos 1 y 2 son
anteriores y no siempre las cumplen (ver [Lecciones aprendidas](#5-lecciones-aprendidas)).

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

En la pierna de 6 GDL (experimento 9), con la regla de la mano derecha y el resto
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

En el bípedo (experimento 10) las dos piernas tienen **los mismos ejes y signos**
(no son espejo): `hip_roll` + lleva cualquiera de los dos pies hacia +y. Los
nombres llevan el lado delante: `left_knee_joint`, `right_hip_roll_motor`.

**Paso de integración.** `timestep = 0.002` s (500 Hz) en todos los modelos.
Desde el experimento 9, integrador `implicitfast` (ver
[Lecciones aprendidas](#5-lecciones-aprendidas)).

**Nombres.** `<parte>_joint`, `<parte>_motor`, y `<parte>_pos` / `_vel` /
`_torque` para los sensores. Los scripts buscan los elementos por nombre, nunca
por índice.

**Tiempo de simulación, no de reloj.** La lógica de control (cuándo cambiar de
objetivo, cuándo imprimir) usa `data.time`. El reloj del ordenador solo sirve
para dormir y que el visor vaya a velocidad real.

---

## 2. Recorrido por experimentos

Cada experimento añade una sola idea nueva. Para aprender, lo mejor es recorrerlas en
orden.

### Experimento 1 — Un cuerpo que cae

```bash
mjpython simulation/scripts/view_falling_box.py
```

`falling_box.xml`: un suelo, una luz y una caja de 1 kg con `<freejoint/>`
(6 grados de libertad, sin restricciones) a 1 m de altura. Cae y choca con el
suelo.

**Qué se aprende:** la estructura de un MJCF (`option`, `worldbody`, `body`,
`geom`), el bucle `mj_step` + `viewer.sync()`, y cómo sincronizar con el tiempo
real.

### Experimento 2 — Una articulación con motor

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
(sección 3).

### Experimento 3 — Primera pierna de 2 GDL

Primera pierna: pelvis fija, muslo y tibia de 0,25 m, cadera y rodilla con
motores `kp = 100`, `kv = 10`, y objetivos que **saltan de golpe** cada 3 s.
El experimento 4 la sustituyó y su archivo se eliminó. Los saltos se pueden reproducir
con el modo `step` de `analyze_leg_2dof.py`.

**Qué se aprende:** cadenas cinemáticas (un `body` dentro de otro), `<default>`
para no repetir atributos, y que un salto brusco del objetivo pide un torque
enorme (`kp × error`). Esa observación llevó al experimento 4.

### Experimento 4 — Pierna de 2 GDL

Ver la [sección 3](#3-la-pierna-de-2-gdl-en-detalle).

### Experimento 5 — Análisis de esfuerzos

```bash
python simulation/scripts/analyze_leg_2dof.py           # informe en la terminal
mjpython simulation/scripts/analyze_leg_2dof.py --view  # además, ver suave + gravedad
```

Ver la [sección 4](#4-análisis-de-esfuerzos-y-resultados). El script devuelve
el código de salida `0` si pasa los criterios y `1` si no, así que se puede
usar como test automático.

### Experimento 6 — Control con trayectorias suaves

```bash
mjpython simulation/scripts/control_leg_2dof.py
```

Mueve la pierna de 2 GDL por la misma secuencia que el análisis, con el mismo perfil
coseno y con compensación de gravedad (experimento 7). Cada 0,5 s imprime ángulo,
objetivo y torque de cada articulación. Usa la secuencia de `analyze_leg_2dof.py` y los tiempos, `interpolate()` y
`GravityCompensator` de `leg_control.py`, así que lo que se ve en el visor es
exactamente lo que se ha analizado.

La pierna espera quieta 3 s al arrancar, porque la secuencia empieza en
(0°, 0°).

### Experimento 7 — Compensación de gravedad

Clase `GravityCompensator` en `leg_control.py`. Elimina el error permanente
que deja la gravedad (sección 3) sin tocar `kp`.

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
apenas cambia (de 5,16 a 5,31 N·m). Ver la sección 4.

**Ojo, el modelo nunca es perfecto.** Aquí el error es prácticamente cero
porque el compensador usa *el mismo* modelo que simula la pierna. Si se le da
un modelo con las masas un 10 % mal (más o menos), el error máximo queda en
**0,18°**: la compensación quita el 90 % del error, pero el 10 % de error del
modelo se queda. En un robot real, la calidad del modelo (masas, centros de
masa) limita lo bien que funciona.

### Experimento 8 — Pie, contacto y sentadilla

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
y 1,5 s de transición + 1,5 s quieto, como en el experimento 5.

**Criterios de aceptación**, fijados *antes* de la primera ejecución:

1. Ninguna saturación de torque.
2. Fuerza en el pie = peso total ± 5 %.
3. El pie no se desplaza más de 5 mm.
4. Altura de la pelvis = teórica ± 1 cm.

**Por qué no vale la compensación del experimento 7.** `GravityCompensator`
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
experimento 7, el objetivo se adelanta `τ / kp`.

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

### Experimento 9 — Pierna de 6 GDL con pie plano

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
- Pelvis en vertical como en el experimento 8, más una **restricción de igualdad**
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
tabla de [Convenciones](#1-convenciones). Ese es el error más típico al montar
un robot: un eje al revés.

**Prueba 2, en apoyo.** Se suelta la pelvis y se hace la sentadilla de la
experimento 8 con `StanceCompensator`, que no hubo que cambiar: con el pie centrado
bajo el tobillo, la fuerza vertical pasa por el tobillo. Para que el pie quede
plano y bajo la cadera:

```
hip_pitch = −rodilla / 2,    ankle_pitch = −rodilla / 2
```

**Criterios de aceptación**, fijados antes de la primera ejecución:

1. Ninguna saturación de torque, en ninguna de las dos pruebas.
2. Aire: error final < 0,2° en todas las articulaciones, y sentido correcto en
   los 6 ejes.
3. Apoyo: los cuatro criterios del experimento 8, más el **pie plano** (inclinación
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

### Experimento 10 — Bípedo: equilibrio en un pie

```bash
python simulation/scripts/balance_biped.py           # informe en la terminal
mjpython simulation/scripts/balance_biped.py --view  # además, verlo (~20 s)
```

Primera vez que **nada sujeta la pelvis**: tiene `freejoint` y solo la
sostienen las piernas, así que el robot se puede caer. Modelo `biped_6dof.xml`:

- Dos piernas de 6 GDL iguales a la del experimento 9, con las caderas en
  y = ±0,1 m (20 cm entre pies).
- Pelvis de 16 × 30 × 10 cm y 2 kg, con un sensor de orientación y posición
  (`pelvis_quat`, `pelvis_pos`), como la IMU de un robot real.
- Mismas ganancias, límites e integrador que el experimento 9. Peso total: 6,2 kg →
  **60,82 N**.

**Secuencia.**

1. De pie, rodillas a 40°.
2. Sentadilla a 80° y vuelta a 40°.
3. Pasar el peso a la izquierda.
4. Levantar el pie derecho 5 cm y bajarlo.
5. Volver al centro.

Cada paso dura 1,5 s de transición + 1,5 s quieto. De pie se usa 40° de
rodilla, y no 20°, porque al desplazar la pelvis de lado la pierna tiene que
alcanzar más lejos y necesita margen para estirarse.

**Novedad 1: cinemática inversa (`leg_ik` en `leg_control.py`).** Hasta ahora
se daban ángulos. Ahora se da *dónde* tienen que estar la pelvis y los pies, y
`leg_ik` calcula los 6 ángulos de cada pierna, con el pie plano y mirando
adelante:

1. `hip_roll = atan2(y, −z)` inclina el plano de la pierna hasta el tobillo.
2. En ese plano, cadera y rodilla forman un brazo de 2 eslabones: la ley del
   coseno da la rodilla, y la cadera apunta al tobillo menos lo que aporta la
   rodilla.
3. El tobillo deshace los giros: `ankle_pitch = −(hip_pitch + knee)`,
   `ankle_roll = −hip_roll`.

Se comprobó contra la cinemática directa de MuJoCo en 5 posiciones: error
0,000 mm, pie plano, yaw 0°. La secuencia se interpola **en el espacio de
tareas** (posiciones de pelvis y pies) y se resuelve la IK en cada paso. Si se
interpolaran ángulos, el pie de apoyo se arrastraría a mitad de transición.

**Novedad 2: dónde poner el centro de masas.** Para sostenerse en el pie
izquierdo, el centro de masas (CdM) tiene que estar encima de él.
`center_com_over` (en `biped_control.py`) mueve la pelvis, en x e y, lo que le
falta al CdM hasta que queda sobre el centro del pie, ya con el pie derecho
levantado. Converge porque el CdM se mueve algo menos que la pelvis. Resultado:
**15,8 cm** de desplazamiento lateral. Es más que los 10 cm hasta el centro
del pie, porque la pierna levantada (2,1 kg) queda al otro lado.

**Novedad 3: compensación con dos pies.** `StanceCompensator` acepta ahora
varios pies y la fracción de peso de cada uno:

- En doble apoyo, la fracción sale de dónde está el CdM entre los dos pies.
- Con un pie en el aire, todo el peso va al otro.

**Criterios de aceptación**, fijados antes de la primera ejecución:

1. Ninguna saturación.
2. Pelvis inclinada < 5° en todo momento (no se cae).
3. Pelvis a < 1 cm de su posición planificada.
4. Ningún pie desliza > 5 mm.
5. Fuerzas: con dos pies, suma = peso ± 5 %; con uno, pie izquierdo = peso y
   derecho = 0.
6. Pie levantado ≥ 4 cm.
7. CdM dentro del pie de apoyo con ≥ 2 cm de margen.
8. Error articular < 0,5°.

**Lo que pasó por el camino:**

1. *Primera ejecución:* no se cayó (inclinación ≤ 2,9°), levantó el pie y no
   saturó nada. Pero la pelvis quedaba a 13 mm de su sitio y el error articular
   llegaba a 0,93°. FAIL. También salió un fallo falso: "pie derecho abajo" se
   evaluaba como si el pie estuviera en el aire, porque quedaba a 1,3 mm del
   suelo. El criterio pasó a usar el plan, no la medida.
2. *La causa:* en sentadilla, muslos y tibias van hacia delante y el **CdM
   queda ~4,6 cm delante de los tobillos**. El compensador aplicaba la fuerza
   del suelo *en el tobillo*, pero en realidad actúa bajo el CdM (en el
   **centro de presiones**). Faltaba el torque de tobillo que evita que el
   cuerpo vuelque hacia delante: el tobillo cedía `τ / kp` y la pelvis se
   inclinaba.
3. *Solución:* el compensador acepta el punto donde actúa cada fuerza
   (`mj_jac` en lugar de `mj_jacBody`). El bípedo usa x = la del CdM, y = la de
   cada pie. Los experimentos 8 y 9 siguen usando el tobillo, que allí vale porque el
   CdM está encima. **PASS**: inclinación < 1°, pelvis a < 1,5 mm, error
   articular < 0,1°.
4. *¿Hace falta compensar?* Sin compensación el robot **se cae** a los 9,7 s,
   a mitad del cambio de peso.

### Experimento 11 — Primeros pasos: marcha estática

```bash
python simulation/scripts/walk_biped.py           # informe en la terminal
mjpython simulation/scripts/walk_biped.py --view  # además, verlo (~55 s)
```

El mismo bípedo del experimento 10 da **4 pasos de 8 cm**. Es una **marcha
estática**: el centro de masas está siempre sobre un pie, así que si el robot
se congelara en cualquier instante no se caería. Es lenta, pero es la base
sobre la que se construye la marcha dinámica.

Cada paso son cuatro fases:

1. **Peso al pie de apoyo:** la pelvis se mueve hasta que el CdM queda sobre
   él (`center_com_over`).
2. **Pie libre arriba** 5 cm.
3. **Pie libre adelante:** aterrizará 8 cm por delante del otro pie. En el
   aire, la pelvis se reajusta para que el CdM siga sobre el pie de apoyo,
   porque la pierna que avanza también lo mueve.
4. **Pie libre abajo.**

Empieza el pie derecho. Tras el 4.º paso, el peso vuelve al centro entre los
pies. Lo que comparten los experimentos 10 y 11 (`Pose`, `Biped`, IK de las dos
piernas, reparto del peso) se movió a `biped_control.py`.

**Criterios de aceptación**, fijados antes de la primera ejecución: los de la
experimento 10, más:

- La pelvis no gira más de 3° (yaw).
- Cada pie está a < 5 mm de **donde se colocó** mientras está en el suelo, no
  solo de donde empezó.
- Ninguna pose pide más del 98 % del largo de la pierna. Se comprueba al
  planificar, antes de simular: la IK recorta en silencio si no llega. Basta
  con mirar las poses de cada fase: la distancia cadera-tobillo de una mezcla
  lineal de dos poses nunca supera la de los extremos.

**Lo que pasó por el camino:**

1. *Alcance.* Con las rodillas a 50°, la comprobación de alcance paró el plan
   antes de simular: al aterrizar el pie adelantado con la pelvis desplazada
   15,8 cm de lado, la pierna se estiraba al **99,9 %**. Con 55° llega al
   98,4 % (aún por encima del límite) y con 60° baja al 96,7 %.
2. *Primera simulación:* da los 4 pasos sin caerse y avanza 29 cm. Pero en el
   cambio de peso de los pasos 2 y 4, **los dos pies** se mueven ~7 mm a la
   vez. FAIL.
3. *Primera causa encontrada:* con los pies escalonados, la pelvis cruza
   31,6 cm de lado en el cambio de peso (el doble que en el paso 1) en el
   mismo 1,5 s. Doble aceleración, más torque de vuelco, y los pies se
   balancean sobre 1 o 2 esquinas. Arreglo: **las transiciones largas duran
   más** (duración ∝ √distancia), para no acelerar más que el experimento 10. Se
   añadió `duration` a `interpolate`. La inclinación baja de 1,8° a 0,9°,
   pero el desplazamiento sigue acumulándose (3 mm por cambio de peso). FAIL.
4. *Dos hipótesis que no eran:*
   - **Fluencia del contacto blando** (MuJoCo deja deslizar un poco bajo
     fuerza lateral). Se probaron `cone="elliptic"`, `impratio=10` y el solver
     `noslip`: lo mejor bajó de 9,3 a 6,0 mm. No era la causa principal y no
     se dejó.
   - **El pie pivotando sobre una esquina** (fricción de torsión casi nula).
     El yaw de los pies no pasaba de 0,3°: tampoco.
5. *La causa real:* al mirar la dirección y la carga, el pie que se movía era
   siempre el **poco cargado**: el recién aterrizado (< 25 N) o el que está a
   punto de despegar (< 5 N). Lo arrastraba el **retraso de los motores**. Un
   motor de posición de MuJoCo hace `τ = kp·(objetivo − q) − kv·q̇`: su
   amortiguamiento frena hacia velocidad *cero*, así que en movimiento la
   articulación va `(kv/kp)·q̇` por detrás del objetivo. En hip_roll son
   ~1,3°, unos 10 mm en el pie. El pie cargado aguanta por fricción; el ligero,
   no.
6. *Solución:* **feedforward de velocidad** en `Biped.step`: se suma
   `(kv/kp)·q̇_deseada` al objetivo. El motor da entonces
   `kp·(objetivo − q) + kv·(q̇_deseada − q̇)` y sigue sin retraso. `q̇_deseada`
   sale de la diferencia entre los objetivos de dos pasos seguidos. **PASS**:
   cada pie a < 1,5 mm de su sitio. Sin el escalado de duración también pasa,
   pero con 3,9 mm: se deja el escalado porque da el triple de margen.

### Experimento 12 — La misma marcha en la mitad de tiempo

```bash
python simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25           # informe
mjpython simulation/scripts/walk_biped.py --speed 1.27 --hold 0.25 --view  # verlo (~27 s)
```

**Objetivo** (de la hoja de ruta): la marcha del experimento 11 en la mitad de
tiempo, con los mismos criterios. La idea de partida era que haría falta una
**compensación dinámica**. Al final no hizo falta, y el porqué es la lección
de este experimento.

`walk_biped.py` tiene tres opciones nuevas:

- `--speed`: divide la duración de las transiciones.
- `--hold`: fija la pausa tras cada transición (por defecto 1,5 s).
- `--dynamic`: usaba la compensación dinámica; se eliminó en el experimento 16.

**Lo que pasó por el camino:**

1. *Medir primero.* Con todas las duraciones a la mitad (×2), la marcha falla
   igual que el experimento 11 al principio: pies a 10 mm de su sitio y pelvis
   inclinada 1,8°, en los cambios de peso. A ×1,5 todavía pasa (4 mm).
2. *Compensación dinámica (dinámica inversa).* En lugar de solo la gravedad,
   el compensador calcula lo que necesita el movimiento planificado:
   `τ = M(q)·q̈ + C(q, q̇) + τ_g(q) − Σ Jᵢᵀ·Fᵢ` (`mj_rne` con aceleración). Las
   aceleraciones salen del plan por diferencias finitas, como la velocidad del
   feedforward. La fuerza del suelo deja de ser solo el peso e incluye la
   fuerza para acelerar el cuerpo, y actúa en el **ZMP** (punto de momento
   cero), no bajo el CdM.
3. *Primera versión del ZMP:* la fórmula del carro y la mesa,
   `ZMP = CdM − h/(g + a_z)·a_xy`. Los pies mejoran a velocidad alta, pero la
   pelvis se inclina **más** que con la estática, incluso a velocidad normal
   (1,02° frente a 0,57°).
4. *Segunda versión, ZMP exacto (`Biped.required_contact`).* La dinámica
   inversa da lo que necesitan los 6 grados de libertad de la pelvis. Como no
   tienen motor, solo el suelo puede aportarlo: las 3 filas lineales son la
   fuerza total, y las 3 angulares dicen **dónde** tiene que actuar (se
   resuelve por mínimos cuadrados). Incluye el momento angular de las piernas.
   Comprobación: en reposo da el peso exacto y el ZMP bajo el CdM (0,0000 mm).
   Los pies mejoran más (×2,5: 79 → 19 mm), pero la inclinación sigue igual.
   El momento angular no era la causa.
5. *Perfil de mínimo jerk.* La inclinación máxima aparecía siempre justo al
   empezar o al acabar una transición, donde la aceleración del perfil coseno
   salta. Se probó `10s³ − 15s⁴ + 6s⁵` (aceleración cero en los extremos):
   **peor** (su aceleración pico es un 17 % mayor). Se quitó.
6. *Mirar el seguimiento.* El error articular a lo largo de toda la trayectoria
   salía **igual** con las dos compensaciones (0,048° a ×1), y los peores eran
   los tobillos en roll. La inercia de los eslabones, que es lo que corrige la
   versión dinámica, no era el problema.
7. *¿El plan es posible?* Se midió el ZMP que pide el plan frente al polígono
   de apoyo. Siempre queda dentro, pero a ×2 se acerca a **8 mm del borde** en
   los cambios de peso, y con la presión tan cerca del borde el pie se balancea.
8. *Dónde se va el tiempo.* De los 54,3 s del experimento 11, **25,5 s son
   pausas** (17 fases × 1,5 s quieto para medir). Dividirlo todo por dos
   acelera también los movimientos, que es lo caro. Con pausas de 0,25 s, para
   llegar a 27,15 s las transiciones solo tienen que ir ×1,27 (aceleración
   ×1,6 en lugar de ×4). **PASS con la compensación estática.**

**Comparación final**, todas en la mitad de tiempo (27,1 s):

| Pausa | Transiciones | Compensación | Pie fuera de sitio máx. | Inclinación máx. | Fallos |
|---|---|---|---|---|---|
| 0,75 s | ×2,00 | estática | 10,0 mm | 1,81° | 13 |
| 0,75 s | ×2,00 | dinámica | 5,7 mm | 3,33° | 4 |
| 0,50 s | ×1,58 | estática | 5,3 mm | 1,16° | 5 |
| 0,50 s | ×1,58 | dinámica | 3,2 mm | 2,22° | 0 |
| **0,25 s** | **×1,27** | **estática** | **2,5 mm** | **0,79°** | **0** |
| 0,25 s | ×1,27 | dinámica | 2,6 mm | 1,48° | 0 |
| 0,10 s | ×1,14 | estática | 1,3 mm | 0,59° | 0 |

Se eligió 0,25 s de pausa: con 0,10 s los números son algo mejores, pero el
robot apenas tiene tiempo de asentarse antes de medir.

**Corrección posterior:** las filas "dinámica" de esta tabla son de una versión
con un error en el reparto entre pies. El experimento 15 lo explica y lo corrige.
La compensación dinámica se eliminó después: ver el experimento 16.

### Experimento 13 — Marcha más natural: pasos largos y giro

```bash
python simulation/scripts/walk_biped.py --knee 56 --step 0.12 --width 0.07 --inset 0.02 --turn 45 --speed 1.27 --hold 0.25
mjpython simulation/scripts/walk_biped.py --knee 56 --step 0.12 --width 0.07 --inset 0.02 --turn 45 --speed 1.27 --hold 0.25 --view  # verlo (~51 s)
```

**Objetivo** (de la hoja de ruta): 4 pasos de 12 cm y un giro de 45°, con la
pelvis más alta y menos desplazamiento lateral. El bípedo es el mismo; lo que
cambia es el plan. `walk_biped.py` tiene opciones nuevas, y por defecto sigue
dando los números exactos de los experimentos 11 y 12:

| Opción | Qué es | Experimentos 11-12 | Experimento 13 |
|---|---|---|---|
| `--knee` | Rodillas de pie | 60° | 56° |
| `--step` | Cuánto adelanta un pie al otro | 8 cm | 12 cm |
| `--width` | Distancia de cada pie al centro | 10 cm | 7 cm |
| `--inset` | CdM hacia dentro del centro del pie de apoyo | 0 | 2 cm |
| `--turn` | Giro final a la izquierda | 0° | 45° |

**Menos balanceo.** Los 16 cm de desplazamiento lateral venían de dos sitios:
los pies separados 20 cm y el CdM exactamente en el centro del pie. Con los
pies a ±7 cm y el CdM 2 cm hacia dentro, la pelvis se mueve **8 cm**, la
mitad. El CdM sigue teniendo 3 cm hasta el borde del pie. Con menos balanceo
la pierna llega más lejos, así que se puede andar más estirado:

| Pies a | CdM hacia dentro | Pelvis de lado | Rodilla mínima (alcance ≤ 98 %) |
|---|---|---|---|
| ±10 cm | 0 | 16,2 cm | 66° |
| ±10 cm | 3 cm | 11,3 cm | 60° |
| ±8 cm | 2 cm | 9,7 cm | 58° |
| **±7 cm** | **2 cm** | **8,0 cm** | **56°** |
| ±6 cm | 3 cm | 4,8 cm | 54° |

No se bajó de ±7 cm: a ±6 cm quedan solo 2 cm entre pies, y al girar un pie
respecto al otro las esquinas se acercan.

**Giro.** `Pose` guarda ahora el giro de la pelvis y de cada pie.
`joint_targets` pone `hip_yaw = giro del pie − giro de la pelvis` y resuelve el
resto de la pierna con `leg_ik` en los ejes del pie. Se comprobó contra MuJoCo
con pelvis y pies girados: error 0,0000 mm, pies planos y giro exacto.

**Pisadas.** `footsteps` genera la lista de pisadas (pie, posición, giro). Cada
una se coloca en los ejes del nuevo rumbo, respecto al pie de apoyo. Tras los 4
pasos rectos, el rumbo sube 15° por paso, con pasos de medio largo, y un último
paso deja los dos pies paralelos: 8 pasos en total. La pelvis gira a medio
camino entre los dos pies.

**Criterios de aceptación**, fijados antes de la primera ejecución: los de la
experimento 11, más:

- La pelvis y cada pie a < 3° de su giro planificado.
- Los pies no se tocan entre sí.
- El margen del CdM se mide en los ejes del pie, porque su rectángulo gira.

**Pasó a la primera.** Al ritmo del experimento 11, la pelvis se inclina menos
(0,39° frente a 0,57°), porque menos balanceo significa menos aceleración; los
pies quedan casi igual (1,6 mm frente a 1,4 mm), con pasos más largos. También
aguanta el ritmo del experimento 12, que es la configuración elegida.


### Experimento 14 — Respuesta a empujones

```bash
python simulation/scripts/push_biped.py                      # informe
mjpython simulation/scripts/push_biped.py --force 20 --view  # ver un empujón
```

Empujón de 0,1 s en la pelvis, de pie, en cuatro direcciones, con motores con
curva de par-velocidad. **Cierra la etapa 19 con límites definidos, sin dar un
paso.** El paso de recuperación no funcionó y pasa a la etapa 26.

**Límite físico.** Sin dar un paso, solo se frena con el ZMP en el borde del
pie: `v/ω ≤ distancia al borde`, con `ω = √(g/h) ≈ 4,8 rad/s`. Son unos 29 N
hacia delante y atrás, y 35 N de lado.

**Resultado** (`push_biped.py`):

| Dirección | Al empezar | Medido | Límite definido | Máximo físico |
|---|---|---|---|---|
| Adelante | 16 N | 25 N | **23 N** | 29 N |
| Atrás | 18 N | 26 N | **24 N** | 29 N |
| Izquierda | 30 N | 27 N | **25 N** | 35 N |
| Derecha | 30 N | 27 N | **25 N** | 35 N |

Recuperarse exige acabar con la pelvis a < 2° y los pies a < 2 cm de su sitio.
Los límites son lo medido menos unos 2 N de margen: se derivan de la medida, no
de un requisito externo. La mejora de "al empezar" a "medido" viene de centrar
el CdM. De lado baja respecto al inicio porque ahora el criterio exige que los
pies no se muevan, los pies están más juntos y los motores tienen curva.

**Qué se intentó y no funcionó:**

1. *Corregir la pelvis con el punto de captura:* inestable. Con articulaciones
   rígidas la pelvis sigue su objetivo, y la corrección altera lo que se mide.
2. *Corregir la pelvis con la inclinación de la IMU:* cae incluso sin empujón.
3. *Limitar el torque del tobillo con la fuerza del pie:* sin efecto.
4. *Paso de recuperación:* sobrevive a 54 N, pero **saltando**: con 50 N el pie de
   apoyo se levanta 10,7 cm y durante 188 ms no hay ningún pie en el suelo. Con un
   paso limpio aguanta 22 N adelante y 28 N de lado, menos o casi igual que sin
   paso. El código está en el historial de git, commit `6c083cc`
   (`recover_biped.py`).

*(Las cifras de la tabla de arriba son del modelo de 60 cm; las del cuerpo de Danny
están en el experimento 16.)*

**Lo que sí funcionó: mirar la caída.** De pie, el CdM estaba **3,4 cm por
delante** de los pies, porque los muslos adelantan masa, y el margen real era de
6,6 cm. Centrarlo con `center_com_over` subió de 16 a 25 N. También se comprobó
que la velocidad no era el problema: con motores de 8 rad/s el resultado apenas
cambiaba. Los 33 rad/s medidos eran un pico de 2 ms por el impacto.

**Limitaciones:** solo de pie, no durante la marcha; el control usa datos exactos
del simulador, sin ruido; el ±20 N·m y los 20 rad/s son provisionales.

### Experimento 15 — Por qué la compensación dinámica inclinaba más

En el experimento 12, la compensación dinámica inclinaba más la pelvis que la
estática y no se sabía por qué. **Etapa 18 cerrada: causa encontrada y corregida.**

**Criterio**, antes de medir: se queda solo si, en la marcha del experimento 13,
iguala o mejora a la estática en inclinación y en pies. Si no, se elimina.

**La prueba directa.** Una compensación correcta debe predecir el par que los
motores aplican de verdad. Se comparó el par real con cada predicción, en cada
instante de la marcha:

| Error RMS del par previsto [N·m] | Estática | Dinámica |
|---|---|---|
| Un pie, en movimiento | 0,114 | **0,104** |
| Dos pies, en movimiento | **0,174** | 0,328 |

Con un pie, la fórmula es correcta. Con dos, falla el reparto entre pies.

| Reparto en doble apoyo | Error RMS [N·m] |
|---|---|
| Por ZMP, puntos en el ZMP, fuerza completa (el original) | 0,328 |
| Por CdM, puntos en el ZMP, fuerza completa | 0,233 |
| Por CdM, puntos en el CdM, fuerza completa | 0,250 |
| Por ZMP, puntos en el ZMP, fuerza vertical | 0,252 |
| **Por CdM, puntos en el CdM, fuerza vertical** | **0,174** |

El peso real de cada pie sigue al CdM (error 0,014 en la fracción), no al ZMP
(0,045): con dos pies apoyados, lo decide la geometría. Y la fuerza horizontal no
se reparte en proporción al peso. La corrección: dinámica inversa completa con
un pie; con dos, reparto por CdM y fuerza vertical. Se añadió además la
aceleración de giro de la pelvis, y ahora vale con `--turn`.

**Resultado**, recta (`--knee 56 --step 0.12 --width 0.07 --inset 0.02`):

| Transiciones, pausa | Compensación | Pie máx. | Inclin. máx. | Fallos |
|---|---|---|---|---|
| ×1,27, 0,25 s | estática | 1,63 mm | 0,66° | 0 |
| ×1,27, 0,25 s | dinámica | 1,83 mm | **0,59°** | 0 |
| ×2, 0,25 s | estática | 4,47 mm | 1,53° | 0 |
| ×2, 0,25 s | dinámica | **2,87 mm** | 1,58° | 0 |

Con giro de 45°:

| Transiciones, pausa | Compensación | Pie máx. | Inclin. máx. | Fallos |
|---|---|---|---|---|
| ×1,27, 0,25 s | estática | 1,71 mm | 0,56° | 0 |
| ×1,27, 0,25 s | dinámica | 1,97 mm | **0,47°** | 0 |
| ×2, 0,25 s | estática | 6,33 mm | 1,31° | 16 |
| ×2, 0,25 s | dinámica | **3,10 mm** | 1,37° | 1 |
| ×2,5, 0,25 s | estática | 6,29 mm | 2,26° | 15 |
| ×2,5, 0,25 s | dinámica | 7,85 mm | 2,19° | 18 |

**Decisión posterior.** Estos números son del modelo de 60 cm. Con el cuerpo de
Danny (experimento 16) la ventaja desaparece: la estática ya pasa a ×2 y a ×2,5
caen las dos. Se aplicó el criterio inicial y **la compensación dinámica se
eliminó**. El código está en el commit `f24ed39` (`required_contact`).

### Experimento 16 — El cuerpo de Danny: 70 cm y 8 kg

**Etapas 13 y 20.** Hasta aquí, el bípedo medía 60 cm, sin torso, con piernas de
50 cm. Los números de los experimentos 10 a 15 de este handbook son de **ese
modelo**, y explican el camino. Los valores actuales, con el cuerpo de Danny, son
los de esta sección y los de `results/`.

Modelo `danny_biped.xml`, entonces llamado `danny.xml` (provisional hasta la etapa 31):

| Parte | Medida | Masa |
|---|---|---|
| Pie | 14 × 8 cm, tobillo a 4 cm del suelo | 0,2 kg × 2 |
| Tibia | 16 cm | 0,5 kg × 2 |
| Muslo | 16 cm | 0,7 kg × 2 |
| Pelvis | caderas a ±7 cm | 1,2 kg |
| Torso, sin brazos ni articulación | hasta los 70 cm | 4 kg |

Medido en el modelo: 8,0 kg, 0,70 m y CdM a 41,6 cm. Las distancias de las marchas
se escalaron con la pierna (factor 0,64): pasos de 5 y 8 cm, pie libre a 3 cm,
pies a ±5 cm. Criterios iguales, con la altura mínima del pie libre escalada a 2,5
cm.

**La vibración.** Al pasar al modelo nuevo, E10 falló: los dos pies se desplazaban
12 mm. No era deslizamiento: la fuerza en los pies oscilaba entre 4 y 106 N, a
**12 Hz**. Los tobillos eran tan ligeros que resonaban con el contacto blando. La
solución física: `armature`, la inercia del motor reflejada por la reductora, que
el modelo ignoraba. Con 0,01 kg·m² la oscilación desaparece y los pies quedan a
menos de 1 mm. `kv` se recalculó con esa inercia.

**Resultados** (`results/`):

| Prueba | Tiempo | Pie máx. | Inclin. máx. | Error art. |
|---|---|---|---|---|
| E11: 4 pasos de 5 cm | 53,6 s | 1,4 mm | 0,79° | 0,12° |
| E12: lo mismo, más rápido | 26,6 s | 3,3 mm | 1,24° | 0,44° |
| E13: 4 pasos de 8 cm y giro de 45° | 50,3 s | 0,8 mm | 0,57° | 0,22° |
| 4 pasos de 8 cm a ×2 | 19,0 s | 1,0 mm | 1,74° | 0,25° |

El "error art." se mide al final de cada sostén. Durante el movimiento es mayor, y
desde la auditoría se limita a 1°: 0,54° en E11, 0,70° en E12, 0,38° en E13 y 0,94°
a ×2, la prueba más justa. A ×2, además, un pie cargado se apoya en 2 puntos o
menos hasta el 11,8 % del tiempo de la peor fase: los pies se balancean.

**Empujones sin paso:** 26 N adelante, 26 N atrás y 34 N de lado, tras ajustar el
amortiguamiento de los tobillos para carga en apoyo. Límites definidos, con 2 N de
margen: 24 y 32 N. El máximo físico estimado es de 28 N y 36 N.

Las cifras actuales y completas están en la
[validación de la fase 3](../documentation/phase-3-validation.md), que manda sobre
las de esta sección si difieren.

**Auditoría técnica (etapa 20).** Una revisión independiente del código encontró
problemas que las pruebas no veían, y se corrigieron:

- *Las métricas solo se medían al final de cada sostén.* Ahora también durante el
  movimiento: error articular < 1° y pelvis < 2 cm. Estos límites se fijaron con
  los datos ya vistos.
- *La prueba de masas no era ciega.* Escalaba todo por igual, así que el CdM no
  cambiaba, y el controlador leía el CdM real del simulador. Ahora lo calcula con
  su modelo nominal y la postura medida (`estimated_com`), y se prueba también
  cambiando solo el torso.
- *La compensación dinámica ya no aportaba* con este cuerpo, y se eliminó.
- `check_kinematics.py` dice "planos" y ahora lo comprueba también en el bípedo.

**Robustez.** La marcha del E13, con el controlador sin saber el cambio:

| Variación | Resultado | Error en movimiento |
|---|---|---|
| Masas e inercias ±10 % | ✅ | 0,60° |
| Pelvis y torso ±10 % | ✅ | 0,51° |
| Fricción 0,6 | ✅ | 0,35° |
| Paso de simulación de 1 ms | ✅ | 0,37° |

**Límites conocidos.** El control sigue su plan sin corregir con la IMU, y eso lo
hace frágil. Medido con la misma marcha, sin que el controlador lo sepa:

| Variación | Resultado |
|---|---|
| CdM de pelvis y torso +0,5 cm | ✅ pie a 4,7 mm, casi en el límite de 5 mm |
| CdM de pelvis y torso ±1 cm | ❌ 10-22 fallos, pie a 8-10 mm |
| Suelo inclinado 0,1° | ✅ pie a 3,2 mm |
| Suelo inclinado 0,25° | ❌ 10 fallos |
| Suelo inclinado 0,5° | ❌ 27-34 fallos, pie a 13 mm |

En estos casos no cae, pero incumple los criterios. La revisión independiente vio
además, con un controlador de prueba fuera del repositorio, caídas con empujones
de 22-23 N si el CdM está desplazado 1 cm, y con un retardo de 6 ms o más en el lazo
de control; no se ha repetido en el proyecto. Toleró sesgo de IMU de 1°, ruido de
encoder de 0,5° y fricción 0,3.

**Más límites:** los empujones se prueban solo en doble apoyo y se juzgan por el
estado final (a 31 N un pie se descarga por completo y sube 1,9 cm), y los límites
de E14 son lo medido menos 2 N, así que solo detectan regresiones. Los rangos
articulares se comprueban al planificar la marcha, no en las demás pruebas. Durante
la transferencia de peso, el CdM queda entre los pies, no sobre uno de ellos.

### Experimento 17 — Un modelo generado desde una sola especificación

**Etapa 3.5.2.** Con 30 articulaciones, copiar XML a mano duplicaría cada definición.
Ahora las medidas, masas, articulaciones y motores viven en `robot_spec.py`.
`build_model.py` construye el modelo con `MjSpec`, la API de MuJoCo, y genera izquierda y
derecha desde una sola descripción. Los controladores importan esos mismos datos.

`danny_biped.xml` (el antiguo `danny.xml`) se genera y se versiona, y **no se edita a
mano**: una prueba comprueba que coincide con la especificación.

**Criterio:** no cambiar la física. Resultado:

- Las 458 matrices numéricas del modelo compilado son idénticas a las del anterior, sin
  ninguna diferencia, y los nombres también.
- 126 pruebas y 16 comprobaciones pasan.
- Las 277 métricas comunes de `results/phase3_metrics.json` no cambian.

Las pruebas que necesitan una variante del robot (pelvis fija, retardo de motor) ya no
editan texto XML: la construyen con `MjSpec`.

---

## 3. La pierna de 2 GDL en detalle

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
de gravedad (experimento 7).

### Sensores

Lo que tendría el controlador de un robot real:

| Sensor | Tipo | Unidad |
|---|---|---|
| `hip_pos`, `knee_pos` | `jointpos` (encoder) | rad |
| `hip_vel`, `knee_vel` | `jointvel` | rad/s |
| `hip_torque`, `knee_torque` | `actuatorfrc` (torque del motor) | N·m |

Se leen con `data.sensor("hip_pos").data[0]`.

---

## 4. Análisis de esfuerzos y resultados

`analyze_leg_2dof.py` ejecuta la secuencia (cadera, rodilla) en grados

```
(0, 0) → (30, 45) → (-20, 90) → (45, 60) → (0, 120) → (0, 0)
```

en tres modos:

- **step**: el objetivo salta de golpe al siguiente valor.
- **smooth**: el objetivo pasa de uno a otro en 1,5 s con un **perfil coseno**,
  `blend = ½·(1 − cos(π·t/T))`. Tiene velocidad cero al empezar y al acabar,
  así que no hay tirón.
- **smooth + gravedad**: como smooth, con compensación de gravedad (experimento 7).

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

### Experimento 8: sentadilla (`squat_leg_2dof.py`)

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

### Experimento 9: pierna de 6 GDL (`analyze_leg_6dof.py`)

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
compensadores solo cubren la parte estática. Ver [Hoja de ruta](../ROADMAP.md).

### Experimento 10: bípedo en equilibrio (`balance_biped.py`)

Valores al final de cada sostén. Inclinación = máximo durante la fase. Con el
feedforward de velocidad del experimento 11, que `Biped` aplica siempre.

| Fase | Error pelvis [mm] | Inclin. máx [°] | Desliz. izq / der [mm] | Pie izq / der [N] | Pie der. sobre el suelo [mm] | Margen CdM [mm] | Error art. [°] |
|---|---|---|---|---|---|---|---|
| Sentadilla 80° | 0,90 | 0,08 | 0,20 / 0,20 | 30,4 / 30,4 | — | — | 0,00 |
| De pie 40° | 0,88 | 0,07 | 0,39 / 0,39 | 30,4 / 30,4 | — | — | 0,00 |
| Peso a la izquierda (15,8 cm) | 0,93 | 0,51 | 0,55 / 1,05 | 60,3 / 0,5 | — | — | 0,07 |
| **Pie derecho arriba 5 cm** | 0,69 | 0,13 | 0,55 / — | **60,8 / 0,0** | **49,4** | **50,0** | 0,01 |
| Pie derecho abajo | 1,05 | 0,27 | 0,62 / 0,93 | 60,3 / 0,5 | — | — | 0,07 |
| Vuelta al centro | 1,52 | 0,39 | 0,93 / 1,49 | 30,4 / 30,4 | — | — | 0,01 |

Ningún motor saturado. Con el pie derecho en el aire, el CdM está a 50,0 mm
del borde del pie izquierdo: en su centro (medio ancho = 50 mm).

La mayor inclinación (0,51°) aparece *durante* el cambio de peso, no en el
sostén: mover 6,2 kg de lado pide aceleración, y la compensación es estática.
Sin el feedforward de velocidad era 0,98°.

### Experimento 11: primeros pasos (`walk_biped.py`)

Rodillas a 60°, pasos de 8 cm, pie libre a 5 cm. Valores al final de cada
sostén; "fuera de sitio" = distancia al punto donde se colocó el pie.

| Fase | Error pelvis [mm] | Inclin. máx [°] | Yaw [°] | Pie izq / der fuera de sitio [mm] | Pie izq / der [N] | Pie libre [mm] | Margen CdM [mm] |
|---|---|---|---|---|---|---|---|
| Paso 1: peso al izquierdo | 0,81 | 0,46 | −0,01 | 0,2 / 0,6 | 60,3 / 0,5 | — | — |
| Paso 1: pie derecho arriba | 0,60 | 0,11 | −0,01 | 0,2 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 1: pie derecho adelante | 0,58 | 0,13 | −0,02 | 0,2 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 1: pie derecho abajo | 0,85 | 0,22 | −0,03 | 0,3 / 0,5 | 60,4 / 0,4 | — | 48,9 |
| Paso 2: peso al derecho | 1,10 | 0,55 | −0,22 | 1,4 / 1,0 | 1,0 / 59,8 | — | — |
| Paso 2: pie izquierdo arriba | 1,44 | 0,11 | −0,23 | — / 1,0 | 0,0 / 60,8 | 49,4 | 50,0 |
| Paso 2: pie izquierdo adelante | 1,37 | 0,29 | −0,23 | — / 1,0 | 0,0 / 60,8 | 49,4 | 50,0 |
| Paso 2: pie izquierdo abajo | 1,35 | 0,22 | −0,21 | 0,9 / 1,0 | 0,4 / 60,4 | — | 48,8 |
| Paso 3: peso al izquierdo | 0,76 | 0,55 | −0,02 | 0,6 / 1,0 | 59,8 / 1,0 | — | — |
| Paso 3: pie derecho arriba | 0,85 | 0,11 | −0,01 | 0,6 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 3: pie derecho adelante | 0,71 | 0,29 | −0,02 | 0,6 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 3: pie derecho abajo | 0,92 | 0,22 | −0,03 | 0,7 / 1,0 | 60,4 / 0,4 | — | 48,9 |
| Paso 4: peso al derecho | 1,46 | 0,55 | −0,22 | 1,1 / 1,2 | 1,0 / 59,8 | — | — |
| Paso 4: pie izquierdo arriba | 1,63 | 0,11 | −0,23 | — / 1,2 | 0,0 / 60,8 | 49,4 | 50,0 |
| Paso 4: pie izquierdo adelante | 1,48 | 0,29 | −0,23 | — / 1,2 | 0,0 / 60,8 | 49,4 | 50,0 |
| Paso 4: pie izquierdo abajo | 1,67 | 0,22 | −0,21 | 0,7 / 1,3 | 0,4 / 60,4 | — | 48,8 |
| Peso al centro | 0,88 | 0,57 | −0,12 | 0,9 / 1,3 | 33,8 / 27,1 | — | — |

La pelvis avanza **27,9 cm** (los pies acaban en x = 24 y 32 cm). Ningún
motor saturado, error articular ≤ 0,07°. Los fallos se comparan así:

| | Pie fuera de sitio máx. | Inclinación máx. |
|---|---|---|
| Transiciones de 1,5 s, sin feedforward | 13,5 mm | 1,78° |
| Duración ∝ √distancia, sin feedforward | 9,3 mm | 0,85° |
| Solo feedforward de velocidad | 3,9 mm | 0,98° |
| **Las dos cosas** | **1,4 mm** | **0,57°** |

### Experimento 12: la marcha en la mitad de tiempo (`walk_biped.py --speed 1.27 --hold 0.25`)

Compensación estática, transiciones ×1,27, pausas de 0,25 s.

| Fase | Error pelvis [mm] | Inclin. máx [°] | Yaw [°] | Pie izq / der fuera de sitio [mm] | Pie izq / der [N] | Pie libre [mm] | Margen CdM [mm] |
|---|---|---|---|---|---|---|---|
| Paso 1: peso al izquierdo | 1,27 | 0,74 | −0,06 | 0,2 / 0,6 | 56,3 / 4,1 | — | — |
| Paso 1: pie derecho arriba | 0,71 | 0,16 | −0,03 | 0,1 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 1: pie derecho adelante | 0,60 | 0,22 | −0,06 | 0,1 / — | 60,8 / 0,0 | 49,4 | 50,0 |
| Paso 1: pie derecho abajo | 0,70 | 0,24 | −0,04 | 0,2 / 0,2 | 59,5 / 1,4 | — | 48,9 |
| Paso 2: peso al derecho | 3,21 | 0,75 | −0,32 | 2,3 / 1,9 | 3,6 / 56,4 | — | — |
| Paso 2: pie izquierdo arriba | 2,23 | 0,14 | −0,31 | — / 1,9 | 0,0 / 60,8 | 49,5 | 49,8 |
| Paso 2: pie izquierdo adelante | 1,93 | 0,45 | −0,25 | — / 1,9 | 0,0 / 60,8 | 49,2 | 50,0 |
| Paso 2: pie izquierdo abajo | 2,00 | 0,24 | −0,30 | 1,2 / 1,9 | 1,3 / 59,7 | — | 48,9 |
| Paso 3: peso al izquierdo | 1,97 | 0,74 | −0,01 | 1,2 / 1,7 | 56,3 / 3,7 | — | — |
| Paso 3: pie derecho arriba | 1,36 | 0,14 | −0,02 | 1,2 / — | 60,8 / 0,0 | 49,5 | 49,8 |
| Paso 3: pie derecho adelante | 0,49 | 0,45 | −0,09 | 1,2 / — | 60,8 / 0,0 | 49,2 | 50,0 |
| Paso 3: pie derecho abajo | 1,08 | 0,24 | −0,03 | 1,2 / 1,3 | 59,7 / 1,3 | — | 48,9 |
| Paso 4: peso al derecho | 3,58 | 0,74 | −0,33 | 2,3 / 2,4 | 3,7 / 56,3 | — | — |
| Paso 4: pie izquierdo arriba | 2,88 | 0,14 | −0,31 | — / 2,5 | 0,0 / 60,8 | 49,5 | 49,8 |
| Paso 4: pie izquierdo adelante | 2,08 | 0,45 | −0,25 | — / 2,5 | 0,0 / 60,8 | 49,2 | 50,0 |
| Paso 4: pie izquierdo abajo | 2,62 | 0,24 | −0,30 | 1,3 / 2,4 | 1,3 / 59,7 | — | 48,9 |
| Peso al centro | 2,27 | 0,79 | −0,16 | 1,9 / 2,4 | 34,0 / 26,8 | — | — |

Duración total **27,1 s** (experimento 11: 54,3 s). La pelvis avanza 27,8 cm.
Ningún motor saturado, error articular ≤ 0,17°. Con pausas cortas, al final
del cambio de peso el pie que se descarga aún lleva 3–4 N: el robot no ha
terminado de asentarse cuando se mide, por eso la pausa no baja de 0,25 s.

### Experimento 13: pasos largos y giro (`walk_biped.py --knee 56 --step 0.12 --width 0.07 --inset 0.02 --turn 45 --speed 1.27 --hold 0.25`)

Por tipo de fase, lo peor de los 8 pasos:

| Medida | Resultado | Límite |
|---|---|---|
| Pasos | 8 (4 rectos + 4 de giro) | — |
| Duración total | 50,6 s | — |
| Pelvis al final | (66,9, 11,4) cm, rumbo **44,9°** | rumbo 45° |
| Pie fuera de sitio | 1,7 mm | 5 mm |
| Inclinación de la pelvis | 0,56° | 5° |
| Error de pelvis | 2,7 mm | 10 mm |
| Error de giro, pelvis / pies | 0,28° / 0,29° | 3° |
| Margen del CdM | 27,5 mm | 20 mm |
| Pie libre | 48,8 mm | 40 mm |
| Error articular | 0,13° | 0,5° |
| Pies que se tocan | no | no |

Ningún motor saturado. Comparación de las tres marchas, cada una a su ritmo:

| | Experimento 11 | Experimento 12 | Experimento 13 |
|---|---|---|---|
| Pasos | 4 de 8 cm | 4 de 8 cm | 4 de 12 cm + 4 de giro |
| Tiempo por paso | 13,6 s | 6,8 s | 6,3 s |
| Rodillas | 60° | 60° | 56° |
| Pelvis de lado | 15,8 cm | 15,8 cm | 8,0 cm |
| Pie fuera de sitio máx. | 1,4 mm | 2,5 mm | 1,7 mm |
| Inclinación máx. | 0,57° | 0,79° | 0,56° |

---

## 5. Lecciones aprendidas

**Unidades mezcladas en el mismo archivo.** Con `<compiler angle="degree"/>`,
MuJoCo convierte a radianes `range` de las articulaciones, pero **no**
`ctrlrange` de los actuadores, que siempre está en radianes. En `single_joint.xml`
(y en la primera pierna) funciona, pero el mismo archivo tiene `range="-90 90"` y
`ctrlrange="-1.5708 1.5708"`, y es fácil equivocarse. Desde el experimento 4 todo va en
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
experimento 7 supone la pierna en el aire; apoyada, empeora el error. El controlador
tiene que saber si el pie está en el suelo.

**Mirar los números antes de cambiar nada.** El pie "deslizaba", pero la
fuerza horizontal era el 30 % de la que hace falta para deslizar. El problema
era la geometría (la esfera soldada a la tibia), no la fricción. Subir la
fricción no lo habría arreglado.

**Fijar los criterios antes de ejecutar.** En el experimento 8 los criterios se
escribieron antes de la primera ejecución y no se relajaron cuando fallaron:
eso obligó a encontrar la causa real.

**Las ganancias de cada eje por separado no bastan.** El `kv` se calculó con
la inercia de cada eje, pero los ejes se acoplan: en ciertas posturas aparece
un modo de inercia mucho menor que cualquier diagonal. Con un integrador
explícito, eso basta para que la simulación se vuelva inestable. Integrador
`implicitfast` desde el experimento 9.

**`range` no es `ctrlrange`.** `range` es el tope mecánico de la articulación;
`ctrlrange` es hasta dónde puede llegar el mando. Una articulación que trabaja
en su tope (la rodilla recta) necesita margen en el mando para que la
compensación no se recorte.

**Comprobar los sentidos de los ejes en simulación.** Un eje al revés es el
error más común al montar un robot, y en el modelo es igual de fácil
equivocarse. El experimento 9 lo comprueba en cada ejecución.

**La fuerza del suelo actúa bajo el centro de masas, no en el tobillo.** Con un
pie fijo y la pelvis guiada da igual, pero con la pelvis libre el cuerpo puede
volcar sobre el tobillo, y el compensador tiene que poner la fuerza donde está
de verdad (el centro de presiones).

**Planificar en el espacio de tareas.** Interpolar posiciones de pelvis y pies
y resolver la IK en cada paso mantiene el pie de apoyo quieto. Interpolar
ángulos no lo garantiza.

**Un motor de posición se retrasa cuando se mueve.** Su amortiguamiento
(`−kv·q̇`) frena hacia velocidad cero, no hacia la velocidad deseada. Con una
trayectoria en movimiento hay que sumar el feedforward de velocidad
(`(kv/kp)·q̇_deseada`); si no, cada articulación va `(kv/kp)·q̇` por detrás.

**Comprobar las hipótesis antes de quedarse con un arreglo.** En el experimento 11
hubo dos explicaciones plausibles (fluencia del contacto, pie pivotando) que
las medidas descartaron. Mirar *qué* pie se movía y *con cuánta carga* llevó a
la causa real. Un arreglo que mejora un poco sin explicar el fallo no se deja.

**Comprobar los límites al planificar, no al simular.** La IK recorta en
silencio si el pie no llega. El experimento 11 mide el alcance de cada pose antes de
simular y para con un mensaje claro.

**Antes de mejorar el controlador, mirar el plan.** En el experimento 12 la
compensación dinámica parecía la respuesta obvia, y no lo era: la mitad del
tiempo se iba en pausas. Recortarlas bastó para cumplir el objetivo con menos
aceleración. Un controlador más sofisticado no arregla un plan que pide más
de lo necesario.

**Una mejora que no mejora el resultado se queda fuera del camino por
defecto.** La compensación dinámica es correcta en teoría y se comprobó (en
reposo da el peso y el ZMP exactos), pero en la marcha inclinaba más la
pelvis. Queda como opción para reproducir la comparación, no como defecto.

**Lo que estabiliza una marcha no siempre es el control.** En el experimento 13 no se
tocó el controlador. Reducir el balanceo lateral en el plan dio pasos un 50 %
más largos, menos inclinación y más alcance. Medir antes de elegir parámetros
(la tabla de rodilla mínima) evitó probar a ciegas.

**Comprobar una fórmula contra lo que hace el sistema.** Comparar el par previsto
con el real localizó en minutos el error de la compensación dinámica, tras tres
hipótesis fallidas: la fórmula estaba bien, y el reparto entre pies, mal.

**Medir con el tiempo de simulación.** Si la lógica usa el reloj del ordenador,
el movimiento cambia según la carga de la máquina. Con `data.time` es
reproducible.

---

## 6. Glosario

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
- **Centro de masas (CdM)**: punto medio de toda la masa del robot. Para no
  caerse, su proyección en el suelo tiene que caer dentro de los pies.
- **Centro de presiones**: punto del suelo donde actúa, en total, la fuerza de
  un pie. Si el robot está quieto, queda bajo el CdM.
- **Cinemática inversa (IK)**: calcular los ángulos de las articulaciones a
  partir de dónde se quiere el pie. La directa es al revés.
- **`freejoint`**: articulación de 6 GDL sin restricciones. Un cuerpo con
  `freejoint` solo se sostiene por sus contactos.
- **Marcha estática**: forma de andar en la que el CdM está siempre sobre un pie;
  el robot no se cae aunque se pare en cualquier momento. La marcha dinámica
  (la humana) deja caer el CdM hacia delante entre pasos.
- **Feedforward de velocidad**: sumar al objetivo de un motor de posición lo
  necesario para que su amortiguamiento siga la velocidad deseada en vez de
  frenar hacia cero.
- **Dinámica inversa**: calcular las fuerzas y torques que hacen falta para que
  el robot siga un movimiento dado (posiciones, velocidades y aceleraciones).
  En MuJoCo, `mj_rne`.
- **ZMP (punto de momento cero)**: punto del suelo donde tiene que actuar la
  fuerza total del suelo para que el robot haga su movimiento sin volcar.
  Quieto, coincide con la proyección del CdM; al acelerar, se aleja en sentido
  contrario a la aceleración. Si se sale de los pies, el robot vuelca.
- **Feedforward**: parte del control que se calcula de antemano con el modelo
  (aquí, el torque de gravedad), en lugar de reaccionar al error medido.
- **qfrc_bias**: fuerzas de gravedad, Coriolis y centrífugas que calcula MuJoCo
  en cada paso. Quieta, la pierna solo nota la gravedad.
- **REP-103**: convención de ejes y unidades de ROS.
