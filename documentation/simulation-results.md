# Simulación en MuJoCo

Tras la [auditoría](design-audit.md) del 2 de noviembre de 2026, las pruebas de pie, tareas y visor usan el servo BAM de
`analysis/danny_servo.py`. Las búsquedas de levantarse aún no se han repetido con él.

Etapa 3, en curso. El MJCF exportado no se edita: `analysis/danny_sim.py` le añade suelo, base libre, suelas, actuadores y
los parámetros del servo. Cifras en `results/standing_check.json` y `results/joint_loads.json`.

## Cómo reproducirlo

```bash
.venv/bin/python analysis/check_standing.py
```

```bash
.venv/bin/python analysis/measure_joint_loads.py
```

Para ver el robot en el suelo, con gravedad y sus motores limitados:

```bash
.venv/bin/python analysis/view_simulation.py
```

Con una tarea, la repite en bucle: `squat`, `single_support`, `rise_from_crouch` y `walk` se simulan al momento, y `get_up_prone` y
`roll_over` son grabaciones de esa simulación:

```bash
.venv/bin/python analysis/view_simulation.py get_up_prone
```

Si el robot cae en el visor, el botón Reset o la tecla Retroceso lo devuelven al inicio. Para regenerar el modelo de entrenamiento:

```bash
.venv/bin/python analysis/export_training_model.py
```

Las búsquedas de levantarse tardan muchos minutos:

```bash
.venv/bin/python analysis/search_get_up.py prone --generations 45 --population 240 --seeds 3
```

```bash
.venv/bin/python analysis/search_get_up.py prone --verify
```

```bash
.venv/bin/python analysis/search_roll_over.py --generations 25 --population 160
```

Si el CAD cambia, antes hay que exportarlo con los comandos de [lower-body-design.md](lower-body-design.md).

## Criterios y estado

| Paso | Criterio | Estado |
|---|---|---|
| 3.1 | De pie 3 s, 2° o menos de inclinación y 5 mm o menos de caída del centro de masas | **Cumple desde `stand`:** 0,08° de 8,4 a 6 V, sin bajar. Falla la tarea en simulación directa: ver abajo |
| 3.2 | Tabla por articulación en `results/` | **Hecho.** El paso del suelo a de pie se probó aparte, por búsqueda |
| 3.3 | Picos de 0,61 N·m a 7,4 V y 0,49 N·m a 6 V, y velocidades por debajo de las de vacío | **Falla por margen:** marcha, 0,68 N·m en `ankle_pitch`, y a un pie, 0,51 N·m a 6 V. Criterio cambiado el 2 de noviembre: es el bloqueo de BAM, 0,912 y 0,739 N·m, entre 1,5, porque el anterior superaba el bloqueo |

## Pico por tarea

Par con el rozamiento del modelo BAM bajo esa carga, en cálculo cuasiestático. La marcha va a 0,25 m/s, con pasos de 100 mm cada 0,40 s y 12 % de doble apoyo a cada lado.

| Tarea | Par pico | Articulación | Velocidad pico | Tensión mínima |
|---|---:|---|---:|---:|
| Marcha | 0,68 N·m | `ankle_pitch` | 7,6 rad/s, rodilla | 5,9 V |
| Apoyo a un pie | 0,51 N·m | `hip_roll` | 2,2 rad/s | Sin límite útil |
| Levantarse desde cuclillas, sin manos | 0,28 N·m | `hip_pitch` | 0,9 rad/s | Sin límite útil |
| Cuclillas, bajando 60 mm | 0,26 N·m | `knee` | 1,0 rad/s | Sin límite útil |
| Boca abajo, elevar el tronco con las manos | 0,18 N·m | `elbow` | 0,8 rad/s | Sin límite útil |
| Boca arriba, sentarse | 0,11 N·m | `knee` | 0,8 rad/s | Sin límite útil |

**Lectura:** el modelo reparte los 100 g de tornillos y cables, y pesa 1,30 kg. Los picos de la
marcha salen de una trayectoria de medida, no optimizada, y una política los cambiará. Las dos filas tumbadas son solo orientativas:
sus trayectorias meten el cuerpo en el suelo.

## Lo que hay que saber

- **Curva del motor.** Con los datos de `xgoduck_rl` y el `max_pwm` de 0,97, el modelo da 0,912 N·m de bloqueo a 7,4 V y 0,739 a 6 V,
  frente a 1,47 N·m de ficha. La marcha necesita 5,9 V, por debajo del corte de 6,4 V, con poco margen. El pico anterior de 0,75 N·m
  y 6,9 V era un artefacto del tope de tobillo.
- **El servo es blando.** Con la ganancia de firmware 5 del XGO-Duck da 1,03 N·m/rad a 7,4 V. Se aguanta de pie gracias al rozamiento bajo
  carga, pero no sigue las tareas: cuclillas y apoyo a un pie caen. Con 1,30 kg, tampoco con ganancia 10 (`results/standing_check.json`). Sin realimentación, estas tareas no
  dicen si el robot puede: la prueba válida es la siguiente. La ganancia real se fija en la 5.1.
- **Puede dar pasos.** `analysis/check_walk_feasibility.py`, con ganancia 5, la física del entrenamiento y 20 ms de retardo, da 4 apoyos alternos, zancada
  media de 98 mm, 15 mm de deslizamiento como máximo y 0,46 m de avance antes de caer a 1,8 s (`results/walk_feasibility.json`). Es una prueba de
  existencia con realimentación mínima: no demuestra una marcha sostenida, ni aleatorización, ni a 6 V.
- **Tope de `hip_roll` cambiado.** Pasó de -3° a -15° hacia dentro, porque llevar el peso sobre un pie pide unos 11°. Los -3°
  venían del choque entre pies, que no ocurre con el otro pie apartado. Ahora se barre con la otra pierna abierta, y los
  pies deben protegerse con colisiones.
- **Tope de `hip_pitch` confirmado.** Se probaron -130° y -105°: el muslo toca el eslabón yaw y el servo de roll. Sigue en -100°.
- **Tobillo ampliado a -45°.** Con -30°, `stand` quedaba a 3,5° del tope, y la marcha pasaba el 38 % del tiempo en él. El barrido del CAD sale limpio.
- **Levantarse, pendiente de repetir.** Con motores más rígidos y 0,94 N·m, antes del tobillo a -45°, una búsqueda de
  entropía cruzada encontró cómo pasar de boca abajo a de pie en 9 s, y otra cómo darse la vuelta desde boca arriba.
  Con los brazos a los lados el robot se voltea sobre la cabeza: las manos deben ir delante de la cabeza, barriendo por el
  costado porque el hombro no pasa por detrás. Luego empujan, y las rodillas se recogen.
- **Esa secuencia no vale con BAM.** Con el servo blando no se levanta, según la auditoría. Sin realimentación solo funcionaba en las condiciones exactas de la búsqueda: falla a 6 V, con
  otro rozamiento y con la masa a ±10 %, e incluso al repetirla con los objetivos redondeados. Una búsqueda que puntuaba varias
  condiciones no lo mejoró. Demuestra que es posible,
  no que sea fiable, y además satura varios motores durante 1 a 2,6 s. Una política con realimentación debe lograrlo en la etapa 4.
  La cadera no se rediseña por ahora.
- **En simulación directa**, sin realimentación, ninguna tarea se sostiene con el servo BAM.

## Qué no se ha comprobado

- Par y curva del servo real: los parámetros son de otro robot con este servo, sin confirmar en los nuestros.
- Reparto entre dos pies: es el de menor par, con contactos sin imponer, y solo se valida el centro de presiones en apoyo simple.
- La ganancia real del firmware, holgura de 0,5° y flexión de las piezas.
- Marcha en bucle cerrado: esta se calcula con dinámica inversa, sin caminar de verdad en simulación.
- Colisiones entre eslabones: ya hay cajas simples por eslabón en `analysis/danny_sim.py`, sin colisionador en los eslabones
  cortos de cadera, tobillo, cuello y hombro. Son aproximadas: la mecánica fina sale del barrido de Fusion.
- Darse la vuelta y levantarse no se han encadenado.
- Calor. Las cifras RMS de `results/joint_loads.json` son la base para estimarlo.
