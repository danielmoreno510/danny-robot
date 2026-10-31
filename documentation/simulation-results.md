# Simulación en MuJoCo

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
| 3.1 | De pie 3 s, 2° o menos de inclinación y 5 mm o menos de caída del centro de masas | **Cumple:** 0,14° y 0,2 mm |
| 3.2 | Tabla por articulación en `results/` | **Hecho.** El paso del suelo a de pie se probó aparte, por búsqueda |
| 3.3 | Picos de 0,98 N·m a 7,4 V y 0,78 N·m a 6 V, y velocidades por debajo de las de vacío | **Cumple en lo medido.** Ver el aviso de la curva del motor |

## Pico por tarea

Par con 0,025 N·m de rozamiento añadido. La marcha va a 0,25 m/s, con pasos de 0,30 s y 12 % de doble apoyo a cada lado.

| Tarea | Par pico | Articulación | Velocidad pico | Tensión mínima |
|---|---:|---|---:|---:|
| Marcha | 0,75 N·m | `ankle_pitch` | 8,2 rad/s, rodilla | 6,9 V |
| Apoyo a un pie | 0,42 N·m | `hip_roll` | 2,2 rad/s | Sin límite útil |
| Levantarse desde cuclillas, sin manos | 0,23 N·m | `hip_pitch` | 0,9 rad/s | Sin límite útil |
| Cuclillas, bajando 60 mm | 0,21 N·m | `knee` | 1,0 rad/s | Sin límite útil |
| Boca abajo, elevar el tronco con las manos | 0,15 N·m | `elbow` | 0,8 rad/s | Sin límite útil |
| Boca arriba, sentarse | 0,10 N·m | `knee` | 0,8 rad/s | Sin límite útil |

**Lectura:** los pares son bajos porque Danny pesa 1,29 kg (E), y hay margen en todo menos en la marcha. Los picos de la
marcha salen de una trayectoria de medida, no optimizada, y una política los cambiará.

## Lo que hay que saber

- **Curva del motor.** Con los datos de `xgoduck_rl`, el modelo da 0,94 N·m de bloqueo a 7,4 V, frente a 1,47 N·m de ficha. Con
  esa curva, la marcha de 0,25 m/s usa el 92 % a 7,4 V, y pide el 118 % a 6 V: necesita unos 6,9 V. Es decir, a batería
  baja habría que andar más despacio o cortar antes. El criterio de 3.3 no ve esto porque mira par y velocidad por separado.
- **Tope de `hip_roll` cambiado.** Pasó de -3° a -15° hacia dentro, porque llevar el peso sobre un pie pide unos 11°. Los -3°
  venían del choque entre pies, que no ocurre con el otro pie apartado. Ahora se barre con la otra pierna abierta, y los
  pies deben protegerse con colisiones.
- **Tope de `hip_pitch` confirmado.** Se probaron -130° y -105°: el muslo toca el eslabón yaw y el servo de roll. Sigue en -100°.
- **Tobillo.** Con el pie plano, -30° limita la cuclilla a unos 60 mm de bajada, con el tronco inclinado 20°.
- **La mecánica permite levantarse.** En simulación directa, con 0,94 N·m por motor y colisiones simples, una búsqueda de
  entropía cruzada encontró cómo pasar de boca abajo a de pie en 9 s, y otra cómo darse la vuelta desde boca arriba.
  Con los brazos a los lados el robot se voltea sobre la cabeza: las manos deben ir delante de la cabeza, barriendo por el
  costado porque el hombro no pasa por detrás. Luego empujan, y las rodillas se recogen.
- **Pero esa secuencia es frágil.** Sin realimentación solo funciona en las condiciones exactas de la búsqueda: falla a 6 V, con
  otro rozamiento y con la masa a ±10 %, e incluso al repetirla con los objetivos redondeados. Una búsqueda que puntuaba varias
  condiciones no lo mejoró. Demuestra que es posible,
  no que sea fiable, y además satura varios motores durante 1 a 2,6 s. Una política con realimentación debe lograrlo en la etapa 4.
  La cadera no se rediseña por ahora.
- **En simulación directa** se sostienen `squat`, `single_support` y `rise_from_crouch`. `walk` cae sin realimentación.

## Qué no se ha comprobado

- Par y curva del servo real: los parámetros son de otro robot con este servo, sin confirmar en los nuestros.
- Reparto entre dos pies: es el de menor par, con contactos sin imponer, y solo se valida el centro de presiones en apoyo simple.
- Fricción dependiente de la carga, holgura de 0,5° y flexión de las piezas.
- Marcha en bucle cerrado: esta se calcula con dinámica inversa, sin caminar de verdad en simulación.
- Colisiones entre eslabones: ya hay cajas simples por eslabón en `analysis/danny_sim.py`, sin colisionador en los eslabones
  cortos de cadera, tobillo, cuello y hombro. Son aproximadas: la mecánica fina sale del barrido de Fusion.
- Darse la vuelta y levantarse no se han encadenado.
- Calor. Las cifras RMS de `results/joint_loads.json` son la base para estimarlo.
