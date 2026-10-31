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

Para ver el robot en el suelo, con gravedad y sus actuadores:

```bash
.venv/bin/python analysis/view_simulation.py
```

Si el CAD cambia, antes hay que exportarlo con los comandos de [lower-body-design.md](lower-body-design.md).

## Criterios y estado

| Paso | Criterio | Estado |
|---|---|---|
| 3.1 | De pie 3 s, 2° o menos de inclinación y 5 mm o menos de caída del centro de masas | **Cumple:** 0,14° y 0,2 mm |
| 3.2 | Tabla por articulación en `results/` | **Hecho**, salvo el paso del suelo a cuclillas |
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
- **Levantarse del suelo sigue sin resolver.** Con cadera a -100° y tobillo a -30°, las manos no llegan al suelo con los pies
  planos, así que no hay una postura sencilla entre el suelo y las cuclillas. Lo medido son los tramos de antes y después.
  Es el mayor riesgo para R2. Se prueba primero con entrenamiento en la etapa 4, y si falla hay que rediseñar la cadera.

## Qué no se ha comprobado

- Par y curva del servo real: los parámetros son de otro robot con este servo, sin confirmar en los nuestros.
- Reparto entre dos pies: es el de menor par, con contactos sin imponer, y solo se valida el centro de presiones en apoyo simple.
- Fricción dependiente de la carga, holgura de 0,5° y flexión de las piezas.
- Marcha en bucle cerrado: esta se calcula con dinámica inversa, sin caminar de verdad en simulación.
- Colisiones entre eslabones: el modelo solo tiene suelas y suelo. Hay que añadirlas para entrenar.
- Calor. Las cifras RMS de `results/joint_loads.json` son la base para estimarlo.
