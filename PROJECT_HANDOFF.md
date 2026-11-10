# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.

**Fecha:** 4 de noviembre de 2026. **Etapa actual:** 3, MuJoCo, con 2.2, 2.3 y 2.4 abiertas por lo físico. La etapa 1 está
cerrada.
**Siguiente paso:** seguir el [plan de la etapa 4](#plan-de-la-etapa-4-fases) por fases. Las fases 0 a 2 están aprobadas: la Fase 2,
movimiento completo, con `fase2_f_rodilla/model_8995.pt`, fusionada en `main` el 4 de noviembre (PR 1), con su ONNX en `policies/` y vídeos en
`results/videos/`. La fase 3, agacharse quieto, está aprobada con `fase3_agacharse/model_11994.pt`, usado solo parado y con ONNX en `policies/` (ver el plan); caminar agachado queda descartado por ahora. `marcha_reloj_1000` ya caminó hacia delante y giró con el modelo
anterior (ver [Entrenamiento en simulación](#entrenamiento-en-simulación-etapa-4-sin-terminar)). El código de `training/` está en la
rama `entrenamiento-danny`, que se actualiza con `origin/main` antes de cada fase. `main` está protegida: los colaboradores entran por pull
request aprobado por el usuario, que es quien cambia el diseño. Levantarse es posible en simulación, pero aún frágil. El robot completo ya está en CAD:
[piernas](documentation/lower-body-design.md), [tronco](documentation/upper-body-design.md) y
[paneles](documentation/panels-and-protection.md).

**Decisión del 4 de noviembre, fuentes de movimiento humano (1.2):** CMU y 100STYLE, que permiten publicar lo derivado
([sección 6](documentation/research-and-requirements.md)). La etapa 1 queda cerrada. LAFAN1 solo en local si hiciera falta.

**Decisión del 4 de noviembre, tensión mínima:** Danny deja de caminar a 6,6 V (3,3 V por celda) y el programa corta a 6,4 V. Se sigue
entrenando de 6 a 8,4 V, pero la saturación de la Fase 2 se mide a 6,6, 7,4 y 8,4 V fijos ([guía](documentation/training-guide.md), 4.2).
La prueba de 5 minutos usa los empujones de la guía, ±0,12 m/s. El tercer intento (`fase2_e_doble`, rama `entrenamiento-danny`)
se midió con ±0,20 m/s, más fuerte que la guía. Con ±0,12 m/s y a 6,6 V, `fase2_f_rodilla` aprueba la Fase 2. Que el paro a 6,6 V mantenga los 20 minutos de R5 no está medido.

**Auditoría del 2 de noviembre:** [design-audit.md](documentation/design-audit.md). Corregidos los 6 bloqueos de montaje, el tobillo a -45°,
la pose de reposo y la simulación con BAM. Quedan puntos importantes: no imprimas piezas definitivas ni compres rodamientos de 10 × 15 × 3.
El modelo cambió, así que hay que reentrenar. Los pasos de verificación están en `check_assembly.py` y `check_standing.py`, y este último sale con
código 1 porque, sin política, las tareas fallan con ganancia 5.

**Segunda auditoría:** al final de [design-audit.md](documentation/design-audit.md) están los fallos, la tornillería, las medidas C1 a C10 y la primera tanda en PLA.
Corregido lo que no depende del servo. Lo primero con los servos es medir C1 a C5, antes de meter ningún tornillo.

**Recibido el 10 de noviembre:** 3 servos HD-1910, la placa FE-URT-2, impresora 3D con filamento, calibrador digital y
báscula de cocina. **Falta por llegar:** la fuente de 3-12 V y 5 A (ArmDojo), necesaria para mover los servos. Ahora toca: medir conectores, muesca y rosca del servo, comprobar encaje y pesar piezas (2.2 y
2.3), y empezar el banco (5.1). Falta el brazo de carga, que se imprime, y los programas de medida. No se compra el resto
hasta pasar 5.1 y 5.2. Claude modela en Fusion por scripts, con `DannyBridge`. **Primeras medidas del servo (10 de noviembre):**
caja de 33,9 × 19,9 × 22,8 mm, dentro de la holgura del CAD; cada bocina sobresale unos 3,4 mm (el CAD supone 3,0); pesa 23 g (el CAD supone 21).
Faltan C2, C5, C6, C8 y C9, y saber dónde se tomaron seis cotas de las fotos ([design-audit.md](documentation/design-audit.md)).

**Fase 6, gestos de pie: criterio aprobado por el dueño el 10 de noviembre, antes de programar.** Un primer intento de adaptar el andar
de 100STYLE y CMU se deshizo a petición del dueño: las rodillas se veían antinaturales por las proporciones de Danny (pie largo, pierna
corta) y por una pierna rara en la grabación de 100STYLE. Se empieza por **gestos de pie**, con un clip: CMU **141_16 "Wave Hello"**, y
después 143_25, 141_21 y 143_35. **Método:** piernas y pelvis fijas en la pose `stand` de la fase 2; de la persona se copian brazos,
cintura y cuello, con cada tramo del brazo de Danny en la dirección del humano y con su propia longitud. **Pasa si cumple todo:**
(1) vídeo de la persona junto a Danny, aprobado por el dueño; (2) mano y codo a 1 cm o menos de su objetivo de mediana, y a 2 cm o menos
en el 95 % del tiempo (estimación, con un brazo de unos 17 cm); (3) ninguna articulación del brazo más del 5 % del tiempo en su tope;
(4) ninguna articulación por encima del 80 % de la velocidad del servo en vacío (8,2 rad/s a 6,6 V); (5) el brazo no atraviesa tronco,
cabeza ni piernas en ningún fotograma; (6) piernas y pelvis exactamente en la pose de pie. Es solo la referencia: seguirla con física
se entrena después, con su propio criterio.

**Primer clip, 141_16 "Wave Hello" (10 de noviembre).** `analysis/retarget_gesture.py` adapta y mide (`results/gesture_cmu_141_16*.json`),
y `analysis/record_gesture_video.py` graba la persona junto a Danny (`results/videos/gesture_cmu_141_16*.mp4`). Datos de CMU en
`external/cmu/` (fuera de git): "The data used in this project was obtained from mocap.cs.cmu.edu. The database was created with funding
from NSF EIA-0196217." Al ritmo de la persona (2,5 s) pasan los puntos 2, 3, 5 y 6 (codo y mano a 1,1 mm de mediana y 2,0 mm de p95; ningún
tope; 1,7 mm de holgura mínima entre brazo y cuerpo; piernas fijas), pero **no el 4**: `right_shoulder_yaw` llega a 14,1 rad/s. **No es
un fallo de la adaptación:** el antebrazo de la persona gira hasta 17,5 rad/s (13,5 de p95), más que el servo en vacío (10,3). Debió
calcularse antes de fijar el criterio. Con `--slowdown 1.8` (4,5 s) pasan los cinco puntos medibles (8,11 rad/s como máximo). **El punto 5
se mide con las mallas del CAD:** las cajas de colisión simplificadas del entrenamiento se tocan 2,6 mm en el hombro derecho aunque las
piezas no, así que al entrenar habrá contacto entre brazo y tronco si se activa el choque entre piezas.

**Decisión del dueño (10 de noviembre): aprobados dos gestos de 141_16.**
- **"Saludar":** `cmu_141_16_lento1.8`, con una mano y 1,8 veces más lento que la persona. Cumple los seis puntos.
- **"Pedir ayuda" o "nervioso":** con las dos manos y al ritmo de la persona, que al dueño también le gustó. Al ritmo humano y con toda su
  amplitud no cumple el punto 4 (14,1 rad/s) y el servo real no llegaría, así que se hizo `cmu_141_16_dos_manos_amplitud0.5`: los dos brazos
  copian el derecho de la persona, en espejo (`--both-hands`), y la agitación rápida se reduce a la mitad sin cambiar cuánto suben las
  manos (`--amplitude 0.5`). Esto último lo hace `shrink_arms`, que suaviza con una gaussiana de 0,1 s. Cumple los puntos 3 a 6 (7,94 rad/s,
  3,2 mm de holgura). El punto 2 vale 0 por construcción, porque mide contra el brazo ya reducido; antes de reducir, el error era el del saludo
  (1-2 mm). **Aprobado por el dueño el 10 de noviembre**, tras ver el vídeo (`results/videos/gesture_cmu_141_16_dos_manos_amplitud0.5.mp4`).
- La versión al ritmo humano con una mano (`cmu_141_16`) queda solo como referencia: no cumple el punto 4.

**Más gestos (10 de noviembre, trabajando solo mientras el dueño estaba fuera; vídeos pendientes de su aprobación).** Se probaron 13 clips
más de CMU: 143_25, 141_21, 143_35, 141_18, 141_20, 141_22, 141_23, 141_24, 141_27, 143_33, 13_26, 18_08 y 19_08. Al principio todos fallaban
la velocidad, y en los tranquilos el robot iba de 3 a 5 veces más rápido que la persona. Ejemplos: conversación 18_08, 18,5 rad/s frente a
6,8; dirigir el tráfico 13_26, 50,5 frente a 10,8. El motivo es que el giro del hombro de Danny va a lo largo del brazo, y con el brazo casi
recto el cálculo lo hacía girar sin mover la mano. Tres cambios en `danny_gestures.py`, todos opcionales, para que los gestos aprobados de
141_16 se reproduzcan igual (comprobado: 0,000° de diferencia):
- `--speed-cap`: ninguna articulación del brazo cambia más deprisa que el 97 % del límite del criterio. Si la persona va más rápido, Danny se
  queda atrás, y el punto 2 lo mide.
- `--body-margin`: el brazo se aparta del cuerpo cuando una pieza queda a menos de ese margen. Si al terminar las 30 iteraciones sigue
  tocando, se hacen hasta 90 más (en 18_08 hacían falta unas 40).
- `--continuity`, hacia la postura anterior: no quedó en ningún gesto final.

Casi todos los roces eran del antebrazo contra el muslo o la cadera, con el brazo colgando: el brazo de Danny es largo en proporción.

**Vibración (vista por el dueño el 10 de noviembre).** El dueño vio el brazo derecho vibrar al principio de 143_25 y en 141_27. Medida como
la segunda diferencia de cada articulación del brazo, era de hasta 7-8° en los cinco gestos nuevos. La causa: el tope de velocidad hace que
el giro del hombro choque con él en un sentido y en el otro en fotogramas seguidos, en posturas en las que ese giro apenas mueve la mano.
La continuidad (1e-2) la bajaba solo a 3-4°, y empeoraba 143_25. **Arreglo: `--smooth 0.05`**, un suavizado gaussiano de 0,05 s de brazos,
cintura y cuello al final. Deja la vibración en 0,7-1,4°. Desde entonces, el punto 2 se mide sobre la trayectoria que se entrega (50 Hz, tras
remuestrear y suavizar), frente al objetivo del fotograma de CMU más cercano. Antes se medía en los fotogramas del cálculo. Por eso cambian
las cifras de los gestos aprobados de 141_16, aunque su trayectoria es la misma (0,000°): el saludo lento da 1,1 / 2,8 mm, y el de las dos
manos 0,3 / 1,7 mm, que antes daba 0 por construcción. Los dos aprobados no tienen suavizado y vibran 2,4° y 3,3°. **Las piernas no se mueven
en ningún gesto, por el criterio:** el dueño lo notó en 141_23 y 141_27, donde la persona da un paso o se inclina.

**Cumplen los puntos 2 a 6** (`results/gesture_<nombre>.json`, vídeos en `results/videos/gesture_<nombre>.mp4`). Todos con `--speed-cap` y
`--smooth 0.05`. **Los cinco, aprobados por el dueño el 10 de noviembre**: 18_08 y 19_08 al verlos, y 141_27, 141_23 y 143_25 tras corregir la vibración.

| Gesto | Nombre | Ajustes | Duración | Codo y mano, mediana / p95 | Tope | Velocidad máx. | Holgura mín. | Vibración máx. |
|---|---|---|---|---|---|---|---|---|
| Conversación, explicar con las manos | `cmu_18_08` | margen 3 mm | 17,4 s | 1,9 / 6,7 mm | 0 % | 5,78 rad/s | 1,4 mm | 0,74° |
| "Mope" (abatido) | `cmu_141_27` | margen 5 mm | 4,6 s | 2,8 / 12,2 mm | 0 % | 3,41 | 3,2 mm | 0,76° |
| Dar la mano | `cmu_141_23` | margen 5 mm | 2,1 s | 4,2 / 15,3 mm | 0 % | 6,33 | 3,2 mm | 0,76° |
| Conversación, la otra persona | `cmu_19_08_lento1.3` | margen 3 mm, × 1,3 más lento | 22,6 s | 2,8 / 15,1 mm | 0,2 % | 7,94 | 0,5 mm | 1,36° |
| Saludar con el brazo alto | `cmu_143_25_lento1.8` | margen 5 mm, × 1,8 más lento | 9,9 s | 3,3 / 16,9 mm | 3,5 % | 7,80 | 3,2 mm | 1,40° |

**Gestos que se quedaron cerca, con margen a los topes (10 de noviembre).** Hay una opción nueva, `--limit-margin 3`: el brazo se queda a
3° de cada tope, para no apoyar el servo en el tope mecánico, que es lo que busca el punto 3. **Con este margen, el punto 3 sale 0 % por
construcción** (cuenta como tope a menos de 2°), así que la prueba que decide es el punto 2: si la persona pide más brazo del que Danny puede
dar, el error sube. Probado con `--speed-cap --body-margin 0.005 --smooth 0.05 --limit-margin 3`:

| Gesto | Nombre | Codo y mano, mediana / p95 | Velocidad máx. | Holgura mín. | Vibración máx. | Resultado |
|---|---|---|---|---|---|---|
| Encoger los hombros | `cmu_141_21` | 2,7 / 12,6 mm | 5,40 rad/s | 2,9 mm | 0,89° | Cumple; aprobado por el dueño |
| Chocar los cinco | `cmu_141_22` | 5,6 / 18,2 mm | 7,35 | 3,2 mm | 1,00° | Cumple por poco; aprobado por el dueño |
| Peek A Boo | `cmu_143_33` | 5,4 / 16,2 mm | 7,82 | 2,5 mm | 0,94° | Cumple; aprobado por el dueño |
| Peek a Boo | `cmu_141_18` | 4,1 / 22,8 mm | | | | No: error |
| Macarena | `cmu_143_35` | 5,8 / 29,8 mm | | | | No: error |
| Dirigir el tráfico | `cmu_13_26` | 4,1 / 33,1 mm | | | | No: error |
| Around the world | `cmu_141_24` | 6,1 / 39,7 mm | | | | No: error |
| Esperar | `cmu_141_20` | 6,1 / 36,6 mm | | | | No: error |

En los que fallan, Danny se queda a varios centímetros de la postura de la persona: el hombro no le da, y el gesto sería otro. Sus informes
(`results/gesture_cmu_<clip>.json`) quedan con estos ajustes.

**Bailes de pie (10 de noviembre).** Se usan los mismos ajustes que con el margen (`--speed-cap --body-margin 0.005 --smooth 0.05
--limit-margin 3`). Las piernas siguen fijas: de los bailes solo se copian brazos, cintura y cuello. Cumplen los puntos 2 a 6, y **el
dueño los aprobó el 10 de noviembre**. Hizo notar que en ninguno se mueven los pies: es lo siguiente, los movimientos con piernas.

| Baile | Nombre | Duración | Codo y mano, mediana / p95 | Velocidad máx. (rad/s) | Holgura mín. | Vibración máx. |
|---|---|---|---|---|---|---|
| Chicken Dance | `cmu_143_34` | 6,5 s | 6,4 / 18,6 mm | 7,57 | 2,6 mm | 0,92° |
| Chicken Dance, pareja A | `cmu_18_15` | 12,8 s | 5,0 / 15,5 mm | 7,34 | 2,2 mm | 1,17° |
| Chicken Dance, pareja A, otra toma | `cmu_20_01` | 10,7 s | 5,7 / 16,8 mm | 7,07 | 2,7 mm | 1,32° |
| Chicken Dance, pareja B, otra toma | `cmu_21_01` | 10,7 s | 7,0 / 19,6 mm | 7,23 | 3,2 mm | 1,31° |
| Brazos en alto y giro del tronco | `cmu_05_12` | 11,3 s | 2,3 / 13,0 mm | 7,92 | 2,2 mm | 1,26° |

**No cumplen:**
- La pareja B de 18_15 (19_15): p95 de 23,5 mm, y de 22,6 mm aun yendo 1,3 veces más lento.
- Brazos en alto con arabesque (49_09): el codo derecho se queda a unos 3,6 cm de su sitio todo el rato, porque el hombro no da para
  tanto.
- Twist (141_12) y pirueta (05_02): los mueven la cadera y el giro del cuerpo, y Danny tiene la pelvis fija (p95 de 62 y 56 mm).

Se corrigió `resample`: con clips como 20_01 y 21_01, el último instante se pasaba del final por redondeo y daba error. No cambia los clips
anteriores, en los que nunca se pasaba.

**Movimientos con piernas: criterio aprobado por el dueño el 10 de noviembre, antes de programar.** **Método:** se parte de la pose `stand`
de Danny, con las rodillas dobladas 47°, y a cada articulación de la pierna se le suma lo que la persona cambia respecto a su propia postura de
pie. Se copia el cambio, no la postura: el primer intento de andar copió la forma de la pierna humana, casi recta, y las rodillas se veían mal.
El cuerpo sube o baja para que el pie apoyado toque el suelo. Se empieza **sin desplazarse**: los cuatro Chicken Dance con sus flexiones de
rodilla y pasar el peso de un pie a otro; andar queda para el final. **Pasa si cumple:** los puntos 1 a 5 de los gestos (el 6 ya no aplica,
porque las piernas se mueven), y además (7) el pie apoyado no resbala más de 5 mm mientras está en el suelo (estimación); (8) el pie no
atraviesa el suelo y ninguna otra pieza de la pierna lo toca; (9) el centro de masas cae dentro de la huella de los pies apoyados el 95 % del
tiempo o más.

**Movimientos con piernas: resultado (10-11 de noviembre).** `--legs` en `retarget_gesture.py` usa `LegRetarget` (`danny_gestures.py`).
Copiar solo el cambio de ángulos no bastaba en 143_34: al agacharse, los pies se separaban 2,5 cm y se inclinaban, porque Danny tiene otras
longitudes. Así quedó el método:
- La postura sale del cambio de la persona.
- Una cinemática inversa de piernas y altura de pelvis mantiene cada pie apoyado fijo y plano donde se apoyó.
- Lleva el centro de masas dentro de la huella, a 8 mm del borde.
- Levanta el pie en el aire a 5 mm o más por cada esquina de la suela.
- Respeta el tope de velocidad del servo.
- Las articulaciones que llegan a su rango dejan de contar, para que la pelvis suba en vez de hundirse los pies.
- Los apoyos y levantadas de menos de 0,1 s se tratan como ruido de la captura.

Fallos encontrados y corregidos por el camino: medir el resbale con el mismo umbral que la altura del pie en el aire, un tope de 0,3 m/s en la
pelvis que despegaba los pies, y la pelvis congelada al sacarla del cálculo por el tope de velocidad. Ajustes: `--speed-cap --body-margin 0.005
--smooth 0.05 --limit-margin 3 --legs`.

| Baile | Nombre | Ajustes | Codo y mano p95 | Resbale máx. | Pie más bajo | Otra pieza más baja | Equilibrio | Puntos |
|---|---|---|---|---|---|---|---|---|
| Chicken Dance (sin los pasos finales) | `cmu_143_34_hasta5.4_piernas_lento1.4` | hasta 5,4 s, × 1,4 más lento | 17,5 mm | 0,3 mm | -0,2 mm | 1,8 mm | 98,4 % | 9/9 |
| Chicken Dance, pareja A, otra toma | `cmu_20_01_piernas` | ritmo de la persona | 16,8 mm | 0,3 mm | -0,2 mm | 1,8 mm | 100,0 % | 9/9 |
| Chicken Dance, pareja A | `cmu_18_15_piernas` | ritmo de la persona | 15,4 mm | 0,9 mm | -0,8 mm | 1,2 mm | 100,0 % | 8/9 |
| Chicken Dance, pareja B, otra toma | `cmu_21_01_piernas` | ritmo de la persona | 19,7 mm | 5,3 mm | -7,8 mm | -5,9 mm | 99,8 % | 6/9 |

- **143_34:** al ritmo de la persona no cumple, porque la rodilla va al tope de velocidad al levantarse de la sentadilla, el pie se inclina
  2-3° y entra 1,2 mm. Además, desde los 5,5 s la persona deja de bailar y da pasos girando, y eso se queda para cuando toque andar, así que el
  clip se corta en 5,4 s (`--end`).
- **18_15:** falla por un fotograma suelto en el que el pie entra 0,8 mm.
- **21_01:** el pie ya está desplazado 5,3 mm desde el primer fotograma, y la cadera derecha llega a su tope de giro.
- 143_34 y 20_01 cumplen los nueve puntos medibles; vídeos en `results/videos/gesture_cmu_143_34_hasta5.4_piernas_lento1.4.mp4` y
  `gesture_cmu_20_01_piernas.mp4`. **Aprobados por el dueño el 11 de noviembre.**

**Antes del margen, no cumplían,** con `--speed-cap` y margen de 5 mm. Por topes del brazo, que el ritmo no arregla, porque la persona lleva el brazo más atrás o
más arriba de lo que permite el hombro de Danny: encoger los hombros 141_21 (5,3 % en `right_shoulder_pitch` a +45°), chocar los cinco
141_22 (6,0 %), Peek a Boo 141_18 y 143_33 (49 % y 25 %), Macarena 143_35 (18 %: codo a -130° y hombro a +45°, con las manos detrás de la
cabeza) y dirigir el tráfico 13_26. Por quedarse atrás: esperar 141_20 y 141_24, que además roza o toca topes. **La holgura de 0,5 mm de 19_08
es justa:** pasa el criterio, pero en el robot real deja poco margen.

## Cómo empezar

Lee en este orden:

1. Este archivo.
2. [documentation/HANDBOOK.md](documentation/HANDBOOK.md): cómo se hace cada etapa.
3. [CLAUDE.md](CLAUDE.md): reglas de trabajo.
4. [ROADMAP.md](ROADMAP.md): plan y estado.
5. [documentation/research-and-requirements.md](documentation/research-and-requirements.md):
   decisiones, requisitos, referencias y actuadores.
6. [documentation/architecture-and-bom.md](documentation/architecture-and-bom.md): articulaciones,
   medidas, piezas, precios y riesgos.

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
.venv/bin/python analysis/estimate_robot_budget.py
```

## Situación

El 31 de octubre de 2026 el usuario reinició el proyecto: los modelos de 70 cm y la propuesta de 40 cm con
STS3215 quedan en la etiqueta `archivo-danny-70cm`. Sirve de consulta, no como base. Ya no son válidos
sus requisitos, sus 31 GDL ni Isaac Lab.

Nueva dirección: humanoide con aspecto de Tesla Optimus en pequeño, hasta 1 500 €. Del XGO-Duck se toman el
servo y el método, no la forma. Altura aprobada: 40 cm, que da esas proporciones a cambio de menos margen de
par. Prioridad: caminar natural y levantarse. Saltar, correr y manipular se intentan si el diseño lo
permite. Entrenamiento con mjlab en la RTX 3080 Ti.

**Candidato a validar:** todo HD-1910, porque el STS3215 pesa más y es más lento. Los picos superan el par
nominal. El usuario decidió diseñar antes de comprar, así que el calor se medirá en la etapa 5.1, y hasta
entonces el diseño usa datos de ficha.

## Entrenamiento en simulación (etapa 4, sin terminar)

Código en `training/`, rama `entrenamiento-danny`. Tarea `Mjlab-Velocity-Flat-Danny`, PPO de RSL-RL. Corridas con
4096 entornos. Todas las evaluaciones son deterministas, sin empujones, 256 entornos y 20 s (`scripts/evaluate_walk.py`).

- **Lo que funciona:** Danny se mantiene de pie 20 s sin caerse (`results/training_walk_*.json`). La prueba de pie
  sin acción da centro de masas a 0,227 m (`results/training_stand_measurement.json`).
- **Lo que no funcionó, sin reloj de marcha:** no caminaba. El error de velocidad es 0,155 m/s en `rewardw_300` (escala de acción 0,25) y
  0,155 m/s en `actionscale_300` (escala 0,5), frente a 0,16 m/s de quedarse quieto. Mejorar la escala de acción no ayudó.
  Esas corridas usan las recompensas de mjlab, con las que quedarse quieto da el 91 % de la recompensa de velocidad.
- **Escalas de la guía** (`marcha_guia_3000`, parada en la iteración 1000): cae en 0,40 s, error 0,54 m/s
  (`results/training_walk_marcha_guia_1000_sin_empujones.json`). Todas las terminaciones son por caída (más de 70°).
  El punto de control sin entrenar se mantiene 17,4 s, así que lo que tumba a Danny es lo aprendido: la acción cruda
  media sube de 0,05 (iteración 0) a 0,63 (iteración 1000), con `ACTION_SCALE` 1,0 y sin recorte de acciones. El ruido
  inicial de la política (desviación 1,0) lo tumba en los primeros episodios. Cambiar la escala de acción al evaluar no lo
  arregla: 0,38 s con 1,0 y 0,5, 0,46 s con 0,25.
- **Cambios propios sobre la guía** (primer intento, sin base en la guía): penalización de la magnitud de la acción
  (`mdp.action_l2`, peso −0,05) y desviación inicial de la política 0,5. Con peso −0,01, 300 iteraciones: 0,66 s. Con
  −0,05, 300 iteraciones (`penalizacion_accion_005_300`): de pie el 97,7 % de los episodios, error 0,155 m/s, igual que
  quieto. Altura máxima de suela en 20 s: 40 mm de media (`results/training_peak_height_penalizacion_accion_005_300.json`,
  máximo de todo el episodio, no por paso).
- **Trampa de los pies** (`scripts/measure_gait.py`, orden fija de 0,25 m/s): con esa configuración Danny se queda sobre
  el pie izquierdo con el derecho a 20–25 mm, sin apoyos y sin avanzar (0,0007 m/s,
  `results/training_gait_penalizacion_accion_005_300.json`). El `air_time` de mjlab paga mientras el pie está en el aire, y
  `foot_swing_height` (−5) castiga cada aterrizaje, así que no apoyar nunca le sale a cuenta. Se probaron dos arreglos,
  300 iteraciones cada uno:
  - `aterrizaje_300`, `air_time` pagado al aterrizar (`mdp.feet_air_time_at_landing`, mismo total por paso que mjlab):
    1,37 apoyos por pie y segundo, pero de un solo pie, sin avanzar (`results/training_gait_aterrizaje_300.json`). Al final
    del entrenamiento: episodio 866 pasos, `peak_height_mean` 23,6 mm y `air_time` 0,80, que ya cumple la comprobación
    de las 1000.
  - `pasos_alternos_300`, pagado solo si aterriza el pie contrario al último: vuelve a quedarse sobre un pie, sin apoyos
    (`results/training_gait_pasos_alternos_300.json`), con `peak_height_mean` 12,8 mm. Se descartó.
  Ningún término premia el pie levantado: es una postura neutra. Lo que falla es que no aprende a avanzar en 300
  iteraciones. xgoduck_rl camina con los mismos valores (`tasks/__init__.py` de su repositorio), pero entrena hasta 50 000.
- **`marcha_aterrizaje_3000`**, la configuración de `aterrizaje_300`, parada en la iteración 1694 para dar paso a la
  siguiente. Pasó la comprobación de la guía a las 1000 (log de entrenamiento: episodio
  unos 970 pasos, `peak_height_mean` 23 mm, `air_time` 1,1). Aun así no camina: de pie el 100 %, error 0,158 m/s
  (`results/training_walk_marcha_aterrizaje_1000.json`) y, con orden fija de 0,25 m/s, avance de 0,0003 m/s, apoyo a un
  pie el 90 % del tiempo y ningún apoyo alterno (`results/training_gait_marcha_aterrizaje_1000.json`): sigue pisando con
  un solo pie. Esas dos medidas leen la velocidad en el origen de `base_link`.
- **Velocidad en el centro de masas:** el origen de `base_link` está en el suelo, a unos 23 cm del centro de masas.
  `track_linear_velocity` se sustituye por `mdp.track_com_linear_velocity`, que lee `root_com_lin_vel_b` (existe en
  mjlab 1.3.0). `evaluate_walk.py` y `measure_gait.py` miden también ahí, así que sus cifras no se comparan directamente
  con las anteriores. Hipótesis de la guía, sin confirmar.
- **`marcha_com_3000`** (parada en la iteración 1402 por el cambio de modelo de la auditoría), todos los valores de la guía más los tres cambios propios (`action_l2`, desviación
  inicial 0,5 y `air_time` al aterrizar) y la velocidad en el centro de masas. Comprobación de la guía a las 1000; criterio
  a las 3000 con `evaluate_walk.py` y `measure_gait.py`. Pasó la comprobación de la guía a las 1000 (log: episodio 923 pasos,
  `peak_height_mean` 27,8 mm, `air_time` 1,04, `track_linear_velocity` 0,89), pero tampoco camina: de pie el 100 %, error
  0,155 m/s (`results/training_walk_marcha_com_1000.json`); con orden fija de 0,25 m/s avanza 0,0003 m/s, con un solo pie
  apoyado el 89 % del tiempo y 0,2 % de apoyos alternos (`results/training_gait_marcha_com_1000.json`). En `model_800` se
  apoya en el izquierdo y sube y baja el derecho unas 3 veces por segundo (medida aparte, sin script en el repositorio).
- **Modelo de la auditoría (2 de noviembre):** `danny_training.xml` nuevo, tobillo de -45° a 40° (`stand` a -26,5°, 14,3° del
  límite blando de -40,75°), brazos en reposo a 8° de hombro y -17,2° de codo, e IMU en el sitio `imu` de `torso_1`. Los puntos de
  control anteriores no valen para este modelo.
- **Aterrizaje repetido:** se probó una penalización de -2 (`mdp.repeated_foot_landing`, ya quitada) contra el atajo de
  subir y bajar un solo pie. No lo arregló (ver `marcha_pies_kp5`).
- **`marcha_pies_kp5`** (modelo de la auditoría, `KP_FW` 5, aterrizaje repetido -2, parada a las 1000): no aprueba
  (`results/training_gait_marcha_pies_kp5_1000.json`). De pie el 100 %, pero avanza 0,001 m/s, solo el 3,6 % de los apoyos
  alternan y el pie izquierdo hace el 2 % de los apoyos. La penalización de aterrizaje repetido no cambió el atajo. Pico de par
  (`results/training_torque_marcha_pies_kp5_1000.json`): rodilla izquierda 0,97 N·m, rodilla derecha 0,85, tobillo de balanceo
  derecho 0,83, tobillo de cabeceo derecho 0,77; ninguna pasa el 90 % del par de parada a 6 V más del 0,2 % del tiempo.
  Con acción cero, 7 de 64 entornos caen en 5 s con este modelo (antes ninguno): `stand` ya no es estable sin política.
- **Reloj de marcha** (`gait_clock.py`, `mdp.gait_contact_phase`): fase de 0,8 s que solo avanza con orden de movimiento;
  seno y coseno en actor y crítico; recompensa de contacto en contrafase, peso 1,5, y con orden cero premia los dos pies
  apoyados. Sustituye a `air_time`, a la penalización de acción y a la de aterrizaje repetido, que se quitaron del código
  (el historial está en git). Lo comprueba `scripts/check_gait_clock.py`.
- **`marcha_reloj_1000`: camina.** Desde cero, 1000 iteraciones, `model_999.pt`, con el modelo de la auditoría, `KP_FW` 5 y
  la velocidad en el centro de masas. Con orden fija de 0,25 m/s, sin empujones, 256 entornos y 20 s
  (`results/training_gait_marcha_reloj_1000.json`): sin caer el 100 %, avance 0,248 m/s, error 0,020 m/s, apoyos alternos
  99,9 %, pie izquierdo el 48,9 % de los apoyos. Con órdenes aleatorias (`results/training_walk_marcha_reloj_1000.json`):
  de pie 99,2 % de los entornos, error 0,028 m/s. Altura máxima de suela 43 mm
  (`results/training_peak_height_marcha_reloj_1000.json`). Log de entrenamiento: episodio de 996 pasos y
  `peak_height_mean` de 19,7 mm. Con el punto de control de la iteración 350 ya caminaba igual (error 0,023 m/s). El usuario
  lo vio caminar hacia delante en el visor.
- **Giro de `marcha_reloj_1000`** (`results/training_gait_reloj_1000_*.json`, 256 entornos, 20 s, sin empujones, giro de ±0,5 rad/s,
  el máximo del entrenamiento): en el sitio gira a +0,40 y -0,40 rad/s y avanza 0,002 m/s; andando a 0,25 m/s gira a +0,42 y
  -0,37 rad/s y avanza 0,249 y 0,249 m/s. Error de giro de 0,12 a 0,14 rad/s, o sea que llega al 75-85 % de la orden. En los cuatro
  casos el 100 % no cae y el 99,8 % de los apoyos alternan. Avance con giro: sin caer y sin perder la marcha.
- **Pares de `marcha_reloj_1000`** (`results/training_torque_marcha_reloj_1000.json`, orden de 0,25 m/s): tobillo de cabeceo
  derecho 1,02 N·m, izquierdo 0,92, rodilla izquierda 0,90, derecha 0,88, cadera de balanceo izquierda 0,70; el resto, por
  debajo de 0,5. El 90 % del par de parada a 6 V (0,762 N·m) se supera el 7,5 % del tiempo en el tobillo de cabeceo derecho
  y el 3,5 % en el izquierdo. La guía pide no saturar ninguna articulación más del 5 %: el derecho no lo cumple a 6 V, aunque
  ese umbral se mide con el par de parada a 6 V y la aleatorización llega a 8,4 V (1,067 N·m). Sin revisar por tensión.
- **Aviso sobre las medidas de órdenes fijas anteriores:** `rel_forward_envs` de mjlab vale 0,2, así que el 20 % de los entornos
  recibe siempre (0,3; 0; 0), sin giro, y `measure_gait.py` y `measure_actuator_torque.py` no lo quitaban. Los JSON
  `training_gait_*` y `training_torque_*` anteriores al 2 de noviembre por la tarde (`aterrizaje_300`, `pasos_alternos_300`,
  `penalizacion_accion_005_300`, `marcha_aterrizaje_1000`, `marcha_com_1000`, `marcha_pies_kp5_1000`) llevan esa mezcla: su
  conclusión (no avanza, un solo pie) no cambia, pero las cifras de orden y de par no son de una orden de 0,25 m/s limpia. Las
  de `marcha_reloj_1000` se repitieron ya sin esos entornos.
- **Sin comprobar de `marcha_reloj_1000`:** no se probó marcha lateral ni los empujones;
  tampoco con armadura y holgura aleatorias. El 94 % del tiempo hay un solo pie apoyado, así que casi no hay fase con los
  dos pies en el suelo. Falta saber si el criterio de la guía (5 minutos simulados sin caer en el 95 % de las pruebas, con
  aleatorización completa) se cumple. La política necesita la fase del reloj como entrada: el robot real tendrá que generarla
  por firmware. Los puntos de control anteriores no cargan con la observación nueva.
- **Pares** (`results/training_torque_*.json`): en ambos casos la mayoría de articulaciones usa menos de 0,4 N·m. Los
  tobillos de cabeceo llegan a 0,94–0,98 N·m con escala 0,5 (el límite a 6 V es 0,762 N·m y a 8,4 V es 1,067 N·m).
  Solo pasan el 90 % del límite entre el 4 % y el 10 % del tiempo. La saturación de par no explica el fallo.
- **Seguimiento de consigna:** el error RMS de las articulaciones es 0,047 rad en `rewardw_300` y 0,096 rad en
  `actionscale_300`. Los tobillos de cabeceo son lo peor (0,35–0,43 rad con escala 0,5).
- **Prueba aislada del tobillo** (escalón de 0,3 rad, sin política, `scratch/diag_ankle_step.py`): el tobillo sube
  hacia la consigna pero se queda corto y oscila. No separa el actuador de la caída del cuerpo.
- **Pies:** las suelas se despegan entre 1 y 2 cm en el entorno 0, y hasta 5 cm en algún entorno de los 64.
  No hay marcha sostenida.

Lo que no se ha comprobado: si el modelo BAM sigue la consigna en un banco sin el cuerpo, si el contacto del pie
(cajas de 8,5 × 4,8 cm, fricción 0,8, pies a 6,4 cm entre centros) limita el paso, y si más iteraciones cambian algo.
Las conclusiones dependen del modelo de actuador (`params/hd1910_m6.json`, de xgoduck_rl) y no están confirmadas
en servos reales.

## Plan de la etapa 4 (fases)

Orden fijo, sin saltarse pruebas. El criterio se escribe antes de cada corrida larga. Si una fase falla, se para, se escribe lo
que se vio y se avisa; no se alargan las corridas para ver si se arreglan solas. Cada fase tiene una corrida nueva con nombre
claro y se conserva el punto de control que aprobó. Todo se reentrenará después de medir los servos reales, porque los
parámetros son de otro robot: lo aprendido no se instala directo en el robot físico. La fase del reloj irá en el programa de
50 Hz de la Radxa.

| Fase | Trabajo | Estado |
|---|---|---|
| 0 | Preparar: main fusionado, modelo comprobado, colisiones, masa, centro de masas y bloqueo de par | Hecho el 2 de noviembre (abajo) |
| 1 | Validar la marcha con el modelo final | Aprobada el 2 de noviembre con la fase 1b (abajo): desde cero no aprendió, y reentrenando desde la política antigua con penalización de choques sí |
| 2 | Movimiento completo (avanzar, retroceder, lateral, girar, quieto) con armadura, holgura y empujones | Aprobada el 4 de noviembre con `fase2_f_rodilla/model_8995.pt` (ver más abajo; PR 1 a main) |
| 3 | Agacharse: orden de altura del cuerpo, hasta 3 000 iteraciones | **Aprobada (agacharse quieto)** el 5 de noviembre con `fase3_agacharse/model_11994.pt`, usado solo parado; caminar agachado, descartado por ahora (ver más abajo) |
| 4 | Levantarse (4.3): política propia, 10 000 iteraciones; primero boca abajo, luego boca arriba | Criterio escrito el 5 de noviembre (abajo); tarea sin implementar, sin entrenar |
| 5 | Sentarse y tumbarse: política propia, hasta 5 000 iteraciones por acción; solo si la fase 4 aprueba | Pendiente |
| 6 | Movimiento natural (4.4): imitar CMU y 100STYLE, retargeteadas con GMR | Pendiente: fuentes elegidas el 4 de noviembre |

Fuera del plan, salvo que el usuario lo pida: correr, saltar y tomar objetos.

**Fase 0, hecha** (`scripts/check_training_setup.py` lo comprueba): `danny_training.xml` con 1,3006 kg, 13 `<pair>`, tobillo de -45° a 40° y
sitio `imu`; las 31 cajas de colisión con `condim` 3 y prioridad 1 (antes 1 y 0, y el suelo les ponía su fricción de 1,0); masa
total de 0,90 a 1,10 veces la nominal y centro de masas de hasta unos 12,6 mm por eje, con `shared_random=True` en todos los cuerpos.
El centro de masas pasa de 10 mm porque el desplazamiento se aplica en el marco de cada cuerpo y el 24 % de la masa está en eslabones
con el marco girado. La inercia no escala con la masa (`dr.body_mass`). Par de bloqueo medido: 0,739 N·m a 6 V y 0,912 N·m a 7,4 V
(a 8,4 V, 1,036 N·m por extrapolación, sin medir). Los scripts de medida aceptan `--vin` para fijar la tensión sin caída de carga.

**Fase 1, criterio (escrito antes de entrenar):** corrida `fase1_marcha_1000`, 1 000 iteraciones desde cero, 4096 entornos, `KP_FW` 5,

reloj de 0,8 s, `std` de velocidad 0,125 y la configuración actual de `env_cfgs.py`. Se evalúa `model_999.pt` con

`measure_gait.py --vin 7.4` (orden de 0,25 m/s, 256 robots, 20 s, sin empujones). Aprueba si, a 7,4 V: apoyos alternos de 0,9 o más,

error de velocidad de 0,08 m/s o menos y 95 % o más de los 256 robots sin caer. Se mide también a 6 y 8,4 V, con el pico de par por

articulación (`measure_actuator_torque.py --vin ...`). Si no aprueba, se compara con `marcha_reloj_1000` (modelo anterior, 1,29 kg,

sin autocolisiones ni cajas de condim 3) para decir si dependía de pesar menos o de atravesar piezas.

**Fase 1, resultado: no aprueba.** Corrida `fase1_marcha_1000` (el modelo final, desde cero, 4096 entornos). Se cayó con un error de
CUDA en la iteración 701 (`Warp CUDA error 700`, coincidió con un cierre forzado de visores; el controlador de NVIDIA registró
tres eventos 153; la causa no está probada) y se reanudó desde `model_700.pt` con 300 iteraciones más (`fase1_marcha_1000_reanudada`,
hasta la 1 000). Se evalúa `model_999.pt`, orden de 0,25 m/s, 256 robots, 20 s, sin empujones
(`results/training_gait_fase1_marcha_1000_*V.json`):

| Tensión | Sin caer | Avance | Error de velocidad | Apoyos alternos | Aterrizajes del pie izquierdo |
|---|---|---|---|---|---|
| 7,4 V | 100 % | 0,001 m/s | 0,250 m/s | 0,008 | 99,5 % |
| 6 V | 100 % | 0,000 m/s | 0,250 m/s | 0,003 | 99,8 % |
| 8,4 V | 100 % | 0,001 m/s | 0,250 m/s | 0,030 | 98,1 % |

Criterio: alternancia de 0,9 o más, error de 0,08 m/s o menos y 95 % sin caer. Solo cumple lo último. Danny se queda quieto y mueve
el pie izquierdo sin avanzar, como las corridas sin reloj. Log: recompensa de velocidad 0,93 desde la iteración 100 (con `marcha_reloj_1000`
llegaba a 2,2 hacia la 300) y `peak_height_mean` de 9 a 10 mm; la altura máxima de suela fue de 25 mm
(`results/training_peak_height_fase1_marcha_1000.json`). Pico de par a 7,4 V (`results/training_torque_fase1_marcha_1000_*V.json`):
hombro de balanceo 0,94 N·m, cadera de guiñada izquierda 0,93; a 6 V el pico es 0,76 y a 8,4 V, 1,06, es decir, el bloqueo en cada tensión.

**Por qué: comparación con `marcha_reloj_1000`, sin entrenar.** La política antigua (`model_999`, que caminaba con el modelo anterior) se
prueba en el modelo nuevo a 7,4 V con orden de 0,25 m/s (`results/training_gait_antigua_en_modelo_nuevo_*.json`, `measure_gait.py` con
`--nominal-mass` y `--drop-link-contacts`):

| Variante en el modelo nuevo | Sin caer |
|---|---|
| Completo (masa y centro de masas aleatorios, 13 contactos entre eslabones) | 1,6 % |
| Masa nominal, con los 13 contactos | 0 % |
| Masa nominal, sin los contactos de brazo | 0 % |
| Masa nominal, sin los de pie (3 pares) | 63 % |
| Masa nominal, sin los de brazo ni los de pie | 64 % |
| Masa nominal, sin ningún contacto | 100 % (error 0,019 m/s) |
| Masa aleatoria, sin ningún contacto | 65 % |

Conclusión: no dependía de pesar menos (con 1,30 kg y sin contactos camina igual); dependía de **atravesar piezas**. Los contactos del pie
izquierdo con el derecho tumban casi todo (de 0 % a 63 % al quitarlos), los del muslo, la tibia y la cadera entre sí aportan el resto,
y los de brazo no influyen por sí solos. La aleatorización nueva de masa y centro de masas cuesta otro 35 % (de 100 % a 65 %). Holguras en
la pose de pie (`mj_geomDistance`): suela con suela 16 mm (suelas de 85 × 48 mm con 64 mm entre centros), cadera con cadera 8,6 mm, muslo con
muslo 23 mm, tibia con tibia 23 mm. Con tan poco margen, cualquier balanceo lateral hace chocar los pies. La política antigua nunca aprendió a evitarlo
porque no existía ese contacto, y la nueva no lo descubrió en 1 000 iteraciones.

**Lo que no se comprobó de la fase 1:** si más iteraciones, otra semilla o una recompensa de separación entre pies lo resuelven (hay una sola
corrida con el modelo final); qué par de contacto concreto roza en cada paso; y si la causa del error de CUDA fue el cierre forzado.

**Fase 1b, criterio (escrito antes de entrenar; el usuario aprobó el intento):** corrida `fase1b_separacion_500`, 500 iteraciones
partiendo de `marcha_reloj_1000/model_999.pt` (la política que caminaba sin contactos), en el modelo final y con la aleatorización de la fase 0.
Dos términos nuevos en `env_cfgs.py`: `link_pair_contacts` (peso -1, un punto por cada contacto activo de los 13 `<pair>`; con la política
antigua da -0,59 por paso, y coincide con el recuento independiente de unos 0,57) y `feet_lateral_clearance` (peso -2, separación lateral
entre suelas por debajo de 10 mm; con la política antigua da -4,2 por paso; de pie hay 16 mm). Se prueba con
`scripts/check_feet_lateral_clearance.py`. El sensor de contactos de mjlab no detecta los choques entre eslabones con sub-árboles (mide 0
con una política que choca miles de veces), por eso el recuento lee los contactos de la simulación. Se evalúa el último punto de control con
`measure_gait.py --vin 7.4` y el mismo criterio de la fase 1: alternancia de 0,9 o más, error de 0,08 m/s o menos y 95 % o más sin caer, y además
reparto de aterrizajes entre 0,4 y 0,6 y un recuento de choques entre eslabones cercano a 0. Se mide también a 6 y 8,4 V, con el pico de par.
Si no aprueba, se para, se escribe lo visto y se pide decidir un cambio de diseño (separar los pies o estrechar las suelas, sin tocar `cad/`
desde aquí). No se alarga la corrida.

**Fase 1b, resultado: aprueba.** Corrida `fase1b_separacion_500` (500 iteraciones desde `marcha_reloj_1000/model_999.pt`, de la 1 000 a la
1 499), punto de control aprobado `model_1498.pt`, guardado fuera del repositorio en `D:\Claude\puntos_de_control\fase1b_separacion_500_model_1498.pt`
(5 MB, no va a git). Orden de 0,25 m/s, 256 robots, 20 s, sin empujones (`results/training_gait_fase1b_separacion_500_*V.json`):

| Tensión | Sin caer | Avance | Error de velocidad | Apoyos alternos | Aterrizajes del pie izquierdo |
|---|---|---|---|---|---|
| 7,4 V | 100 % | 0,263 m/s | 0,032 m/s | 0,993 | 48,9 % |
| 6 V | 100 % | 0,244 m/s | 0,031 m/s | 0,997 | 48,9 % |
| 8,4 V | 100 % | 0,266 m/s | 0,034 m/s | 0,993 | 49,2 % |

Cumple todo el criterio a las tres tensiones: alternancia de 0,9 o más, error de 0,08 m/s o menos, 95 % o más sin caer, y reparto entre pies de
0,4 a 0,6. Choques entre eslabones: 0,000 por paso con la política determinista (recuento propio) y `feet_lateral_clearance` de -0,04 por paso
(antes -4,2). En el log, al final: episodio de 987 pasos, recompensa de velocidad 2,12, `peak_height_mean` de 17,8 mm y el 1,3 % de los
entornos con algún choque (con aleatorización y empujones). Altura máxima de suela de 40 mm (`results/training_peak_height_fase1b_separacion_500.json`).

Pico de par (`results/training_torque_fase1b_separacion_500_*V.json`, bloqueo de 0,739 N·m a 6 V, 0,912 a 7,4 V y 1,036 a 8,4 V): los picos llegan
al bloqueo en las rodillas y los tobillos de cabeceo (0,76 / 0,94 / 1,07 N·m, hasta un 3 % por encima de la cifra de bloqueo). Por encima del 5 % del
tiempo sobre el 90 % del bloqueo: a 7,4 V y a 8,4 V, el tobillo de cabeceo derecho (9,9 % y 9,1 %; el izquierdo, 3,1 % y 2,3 %); a 6 V, las dos rodillas
(7,9 % y 7,5 %) y ese tobillo (8,1 %). La guía pide no saturar ninguna articulación más del 5 % del tiempo: **no se cumple todavía**, y es parte del criterio
de cierre de la fase 2, no de esta.

**Lo que no se comprobó de la fase 1b:** solo se probó hacia delante a 0,25 m/s (ni giro, ni lateral, ni marcha atrás, ni con empujones); el
recuento de choques se validó con la política antigua y contra un recuento independiente, pero `Metrics/link_contact_fraction` salió `nan` en mi prueba;
no se probó otra semilla; no hay vídeo (la grabación automática con `play --video` no terminó en 9 minutos). La política aprobada parte de
una antigua: la que se entrenó desde cero con el modelo final no aprendió a andar (`fase1_marcha_1000`), así que el entrenamiento desde cero
necesita estas penalizaciones desde el principio o un arranque como este.

**Fase 2, criterio (escrito antes de entrenar):** una sola política con avanzar, retroceder, paso lateral, girar y parar con orden cero. Parte de
`fase1b_separacion_500/model_1498.pt` y entrena 10 000 iteraciones en tres corridas seguidas, cada una reanudada de la anterior, con más
aleatorización en cada una (`DANNY_DR_STAGE` en `env_cfgs.py`; sin la variable y al evaluar vale la 3, la completa):

| Corrida | Iteraciones | Etapa | Armadura de los servos | Holgura máxima | Empujones (XY) |
|---|---|---|---|---|---|
| `fase2_a` | 3 000 | 1 | ±10 % | 0,15° | ±0,12 m/s |
| `fase2_b` | 3 000 | 2 | ±20 % | 0,30° | ±0,16 m/s |
| `fase2_c` | 4 000 | 3 | ±30 % | 0,50° | ±0,20 m/s |

La holgura es un juego mecánico sobre la posición objetivo (`actuator.apply_backlash`, comprobado con `scripts/check_backlash.py`): una aproximación,
no el modelo de un servo real. Órdenes: avance de -0,25 a 0,25 m/s, lateral de -0,1 a 0,1 m/s y giro de -0,5 a 0,5 rad/s (11° por paso de 0,4 s, por
debajo de los 20° por paso del límite real), con el 20 % de los entornos a orden cero y sin los entornos forzados a avanzar de mjlab. Controles al final de
cada corrida (3 000, 6 000 y 10 000 iteraciones) con `scripts/evaluate_movements.py` a 7,4 V, aleatorización completa y sin empujones: cada uno de
los siete movimientos (adelante, atrás, lateral a cada lado, giro a cada lado, quieto) cumple si el 95 % o más de 256 robots no cae en 20 s y el error
de velocidad es de 0,08 m/s o menos y el de giro de 0,15 rad/s o menos; quieto, además, con velocidad media de 0,03 m/s o menos y giro medio de
0,10 rad/s o menos. Los controles intermedios informan y no frenan la corrida, salvo que la política colapse (menos del 50 % sin caer en adelante).
La fase se aprueba con la corrida `fase2_c` si cumplen los siete movimientos y además la prueba de la guía: 0,25 m/s durante 300 s simulados
(`measure_gait.py --pushes --seconds 300`, sin fijar la tensión) sin caer en el 95 % de los robots, y sin saturar ninguna articulación más del 5 %
del tiempo (`measure_actuator_torque.py --pushes --seconds 300`, contra el bloqueo a 6 V). Si no aprueba, se para y se explica; no se alarga.
Si aprueba: ONNX, vídeos si caben, un PR a `main` y aviso al dueño.

## Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que las cifras de `results/actuator_comparison.json` sean de diseño: son de primer orden.

## Pendiente de decidir por el usuario

- Licencia del repositorio, que es público.

## Última verificación

`analysis/compare_actuators.py`, `analysis/estimate_robot_budget.py`, `analysis/inspect_model.py` con un modelo mínimo
y pyflakes, el 31 de octubre de 2026. El handbook se probó dos veces con agentes sin contexto: en ambas dijeron qué
toca y por qué, y se corrigieron sus hallazgos. La segunda reprodujo igual los JSON de `results/`. Tras esas
correcciones no se repitió la prueba. La ventana del visor no se ha visto. No hay simulador, pruebas ni CAD
en esta nueva etapa.

Etapa 2.1 superada el 31 de octubre de 2026 en Fusion 2706.1.17, en macOS: `BuildExportTest` y `ExportModel` dieron
`cad/export/export_test/`, que carga en MuJoCo con 0,7536 kg, un 0,00 % de diferencia con el CAD, y el eje en X,
como en el CAD. Ese modelo mantiene su punto de giro al mover la bisagra. El complemento `DannyBridge` arrancó solo
al reiniciar Fusion. No se ha comprobado: la ventana del visor, el botón de ACDC4Robot en Fusion, ni un modelo con
ocurrencias giradas o varias articulaciones. La vista de Fusion sigue con Y arriba, así que los modelos se ven
tumbados, aunque sus coordenadas sean correctas. La pieza de prueba no se guardó en Fusion.

Etapas 2.2 y 2.3 en curso: el diseño de Fusion tampoco se guarda, se regenera con los seis comandos de
`documentation/lower-body-design.md`. `check_robot.py` da cero fallos con 24 cuerpos y 23 articulaciones, y
falla con resultados vacíos, obsoletos o con choques. Esa comprobación no ve el montaje: tanto piernas como
tronco la pasaron sin poder montarse. Por eso una revisión independiente sondeó los STL en cada iteración. Nada se ha
impreso ni montado. Lee las limitaciones de esos dos documentos antes de seguir.

### Fase 2, control 1 (fase2_a, etapa 1, 3000 iteraciones, model_4497)

Medido a 7,4 V con la aleatorización completa (etapa 3) y sin empujones (`results/training_movements_fase2_a_4497.json`): los siete movimientos cumplen (sin caer 1,000; error lineal 0,026-0,065 m/s; error de giro 0,035-0,132 rad/s; quieto 0,0013). Giro en el sitio queda cerca del límite (0,132 y 0,118 frente a 0,15). No se ha medido aún con empujones, durante 5 min ni la saturación de par: eso es la prueba final. Sigue `fase2_b` (etapa 2, 3000 iteraciones).

### Fase 2, control 2 (fase2_b, etapa 2, 3000 iteraciones más, model_7496)

Mismas condiciones que el control 1 (`results/training_movements_fase2_b_7496.json`): seis de siete movimientos cumplen; **giro a la derecha no cumple** (error de giro 0,196 rad/s, límite 0,15) y giro a la izquierda queda justo en el límite (0,150). Sin caer 1,000 en todos; adelante, atrás, laterales y quieto cumplen. El control es informativo y no detiene la corrida; empeora el giro respecto al control 1 (0,132 y 0,118), probablemente por la aleatorización más dura de la etapa 2 (causa no medida). Sigue `fase2_c` (etapa 3, 4000 iteraciones, desde model_7496).

### Fase 2, resultado final (fase2_c, etapa 3, 4000 iteraciones más, model_11495): NO CUMPLE

Corridas: `fase2_a` (etapa 1, 1498 a 4497), `fase2_b` (etapa 2, hasta 7496), `fase2_c` (etapa 3, hasta 11495). Puntos de control en `D:\Claude\puntos_de_control\` (fuera de git). Se paró aquí, sin alargar la corrida, como pide el plan.

- **Movimientos** (`results/training_movements_fase2_c_11495.json`, 7,4 V, aleatorización completa, sin empujones): adelante, atrás, laterales y quieto cumplen. **Giro a ambos lados no cumple**: error de giro 0,219 y 0,232 rad/s (límite 0,15). Causa medida: la velocidad de giro lograda es 0,28 y 0,27 rad/s con orden de 0,5, y baja con cada etapa (control 1: 0,39 y 0,42; control 2: 0,36 y 0,31; control 3: 0,28 y 0,27). El giro queda corto, no inestable (sin caer 1,000).
- **Prueba de la guía** (0,25 m/s, 300 s, 256 robots, empujones, aleatorización completa; `results/training_gait_fase2_c_prueba_guia.json`): **sin caer 0,883, por debajo del 0,95 exigido**. Entre los que no caen: velocidad 0,249 m/s, error 0,037, paso alterno 0,994.
- **Par** (`results/training_torque_fase2_c_prueba_guia.json`, 64 robots, 300 s, empujones): **rodilla izquierda satura (más del 90 % del bloqueo) el 6,35 % del tiempo**, por encima del 5 % exigido; rodilla derecha 3,5 %, cadera roll izquierda 0,7 %, tobillo izquierdo 0,6 %. Par máximo 1,067 N·m (bloqueo a 8,4 V: 1,036 extrapolado).
- **No comprobado:** por qué cae el 11,7 % en 5 min (si es por empujones, por una tensión baja o por deriva con el tiempo; no se separó). No se probó si el giro mejora con una orden menor ni con más peso de seguimiento de giro. No hay vídeo ni ONNX ni PR a main: la fase no pasó.
- **Siguiente paso, decide el dueño:** (a) endurecer la recompensa de giro y reentrenar desde `fase2_a` (que cumplía los siete movimientos sin empujones), (b) bajar la orden de giro por debajo de 0,5 rad/s si 20° por paso lo permite, (c) revisar el par de rodilla, que es de diseño del robot (no se modifica `cad/`).

### Fase 2, segundo intento (opción 1 del dueño: recompensa de giro más exigente, desde fase2_a)

**Cambio:** `track_angular_velocity` pasa de peso 2 y std 0,71 rad/s a peso 3 y std 0,25 (`env_cfgs.py`). **Motivo medido:** con el valor por defecto un error de giro de 0,22 rad/s aún da el 91 % de la recompensa, y el giro quedó en 0,28 de 0,5 rad/s. Es una hipótesis: no se probó que sea la causa.

**Plan antes de ejecutar:** `fase2_d` (etapa 2, 3000 iteraciones, desde `fase2_a` model_4497), control; `fase2_e` (etapa 3, 4000 iteraciones), control y prueba final. Mismo criterio de la fase 2 que arriba. Los controles informan y no paran la corrida salvo colapso (menos del 50 % sin caer en avance).

**Lo que este cambio no aborda:** el 88,3 % sin caer en 5 min con empujones y la rodilla izquierda saturada el 6,35 % del tiempo. Si el giro se arregla y esos dos siguen fallando, la fase 2 seguirá sin cumplir y lo diré.

#### Segundo intento, control 1 (fase2_d, etapa 2, model_7496)

`results/training_movements_fase2_d_7496.json` (7,4 V, aleatorización completa, sin empujones): sin caer 1,000 en todo; cumplen adelante, lateral izquierda y quieto; **no cumplen atrás (error de giro 0,522 rad/s), lateral derecha (0,165) ni los dos giros (0,199 y 0,202)**. Peor que el primer intento en atrás y lateral (control 2 anterior: atrás 0,078). El giro medio sí mejoró (0,40 y -0,34 rad/s con orden de 0,5, frente a 0,36 y -0,31), pero el error de giro por paso sigue alto porque la velocidad de giro oscila durante la marcha: en atrás la media es 0,026 y la media del valor absoluto 0,522. Es decir, el criterio mide sobre todo la oscilación del paso, no el seguimiento medio. No se ha medido de dónde sale esa oscilación. Se lanza `fase2_e` (etapa 3) según el plan; el control es informativo.

#### Segundo intento: `fase2_e` parada y análisis del giro (GPU libre)

`fase2_e` (etapa 3) se paró por orden del dueño en la iteración ~8200 de 11 496 (último punto `model_8200.pt`, sin evaluar; no cuenta como resultado). Se cerró a la fuerza porque `train.exe` tenía procesos hijos y no respondía a un cierre normal.

**Medidas** (`results/training_yaw_oscillation_fase2.json`, `scripts/measure_yaw_oscillation.py`, 64 robots, 12 s, 7,4 V, sin empujones):
- La oscilación de la velocidad de giro es el ritmo del paso: frecuencias dominantes 2,5 y 1,25 Hz (ciclo de 0,8 s). Quieto no oscila (0,002).
- Adelante y atrás: casi todo el error por paso es esa oscilación (`fase2_a` adelante: 0,060 por paso, 0,019 promediado a 0,8 s).
- **El giro de `fase2_c` falla por giro medio corto, no por oscilación:** error por paso 0,220, promediado 0,217; giro medio 0,283 de 0,5 rad/s. Corrige la lectura anterior de este documento (control 1 del segundo intento), que atribuía el criterio sobre todo a la oscilación: eso vale para adelante y atrás, no para el giro de `fase2_c`.
- `fase2_d` (recompensa de giro más exigente) **duplicó la oscilación** (RMS adelante 0,11 a 0,28 rad/s; atrás 0,14 a 1,05) y subió el giro medio (0,405), pero el error por paso no baja del criterio.
- **El entorno duro no explica el giro corto:** evaluada con aleatorización de etapa 1 o de etapa 3, `fase2_a` da 0,388 y 0,389 rad/s, y `fase2_c` 0,282 y 0,281. Es lo que aprendió la política.
- `fase2_c` gira poco a todas las órdenes, no solo a la máxima (orden 0,2: 0,066; 0,3: 0,135; 0,4: 0,205; 0,5: 0,281 rad/s).
- **No medido:** por qué la política entrenada con etapas 2 y 3 aprende a girar menos que `fase2_a`. Candidatas sin probar: los empujones durante el entrenamiento, el peso de la penalización de oscilación angular o la combinación de ambos.

#### Medidas previas al tercer intento (sobre fase2_c, model_11495)

**Recompensa doble** (propuesta del dueño tras revisar b0dcf8e): se vuelve a `track_angular_velocity` por defecto (peso 2, std 0,71 rad/s) y se añade `track_angular_velocity_fine` (peso 1, std 0,25). **El cambio a std 0,25 con peso 3 solo empeoró atrás (0,078 a 0,522 rad/s) y el lateral derecho**, sin llevar el giro por paso al criterio. Ambos términos incluyen la velocidad angular en x e y (balanceo del tronco), como la función de mjlab: con std 0,25 eso penaliza mucho el balanceo. Es una lectura del código, no está medida como causa.

**Par por tensión** (`results/training_torque_fase2_c_{6.0,7.4,8.4}V.json`, 300 s, empujones, 64 robots, bloqueo de su propia tensión): la rodilla izquierda supera el 90 % del bloqueo el **5,7 % del tiempo a 6 V** (derecha 2,5 %), el 1,4 % a 7,4 V y nada a 8,4 V. El 6,35 % medido antes comparaba a todos con el bloqueo a 6 V y sobrestimaba. El criterio de saturación (menos del 5 %) falla solo a 6 V. Valores de simulación; el HD-1910 no está confirmado.

**Caídas en 5 minutos** (`results/training_gait_fase2_c_5min_*.json`, 256 robots, 0,25 m/s): sin empujones a 7,4 V, **100 % sin caer**; con empujones a 7,4 V, **91,8 %**; con empujones a 6 V, **85,2 %**. Las caídas vienen de los empujones, y la tensión baja las empeora. La prueba de la guía (empujones y tensión de 6 a 8,4 V) dio 88,3 %, entre esos dos casos.

Cambios de herramientas: `check_training_setup.py` usa ahora la cota exacta del centro de masas por entorno (el límite fijo de 13,5 mm fallaba al azar entre 12,1 y 13,6 mm); `evaluate_movements.py` imprime el giro medio y la media de su valor absoluto; la nota de `measure_actuator_torque.py` indica contra qué bloqueo compara.

#### Tercer intento, control 1 (fase2_d_doble, etapa 2, model_7496, 3000 iteraciones desde fase2_a model_4497)

`results/training_movements_fase2_d_doble_7496.json` (7,4 V, aleatorización completa de la etapa 3 de evaluación, sin empujones): **cumplen los siete movimientos**, sin caer 1,000 en todos. Error de giro por paso: adelante 0,070, atrás 0,073, lateral izquierda 0,030, lateral derecha 0,034, giro izquierda 0,125 (media 0,424), giro derecha 0,114 (media -0,421), quieto 0,001. Error lineal máximo 0,070 m/s (laterales). La recompensa doble corrige lo que empeoró con std 0,25 solo: atrás vuelve de 0,522 a 0,073. Es solo el control de la etapa 2 de entrenamiento; falta la etapa 3 (`fase2_e_doble`, 4000 iteraciones), el control final y la prueba de la guía. No se ha medido todavía con empujones ni a 6 V.

#### Tercer intento, resultado final (fase2_e_doble, etapa 3, model_11495, 4000 iteraciones desde fase2_d_doble model_7496): NO CUMPLE

**Criterio** (el de la fase 2): cada movimiento por separado con 256 robots, 20 s: >=95 % sin caer, error <0,08 m/s o <0,15 rad/s, quieto se queda quieto; prueba de la guía: 0,25 m/s, 5 min, >=95 % sin caer con aleatorización completa, y ninguna articulación saturada (>90 % del bloqueo) más del 5 % del tiempo.

**Movimientos** (`results/training_movements_fase2_e_doble_11495.json`, 7,4 V, sin empujones): sin caer 1,000 en los siete. Error de giro por paso: adelante 0,067, atrás 0,086, lateral izquierda 0,027, lateral derecha 0,033, giro izquierda 0,135 (media 0,385), **giro derecha 0,158 (media -0,350), no cumple por 0,008 rad/s**, quieto 0,002. Error lineal máximo 0,076 m/s (lateral izquierda). Los demás cumplen.

**Prueba de la guía** (`training_gait_fase2_e_doble_prueba_guia.json`, empujones, 6 a 8,4 V, 5 min, 256 robots): **82,4 % sin caer, no cumple** (criterio 95 %).

**Caídas separadas** (`training_gait_fase2_e_doble_5min_*.json`): sin empujones a 7,4 V 100 %; con empujones a 7,4 V 83,2 %; con empujones a 6 V 82,8 %. Las caídas vienen de los empujones. Peor que fase2_c (91,8 % y 85,2 %).

**Par** (`training_torque_fase2_e_doble_{6.0,7.4,8.4}V.json`, 300 s, empujones, 64 robots, bloqueo de su propia tensión): la rodilla izquierda supera el 90 % del bloqueo el **13,2 % del tiempo a 6 V** (derecha 10,1 %), **4,9 % a 7,4 V** (derecha 3,4 %) y 1,8 % a 8,4 V (derecha 1,6 %; tobillo derecho 1,2 %). **No cumple a 6 V**; a 7,4 y 8,4 V queda por debajo del 5 %, la izquierda a 7,4 V por muy poco. El par máximo pasa un poco del bloqueo en las tres tensiones (0,762 frente a 0,739 N·m a 6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V). Peor que fase2_c a 6 V (5,7 % y 2,5 %).

**Resultado:** la fase 2 no se aprueba. Fallan la prueba de la guía (82,4 %), la saturación de la rodilla a 6 V, y el giro a la derecha por 0,008 rad/s. La recompensa de giro doble resolvió el seguimiento de giro en la etapa 2 (fase2_d_doble cumplía los siete movimientos), pero la etapa 3 con empujones de 0,20 m/s empeoró las caídas y la saturación respecto a fase2_c. No hay ONNX, vídeos ni PR a main. La fase 3 (agacharse) no se empieza.

**No comprobado:** la causa de que la etapa 3 empeore las caídas (hipótesis sin medir: empujones de entrenamiento demasiado fuertes para la rodilla a baja tensión); si fase2_d_doble pasa la prueba de la guía (no se midió con empujones ni a 6 V); parámetros reales del HD-1910 (todo se reentrena tras medirlos); el reloj de marcha lo tendría que generar el firmware en la Radxa a 50 Hz. Con orden cero y acción cero (`stand`) caen 7 de 64 robots con el modelo anterior.

**Siguiente paso a decidir por el dueño:** (a) medir `fase2_d_doble` con la prueba de la guía y el par, para ver si la etapa 2 basta; (b) repetir la etapa 3 con empujones de 0,16 m/s; (c) pedir el criterio de saturación solo a 7,4 V, donde cumple, si el robot no va a funcionar a 6 V. Ninguno se lanza sin su decisión.

#### Medidas con los empujones de la guía y a 6,6 V (fase2_d_doble model_7496 y fase2_e_doble model_11495)

Tras la decisión del 4 de noviembre (tensión mínima de marcha 6,6 V, empujones de la guía de ±0,12 m/s), se repitió todo con la opción nueva `--push 0.12` de `measure_gait.py` y `measure_actuator_torque.py`. **La etapa 3 entrenaba y se medía con empujones de ±0,20 m/s, más fuertes que los de la guía; por eso el 82 % anterior no era la prueba de la guía.** Resultados en `results/training_*_push012_*.json` y `training_movements_*_6.6V.json`.

| | fase2_d_doble (etapa 2) | fase2_e_doble (etapa 3) |
|---|---|---|
| Prueba de la guía (5 min, 6 a 8,4 V, 256 robots) | 99,6 % sin caer | 100 % |
| 5 min a 6,6 V | 99,2 % | 100 % |
| 5 min a 7,4 V | 100 % | 100 % |
| Rodilla izq./der. sobre el 90 % del bloqueo a 6,6 V | **8,2 % / 7,5 %** | **8,4 % / 6,5 %** (tobillo der. 2,2 %) |
| Ídem a 7,4 V | 4,9 % / 3,8 % | 3,7 % / 2,8 % |
| Ídem a 8,4 V | 2,2 % / 1,8 % | 1,8 % / 1,8 % |
| Siete movimientos a 6,6 V, sin empujones | **cumplen los siete** (giro izq. 0,137, giro der. 0,121 rad/s) | cumplen seis; **giro der. 0,163, no cumple** (giro izq. 0,149) |

**Conclusión: ninguno cumple todo.** Los dos pasan la prueba de caídas con holgura. Los dos fallan la saturación de la rodilla a 6,6 V (más del 5 %); a 7,4 y 8,4 V cumplen. fase2_e_doble además falla el giro a la derecha. El par máximo supera el bloqueo en las tres tensiones (0,838 frente a 0,813 N·m a 6,6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V) en los dos modelos; el bloqueo a 8,4 V es extrapolado. Candidato más cercano a cerrar la fase 2: fase2_d_doble, que solo falla la rodilla a 6,6 V. No se ha reentrenado nada; la decisión es del dueño.

No comprobado: si una corrida corta más con penalización de par en la rodilla baja la saturación a 6,6 V sin perder el giro; los siete movimientos con empujones; el HD-1910 real. Los movimientos se midieron sin empujones, como en los controles anteriores.

#### Cuarto intento: `fase2_f_rodilla` (castigo de par en rodillas y tobillos), resultado final: CUMPLE

**Criterio escrito antes de lanzar (plan del dueño tras 2f36307):** desde `fase2_d_doble/model_7496`, ~1500 iteraciones con etapa 2 de aleatorización y empujones de ±0,12 m/s. Nuevo castigo `servo_torque_excess` = -peso · Σ max(0, |τ| − 0,8·τ_bloqueo(vin del entorno))², solo en rodillas y cabeceo de tobillos (no castiga el par normal, solo el exceso). Peso **−317**, elegido para que valga ≈7,5 % de la recompensa por paso al principio (0,00184 de castigo crudo con `model_7496`; 5 %, 7,5 % y 10 % darían 211, 317 y 422). `check_training_setup.py` comprueba que vale 0 hasta el 80 % del bloqueo a 6, 7,4 y 8,4 V, crece por encima y vale 0 de pie con acción cero. Para cumplir: los 7 movimientos a 6,6 V, la prueba de la guía (≥95 %), 5 min a 6,6 V, rodilla sobre el 90 % del bloqueo por debajo del 5 % a 6,6, 7,4 y 8,4 V, y la flexión de rodilla en apoyo comparada con fase2_d_doble.

**Corrida:** `fase2_f_rodilla`, 1500 iteraciones (7496 → 8995), `model_8995.pt`, sin caídas durante el entrenamiento (episodio de 1000 pasos); el castigo bajó de −0,277 a −0,030 por episodio. Opciones nuevas: `DANNY_PUSH_M_S` (fuerza del empujón en entrenamiento) y métrica `mean_knee_angle_stance_deg` en `measure_gait.py`.

| Medida (empujones ±0,12 m/s) | fase2_d_doble (7496) | fase2_f_rodilla (8995) |
|---|---|---|
| Prueba de la guía (5 min, 6 a 8,4 V) | 99,6 % sin caer | 100 % |
| 5 min a 6,6 V | 99,2 % | 100 % |
| Rodilla izq./der. sobre el 90 % del bloqueo a 6,6 V | 8,2 % / 7,5 % | **0,01 % / 0,04 %** |
| Ídem a 7,4 V | 4,9 % / 3,8 % | 0,01 % / 0,02 % |
| Ídem a 8,4 V | 2,2 % / 1,8 % | 0,01 % / 0,01 % |
| Peor articulación (cualquiera) a 6,6 / 7,4 / 8,4 V | rodilla izq. 8,2 / 4,9 / 2,2 % | cadera roll der. 0,67 / 0,27 / 0,04 % |
| Flexión de rodilla en apoyo a 6,6 V | 60,6° | 61,3° (sin enderezar la pierna) |
| Siete movimientos a 6,6 V, sin empujones | cumplen los siete | **cumplen los siete**: giro izq. 0,124, giro der. 0,135 rad/s (límite 0,15); adelante/atrás 0,036; lateral 0,067; quieto 0,0006 |

Resultados en `results/training_*_fase2_f_rodilla_*.json`. **Fase 2 cumple todos los criterios con `fase2_f_rodilla/model_8995.pt`.** Copia del punto de control en `D:\Claude\puntos_de_control\fase2_f_rodilla_model_8995.pt` (fuera del repositorio).

**No comprobado / límites:** el par máximo sigue 3 % por encima del bloqueo lineal en las tres tensiones (0,838 frente a 0,813 N·m a 6,6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V), igual que en fase2_d_doble: es un pico puntual del modelo BAM, no saturación sostenida; el bloqueo a 8,4 V es extrapolado. Los movimientos se miden sin empujones. La caída de tensión bajo carga (≤0,15 V) no entra en el castigo. El reloj de marcha exige firmware a 50 Hz en la Radxa. Los parámetros reales del HD-1910 no están confirmados: todo se reentrena tras medir el servo real. Error de importación circular en `measure_actuator_torque.py` (importar `actuator` antes que `evaluation`) corregido con el orden de imports; las tres pruebas de par se repitieron.

**Entregables de la fase 2 (con `fase2_f_rodilla/model_8995.pt`):**
- Política en ONNX: `policies/danny_fase2_movimiento_completo_2026-11-04.onnx` (837 KB). Entrada `obs` de 80 valores (`base_ang_vel`, `projected_gravity`, `joint_pos`, `joint_vel`, `actions`, `command`, `gait_clock`), salida `actions` de 23 valores. Lo exporta el propio ejecutor de mjlab al guardar. `scripts/verify_policy_onnx.py` lo compara con el punto de control en 500 pasos y 16 robots: diferencia máxima 1,5e-6 (límite 1e-4), en `results/training_onnx_fase2_f_rodilla.json`. Aviso: los metadatos del ONNX dicen rigidez 1, amortiguamiento 0 y escala de acción 1,0; son valores de relleno del modelo BAM (la escala 1,0 sí es la del entrenamiento, `ACTION_SCALE`), no ganancias de un PD real. El programa de la Radxa debe usar la lista `joint_names` y `default_joint_pos` de los metadatos y el reloj de marcha a 50 Hz.
- Vídeos de 8 s a 6,6 V, sin empujones, 0,4 MB cada uno: `results/videos/fase2_f_rodilla_{adelante,atras,lateral_izquierda,giro_izquierda,quieto}.mp4` (`scripts/record_walk_video.py`).
- No comprobado: el ONNX no se ha ejecutado en la Radxa ni a 50 Hz reales; los vídeos son de 320×240.

### Fase 3, agacharse: criterio escrito antes de entrenar (4 de noviembre)

**Lo que permite el cuerpo (cálculo estático, no entrenamiento).** Con los pies planos y el tronco recto, el centro de masas queda sobre los pies con cadera -21° y rodilla 47° (la pose de pie). Al agacharse, el tobillo debe valer -(cadera + rodilla) y su límite es -45°, así que la rodilla llega a unos 82° y **el centro de masas baja unos 22 mm** (de 0,212 a 0,190 m, un 10 %). El par estático de la rodilla pasa de 0,13 a 0,24 N·m, el 29 % del bloqueo a 6,6 V (0,813 N·m): el par no es el límite, el rango del tobillo sí. Bajar más exige levantar talones o inclinar el tronco, y eso no se intenta. **Si el dueño quiere un agachado más profundo, es un tema de diseño (rango del tobillo).** Cálculo con `danny_training.xml` y la masa nominal; la masa y el centro de masas aleatorizados lo mueven unos milímetros.

**Orden.** Una orden `height` en metros, de -0,02 a 0 m respecto a la altura de pie, que se suma a la altura objetivo del tronco; se resortea cada 4 a 8 s y el 20 % de los entornos recibe 0. La política la ve como una observación más (82 entradas en lugar de 80).

**Corrida.** `fase3_agacharse`, hasta 3 000 iteraciones desde `fase2_f_rodilla/model_8995.pt`, ampliando la primera capa con columnas a cero para las entradas nuevas (así empieza caminando como en la fase 2), etapa 2 de aleatorización y empujones de ±0,12 m/s. Se mantiene el castigo de par (-317). Se para, sin alargar, si el 50 % o más de los robots cae hacia delante.

**Aprueba** (256 robots, 20 s, 6,6 V fijos, sin empujones, salvo la última prueba) si:
1. Con orden de altura de 0, -0,01 y -0,02 m y orden de velocidad cero: 95 % o más sin caer, error medio de altura de 4 mm o menos y velocidad media de 0,03 m/s o menos.
2. Caminando a 0,25 m/s con altura -0,02 m: 95 % o más sin caer y error de velocidad de 0,08 m/s o menos.
3. Los siete movimientos de la fase 2 con altura 0 siguen cumpliéndose (no se pierde lo aprendido).
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, en el agachado quieto a -0,02 m y en la marcha agachada.
5. Prueba de la guía (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, altura aleatoria): 95 % o más sin caer.
Si no cumple, se para y se anota; no se amplía la corrida.

### Fase 3, agacharse: criterio escrito antes de entrenar (4 de noviembre)

**Cambio de criterio, 4 de noviembre, a propuesta del dueño** (la primera versión suponía el tronco recto y limitaba la bajada a 20 mm; con el tronco inclinado como una persona, el cuerpo baja más). Orden de altura de **0 a -0,05 m medida en la pelvis** (`pelvis_bottom`), con el tronco inclinado hacia delante en proporción a la orden: 20° a -0,05 m. La tarea `squat` de `analysis/danny_motion.py` baja la pelvis 45 mm respecto a su pose de pie (60 mm respecto al origen del modelo, 0,241 m) con el tronco a 0,35 rad. Ojo: respecto a la pose de pie de la política (pelvis a 0,228 m), -0,05 m son 5 mm más que lo comprobado allí, por eso se comprueba también a mano (siguiente párrafo).

**Cálculo estático previo** (`scripts/check_crouch_pose.py`, `results/training_crouch_static.json`; pies planos, centro de masas sobre las suelas, peso repartido por igual, 6,6 V, tronco inclinado 20° por cada 50 mm):

| Bajada de la pelvis | Inclinación | Cadera | Rodilla | Tobillo (límite -45°) | Par de rodilla (% del bloqueo a 6,6 V) |
|---|---|---|---|---|---|
| 0 | 0° | -20,9° | 47,4° | -26,6° | 0,13 N·m (16 %) |
| -0,025 m | 10° | -53,0° | 79,6° | -36,6° | 0,20 N·m (24 %) |
| -0,045 m | 18° | -75,7° | 96,9° | -39,2° | 0,21 N·m (26 %) |
| **-0,050 m** | **20°** | -81,3° | 100,5° | **-39,3°** | **0,21 N·m (26 %)** |
| -0,060 m | 24° | -92,4° | 107,2° | -38,8° | 0,21 N·m (26 %) |

A -0,05 m el tobillo queda dentro de su rango (5,7° de margen), la cadera (límite -100°) y la rodilla (130°) también, y el centro de masas cae sobre los pies (error de 0 mm, por construcción). Es estático: el par dinámico y la aleatorización de masa y centro de masas no están incluidos.

**Orden.** Una orden `height` en metros, de -0,05 a 0 respecto a la pelvis de pie (0,228 m), que la política ve como una observación más (81 entradas en lugar de 80; el crítico, 96 en lugar de 95). Se resortea cada 4 a 8 s y el 20 % de los entornos recibe 0. Con esa orden:
- la recompensa de postura se desplaza: la referencia de cadera, rodilla y tobillo es la de la tabla de arriba (interpolada);
- `upright` pide un cabeceo del tronco de 0,35 rad por cada 0,05 m (7 rad/m), en vez de tronco recto;
- recompensa de altura de la pelvis (peso 3, std 6 mm), **solo con orden de velocidad cero**: al caminar la pelvis baja sola unos 12 mm (medido con `fase2_f_rodilla`: -0,0127 m de pie, -0,0252 m a 0,25 m/s), así que caminando se exige la bajada relativa, no la absoluta.

**Corrida.** `fase3_agacharse`, hasta 3 000 iteraciones desde `fase2_f_rodilla/model_8995.pt`, ampliando la primera capa del actor y del crítico con una columna a cero (y su normalizador) para la entrada nueva, de modo que empieza caminando como en la fase 2; etapa 2 de aleatorización y empujones de ±0,12 m/s; se mantiene el castigo de par (-317). Se para, sin alargar, si el 50 % o más de los robots cae hacia delante.

**Aprueba** (256 robots, 20 s, 6,6 V fijos, sin empujones, salvo la última prueba) si:
1. Quieto (orden de velocidad cero) con altura 0, -0,025 y -0,05 m: 95 % o más sin caer, error medio de altura de la pelvis de 5 mm o menos y velocidad media de 0,03 m/s o menos.
2. Caminando a 0,25 m/s con altura -0,025 m: 95 % o más sin caer, error de velocidad de 0,08 m/s o menos y pelvis al menos 15 mm más baja que caminando con altura 0.
3. Los siete movimientos de la fase 2 con altura 0 siguen cumpliéndose (no se pierde lo aprendido).
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, en el agachado quieto a -0,05 m y en la marcha agachada a -0,025 m.
5. Prueba de la guía (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, altura aleatoria entre 0 y -0,05): 95 % o más sin caer.
Si no cumple, se para y se anota; no se amplía la corrida. Ejecución automática de la fase 3, sin esperar permisos, a petición del usuario.

### Fase 3, resultado: NO CUMPLE (`fase3_agacharse`, 3 000 iteraciones, 8995 → 11994, `model_11994.pt`)

Corrida del 4 al 5 de noviembre, de 23:05 a 02:00, etapa 2 de aleatorización y empujones de ±0,12 m/s, sin colapso (episodios de 987 a 1000 pasos). Se midió automáticamente con el criterio de arriba, a 6,6 V (`results/training_*fase3_agacharse*.json`). Punto de partida: `fase2_f_rodilla/model_8995.pt` con la primera capa ampliada (`scripts/extend_checkpoint_inputs.py`); con altura 0 y ese punto de partida, antes de entrenar, caminaba como en la fase 2 (error de velocidad 0,0375 m/s, 100 % sin caer).

| Prueba del criterio | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m | 100 % sin caer; error de altura 2,2 / 2,1 / 4,6 mm (límite 5 mm); velocidad media 0,002 / 0,002 / 0,001 m/s | **Cumple** |
| 2. Marcha a 0,25 m/s a -0,025 m | 100 % sin caer, error de velocidad 0,043 m/s; pero la pelvis baja **4,6 mm** respecto a caminar con altura 0 (0,2065 → 0,2019 m); se pedían 15 mm | **No cumple** |
| 3. Siete movimientos a altura 0 | cumplen 3 de 7: lateral izq. 0,104, lateral der. 0,137 y quieto. Fallan **adelante** (giro 0,159 rad/s), **atrás** (0,336), **giro izq.** (0,321) y **giro der.** (0,406) | **No cumple** |
| 4. Rodilla y tobillo sobre el 90 % del bloqueo, 6,6 V | 0 % en rodillas y tobillos, quieto a -0,05 m y marchando a -0,025 m (con empujones de 0,12). Peor articulación: codo izq. 1,5 % (quieto), cadera izq. 3,8 % (marcha) | **Cumple** |
| 5. Prueba de la guía (5 min, 6 a 8,4 V, ±0,12, altura aleatoria) | 100 % sin caer, error de velocidad 0,044 m/s | **Cumple** |

**Qué se ve.** Quieto, la política baja la pelvis hasta 50 mm con el tronco a 12,6° (se pedían 20°), con el par muy por debajo del límite. Lo que falla es la marcha: (a) caminando agachado no sigue la orden de altura, solo 4,6 mm de los 25 pedidos (la recompensa de altura solo actúa parado y la de postura, que sí actúa caminando, no bastó); (b) con altura 0 la marcha cambió respecto a la fase 2: la pelvis camina 8 mm más baja (0,2065 frente a 0,2149 m) y el tronco inclinado 9,8° (4,2° antes), y el seguimiento del giro se rompió (error de giro 0,16 a 0,41 rad/s caminando). La velocidad lineal se mantiene (0,038 m/s). La causa de (b) no está medida: la hipótesis es que la inclinación y la postura agachada, premiadas quieto, se filtraron a la marcha con altura 0 porque la política comparte pesos; no se probó.

**Lo que no se comprobó:** otras ponderaciones, un punto de control intermedio (solo se midió el último), ni si la regresión del giro aparece ya en las primeras iteraciones. Según el plan, no se alarga la corrida ni se reentrena sin decisión del dueño.

**Siguientes pasos posibles (decide el dueño):** (1) política de agacharse solo quieto, separada de la marcha: no estropea la fase 2 y lo que funciona (criterios 1, 4 y 5) ya cumple; (2) reentrenar con recompensa de altura también caminando (referida a la marcha de altura 0) y con las órdenes de giro de la fase 2 vigiladas en cada control; (3) mezclar con datos de la fase 2 (congelar la marcha con altura 0). La fase 2 sigue aprobada con `fase2_f_rodilla/model_8995.pt` (PR 1 a main); la fase 4 (levantarse) no depende de la 3.

### Fase 3 (segundo intento): agacharse solo quieto, criterio escrito antes de entrenar (5 de noviembre)

**Decisión del dueño, 5 de noviembre:** agacharse quieto como política aparte; caminar agachado queda **descartado por ahora**. Hipótesis del dueño para la regresión de `fase3_agacharse` (sin medir): el 80 % de los entornos caminaba con orden de altura y referencias de cuclillas estáticas, y solo el 20 % practicaba la marcha de la fase 2.

**Corrida.** `fase3_quieto`, hasta 1 500 iteraciones desde `fase2_f_rodilla/model_8995.pt` con la primera capa ampliada (el mismo punto de partida `fase3_inicio`), orden de velocidad siempre cero (`DANNY_STAND_ONLY=1`: todos los entornos reciben orden cero) y orden de altura de 0 a -0,05 m (20 % de ceros), etapa 2 de aleatorización, empujones de ±0,12 m/s, castigo de par -317 y las mismas recompensas de altura, postura e inclinación. Se para, sin alargar, si el 50 % o más de los robots cae. Los puntos de control se guardan cada 50 iteraciones.

**Aprueba** (256 robots, 6,6 V fijos, sin empujones salvo donde se indica) si:
1. Quieto con altura 0, -0,025 y -0,05 m, 20 s: 95 % o más sin caer, error medio de altura de la pelvis de 5 mm o menos y velocidad media de 0,03 m/s o menos.
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, quieto a -0,05 m con empujones de ±0,12 m/s.
5. Prueba de la guía quieta (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, orden de velocidad cero, altura aleatoria entre 0 y -0,05): 95 % o más sin caer.
6. **Cambio de política (nueva).** De pie con la política de la fase 2 y orden cero, a los 3 s se cambia a esta con orden cero y altura 0, -0,025 o -0,05 m: 95 % o más sin caer en los 5 s siguientes y velocidad media de 0,03 m/s o menos entre los 2 y los 5 s tras el cambio (`scripts/measure_policy_switch.py`; la política de la fase 2 es `fase3_inicio`, su equivalente con la entrada de altura a cero). El error de altura se anota, no decide.
Las pruebas 2 y 3 de la fase 3 anterior (marcha agachada y siete movimientos con la política nueva) ya no aplican: la marcha es la de la fase 2, sin cambios.
Si cumple, la fase 3 se anota aprobada (agacharse quieto). Si no, se para y se anota; no se amplía la corrida. Después se escribe el criterio de la fase 4 antes de entrenar. Ejecución automática, a petición del dueño.

### Fase 3 (segundo intento), resultado: NO CUMPLE (`fase3_quieto`, 1 500 iteraciones, 8995 → 10494, `model_10494.pt`)

Corrida del 5 de noviembre (empezó a las 10:40), `DANNY_STAND_ONLY=1`, etapa 2, empujones de ±0,12 m/s, sin colapso (episodios de 1 000 pasos, ninguna caída en el entrenamiento). Medido automáticamente a 6,6 V (`results/training_*fase3_quieto*.json`).

| Prueba | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m, 20 s | 100 % sin caer, velocidad 0,0002–0,0003 m/s; **error de altura de la pelvis 31,8 / 7,5 / 16,6 mm** (límite 5 mm). La pelvis queda en 0,196 / 0,196 / 0,195 m para las tres órdenes (objetivos 0,228 / 0,203 / 0,178) | **No cumple** |
| 4. Rodilla y tobillo sobre el 90 % del bloqueo, quieto a -0,05 m con empujones | 0 % en rodillas y tobillos (rodilla izq. 0,48 N·m máx., der. 0,54; tobillo 0,38 / 0,34) | Cumple |
| 5. Prueba de la guía quieta (5 min, 6 a 8,4 V, ±0,12, altura aleatoria) | 100 % sin caer, velocidad 0,0007 m/s | Cumple |
| 6. Cambio de la política de la fase 2 a esta (orden 0, 5 s tras el cambio) | 100 % sin caer en las tres alturas, velocidad 0,0005 m/s; error de altura 34 / 9,9 / 14,1 mm (se anota, no decide) | Cumple |

**Qué se ve.** La política se queda quieta y estable, pero **no obedece la orden de altura**: la pelvis queda en unos 0,195 m (33 mm por debajo de la de pie) y el tronco casi recto (0 a 0,9°, se pedía hasta 20°) para cualquier orden. Es un punto medio fijo, no un seguimiento. Los puntos de control intermedios (`results/training_gait_fase3_quieto_diag_*.json`, quieto a 0 y -0,05 m) dan lo mismo: pelvis a 0,198 / 0,197 (iteración 9000, solo 5 iteraciones después de reanudar), 0,205 / 0,205 (9250), 0,199 / 0,199 (9500) y 0,198 / 0,197 m (10000). Es decir, **la orden de altura nunca se siguió**, y la pelvis ya estaba baja a las 5 iteraciones, cuando la política de la fase 2 de pie la tenía en 0,2275 m (`results/training_switch_fase2_sin_cambio.json`, error 0,5 mm a altura 0).

**Hallazgo aparte: `right_hip_yaw` está al tope de par el 99,97 % del tiempo** (0,8135 N·m, el bloqueo a 6,6 V) en el agachado quieto a -0,05 m con empujones; el izquierdo, 0 %. El criterio 4 solo cubre rodilla y tobillo, así que no decide, pero esa pose no es válida para un robot real (el servo estaría bloqueado todo el tiempo). La causa no se midió; la hipótesis es que la cadera derecha empuja contra un límite de articulación.

**Lo que no se comprobó:** por qué la pelvis cae tan rápido en las primeras iteraciones (hipótesis sin medir: sin el estado del optimizador, las primeras actualizaciones son grandes y el castigo de la rodilla y la postura de cuclillas atraen a un mínimo local fijo); si con más iteraciones, otro peso de la recompensa de altura o empezando con la entrada de altura con pesos no nulos sí la seguiría; la pose de `right_hip_yaw`. Según el plan, no se alarga la corrida ni se reentrena sin decisión del dueño.

**Contraste con `fase3_agacharse`**, que sí seguía la orden de altura quieto (error 2,2 / 2,1 / 4,6 mm) pero con las órdenes de velocidad de la fase 2 mezcladas. Lo que cambia entre ambas corridas es solo `DANNY_STAND_ONLY=1`, 1 500 en lugar de 3 000 iteraciones y que no hay caminata; no se sabe cuál de las tres causa la diferencia.

**Estado de la fase 3:** sigue **no aprobada**. Opciones, decide el dueño: (a) usar `fase3_agacharse/model_11994.pt` solo para agacharse quieto: cumple 1, 4 y 5 quieto, pero cambia la marcha de la fase 2, así que habría que usarlo solo parado y cambiar de política con la prueba 6 (no medida con ese punto de control); (b) repetir `fase3_quieto` con una corrección que se decida después de medir la causa; (c) dejar el agacharse para después de medir el servo real. La fase 4 (levantarse) no depende de la 3 y no se ha empezado: su criterio no está escrito.

### Fase 3, aprobada (agacharse quieto) con `fase3_agacharse/model_11994.pt`, usado solo parado (5 de noviembre)

**Decisión del dueño, 5 de noviembre:** opción (a). El modelo se usa solo parado y se cambia a él desde la marcha de la fase 2 con orden de velocidad cero. Caminar agachado queda **descartado por ahora**, y las pruebas 2 y 3 de la fase 3 original no aplican. La marcha sigue siendo `fase2_f_rodilla/model_8995.pt`.

| Prueba (modelo `model_11994`, 6,6 V) | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m, 20 s (ya medido) | 100 % sin caer, error de altura 2,2 / 2,1 / 4,6 mm, velocidad ≤ 0,002 m/s | Cumple |
| 4. Saturación (>90 % del bloqueo), quieto a -0,05 m con empujones ±0,12, 120 s, **todas las articulaciones** | Máximo: codo izquierdo 1,54 %; rodillas, tobillos y las dos caderas en giro, 0 % (`results/training_torque_fase3_agacharse_quieto_h-0.05_6.6V.json`, 64 robots) | Cumple |
| 5. Prueba de la guía **quieta** (5 min, 6 a 8,4 V, ±0,12, orden de velocidad cero, altura aleatoria; `training_gait_fase3_agacharse_quieto_push012_prueba_guia.json`) | 99,6 % sin caer (255 de 256), velocidad media 0,0046 m/s | Cumple |
| 6. Cambio de política: de pie con la de la fase 2, a los 3 s a esta, 5 s después (`training_switch_fase3_agacharse.json`) | 100 % sin caer en las tres alturas; velocidad 0,0027 / 0,0025 / 0,0015 m/s (límite 0,03); error de altura 0,9 / 1,4 / 5,3 mm | Cumple |

La prueba 5 de antes se había medido caminando a 0,25 m/s; esta es la versión quieta, medida hoy. El ONNX está en `policies/danny_fase3_agacharse_quieto_2026-11-05.onnx` (81 entradas → 23 acciones; la entrada nueva, la última, es la bajada de la pelvis en m, de 0 a -0,05). Coincide con el punto de control (`results/training_onnx_fase3_agacharse.json`, diferencia máxima 2,4e-6). Vídeo: `results/videos/fase3_agacharse_agacharse.mp4` (0,62 MB; la orden baja de 0 a -0,05 m en 4 s, se mantiene 3 s y vuelve a 0). Se ve con `scripts/record_crouch_video.py`.

**Lo que muestra el vídeo y los números no ocultan:** en el vídeo (un solo robot) la pelvis llega a 0,188 m con la orden de -0,05 m (objetivo 0,178), 10 mm por encima; la media de 256 robots es de 4,6 mm de error. Al subir tarda más de 1 s en llegar a 0,224 m (de pie, 0,228). **No se ha comprobado:** la transición contraria (de esta política a la marcha de la fase 2), ni el cambio con empujones, ni la rodilla y el tobillo de la política con 256 robots en la prueba de saturación (se midieron 64). El tronco se inclina 12,6° a -0,05 m, no los 20° del criterio original; el criterio de aceptación no lo exigía.

### Diagnóstico de la caída de la pelvis al empezar (sin entrenar, 5 de noviembre)

Pregunta: `fase3_quieto` tenía la pelvis a 0,198 m en `model_9000`, 5 iteraciones después de empezar. ¿Por qué? Se compararon el normalizador y los pesos y se midieron los puntos tempranos de `fase3_agacharse` (`results/training_normalizador_fase3.json`, `results/training_gait_fase3_agacharse_diag_*.json`).

| Punto de control | Pelvis con orden 0 / -0,05 m | Sin caer |
|---|---|---|
| `fase3_inicio` (política de la fase 2, sin entrenar) | 0,2275 m / sin seguir la orden | 100 % |
| `fase3_quieto` 9000 (a las 5 iteraciones) | 0,198 / 0,197 | 98 % / 98 % |
| `fase3_quieto` 10000 | 0,198 / 0,197 | 100 % |
| `fase3_agacharse` 9000 (a las 5 iteraciones) | 0,200 / 0,200 | 54 % / 61 % |
| `fase3_agacharse` 9050 | 0,202 / 0,202 | 100 % |
| `fase3_agacharse` 9500 | 0,2266 / 0,2265 (error 1,4 / 48,5 mm) | 100 % |

- **El normalizador no es la causa.** `_mean`, `_std` y `count` casi no cambian en las primeras 5 iteraciones (diferencia máxima 0,09 % de la desviación; `count` pasa de 884,7 a 885,3 millones y sigue creciendo). La entrada nueva (altura) empieza con media 0 y desviación 1 y se queda ahí.
- **El salto ocurre igual en las dos corridas** (`fase3_quieto` y `fase3_agacharse`), así que no lo causa `DANNY_STAND_ONLY`. En ambas, en 5 iteraciones los pesos cambian mucho: primera capa 3,5 %, última 9 % (norma relativa).
- **No se encontró la causa del salto.** Hipótesis, sin probar: al ampliar la capa se descarta el estado del optimizador, y las primeras actualizaciones de un Adam nuevo son grandes. Para comprobarlo habría que entrenar 5 iteraciones con otro ritmo de aprendizaje o conservando el optimizador; no se hizo.
- **Por qué una se recupera y otra no, tampoco se encontró.** `fase3_agacharse` vuelve a 0,2266 m a las 500 iteraciones y acaba siguiendo la orden; `fase3_quieto` se queda en 0,198 m. Datos de acompañamiento: al final, la columna de la altura en la primera capa pesa 29,7 en `agacharse` y 8,4 en `quieto`, y la desviación media de la acción es 0,51 y 0,29. Podría ser que sin marcha no haya señal suficiente, pero no se probó.

**Qué hacer en las fases 4 y 5 (precaución, no regla probada):** si una corrida arranca de un punto de control con una capa ampliada o un optimizador nuevo, medir la política a las 5 y a las 50 iteraciones (altura de la pelvis, caídas) antes de dejarla correr, y parar si la política de partida se rompe. Eso cuesta unos minutos.

**Regla nueva del criterio de saturación (5 de noviembre, a petición del dueño):** desde ahora el criterio de «más del 90 % del bloqueo menos del 5 % del tiempo» cubre **todas** las articulaciones, no solo rodillas y tobillos. Motivo: en `fase3_quieto`, `right_hip_yaw` estuvo al tope el 99,97 % del tiempo y el criterio de entonces no lo veía.

**Estado de la fase 3:** **aprobada (agacharse quieto)** con `fase3_agacharse/model_11994.pt`, usado solo parado; caminar agachado, descartado por ahora.

### Fase 4, levantarse: criterio escrito antes de implementar y entrenar (5 de noviembre)

**Qué se sabe del cuerpo (no se ha calculado para esta fase; viene de la etapa 3).** Una búsqueda sin realimentación (`results/get_up_prone.json`) logra ponerse de pie boca abajo en simulación, con el modelo nominal (altura del centro de masas 0,217 m, inclinación 0,3°), pero **solo en el caso nominal: falla con 6 V, con suelo de 0,5 y de 1,2, y con ±10 % de masa** (altura final 0,04 m, tumbado). El gesto necesita la cadera a -100° (su límite) y el tobillo a -45°. Boca arriba, `roll_over.json` da la vuelta a boca abajo primero. Por tanto, **no está demostrado que se pueda con 6,6 V ni con aleatorización**; la fase puede acabar diciendo que no, y entonces el cambio de diseño (cadera, par) lo decide el dueño. Lo que no hay que hacer: copiar la secuencia de la búsqueda como objetivo (la guía dice que no lo es).

**Tarea nueva** (hay que implementarla; no existe en `training/`): `Mjlab-GetUp-Danny`. El robot empieza tumbado (boca abajo o boca arriba según la corrida) con guiñada aleatoria, articulaciones con ruido pequeño y brazos en la pose de reposo; la observación es la del entrenamiento actual sin el reloj de marcha ni la orden de altura. Recompensas: altura de la pelvis y del tronco, tronco vertical, pies apoyados, mantenerse de pie, y castigos de par excedido (como en la fase 2), cambio de acción, límites de articulación y choques. Sin imitar las secuencias de referencia. Aleatorización de la etapa 3 completa en la prueba; en el entrenamiento, etapa 2 al principio.

**Se levanta** un intento si, antes de 20 s, durante 3 s seguidos se cumple a la vez: `pelvis_bottom` a 0,20 m o más (de pie vale 0,228 m), tronco inclinado 15° o menos, los dos pies en el suelo, velocidad media de 0,05 m/s o menos y ningún eslabón distinto de los pies tocando el suelo. Los 0,20 m son una estimación (88 % de la altura de pie); si se cambian, se dice.

**Corridas** (nombres claros, se conserva el punto de control que aprueba):
1. `fase4_boca_abajo`: desde cero, hasta 5 000 iteraciones, 4096 entornos, empezando siempre boca abajo.
2. `fase4_boca_arriba`: solo si la 1 aprueba; arranca del punto de control aprobado de la 1, con la mitad de los entornos boca abajo y la mitad boca arriba (para no olvidar), hasta 5 000 iteraciones más (10 000 en total, como en el plan).
Si una corrida arranca de un punto de control con capa ampliada u optimizador nuevo, se mide a las 5 y a las 50 iteraciones (altura, caídas) antes de dejarla correr (precaución del diagnóstico anterior).

**Comprobaciones durante la corrida 1** (informan; paran la corrida si fallan, sin alargarla): a las 1 000 iteraciones, al menos el 5 % de los entornos levanta la pelvis a 0,10 m o más en algún momento del episodio; a las 3 000, al menos el 5 % se levanta del todo según la definición de arriba. Si no, se para y se explica por qué (puede significar rediseñar la cadera, y eso lo decide el dueño).

**Aprueba la corrida 1** (boca abajo; 256 robots, 20 s, sin empujones, aleatorización completa de la etapa 3): se levanta el **80 % o más** de los intentos (8 de cada 10)
 a. a 6,6 V fijos (decide);
 b. con la tensión aleatoria de 6 a 8,4 V (decide);
 c. a 7,4 V fijos (informa: dice si un fallo en a es de par).
**Aprueba la corrida 2** (la fase 4 entera): lo mismo, boca abajo y boca arriba por separado, a 6,6 V y con la tensión aleatoria. Una sola política para los dos casos.

**Saturación (todas las articulaciones, regla de hoy):** mientras se mantiene de pie los 3 s, ninguna articulación pasa del 90 % del bloqueo más del 5 % del tiempo. **Mientras se levanta, solo se mide** (máximo y fracción por articulación): subir del suelo exige picos altos y no hay medida previa para fijar un umbral; tras la primera corrida se propone uno con números y se anota el cambio.

**Si no cumple:** se para, se anota lo visto (qué caso falla, a qué tensión, qué articulación satura) y no se alarga la corrida. La fase 5 (sentarse y tumbarse) no empieza si la 4 no aprueba, y la fase 6 no empieza.

**Cómo se verá:** `scripts/record_getup_video.py` (por escribir, como `record_crouch_video.py`), un MP4 por caso en `results/videos/`.
