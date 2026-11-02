# Diseño de torso, brazos y cabeza

Etapa 2.3, cuarta iteración en CAD. Sigue abierta: faltan los conectores reales y sujetar las placas. Tres revisiones
independientes sondearon los STL; las correcciones de la última no se revisaron de nuevo. Fuente de las medidas:
`cad/fusion/BuildRobot/BuildRobot.py`.

## Criterios de aceptación

Fijados antes de modelar, salvo los dos cambios indicados debajo. `analysis/check_robot.py` comprueba los automáticos.

| Criterio | Límite | Estado |
|---|---|---|
| Electrónica, IMU, regulador, interruptor, cámara y servos caben | Menos de 0,01 mm³ en común dentro del tronco, y cámara dentro de la cabeza | Cumple, sin conectores reales |
| Reserva de voz | Hueco libre de 25 × 64 × 4,7 mm | Cumple |
| Reserva de manos | Dos servos en cada antebrazo | Cumple por cálculo: hueco de 23,6 × 20,1 mm y 67 mm de largo |
| Rango de cada articulación | Menos de 1 mm³ en 9 ángulos, desde su pose de partida | Cumple |
| Pose cero y 25 muestras de poses combinadas | Sin interferencias | Cumple |
| Modelo exportado | Carga, 24 cuerpos, 23 articulaciones | Cumple |
| Masa, ejes y topes | Como en la 2.1 y la 2.2 | Cumple |
| R4 | 1,8 kg o menos, sumando cables, tornillos y paneles | Cumple: 1,290 kg (E) |
| R10 | Altura de 380 a 410 mm, pierna 49 % o más y hombros 34 % o menos | Cumple: 395 mm, 50,6 % y 33,9 % |
| Centro de masas dentro del apoyo | Pies, en pose cero | Cumple: x = 3,8 mm, con apoyo de -30 a 55 mm |
| Montaje y cableado | Revisión independiente sobre los STL | **Parcial: ver Pendiente para cerrar** |

**Criterios cambiados:**

- **Rangos desde una pose de partida.** Con el brazo colgando hay 0,5 mm a la pelvis, así que la regla de la etapa 2.2 ("el
  resto en pose cero") es físicamente imposible: una pierna abierta 35° atraviesa el brazo, igual que en una persona.
  Piernas y cintura se prueban con los brazos abiertos 60°. `shoulder_roll` parte de `shoulder_pitch` -170°, y
  `shoulder_yaw` de `shoulder_roll` 45°.
- **`shoulder_roll` hacia dentro.** El tope de -10° solo se alcanza con el brazo alto. Con el brazo abajo, el contacto real
  llega cerca de 0°: -0,35° según la revisión. Hacen falta colisiones reales en la etapa 3.

R10 se mide en los hombros, como siempre: 133,8 mm. La anchura mayor es 147 mm, en las chapas de los codos, un 37,2 %
de la altura.

Cifras en `results/robot_check.json`, `results/robot_cad.json` y `results/robot_interference.json`.

## Decisiones de diseño

| Decisión | Por qué |
|---|---|
| Pelvis de 81 mm, antes 114 mm, con los servos de yaw de cadera a lo largo de X | Con 114 mm el brazo no pasa junto a la pelvis sin superar R10. Las caderas siguen a ±29 mm |
| Cintura, hombros y cuello en x = 7,5 mm, con el tronco centrado ahí | El tronco gira sobre sí mismo, y el centro de masas se acerca al del pie |
| Tronco de 78 × 54,4 × 68,5 mm, en dos pisos | Abajo, batería y placas de pie. Arriba, servos de hombro y cuello. Sigue los 340 mm previstos de cuello |
| Batería de pie detrás, con trampilla | 37 mm de alto caben en los 38 mm del piso, y se cambia sin desmontar nada más |
| Ordenador y placa de energía sobre un carro | Sale recto por la trampilla, una vez quitada la batería: sus orejas pasan por muescas junto al suelo, y lleva el tope de la batería. Se comprueba con `analysis/check_assembly.py` |
| Brazo con un servo de roll en el propio brazo | Hombro compacto: el eslabón del hombro es solo una horquilla |
| Servo de codo en el antebrazo, con el extremo largo adelante | Hacia atrás, al doblar, bajaría contra muslo y cadera |
| Yaw de brazo, codo y `shoulder_pitch` sobre una sola bocina | No hay sitio para una horquilla sin superar R10. Sin comprobar: ver Pendiente |
| Cabeza con cuello en 55 mm, con cráneo que baja desde arriba | El cráneo de 46 × 50 × 37 mm empieza a z = 358 mm para no tocar el tronco al mirar abajo 40° |
| Cables de brazos, piernas y cabeza por fuera, con bucle | Dentro, cada giro los pellizca. Las salidas están fuera del barrido de hombro y cuello (r ≥ 21 mm) |
| Rodamiento de 16 × 22 × 4 mm en la cintura | Añadido en la etapa 2.4: ver [panels-and-protection.md](panels-and-protection.md) |

## Tornillos del servo

El plano oficial marca ⌀1,6 en los cuatro agujeros de fijación y en los de la bocina, en las dos caras. Son pilotos para
**tornillos autorroscantes M2**, desde fuera y por cada cara: es lo que usan robots ya montados con este servo. XGO-Duck
usa M2 × 6 avellanados, y la réplica de Microduck M2 × 5, 6 y 8 de punta plana con fijador 243. Con M2 × 6 entran unos 3 mm
en el servo (E), con paredes de 2,5 a 3 mm. La profundidad exacta sigue sin publicarse. El tornillo central del eje es M2,5 × 6.

## Montaje

Revisado sobre los STL con rayos, sin montar nada real. Pelvis y piernas, en [lower-body-design.md](lower-body-design.md).

1. Torso sobre la cintura: 4 tornillos en el suelo, con llave recta por el agujero del estante, antes del servo de cuello.
2. Carro con ordenador y placa de energía, por la trampilla. Se fija con dos M1,6 × 4 de cabeza hexagonal desde la trampilla, con una llave de bola en L. Batería y trampilla, sin espuma: la batería se apoya en la tapa, y deja delante el paso del cable del cuello.
3. Servos de hombro: entran por la ventana lateral. Los tornillos interiores se ponen con llave en L por el hueco del
   cuello. Después, su tapa.
4. Servo de cuello: baja desde arriba, entre los tabiques. Regulador, IMU e interruptor. Tapa superior.
5. Eslabón del hombro: tornillos de bocina antes de poner el brazo.
6. Brazo: servo de roll con su tapa, servo de yaw en la copa, y la repisa sobre la copa con 2 M1,6 detrás. El pie de la tapa de roll añade 2 M1,6 delante. Entra en la horquilla.
7. Eslabón del codo sobre la bocina inferior de yaw, antes del antebrazo. Antebrazo con su servo y tapa, y luego su
   tornillería por fuera.
8. Plato del cuello. Servo y tapa en el bloque de la cabeza, que baja entre la horquilla. Cámara. Cráneo por arriba, con
   2 tornillos arriba y 1 detrás.

## Cables

| Tramo | Recorrido |
|---|---|
| Entre pisos | Paso de 8,9 × 10 mm en el estante, en x = 18 a 27 mm |
| Piernas | Ranuras del suelo en x = 29 a 32 mm, delante de la pelvis, y al servo de yaw por la ventana alta de su costado. Se sujeta por encima de z = 261 mm, para que el antebrazo no lo pellizque |
| Brazos | Salidas en el frontal, y a y = ±30 a 36,5 mm, con bucle de unos 25 a 30 mm (E) por fuera del hombro |
| Cabeza | Ranura de la tapa superior en r = 21 a 24,7 mm, y muesca del cráneo |

La cintura y `neck_yaw` sacan el cable por una cara de bocina: arriba de la pelvis y por el estante.

Hay que anclar los bucles para que no entren en las rendijas de 3,1 mm entre brazo y tronco, y de 3,5 mm entre tronco y
pelvis.

## Cómo reproducirlo

Los comandos y cómo verlo están en [lower-body-design.md](lower-body-design.md#cómo-reproducirlo).

## Pendiente para cerrar la 2.3

- **Conectores: dimensiones.** Cada cara ancha del servo tiene una muesca con un conector AMP 2.0 de 3 pines, a 2 mm de paso.
  Según las fotos de la guía de XGO-Duck, la clavija entra paralela al eje, y el cable sale por esa muesca. Hay que
  enchufar antes de atornillar el servo. Se encadenan por los dos puertos en XGO-Duck y Microduck. Falta la medida
  exacta de la muesca, y en los servos más encerrados el cable debe seguir su ventana sin forzarse.
- **Bocina única.** La ficha no da carga radial. La cintura ya lleva rodamiento alrededor de la bocina, como hace XGO-Duck.
  `shoulder_pitch`, yaw de brazo y `neck_yaw` no lo llevan. En una caída de unos 30 N en el hombro llegan 1,7 N·m a la
  cintura (E). Sigue siendo criterio de la etapa 5.1.

## Limitaciones conocidas

- **Dimensiones:** batería, conector de carga, cámara y placas salen de medidas estimadas. La cámara debe comprobarse con el
  módulo elegido. El paquete de batería solo tiene 68 mm de paso.
- **Sujeción:** ordenador y placa solo van guiados 7 mm en el carro, sin retención arriba, y el regulador no tiene fijación.
  **Falta para poder montarlos:** se decide con las placas en la mano, porque depende de sus agujeros.
- **Hombro y cuello:** las placas dan 32 y 37 MPa a par de bloqueo (E). Hay que limitar el par o añadir nervios.
- **Suelo del torso:** imprimiéndolo sobre la cara trasera, las capas quedan cruzadas en la raíz del saliente de la cintura,
  con unos 28 MPa a 0,8 N·m (E). Partir el torso por el estante lo resolvería, y daría acceso, a costa de más tornillos.
- **Servicio:** para llegar a regulador, IMU o interruptor hay que quitar la cabeza, el plato y la tapa superior.
- **Calor:** hay rejillas, sin cálculo ni medida.
- **Resistencia:** órdenes de magnitud, sin simulación ni ensayo.
- **Aspecto:** ver [panels-and-protection.md](panels-and-protection.md).
