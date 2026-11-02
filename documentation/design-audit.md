# Auditoría del diseño

Revisión independiente del 2 de noviembre de 2026, con tres agentes sin contexto previo: mecánica y montaje, simulación, y forma y
movimiento. Se hizo sobre los STL, los XML y `BuildRobot.py`, sin imprimir ni montar nada.

**Estado:** corregidos los 6 bloqueos, el tobillo a -45°, la pose de reposo, los parámetros del servo y la simulación con BAM.
`analysis/check_assembly.py` comprueba los bloqueos del 1 al 5 y falla con las piezas anteriores. Los puntos 6 y 7 de simulación y el resto
de las listas siguen abiertos. Lo marcado como importante debe resolverse antes de imprimir piezas definitivas.

## Bloquea el montaje, corregido

| # | Fallo | Medida | Arreglo |
|---|---|---|---|
| 1 | El carro de electrónica no entra ni sale | Mide 72,8 mm con orejas, la trampilla 68 mm, y el tope de batería cierra el paso | Muescas para las orejas, topes en el carro, ranuras sobre las cabezas de cintura y 2 M1,6 × 4 de cabeza hexagonal desde la trampilla |
| 2 | Dos tornillos del servo de `shoulder_roll`, en cada brazo | El pie de la tapa deja 4 mm de eje para un M2 × 6 | Ranura de 4,4 mm y tornillos a y = 54 y 59 |
| 3 | Pestaña de la chapa de `hip_roll` | Cabeza dentro del eslabón yaw, 7,4 mm³, y la bocina, 4,5 mm³ | Salientes de 3 mm sobre la caja, y pestañas con M1,6 × 5 a ±13: libran la bocina y el agujero del servo |
| 4 | Pernos de `hip_pitch` contra el antebrazo | 47,7 mm³ en pose cero, y choque en 82 poses | Cajera y `FOREARM_Y[0]` = 47 |
| 5 | Cable de `neck_yaw` sin salida | La ventana da a la batería, con 0,8 mm de hueco | Sin espuma: batería 3 mm atrás, y ventana ampliada |
| 6 | Tornillería de la lista | Hacen falta M2 × 12, M2 × 10 y M1,6 × 18; la lista llega a M2 × 8, sin M1,6 | Lista corregida |

**Por qué pasó las comprobaciones:** `SweepJointRanges` mueve sólidos sin tornillos, con un umbral de 1 mm³, y nadie prueba entradas,
longitudes de tornillo ni salidas de cable.

## Importante en mecánica

- La clavija del cuello: quedan 4,1 × 4,2 mm, y una de 3 pines mide unos 8 mm de ancho (E). Hay que medirla con el servo real.
- El antebrazo solo libra el muslo con el brazo abierto: con el brazo a 8° hay sitio hasta unos 35° de `hip_roll`, y en pose cero no. `training/`
  no tiene autocolisiones, así que debe evitar poses en las que atraviese la pierna.

- Cable de `hip_pitch` sin ruta: 0,7 mm a +20°.
- Ranuras de cable del suelo: desde ±49° de cintura quedan sobre la tapa, con 0,5 mm.
- Rodamiento de cintura: pared de 0,43 mm y apoyo axial en el 18 % del aro.
- Panel de muslo: lengüeta suelta de 109 mm³, y las cabezas de `hip_roll` lo pisan a -100°.
- Piloto del panel de tibia: paredes de 0,55 y 0,65 mm.
- Conectores de `ankle_roll` y `hip_roll` sin ventana.
- El rodamiento de 10 × 15 × 3 no tiene asiento en ninguna pieza, y no debe comprarse aún.
- Las rejillas traseras dan a espuma y batería.
- Batería: 65 × 37 × 19 mm no parece dejar sitio al BMS y las pletinas, y el XT30 no cabe.
- Pie: talón en voladizo de 51 mm con placa de 3 mm, 4,7 mm de flecha a 40 N (E).
- Bocina de yaw de brazo: 3,7 N·m de vuelco con 30 N en la mano (E), peor que la cintura. El rodamiento debería ir primero aquí.
- Faltan orientaciones de impresión.

Se comprobó que están bien: 62 mallas estancas, simetría exacta, la pieza mayor mide 103 mm, los servos entran y los rangos
quedan limpios sin contar tornillería.

## Simulación

| # | Fallo | Medida | Consecuencia y arreglo |
|---|---|---|---|
| 1 | El actuador de `danny_sim.py` no es el servo | 8 y 20 N·m/rad, frente a 1,03 del modelo BAM | `stand` con BAM se inclina 3,71° a 7,4 V y 5,47° a 6 V, por encima de los 2° de 3.1. `squat`, `single_support`, `rise_from_crouch` y levantarse caen. Sin rozamiento de carga cae también `stand`. Usar BAM en estas pruebas, y medir el `kp` real del firmware |
| 2 | El pico de marcha es un artefacto | De 0,724 N·m, 0,703 son armadura al salir del tope de tobillo | La fila de marcha y los 6,9 V no valen. Rehacer sin recorte: sin tope da 0,48 N·m y 6,1 V, o 6,7 V con rozamiento de carga |
| 3 | Faltan parámetros de BAM en nuestro JSON | Rozamiento bajo carga de 0,186 N·m, frente a 0,0254. Bloqueo de 0,912 N·m por `max_pwm` 0,97 | Copiar `1910_m6.json` entero y aplicar el 0,97. `training/` ya usa el archivo completo |
| 4 | `danny_training.xml` no trae IMU | 0 sensores | `training/` la añade en `torso_1`, que es correcto. Falta fijarla en la exportación y su orientación física |
| 5 | La fricción del suelo por debajo de 0,8 no hace nada en `danny_sim` | MuJoCo toma el máximo | `training/` ya usa prioridad en las suelas. Copiarlo |
| 6 | Trayectorias imposibles | `sit_up_from_back` mete el antebrazo 70,9 mm en el suelo | Rechazar penetraciones y fricción mayor de 0,8 |
| 7 | Los pares usan 1,19 kg | Los pares de gravedad suben un 8,4 % con 1,29 kg | Repartir los 100 g de tornillos y cables en el modelo |

Se comprobó que están bien: 23 ejes, signos, anclajes, rangos, simetría, inercias, centros de masas, y que `danny_training.xml` es
el mismo robot. Los scripts reproducen `results/`.

## Forma y movimiento

Hoy parece un robot de kit, no un Optimus: es una opinión, sin dato público fiable de las medidas de Optimus.

| Prioridad | Cambio | Motivo | Coste |
|---|---|---|---|
| Alta | `ankle_pitch` a -45° | Está en el tope el 38 % de la marcha, y sin él baja a 0,48 N·m. Las mallas no chocan hasta -60°, sin comprobar cable | 0 g, barrido, exportar y reentrenar |
| Alta | Brazo en reposo a 6-8° y codo a -15 o -20° | A 2,9° atraviesa el muslo a un pie | Solo pose, y reentrenar |
| Alta | Cabeza de unos 42 mm de ancho y forma de huevo | 50 × 37 mm parece una caja | Menos de 10 g (E) |
| Media | Cubrir codo y antebrazo, falda de pelvis, biseles | Hueco de 28 mm y sin cintura visible | 40-60 g (E), con 78 g libres en paneles |
| Media | Paso de 100 mm a 0,40 s de referencia | 78 % a 6 V, frente a 104 % con 75 mm a 0,30 s | Nulo |
| Decisión | R7 a 0,4 m/s | Rodilla por encima de la velocidad en vacío, tope práctico de unos 0,3 m/s con ese perfil | Rebajar R7 u otro actuador |

No conviene añadir cintura pitch o roll: 1 servo por eje, unos 36 g y 25 mm de altura (E), sin hacer falta para caminar. Los pies
tampoco: su anchura viene del servo de `ankle_roll`.

R6 no se cumple con reserva o con manos: 1 546 € y 1 533 €. R10 mide 133,8 mm de hombros, pero la anchura real con codos es 147 mm.

## Sin comprobar

Plano real del servo, tolerancias, entradas con giro, calor, longitud de cables, rosca real, mjlab en esta auditoría y `roll_over` con BAM.
