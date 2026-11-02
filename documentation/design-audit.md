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

## Segunda auditoría, 2 de noviembre de 2026

Dos revisores independientes sobre el commit `aa7dd8c`: uno recorrió cada acción con las mallas, tornillos y servo BAM, y otro comparó la versión
final con el plano del servo. Nada se imprimió. **Conclusión: todavía no conviene imprimir piezas definitivas.** Se confirmó la caída hacia atrás
con su simulación, y en el código que ninguna cara de bocina tiene alivio central. El resto sale de las medidas de cada revisor.

### Prioridad

| # | Fallo | Medida | Arreglo |
|---|---|---|---|
| 1 | En una caída hacia atrás, el cráneo golpea primero | 1,24 J y 423 N, unos 14 N·m sobre `neck_yaw` (E), con un bloqueo de 0,9 N·m | Hecho: joroba de TPU hasta x = -29, de unos 6 g (E). Sin volver a simular la caída |
| 2 | Las 38 caras de bocina atornilladas no tienen alivio para el tornillo central | Sólido a 14,5-14,7 mm del eje, y la cabeza llegaría a unos 15,2 mm según el plano | Cajera de ⌀7 × 1 mm, tras medir C1 y C2 |
| 3 | Longitud de tornillos | 28 tornillos de bocina necesitan M2 × 4: uno más largo toca la caja | Tabla de totales, abajo |
| 4 | Ventanas de conector | La muesca va solo en una cara, de h = 3,8 a 11,5, y la ventana de -6 a +6 | Ventana de h = 3 a 13, tras medir C7 y C8 |
| 5 | La simulación y el entrenamiento no ven ciertos choques | `roll_over` mete `hip_roll_link` 13,6 mm en el suelo. Sin caja: `hip_yaw`, `hip_roll`, tobillo, hombro y cuello | Hecho: todas las piezas con colisión y 13 pares entre eslabones. Hay que reentrenar |
| 6 | Pared de 0,30 mm en la chapa de `hip_roll`, y de 0,45 mm en la cintura | Agujeros y asiento demasiado justos | Hecho, y con reborde de 0,4 mm bajo el rodamiento |
| 7 | Cables | Cintura y hombro apenas tienen margen, y `hip_pitch` 0,7 mm | Chaflán hecho, y ranuras del suelo reducidas. El bucle de 40 mm queda para el montaje |

También hechos: masa del modelo a 1,30 kg, nervio del talón, panel de tibia, cajeras delanteras de `hip_roll` y agujeros de orejas. Siguen abiertos los puntos 2 y 4, que
dependen del servo real, además de rejillas traseras, BMS y `arm_yaw` con rodamiento.

### Límites por acción

- **Posible:** de pie, cuclillas, brazos arriba, adelante y a los lados, bracear, mirar y saludar a 1,5 Hz.
- **Con límite:** caminar con paso de hasta 140 mm, girar 20° por paso, paso lateral, a un pie, sentarse con los pies bajo el asiento, sentarse sobre los talones, tocar
  el suelo con la mano y llevar las manos a cabeza y cintura.
- **Imposible hoy:** arrodillarse erguido y llevar la mano al hombro contrario.
- **Sin demostrar:** sobrevivir a caídas, darse la vuelta y levantarse con servo BAM.
- **Límites nuevos para la política:** cintura de 44° con los brazos colgando. Con brazo a 8°, `hip_roll` llega a -20,9° antes de chocar. Las caderas chocan
  al abrir ambas 14°. Unos 5 min a un pie llevarían `hip_roll` al corte de temperatura (E).

### Tornillería prevista

Sale de medir agarre y rosca en cada unión, y se revisará tras C1 a C5: 327 M2 (64 × 4, 205 × 5, 28 × 6, 12 × 8, 18 × 10) y 31 M1,6 (2 × 3, 4 × 4, 4 × 5, 9 × 6, 8 × 8, 2 × 16, 2 × 18).
Unos 70 g (E). No hay ningún inserto ni M2 × 12. Un autorroscante de cabeza alomada, de 1,4-1,6 mm (E), no entraría en cajeras de 1,2 mm.

### Medidas con el servo real

| # | Cota | Plano | Decide |
|---|---|---|---|
| C1 | Sobre cabezas centrales, cara a cara | Unos 30,3 mm | Cajera |
| C2 | Alto y ⌀ de cada cabeza; si la de la cara libre se quita sin soltar nada | 0,6-0,7 y ⌀5 | Alivio |
| C3 | Disco de bocina y hueco a la caja, por cara | 2,6 y 0,4 | Longitud máxima |
| C4 | ⌀ de bocina | 16 ± 0,2 | Rodamiento |
| C5 | Fondo de agujeros de bocina y fijación, y si son pasantes | Sin dato | Rosca útil |
| C6 | Cuerpo y fijación 30 × 16, en los 3 servos | ± 0,2 | Hueco |
| C7 | Muesca: 4,6-12,7 a lo largo, 4,6-10 de ancho, 7,7 de fondo | Leído | Ventanas |
| C8 | Clavija: sección, largo y radio del cable | Sin dato | Ventanas |
| C9 | Qué cara es motriz, y si giran juntas | Sin dato | Uniones de una bocina |
| C10 | Masa y longitud de cable | 21-22,5 g y 15 cm | Masas |

**Antes de medir C1 a C5, no metas ningún tornillo en bocina ni servo.**

### Primera tanda de prueba en PLA

Sin rodamiento ni electrónica: `left_hip_roll_link_structure`, `cover` y `plate`; `left_shoulder_link_structure`; `left_upper_arm_plate`, `structure` y `cover`; y `torso_tray_plate`.
Prueba la entrada del servo de `hip_roll`, inclinado 7° y sin margen, el asiento en bocina, el acceso de herramienta y las paredes finas.

### Cifras por corregir en otros documentos

Masa de pierna y pelvis, centro de masas, tornillos de chapa, pared del tornillo del servo, tornillos superiores de cintura, reserva de manos y paso de 68 mm entre carriles.
