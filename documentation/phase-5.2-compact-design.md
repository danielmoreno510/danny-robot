# Fase 5.2: Danny compacto de 40 cm

**Estado: propuesta para tu revisión.** Nada está comprado. El esqueleto de Fusion **sigue a 70 cm** a
propósito: se actualiza cuando apruebes esta arquitectura. Los modelos de 70 cm no cambian.

**Conclusión:** **recomiendo 40 cm, con 22 GDL y 22 servos STS3215 de 12 V, por unos 1 060 € sin Jetson.**
Las cuentas salen, pero con poco margen. Es una apuesta razonable, no una garantía: **no encontré
ningún humanoide con brazos de este tamaño que camine dinámicamente con estos servos.**

**Etiquetas:** **V**, dato del proyecto o del fabricante. **S**, tienda. **E**, estimación mía. **?**,
desconocido. Fuentes en la sección 9.

## 1. Lo aprendido de Open Duck Mini y Microduck

| | Open Duck Mini v2 | Microduck | Danny 40 cm |
|---|---|---|---|
| Altura | 42 cm V | 25 cm V | 40 cm |
| Masa | 2,107 kg, suma del modelo V; real ? | Menos de 800 g V | 2,69 kg (E) |
| Motores | 14 STS3215 de 7,4 V V | 15, modelo ? | 22 STS3215 de 12 V |
| Pierna | 5 GDL, sin roll de tobillo V | ? | 6 GDL |
| Brazos, cintura | No V | No | Sí |
| Lista de materiales | 398 € V | 399 $ de venta V | Unos 1 060 € |
| Velocidad entrenada | ±0,15 m/s V | ? | Unos 0,2 m/s (cálculo) |

**Aprovechamos:** identificación del servo con BAM, MuJoCo con aprendizaje por imitación, bus serie,
control a 50 Hz y el STEP del STS3215 del proyecto SO-ARM100.

**No aprovechamos:** sus políticas, sus parámetros de 7,4 V en servos de 12 V, ni su límite de ±3,35
N·m del simulador, que es un ajuste de modelo y no un par de diseño. Un constructor cambió las rodillas
a 12 V y la política dejó de funcionar: **los servos de 12 V necesitan su propia identificación.**

## 2. Comparación de 35, 40 y 45 cm

Masa pieza a pieza y pares calculados con el modelo. Salen de `results/compact_sizes.json`. Los
pares son sin margen.

| | 35 cm | **40 cm** | 45 cm |
|---|---|---|---|
| Masa (E) | 2,44 kg | **2,69 kg** | 2,98 kg |
| Sin Jetson | 2,25 kg | 2,51 kg | 2,79 kg |
| Cadera roll, un pie | 0,96 N·m | **1,06 N·m** | 1,17 N·m |
| Rodilla, en cuclillas | 0,61 N·m | **0,80 N·m** | 1,01 N·m |
| Tobillo, marcha escalada | 0,77 N·m | **0,96 N·m** | 1,17 N·m |
| Marcha con velocidad del servo | 0,19 m/s | **0,21 m/s** | 0,23 m/s |
| Altura útil del pecho | 60 mm | **80 mm** | 100 mm |
| Placa del Jetson (59 mm) con paredes | ❌ No cabe | ✅ Justa | ✅ Holgada |
| Se sostiene de pie en simulación | Sí | **Sí, 0,54 mm** | Sí |
| Coste de la lista | Igual | **Unos 1 060 €** | Igual |

**Por qué 40 cm:** a 35 cm el Jetson no cabe de pie en el pecho. A 45 cm todo sube un 10-25 % en par
sobre un servo que no cambia, y apenas se gana velocidad. Ninguna otra altura mejora claramente.

**Los componentes no escalan.** El pie, la cadera y el pecho tienen mínimos fijados por el servo y el
Jetson. Por eso el modelo no es el de 70 cm reducido.

## 3. Proporciones y arquitectura

| Medida | 40 cm |
|---|---|
| Suelo a tobillo / tibia / muslo | 30 / 85 / 85 mm; cadera a 200 mm, 50 % |
| Caderas | ±40 mm |
| Hombros | ±80 mm |
| Pecho | 120 × 80 × 80 mm |
| Brazo / antebrazo / mano | 60 / 55 / 25 mm |
| Cabeza | Ø70 mm |
| Pie | 90 × 50 mm |
| CdM de pie | 217 mm |

**Consecuencia estética:** el pecho de 120 mm viene de la placa del Jetson, y hace a Danny **ancho de
hombros**: unos 19 cm en 40 cm de alto. Sin Jetson podría ser bastante más estrecho.

**22 GDL, de 31:**

| Zona | GDL | Qué se pierde |
|---|---|---|
| Piernas | 6 × 2 = 12 | Nada. Se mantiene el roll de tobillo que Open Duck no tiene: ayuda con pies estrechos |
| Cintura | 1, yaw | Inclinar el torso: se agachará con cadera y rodillas |
| Brazos | 4 × 2 = 8 | Muñecas: las manos quedan rígidas. Se puede añadir después |
| Cuello | 1, yaw | Asentir con la cabeza |

**Se conservan** la contrarrotación del torso, el balanceo de brazos y el giro al andar.

**Colocación de servos:** 3 en la pelvis (2 caderas yaw y cintura), 3 en cada muslo, 2 en cada tibia, 2 en
el pecho (hombro pitch) más el cuello, y 3 en cada brazo. **El 45 % de la masa** son servos.

| Masa | kg |
|---|---|
| 22 servos | 1,21 |
| Estructura y paneles (E, calibrada con Open Duck) | 1,08 |
| Jetson (E) | 0,20 |
| Batería | 0,115 |
| Bus, cables y regulador | 0,086 |
| **Total** | **2,69** |

## 4. Motores

| Modelo | Tensión | Bloqueo | Nominal | Vacío | Masa | Precio | Uso |
|---|---|---|---|---|---|---|---|
| **Feetech STS3215 C018** | 12 V | 2,94 N·m S | 0,98 N·m S | 4,7 rad/s S | 55 g S | 26,51 € con IVA, en reserva S | **Todos** |
| STS3215 C001 | 7,4 V | 1,91 S | 0,49 S | 5,4 S | 55 g S | 24,70 € S | Descartado: cadera y rodilla al límite |
| STS3215 C044 | 7,4 V | 2,7 o 1,6 S, en conflicto | 0,88 S | 9,0 S | 55 g S | 21,89-23,57 € S | Candidato si el conflicto se aclara |
| Waveshare ST3215-HS | 12 V | 1,96 V | ? | 11,1 V | 68 g V | ? | Candidato para rodilla y tobillo |
| Feetech STS3250 | 12 V | 4,9 S | ? | 7,9 S | 75 g S | 115 $ S | Reserva para cadera roll, si hace falta |

Todos miden 45,2 × 24,7 × 35 mm (S), salvo el STS3250. Encoder de 12 bits, holgura de hasta 0,5° y
engranajes de cobre (S). **El par continuo real no está publicado:** ni la curva par-velocidad.

**Margen del C018 a 40 cm, con el factor de 1,5:**

| Articulación | Necesario | % del bloqueo | ¿Supera los 0,98 N·m nominales? |
|---|---|---|---|
| Cadera roll | 1,59 N·m | 54 % | **Sí**, en apoyo a un pie |
| Tobillo pitch | 1,44 N·m | 49 % | **Sí**, en picos |
| Rodilla | 1,20 N·m | 41 % | **Sí**, en cuclillas |
| Cadera pitch | 0,90 N·m | 31 % | No |
| Resto | Menos de 0,4 N·m | Menos del 14 % | No |

**La velocidad limita más que el par.** La marcha humana escalada pide 15 rad/s de rodilla, y el servo da
4,7 en vacío. Eso deja unos **0,21 m/s**, equivalente a una persona paseando a **0,44 m/s**: lento pero
creíble, y coherente con los ±0,15 m/s de Open Duck. **Un paseo normal no es alcanzable con este servo.**

**Prueba independiente del servo:** un usuario sostuvo unos 1,47 N·m a 12 V más de una hora a 48 °C, y
con oscilación de ±90° llegó a 71 °C. Es una sola medida, sin replicar.

## 5. Jetson a 40 cm

**Cabe, pero condiciona el robot.**

| Criterio | Resultado |
|---|---|
| Kit de desarrollo | ❌ 100 × 79 × 21 mm: demasiado grande |
| Pila compacta | Módulo 69,6 × 45 mm, placa Hadron-DM 82,6 × 58,8 mm y 56 g, disipador 48 g |
| Masa | Unos 200 g (E): la del módulo no se ha encontrado. Un 7 % del robot |
| Altura de la pila | ? Estimo 45 mm con ventilador; **hay que confirmarla** |
| Potencia | 7 W mínimo V; 15-25 W Super V |
| Consumo total | 30 W aprox. (E): 14,4 Wh dan unos 30 minutos |

**Mi propuesta: reservar el espacio y empezar a andar sin él.** La política de locomoción cabe en una Pi
Zero 2W, como en Open Duck (15 $, unos 16 g, 3 W). El Jetson entraría para visión. Es tu decisión: **no
lo quito**; el modelo actual lo incluye en masa y volumen.

## 6. Lista de materiales

Sin Jetson, impresora, filamento ni ordenadores.

| Concepto | Cant. | Unidad | Total | Base |
|---|---|---|---|---|
| STS3215 C018 | 22 | 26,51 € | 583,22 € | S, con IVA |
| Servos de repuesto | 3 | 26,51 € | 79,53 € | S |
| Waveshare Bus Servo Adapter (A) | 1 | 5 € | 5,00 € | V 4,99 $ |
| IMU BMI088 | 1 | 25,58 € | 25,58 € | S, con IVA añadido |
| Batería 3S 1300 mAh | 2 | 15 € | 30,00 € | S |
| Cargador | 1 | 35 € | 35,00 € | E |
| Regulador de 5 V, 5 A | 1 | 14,90 € | 14,90 € | S |
| Interruptor y fusible | 1 | 15 € | 15,00 € | E |
| Cables y conectores | 1 | 30 € | 30,00 € | E |
| Rodamientos | 1 | 20 € | 20,00 € | E |
| Tornillos e insertos | 1 | 25 € | 25,00 € | E |
| Ejes y piezas metálicas | 1 | 20 € | 20,00 € | E |
| Envíos a España | 1 | 40 € | 40,00 € | E: coste real ? |
| **Subtotal** | | | **923,23 €** | |
| Reserva del 15 % | | | 138,48 € | |
| **Total** | | | **1 061,71 €** | |

**Con la placa y el Jetson aparte.** Queda muy por debajo de los 3 000 €. Avisos: el C018 aparece en
reserva, y su vendedor envía desde China si no hay stock.

**Primera compra propuesta, solo con tu aprobación:** 1 C018, el adaptador y una fuente de 12 V, unos 60 €.

## 7. Impacto en MuJoCo

| Parte | Decisión |
|---|---|
| `danny_compact.xml` | **Nuevo.** Generado desde `compact_spec.py` |
| Modelos de 70 cm y sus pruebas | Se conservan sin cambios, como referencia |
| Nombres, ejes y topes | Reutilizados; una prueba exige que coincidan |
| Controladores (`Biped`, IK) | No sirven todavía: tienen fijas las longitudes de 70 cm |
| Pruebas de realismo y auditorías | Reutilizables cuando el controlador acepte otras medidas |
| Parámetros del servo | De Open Duck a 7,4 V: **provisionales** hasta medirlo a 12 V |
| Curva par-velocidad | Falta en el modelo compacto |

**Duplicación aceptada por ahora:** `compact_spec.py` repite parte de `build_model.py`. Si apruebas los 40 cm,
el compacto pasa a ser el robot principal y se unifica.

## 8. Riesgos

1. **Sin precedente con brazos:** el más cercano es Z-Bot, de unos 40 cm, cuyos datos no pude confirmar.
2. **Calor:** cadera, tobillo y rodilla superan el nominal. Hay que medirlo.
3. **Velocidad:** el movimiento será lento. Sin pasos rápidos, recuperar el equilibrio será limitado.
4. **Masa estimada:** si la estructura pesa más, el margen baja deprisa.
5. **Holgura de 0,5°:** se nota más en un robot pequeño.
6. **Levantarse del suelo:** sin calcular.
7. **Isaac:** Open Duck entrena en MuJoCo Playground. Si Isaac Lab da problemas con la 3080 Ti, es una
   alternativa probada para este tamaño.

**Decisiones que te corresponden:** aprobar 40 cm y los 22 GDL, y qué hacer con el Jetson.

## 9. Fuentes

[Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini/tree/v2) ·
[Lista de materiales](https://docs.google.com/spreadsheets/d/1gq4iWWHEJVgAA_eemkTEsshXqrYlFxXAPwO515KpCJc) ·
[Sim-to-real](https://github.com/apirrone/Open_Duck_Mini/blob/v2/docs/sim2real.md) ·
[Parámetros](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/joints_properties.xml) ·
[Cambio a 12 V](https://github.com/apirrone/Open_Duck_Mini/issues/52) ·
[Microduck](https://pollen-robotics.com/microduck/blog/introducing-microduck/) ·
[BAM](https://github.com/Rhoban/bam) ·
[STS3215 C018](https://openelab.io/products/feetech-sts3215-c018-servo-12v) ·
[C001](https://openelab.com/products/feetech-sts3215-c001-servo-7) ·
[C044](https://openelab.io/products/feetech-sts3215-c044-servo-7) ·
[ST3215-HS](https://www.waveshare.com/st3215-hs-servo-motor.htm) ·
[STS3250](https://servodatabase.com/servo/feetech/sts3250) ·
[Prueba del servo](https://github.com/TheRobotStudio/SO-ARM100/issues/134) ·
[STEP](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STEP/SO100) ·
[Adaptador](https://www.waveshare.com/bus-servo-adapter-a.htm) ·
[Modos del Jetson](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html) ·
[Placas compactas](https://www.cnx-software.com/2025/06/27/recomputer-mini-is-a-compact-carrier-board-for-the-nvidia-jetson-orin-nano-nx-module/) ·
[Z-Bot](https://github.com/zeroth-robotics/zeroth-bot) ·
[ToddlerBot](https://arxiv.org/html/2502.00893v4)
