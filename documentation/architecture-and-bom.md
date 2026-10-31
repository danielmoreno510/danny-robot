# Arquitectura y desglose de piezas

**Etapa 1.3, aprobada por el usuario con 40 cm.** Las cifras salen de `analysis/estimate_robot_budget.py` y
`analysis/compare_actuators.py`, y son previas al CAD: la masa medida en CAD es menor, 1,29 kg (E), en
`results/robot_check.json`. **Comprado por el usuario, sin recibir ni probar a 1 de noviembre de 2026:** 3 servos
HD-1910, FE-URT-2, fuente de alimentación, calibrador digital, báscula de cocina, impresora 3D y filamentos. El
resto de esta lista espera a pasar las etapas 5.1 y 5.2.

**Base:** **V** fabricante, **S** tienda, **E** estimación. Precios en euros con IVA, consultados el 31 de
octubre de 2026.

**Objetivo de forma:** humanoide con proporciones de Tesla Optimus, en pequeño. Del XGO-Duck se toman el
servo y el método de entrenamiento, no la forma.

## 1. Altura: 40 cm

Las medidas del servo no escalan: una altura pequeña obliga a hombros anchos y piernas cortas.

| | Compacto de 36 cm | **Optimus de 40 cm** |
|---|---|---|
| Altura | 360 mm | **395 mm** |
| Pierna / altura | 45,8 %: pierna corta | **50,6 %: humana** |
| Ancho de hombros / altura | 38,9 %: muñeco | **33,4 %: esbelto** |
| Cabeza / altura | 15,3 % | **13,9 %** |
| Masa | 1,38 a 1,54 kg (E) | **1,66 a 1,78 kg (E)** |
| Masa con manos | 1,49 a 1,62 kg (E) | **1,77 a 1,87 kg (E)** |
| Par pico con margen de 1,5 | 51 % del bloqueo | **65 % del bloqueo** |
| Coste | 1 372 € | **1 405 €** |

Las dos masas de 40 cm vienen de dos cálculos distintos: pieza a pieza, y escalando desde Open Duck.

**Decisión: 40 cm.** Es la altura más baja que da proporciones de Optimus. A cambio, **reduce el margen de
par**, y R4 sube a 1,8 kg sin manos y 1,9 kg con manos. Con la estimación alta quedan solo 18 g de margen sin
manos. Los límites de Optimus (hombros 34 % o menos, pierna 49 % o más) son estimación propia, sin medir del
robot real.

Con cualquiera de las dos, **el HD-1910 sigue siendo el candidato**: a 40 cm, un STS3215 C044 en las piernas
sube la masa a 2,19 kg y el par al 73 %.

## 2. Articulaciones

**23 GDL al empezar, 27 con manos.**

| Zona | GDL | Articulaciones | Dónde va el servo |
|---|---|---|---|
| Pierna, 6 × 2 | 12 | Cadera yaw, roll y pitch; rodilla; tobillo pitch y roll | Yaw en pelvis; roll y pitch en cadera; rodilla en muslo; tobillo en tibia y pie |
| Cintura | 1 | Yaw | Pelvis, junto a los de yaw de cadera. Sale hacia el torso |
| Brazo, 4 × 2 | 8 | Hombro pitch, roll y yaw; codo | Pitch dentro del torso; roll en hombro; yaw y codo en el brazo |
| Cuello | 2 | Yaw y pitch | Cuello y cabeza |
| Manos, después | 4 | Muñeca y pinza, × 2 | Antebrazo y mano: espacio reservado |

La pierna lleva 6 GDL, no 5 como el XGO-Duck: el roll de tobillo permite apartar el cuerpo de lado sin
inclinar el pie, y un humanoide de pies estrechos lo necesita. Cuesta 2 servos y 42 g cerca del suelo.

## 3. Medidas por tramo

Previas al CAD. El robot ya está modelado, en [lower-body-design.md](lower-body-design.md) y
[upper-body-design.md](upper-body-design.md): la altura sigue en 395 mm, y la pierna en 200 mm. La pelvis llega hasta unos
72 mm sobre el eje de cadera porque aloja el servo de cintura, así que el torso empieza más arriba.

| Tramo | 36 cm | 40 cm |
|---|---|---|
| Suelo a tobillo | 35 mm | 35 mm |
| Tibia | 65 mm | 82 mm |
| Muslo | 65 mm | 83 mm |
| Cadera | 48 mm | 48 mm |
| Torso | 88 mm | 88 mm |
| Holgura de cuello | 4 mm | 4 mm |
| Cabeza con cuello | 55 mm | 55 mm |
| **Total** | **360 mm** | **395 mm** |

| Pieza | Forma y medidas de partida |
|---|---|
| Torso | Caja de 78 × 54,4 × 68,5 mm en CAD, en dos pisos: abajo electrónica, arriba servos de hombro y cuello. Se afina con los paneles en la 2.4 |
| Hombros | 133,8 mm en CAD: 78 de torso y unos 28 por brazo |
| Pelvis | 81 mm de ancho, con los dos servos de yaw de cadera a lo largo de X, caderas a ±29 mm, y el de cintura entre ellos |
| Muslo y tibia | Viga con el servo dentro, de unos 24 × 29 mm en la caja y 30 × 35 mm arriba, con tapa atornillada |
| Pie | Placa de 85 × 48 mm con suela de TPU |
| Cabeza | 46 × 50 × 37 mm en CAD, más cuello, con cámara detrás de una ventana. El micrófono queda en el pecho, en la reserva de voz |
| Brazo | 177 mm en CAD, un 45 % de la altura. Reserva dos servos en el antebrazo para muñeca y pinza |

Todo el aspecto de Optimus viene de **paneles lisos atornillados sobre el esqueleto**, previstos en 100 g.
Son lo primero que se quita si sobra masa.

## 4. Desglose de piezas

Del robot de 40 cm. Coste por concepto, con repuestos.

| Pieza | Cant. | Medidas (mm) | Forma | Masa montada | Precio ud. | Total | Base |
|---|---:|---|---|---:|---:|---:|---|
| Servo HD-1910-C001 | 23 | 34 × 20 × 23 (29 con ejes) | Caja, eje doble | 483 g | 31,98 € | 735,54 € | V/S |
| Servos de repuesto | 3 | Igual | Caja | - | 31,98 € | 95,94 € | S |
| Radxa Zero 3W 4 GB/32 GB | 1 | 65 × 30 × 8 | Placa | 15 g | 59,83 € | 59,83 € | V/S, masa E |
| Cámara IMX219 | 1 | 25 × 24 × 9 | Placa con lente | 4 g | 20 € | 20,00 € | E |
| IMU BNO085 | 1 | 26 × 18 × 5 | Placa | 3 g | 25 € | 25,00 € | E |
| Regulador 5 V 5 A D24V50F5 | 1 | 18 × 20 × 9 | Placa | 3 g | 30 € | 30,00 € | V/S |
| Placa propia de energía y bus | 1 | 60 × 30 × 12 | Placa a medida | 15 g | 30 € | 30,00 € | E |
| Paquete 2S Molicel P30B con BMS | 1 | 65 × 37 × 19 | Dos cilindros | 104 g | 30 € | 30,00 € | V celda, E paquete |
| Paquete de repuesto | 1 | Igual | Dos cilindros | - | 30 € | 30,00 € | E |
| Interruptor, fusible y XT30 | 1 | 30 × 15 × 12 | Varios | 8 g | 12 € | 12,00 € | E |
| Cables y conectores | 1 | AMP 2.0, 3 pines | Mazo | 40 g | 20 € | 20,00 € | E |
| Rodamientos, packs de 14 | 2 | 10 × 15 × 3 y 16 × 22 × 4 | Anillos | 32 g | 35,95 € | 71,90 € | S, masa E |
| Tornillos M2 autorroscantes e insertos | 1 | M2 × 5 a 8, M2,5 × 6 y fijador | Varios | 60 g | 25 € | 25,00 € | E |
| Estructura en PETG-CF negro y paneles en PETG blanco | 3 bobinas | 2 + 1 kg | Impresa | 880 g | 33 € | 99,00 € | S, masa E; CAD: 555 g |
| Suelas TPU 95A | 1 bobina | 1 kg | Impresa | 10 g | 35 € | 35,00 € | S, masa E |
| FE-URT-2 | 1 | Placa | Banco | - | 15,96 € | 15,96 € | S |
| Fuente de 3-12 V y 5 A | 1 | Caja | Banco | - | 18 € | 18,00 € | S |
| Cargador de 8,4 V | 1 | Caja | Banco | - | 12 € | 12,00 € | E |
| Envíos | 1 | - | - | - | 40 € | 40,00 € | E |
| **Total** | | | | **1 657 g** | | **1 405,17 €** | |

| Coste | 36 cm | 40 cm |
|---|---|---|
| Robot sin manos | 1 372,17 € | 1 405,17 € |
| Con manos (4 servos más) | 1 500,09 € | 1 533,09 € |
| Sin manos, con reserva del 10 % | 1 509,39 € | 1 545,69 € |

**El presupuesto está justo:** los servos son el 59 %. Sin manos entra en 1 500 €, pero la reserva no cabe.
Palancas, sin negociar aún: presupuesto por cantidad a OpenELAB, ArmDojo con 15 servos a unos 25 € sin IVA
ni aranceles, y dejar las manos para después.

**Voz, después (R11):** micrófono digital, amplificador y altavoz, unos 18 g y 18 € (E, sin elegir modelos). No
entra en los totales. Altavoz en la cabeza o en el pecho, según quepa. El uso del servicio en la nube se paga aparte.

Sin contar: impresora, herramientas y soldador, ordenador, boquilla endurecida para PETG-CF, y tornillería
o fallos de impresión por encima de lo estimado.

## 5. Por qué estas piezas

| Elección | Por qué | Descartado |
|---|---|---|
| Radxa Zero 3W | Cabe a lo ancho, muy justa; mismo chip RK3566 que Microduck, que mueve política y cámara | **Raspberry Pi 5:** 85 mm no caben en los 66 mm del torso. Pi Zero 2W: poca memoria para visión |
| Voz en la nube | El modelo grande no cabe a bordo; la placa solo mueve audio, en un proceso aparte del bucle de 50 Hz | IA dentro del robot: más calor, masa y consumo |
| Cámara IMX219 | Radxa la lista como compatible | Camera Module 3: compatibilidad sin confirmar en esta placa |
| BNO085 | Da orientación ya calculada; Open Duck usa un sensor similar | Grove BMI088: descatalogado |
| Paquete 2S de 18650 | Lata de acero, más segura en caídas, y 21,6 Wh con unos 94 g de celdas | LiPo 2S 1500: 11 Wh y blanda ante golpes |
| Placa propia | Reparte corriente en tres cadenas y hace el bus semidúplex | Waveshare Adapter (A): pide 9-12,6 V |
| PETG-CF | Rígido, sin secado exigente y apto para impresora cerrada | PA6-CF: más fuerte, para piezas críticas si hace falta. PLA: solo pruebas |

## 6. Energía y bus

| | Valor (E) | Lectura |
|---|---|---|
| Consumo caminando | 46,5 W | Sin medir |
| Autonomía | 22,3 min | Cumple R5 por poco |
| Pico de piernas | 14,4 A | El BMS debe dar 20 A o más |
| Tiempo de bus por ciclo | 5,7 ms de 20 ms | Cabe; 1 Mbit/s y 23 servos |

Cada servo trae 15 cm de cable y dos conectores. Cada pierna y la parte superior llevan cadena propia, con
la alimentación repartida desde la placa.

## 7. Riesgos de esta arquitectura

1. **Calor:** a 40 cm el pico es 1,75 veces el par nominal, y 2,6 con el margen de 1,5. No hay ficha de par
   continuo. Medir en 5.1.
2. **Sin precedente** de humanoide con brazos y este servo.
3. **Tensión:** la batería cae de 8,4 V a unos 6 V. El servo pierde velocidad y par, y el regulador pide 6 V
   mínimo: hay que entrenar con tensión variable y cortar antes de llegar a 6 V.
4. **Levantarse:** con brazos de 177 mm, sin calcular. Se comprueba en la etapa 3.
5. **Radxa:** stock sin confirmar; la NPU de este chip no está garantizada, y la visión debe funcionar en CPU.
6. **Masa de paneles:** estimada, y sube con cada iteración.
7. **Masa de estructura:** si supera lo previsto, el margen de par baja deprisa.
8. **Conectores del servo:** se sabe que van en muescas de las caras anchas, pero no su medida exacta. Ver
   [upper-body-design.md](upper-body-design.md).
9. **Bocina única:** cintura, hombro, yaw de brazo y cuello cuelgan de una bocina, sin dato de carga radial. Se mide en la 5.1, y
   el rodamiento de 16 × 22 × 4 mm alrededor de la bocina es la mejora prevista.
10. **Batería casera:** los paquetes deben llevar soldadura por puntos y BMS. No se sueldan celdas a mano.

## 8. Para empezar en Fusion

1. Modelo del servo: `BuildRobot` usa el [plano oficial](https://ftservo.github.io/ftservo-wiki/en/products/models/hd-1910-c001/main/).
   **No uses el STEP de esa página:** `HD-1910-C001-20260902.stp` es de un HL-2915, y mide unos 37 × 23 mm.
2. Imprimir una [maqueta del servo a escala 1:1](https://makerworld.com/en/models/3502393-feetech-1910-servo-mock-1-1-dummy)
   para probar encajes sin comprar.
3. Crear el esqueleto con los tramos de la sección 3 como parámetros.
4. Modelar primero la cadera: es lo que más limita la altura.

## 9. Fuentes

Las de [research-and-requirements.md](research-and-requirements.md), más:
[Radxa Zero 3W](https://radxa.com/products/zeros/zero3w/) ·
[Precio RS](https://befr.rs-online.com/web/p/rock-sbc-boards/2564694) ·
[Cámaras de Radxa](https://docs.radxa.com/en/zero/zero3/getting-started/accessory-usage) ·
[Regulador Pololu](https://www.pololu.com/product/2851/specs) ·
[Precio del regulador](https://exp-tech.de/en/products/pololu-5v-5a-step-down-voltage-regulator-d24v50f5) ·
[Molicel P30B](https://nkon.nl/novat/molicel-inr18650-p30b-3000mah-30a.html) ·
[Waveshare Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) ·
[Filamentos Bambu](https://eu.store.bambulab.com/en/collections/fiber-reinforced) ·
[BMI088](https://www.distrelec.de/en/axis-accelerometer-and-gyroscope-seeed-studio-101020584/p/30127498) ·
[Hardware de XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware) ·
[Réplica de Microduck con HD-1910](https://github.com/fanhao375/microduck-replica-cad) ·
[IndieDuck](https://github.com/armature-ai-labs/indieduck)
