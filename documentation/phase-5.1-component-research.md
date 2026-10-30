# Fase 5.1: anexo de componentes

Investigación del 30 y 31 de octubre de 2026. **V**, oficial. **S**, tienda o fuente secundaria.
**?**, desconocido. Varias páginas limitaron el acceso, así que los precios de OpenELAB son
instantáneas del buscador. Las curvas se leyeron de las gráficas de los manuales. Confirma todo antes
de comprar. **Nada se ha comprado.**

## Motores

| Modelo | Reducción | Tamaño [mm] | g | Nominal / pico [N·m] | Vacío [rad/s] | Tensión y bus | Precio |
|---|---|---|---|---|---|---|---|
| RobStride RS00 | 10:1 | Ø57 × 51 V | 310 V | 5 / 14 V | 33,0 V | 48 (24-60); CAN | [160,55 €](https://openelab.io/products/robstride00-qdd-14n-m-integrated-joint-motor-module) S |
| RS01 | 7,75:1 | Ø78,5 × 40 V | 380 V | 6 / 17 V | 33-36 V | 36 (24-50); CAN | [138,99 €, agotado](https://openelab.io/a/s/products/robstride01-qdd-17n-m-integrated-joint-motor-module) S |
| RS02 | 7,75:1 | Ø78,5 × 45,5 S | 380 V | 6 / 17 V | 42,9 V | 48 (24-60); CAN | [164,95 €](https://openelab.io/products/robstride02-qdd-17n-m-joint-motor-module-dual-encoders) S |
| RS05 | 7,75:1 | 46 × 46 × 44 S | 191 V | 1,6 / 5,5 V | 50,3 V | 48 (15-60); CAN | [126,45 €](https://openelab.io/products/robstride05-qdd-joint-motor-module) S, stock contradictorio |
| RS06 | 9:1 | 88 × 88 × 49 S | 621 V | 11 / 36 V | 50,3 V | 48; CAN | [189,95 €](https://openelab.io/products/robstride06-qdd-joint-motor-module) S |
| Damiao J4310-2EC | 10:1 | Ø57 × 46 S | 300 S | 3 / 7 S | 20,9 a 24 V S | 24 o 48; CAN | [119,95-166,90 €](https://openelab.io/collections/damiao-motor) S |
| Damiao J4340-2EC | 40:1 | Ø57 × 53,3 S | 362 S | 9 / 27 S | 10,5 a 48 V S | CAN | [188,95-236,12 €](https://openelab.io/collections/damiao-motor) S |
| Unitree GO-M8010-6 | 6,33:1 | 96,5 × 92,5 × 42,3 V | 530 V | ? / 23,7 V | 30 V | 24; RS485 | [280 $ a 631 £](https://robosavvy.co.uk/unitree-8010-motor.html) S |
| Steadywin GIM6010-8 | 8:1 | ? | ? | 5 / 11-15 S | 44 S | 24 | [85-105 $](https://openelab.io/a/s/products/steadywin-gim6010-8-robot-motor) S |
| Feetech STS3250 | 345:1 | 45 × 25 × 35 S | 74,5 S | 1,6 / 4,9 bloqueo S | 7,9 S | 12; TTL | [125 $, agotado](https://rcdrone.top/products/feetech-sts3250-servo-motor) S |
| Waveshare ST3215 30 kg | ? | ? | 69 S | ? / 2,9 bloqueo S | 4,7 S | 12; serie | [29,90 €](https://grobotronics.com/servo-motor-30kg.cm-45rpm-metal-gears-uart-st3215.html?sl=en) S |
| Hiwonder HTD-85H | ? | 65 × 30 × 62 S | 153 S | ? / 8,3 bloqueo S | 5,2 S | 11,1; UART | [50,55 €](https://openelab.io/a/s/products/htd-85h-bus-servo) S |
| Dynamixel XL430-W250 | ? | 28,5 × 46,5 × 34 V | 57,2 V | ? / 1,5 bloqueo V | 6,4 V | 12; TTL | [54,89 $](https://www.roboticscenter.ai/ar/store/product/robotis-dynamixel-xl430-w250-t) S |
| Dynamixel XM540-W270 | ? | 33,5 × 58,5 × 44 V | 165 V | ? / 10,6 bloqueo V | 3,1 V | 12; serie | [332-398 £](https://robosavvy.co.uk/robotis-dynamixel-xm540-w270-r.html) S |

**Manuales:** [RobStride](https://github.com/RobStride/Product_Information). **Curva a 48 V:** RS00, 14
N·m hasta 10,5 rad/s y 10 N·m a 20 rad/s. RS02, 17 N·m a unos 23 rad/s y 10 N·m a unos 34 rad/s.

**Holgura:** publicada solo en XM540, 15 arcmin V, y GIM6010-8, 15 arcmin S. En RobStride y Damiao es
desconocida, igual que el eje hueco y el par de retroceso.

**Descartados:** MyActuator (355-715 $, datos contradictorios); CubeMars AK60-6 (186-299 $ por 3/9
N·m); DS3235 (sin datos verificados); motor con reductora propios (ODrive Micro 89 $, moteus c1 69 $,
y sin coste real del conjunto).

**Importación:** RobStride en la UE por OpenELAB, con almacén en Múnich o Shenzhen. Desde el 1 de julio
de 2026 los envíos desde China [pagan 3 € de arancel por artículo](https://www.tullverket.se/en/startpage/business/infocus/futuredevelopmentsinthecustomsarea/changestolowvalueconsignments.4.389ee53119d71f234982f159.html),
además del IVA.

### Reparto por opción

| Articulaciones | Cant. | A | B | C |
|---|---|---|---|---|
| Rodilla, cadera pitch | 4 | RS00 | RS02 | RS06 |
| Cadera roll, tobillo pitch, cintura pitch | 5 | RS00 | RS02 | RS02 |
| Cadera yaw, tobillo roll, cintura roll y yaw | 6 | HTD-85H | RS05 | RS05 |
| Hombros | 6 | HTD-85H | HTD-85H | RS05 |
| Codos | 2 | ST3215 | HTD-85H | RS05 |
| Muñecas y cuello | 8 | ST3215 | ST3215 | ST3215 |

**A:** 9 × 160,55 + 12 × 50,55 + 10 × 29,90 = 2 350,55 €. **B:** 9 × 164,95 + 6 × 126,45 + 8 × 50,55 + 8 ×
29,90 = 2 886,85 €. **C:** 4 × 189,95 + 5 × 164,95 + 14 × 126,45 + 8 × 29,90 = 3 594,05 €.

**Avisos:** HTD-85H usa potenciómetro y gira 240°, y sus 5,2 rad/s limitan el balanceo. La cintura yaw no
tenía requisito propio: se supusieron unos 3 N·m.

## Marcha humana

| Dato | Usado | Fuente | Veredicto |
|---|---|---|---|
| Tobillo | 1,6 N·m/kg | [1,5 ± 0,2](https://doi.org/10.3390/s21217114) | Conservador |
| Rodilla | 0,9 | [0,52 a 1,3 m/s, un sujeto](https://pmc.ncbi.nlm.nih.gov/articles/PMC7160724/) | Alto: margen deliberado |
| Cadera | 1,0 | [0,65 en flexión](https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0270078) | **Sin verificar** |
| ω tobillo | 6,3 rad/s | [Tarea exigente, 75 kg](https://pmc.ncbi.nlm.nih.gov/articles/PMC7720653) | Valor usado |
| ω rodilla | 7 rad/s | [5,8](https://pmc.ncbi.nlm.nih.gov/articles/PMC7720653) | Prudente |
| ω cadera | 3,5 rad/s | Sin fuente | **Sin verificar** |

## Cómputo

| Dato | Valor | Etiqueta |
|---|---|---|
| Módulo | [69,6 × 45 mm, SO-DIMM de 260 pines](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) | V |
| Potencia | [15 y 25 W, y MAXN SUPER](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules) | V |
| Masa del módulo | ? | ? |
| Precio del módulo | [327,35 €](https://elecena.pl/product/24138825/900-13767-0030-000-nvidia-jetson-orin-nano-40-tops-8-gb-ram) | S |
| Kit de desarrollo | [100 × 79 × 21 mm, 9-19 V](https://www.tomshardware.com/news/nvidias-new-orin-nano-developer-kit-like-a-raspberry-pi-for-ai), 340-590 € | S |
| CAN del Jetson | [1 controlador, FD, transceptor externo](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/HR/ControllerAreaNetworkCan.html) | V |

**Placas:** [Boson-22](https://connecttech.com/?p=19346), [DFRobot A603](https://www.dfrobot.com/product-2806.html),
[Auvidea JNX42](https://store.rebotnix.com/product/jnx42/) y
[Seeed Robotics J401](https://wiki.seeedstudio.com/recomputer_robotics_j401_getting_started/), esta con
1 100 g: demasiada masa. **Disipadores:** [CTI activo](https://connecttech.com/product/nvidia-jetson-orin-nano-active-heat-sink/),
58 × 38 × 18 mm y 49 g.

**Microcontroladores:** [Teensy 4.1](https://www.tinytronics.nl/en/development-boards/microcontroller-boards/teensy/teensy-4.1),
36 € S. ESP32-S3 [no admite CAN FD](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32s3/api-reference/peripherals/twai.html).
**IMU:** [BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/product_flyer/bst-bmi088-fl000.pdf)
e [ICM-45686](https://invensense.tdk.com/?p=18248).

**Baterías:** [Turnigy 6S 5 Ah](https://speedyfpv.com/collections/lipo-batteries/products/turnigy-rapid-6s-5000mah-22-2v-100c-lipo-battery-w-xt90-for-rc-cars),
713 g y 61,79 $ S. [Tattu 6S 10 Ah](https://gensace.de/de/collections/10000mah-lipo-battery), 1 357 g y
214,99 € S. [Gens Ace 12S 5 Ah](https://gensace.de/collections/12s-lipo-battery), 1 488 g y 323 mm S.
[Molicel P45B](https://www.imrbatteries.com/content/molicel_p45b.pdf): 6S1P da 97 Wh con 420 g de celdas.
**Transporte:** un pack de 111 Wh supera los [100 Wh de la exención SP188](https://origin.tnt.com/dam/tnt_express_media/de_ch/download_documents/HELPCENTER_downloads/2017-ROAD_and_SEA_Transport_of_SP188_Lithium_Batteries-December%202016_1.pdf), fuente de 2017.

## Impresora y materiales

Centauri Carbon 2, [fuente secundaria](https://printer-hub.com/posts/elegoo-centauri-carbon-vs-centauri-carbon-2):
256 mm³, cama a 110 °C, boquilla a 350 °C, sin cámara calefactada. [311,58 € base](https://www.3djake.com/elegoo/centauri-carbon-2?sai=43462)
y [361,99 € combo](https://www.3djake.com/elegoo/centauri-carbon-2-combo?sai=40725), con IVA.

**Hojas de materiales:** [ASA](https://cdn.shopify.com/s/files/1/0584/7236/6216/files/Bambu_ASA_Technical_Data_Sheet.pdf?v=1690434954),
[PET-CF](https://polyalkemi.no/wp-content/uploads/2023/11/Bambu_PET-CF_Technical_Data_Sheet_V2.pdf),
[PAHT-CF](https://uwaterloo.ca/rapid-prototyping-centre/sites/default/files/uploads/documents/bambu_paht-cf_technical_data_sheet_v2.pdf),
[PA12-CF](https://cdn.shopify.com/s/files/1/0603/6609/files/PolyMide_PA12_CF_TDS_V5.1.pdf).

## Isaac

[Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) pide RTX 4080
de 16 GB, 32 GB de RAM y Ubuntu o Windows 11. [Isaac Lab 3.0](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)
usa [Newton por defecto](https://docs.nvidia.com/isaaclab/isaac-lab/installation), sin Isaac Sim, y sin
RTX si no renderiza. No hay tabla oficial de entornos por VRAM: hay que probarlo. macOS no aparece.

## Fusion

[Manual de scripts](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WritingDebugging_UM.htm).
[Licencia personal](https://autodesk.com/products/fusion-360/personal), condiciones por confirmar.
[ACDC4Robot](https://github.com/ACDC4Robot/Fusion360) exporta a MJCF.
