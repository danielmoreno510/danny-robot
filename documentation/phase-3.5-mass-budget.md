# Presupuesto de masa con motores de catálogo (etapa 3.5.1)

**Resultado:** el humanoide de 30 GDL es viable, pero el presupuesto da **10,7 kg**, no
los 10 kg aprobados, y reparte la masa de otra forma. **Decisión del usuario: objetivo de
11 kg**, con 0,3 kg de margen, y el reparto de la sección 4.

Este documento no elige motores. La elección definitiva es la etapa 31. Sirve para
comprobar que la propuesta se puede construir, y para dar al modelo masas creíbles.

Todas las cifras son estimaciones. Los datos de catálogo vienen de búsquedas web, en
muchos casos de tiendas y no de hojas de datos oficiales, y deben confirmarse antes de
comprar nada.

## 1. Par que necesita cada articulación

| Articulación | Necesidad estimada | De dónde sale |
|---|---|---|
| Cadera roll | 8,1 N·m de pico, 4,9 N·m eficaz | Marcha medida a 8 kg, escalada a 11 kg |
| Rodilla | 6,8 N·m de pico, 4,0 N·m eficaz | Marcha medida a 8 kg, escalada a 11 kg |
| Cintura pitch | 3,2 N·m | Cálculo estático: inclinada 45°, sin carga |
| Hombro | 1,8 N·m | Brazo horizontal con 0,3 kg en la mano |
| Codo | 0,8 N·m | Antebrazo horizontal con 0,3 kg |
| Muñeca | 0,2 N·m | Con 0,3 kg |
| Cuello | 0,1 N·m | Inclinado 45° |

Son pares de marcha estática lenta y de posturas quietas. **No incluyen levantarse tras
una caída ni marcha dinámica**, que pedirán bastante más. La fase 4 los medirá.

## 2. Motores de catálogo

| Motor | Par de pico [N·m] | Par nominal [N·m] | Velocidad [rad/s] | Masa [g] |
|---|---|---|---|---|
| [Unitree GO-M8010-6](https://www.unitree.com/go1/motor) | 23,7 | sin confirmar | 30 | 530 |
| [RobStride 01](https://www.pakronics.com.au/products/robstride-01-qdd-17n-m-integrated-actuator-module-ss100029452) | 17 | 6 | 33 en vacío | 380 |
| [RobStride 00](https://aifitlab.com/collections/servo-motor/products/robstride-00-motor) | 14 | 5 | sin aclarar: las fuentes discrepan | 310 |
| [Damiao DM-J4310 V1.2](https://eckstein-shop.de/dm-j4310-2ec-v1-2-24v-dual-encoder-servo-motor_1) | 12,5 | 3,5 | 12,6 nominal | unos 325 |
| [Damiao DM-J4340](https://aifitlab.com/collections/servo-motor/products/damiao-dm-j4340-2ec-servo-motor) | 27 | 9 | 3,8 nominal | unos 362 |
| [Dynamixel XM540-W150](https://generationrobots.com/en/403362-dynamixel-xm540-w150-t-servo-motor.html) | 7,3 | — | 5,6 en vacío | 165 |
| [Dynamixel XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350) | 4,1 | — | 4,8 en vacío | 82, sin confirmar |
| [Feetech STS3215](https://core-electronics.com.au/feetech-sts3215-smart-servo.html) | 1,9 a 2,9 según versión | — | sin confirmar | 55 |

**Por qué no el DM-J4340 en las piernas:** tiene mucho par y pesa poco, pero su velocidad
nominal de 3,8 rad/s deja poco margen. La marcha estática pide 0,83 rad/s, y una marcha
dinámica o recuperarse de un tropiezo pedirán mucho más.

**Por qué no solo servos:** el XM540 da 7,3 N·m de pico, por debajo de los 8,1 N·m de la
cadera. Además, los servos con mucha reducción devuelven mal la fuerza del suelo, algo
que importa para pasar del simulador al robot real.

## 3. Asignación propuesta

| Articulaciones | Motor | Unidades | Masa [kg] |
|---|---|---|---|
| Cadera roll y pitch, rodilla | RobStride 01 | 6 | 2,28 |
| Cadera yaw, tobillos, cintura | RobStride 00 | 8 | 2,48 |
| Hombro pitch y roll, y yaw | Dynamixel XM540 | 6 | 0,99 |
| Codo | Dynamixel XM430 | 2 | 0,16 |
| Muñecas y cuello | Feetech STS3215 | 8 | 0,44 |
| **Total** | | **30** | **6,35** |

Los motores son el 59 % de la masa. Margen sobre la necesidad de pico: cadera y rodilla
unas 2 veces, hombro 4 veces, codo 5 veces y muñeca unas 10 veces.

## 4. Presupuesto por segmento

| Segmento | Motores | Otros | Total [kg] | Propuesta aprobada [kg] |
|---|---|---|---|---|
| Cabeza | STS3215 | sensores y estructura 0,25 | 0,31 | 0,55 |
| Cuello | STS3215 | 0,03 | 0,09 | 0,10 |
| Torso superior | RobStride 00 y 2 XM540 | batería 0,60; ordenador 0,30; electrónica 0,25; cables 0,20; estructura 0,50 | 2,49 | 3,00 |
| Cintura | — | 0,20 | 0,20 | 0,35 |
| Pelvis | 3 RobStride 00 y 2 RobStride 01 | 0,31 | **2,00** | **1,30** |
| Hombro × 2 | XM540 | 0,05 | 0,43 | 0,24 |
| Brazo × 2 | XM540 y XM430 | 0,10 | 0,69 | 0,50 |
| Antebrazo × 2 | STS3215 | 0,06 | 0,23 | 0,36 |
| Mano × 2 | 2 STS3215 | 0,05 | 0,32 | 0,20 |
| Muslo × 2 | 2 RobStride 01 | 0,24 | **2,00** | **1,70** |
| Tibia × 2 | 2 RobStride 00 | 0,15 | **1,54** | **1,20** |
| Pie × 2 | — | 0,20 | 0,40 | 0,50 |
| **Total** | **6,35** | **4,34** | **10,69** | **10,00** |

**Qué cambia:**

- La masa baja hacia pelvis y piernas (5,9 kg frente a 4,7 kg), y sale de cabeza y torso.
- Eso es bueno para el equilibrio: el CdM baja.
- Los tobillos pesan en la tibia. Moverlos más arriba con bielas aligeraría la pierna, y
  la decisión corresponde al diseño mecánico (fase 5).

## 5. Dudas abiertas

- **Estructura:** los unos 2,8 kg asignados a estructura, sumando todos los segmentos,
  son una estimación, sin diseño. Es el dato más incierto.
- **Tensión mixta:** los RobStride trabajan a 24-60 V, y los servos a unos 7-12 V. Hace
  falta conversión de tensión, y añade masa y calor.
- **Batería:** 0,6 kg sin cálculo de autonomía.
- **Coste:** no evaluado.
- **Velocidad de hombros:** 5,6 rad/s puede limitar el balanceo de brazos en la marcha
  dinámica.

## 6. Recomendación

1. **Subir el objetivo de 10 a 11 kg**, y usar el reparto de la sección 4 en el modelo.
   Aprobado.
2. Mantener todo lo demás aprobado: 30 GDL, 70 cm y las proporciones.
3. Usar los límites de par y velocidad de la sección 2 en la fase 4, en lugar de los ±20
   N·m provisionales.
