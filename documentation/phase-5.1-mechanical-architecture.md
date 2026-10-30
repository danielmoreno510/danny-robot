# Fase 5.1: arquitectura mecánica y esqueleto de CAD

**Estado: entregada para tu revisión.** Nada está comprado ni decidido de forma irreversible. El
script de Fusion **no se ha ejecutado en Fusion**: Fusion no está instalado aquí. Se probó con un
sustituto de su API.

**Conclusión:** Danny se puede construir con 70 cm y unos 11 kg, con piernas de motor BLDC integrado
por unos 3 500-5 300 € en total. La lámina de concepto sirve como objetivo visual, pero sus motores,
su masa de 5-7 kg y su batería no valen. Hay tres riesgos que debes conocer antes de seguir:

1. **Calor:** ningún motor asequible cumple con seguridad el par continuo de rodilla y cadera.
2. **Cintura, cadera y cuello:** en el modelo, sus ejes se cruzan en un punto. Con motores reales
   no caben así, y las alturas tendrán que crecer.
3. **Isaac Sim:** la RTX 3080 Ti queda por debajo de su mínimo publicado.

**Etiquetas:** **V**, dato oficial. **S**, tienda o fuente secundaria. **E**, estimación propia.
**?**, desconocido. Los precios son de octubre de 2026, sin envío y con el IVA sin confirmar.

## 1. Auditoría del humanoide de MuJoCo

Datos de `simulation/models/danny_humanoid.xml`, generado desde `robot_spec.py`.

| Propiedad | Valor | Estado |
|---|---|---|
| Altura | 700 mm | Validada en el modelo |
| GDL | 31: piernas 6 × 2, cintura 3, brazos 7 × 2, cuello 2 | Validados en simulación |
| Masa | 11,004 kg | **Provisional:** presupuesto, sin diseño |
| CdM en `stand` | 370 mm sobre el suelo | Provisional, depende de la masa |
| Inercias | De cajas y cápsulas sólidas | **Provisional:** sin relación con piezas reales |
| Ejes | REP-103; x adelante, y izquierda, z arriba | Validados |
| Motores | Par y velocidad de catálogo | Provisional; `armature` y amortiguamiento son estimaciones |
| Colisiones | Cajas, cápsulas y esferas simples | Provisional: no contienen motores |

**Jerarquía:** pelvis → cintura yaw → roll → pitch → torso → hombros y cuello. Pelvis → cadera yaw,
roll, pitch → rodilla → tobillo pitch, roll.

**Longitudes [mm]:** suelo a tobillo 40, tibia 160, muslo 160, cadera a centro de pelvis 50,
caderas ±70, pelvis a cintura 40, cintura a hombros 140, hombros ±110, brazo 130, antebrazo 120,
mano 50, cabeza Ø90, pie 140 × 80.

**Masas [kg]:** pelvis 2,0; cintura 0,51; torso 2,48; hombro 0,215 × 2; brazo 0,347 × 2; antebrazo
0,115 × 2; mano 0,16 × 2; cuello 0,09; cabeza 0,31; muslo 1,0 × 2; tibia 0,77 × 2; pie 0,2 × 2.

**Topes [°]:** cadera yaw ±40, roll ±30, pitch -90 a 30; rodilla 0 a 130; tobillo pitch -65 a 45,
roll ±30; cintura yaw ±45, roll ±20, pitch -15 a 45; hombro pitch -150 a 60, roll -10 a 120, yaw
±90; codo 0 a 140; muñeca ±90, ±45, ±45; cuello ±80, -30 a 45.

**Diferencias con el catálogo real:**

- El manual de RobStride da 6 N·m nominales en el RS01 y RS02, con disipador de 260 × 280 mm. El RS00
  da 5 N·m con 90 × 85 mm. **Un robot no tendrá esos disipadores.**
- La velocidad del RS00 queda resuelta: 33 rad/s, como usa el modelo.
- Los Dynamixel publican par de bloqueo, no nominal: el modelo usa ese dato como pico.

## 2. Requisitos mecánicos

Calculados con `simulation/scripts/estimate_joint_loads.py`. Salen de
`results/joint_load_estimates.json`, con margen de 1,5 sobre el mayor de dos casos: posturas quietas
del modelo y marcha humana escalada con número de Froude. El continuo se estima como la mitad del
pico.

| Articulación | Estático | Marcha escalada | **Pico** | **Continuo** | Velocidad |
|---|---|---|---|---|---|
| Rodilla | 8,8 | 4,0 | **13,2 N·m** | 6,6 | 11 rad/s |
| Cadera roll | 8,0 | 4,0 | **12,0** | 6,0 | 2,4 |
| Tobillo pitch | 0,3 | 7,0 | **10,6** | 5,3 | 10 |
| Cadera pitch | 1,4 | 4,4 | **6,6** (usar unos 10) | 3,3 | 5,5 |
| Cintura pitch | 4,2 | — | **6,4** | 3,2 | unos 3 (E) |
| Cintura roll | 2,0 | — | 3,0 | 1,5 | ? |
| Tobillo roll | 0 | 1,8 | 2,6 | 1,3 | 2,4 |
| Hombro | 1,7 | — | 2,5 | 1,3 | unos 5 (E) |
| Cadera yaw | 0 | 1,3 | 2,0 | 1,0 | 2,4 |
| Codo | 0,7 | — | 1,1 | 0,5 | unos 5 (E) |
| Muñeca y cuello | 0,2 a 0,3 (E) | — | — | — | ? |

**Lo que no cubre:** levantarse del suelo, recuperarse de un tropiezo ni una marcha dinámica medida.
Solo los casos quietos salen del modelo. La escala humana da un orden de magnitud: tobillo y rodilla
se contrastaron, y **la cadera no se pudo verificar**. La rodilla humana típica ronda 0,5 N·m/kg, y
dejé 0,9 como margen.

Otros requisitos: 31 GDL, topes de la sección 1, altura de 700 ± 20 mm, masa objetivo de 11 kg,
piezas dentro de 256 mm de impresora, y mantenimiento sin desmontar el robot entero.

## 3. Proporciones

**Se mantienen las proporciones del modelo como punto de partida:** ya están validadas, y la cadera a
51 % de la altura es parecida a la humana. La cintura, la cadera y el cuello crecerán al colocar
motores reales (sección 12), y eso se decidirá en el diseño de la pelvis y las piernas.

**La lámina de concepto frente a la realidad:**

| Elemento de la lámina | Evaluación |
|---|---|
| 700 mm, cuerpo blanco y juntas negras, cabeza negra lisa | ✅ Objetivo visual válido |
| Ancho de 280 mm | ✅ El modelo da 290 mm de hombro a hombro |
| 5-7 kg | ❌ Solo los motores suman unos 6,3 kg |
| XL430 y STS3215 en cadera y rodilla | ❌ 1,5 a 2,9 N·m frente a 12-13 N·m |
| DS3235 | ❌ 3,4 N·m según la lámina, y sin datos verificados |
| Raspberry Pi 5 y PCA9685 | ❌ Decidiste Jetson, y los buses necesitan CAN |
| LiPo 4S | ❌ Los RobStride trabajan a 24-60 V; 4S da 12-16,8 V |
| Manos de 1-2 GDL | Opcional: el modelo tiene manos rígidas |

Es diseño propio inspirado en esa estética, no una copia del robot de Tesla.

## 4. Arquitectura de articulaciones

**Propuesta: mantener los 31 GDL.** Cadera de 3 ejes, cintura de 3 y brazo de 7 son los que permiten
contrarrotar torso y pelvis y balancear los brazos.

| Cuestión | Propuesta |
|---|---|
| Recortar sin dañar la marcha | Muñecas de 3 a 1 GDL y cuello a yaw: ahorro bajo, unos 30 € por servo |
| No recortar | Las 12 de pierna, cintura yaw y hombros |
| Tobillo | Evaluar bielas para subir la masa a la tibia; decide el diseño de la pierna |
| Ejes cruzados | Aceptar desplazamientos pequeños y llevarlos al modelo de MuJoCo |
| Manos | Rígidas en esta versión |

## 5. Motores

Los datos completos, con fuentes, están en el [anexo de componentes](phase-5.1-component-research.md).

| Modelo | Tamaño [mm] | Masa | Nominal / pico | Vacío | Precio |
|---|---|---|---|---|---|
| **RobStride RS02** | Ø78,5 × 45,5 S | 380 g V | 6 / 17 N·m V | 42,9 rad/s V | 164,95 € S |
| RobStride RS00 | Ø57 × 51 V | 310 g V | 5 / 14 V | 33 V | 160,55 € S |
| RobStride RS05 | 46 × 46 × 44 S | 191 g V | 1,6 / 5,5 V | 50,3 V | 126,45 € S |
| RobStride RS06 | 88 × 88 × 49 S | 621 g V | 11 / 36 V | 50,3 V | 189,95 € S |
| Damiao J4340-2EC | Ø57 × 53,3 S | 362 g S | 9 / 27 S | 10,5 a 48 V S | 188,95-236,12 € S |
| Hiwonder HTD-85H | 65 × 30 × 62 S | 153 g S | ? / 8,3 bloqueo S | 5,2 S | 50,55 € S |
| Waveshare ST3215 30 kg | ? | 69 g S | ? / 2,9 bloqueo S | 4,7 S | 29,90 € S |

**Recomendación: RS02 para las piernas.** Es la mejor mezcla de par, velocidad y tamaño, y se
compra en la UE por OpenELAB. No encontré distribuidor en España. Tiene un riesgo: sus 6 N·m
nominales requieren un disipador que el robot no tendrá.

## 6. Cómputo con NVIDIA Jetson

**Propuesta: módulo Jetson Orin Nano 8 GB (327 € S) con placa compacta, no el kit de desarrollo.** El
kit mide 100 × 79 × 21 mm y sirve para empezar a programar.

| Placa | Tamaño [mm] | Entrada | Masa | Precio | Nota |
|---|---|---|---|---|---|
| Connect Tech Boson-22 | 90 × 75 | 9-36 V | 80 g | 609 $ S | Admite 6S directo, modo Super V; cara |
| DFRobot A603 | 87 × 52 × 26 | 9-19 V | 100 g | 199 $ S | Necesita conversor; CAN por confirmar |
| Auvidea JNX42 | ? | 12-48 V | ? | 215 € sin IVA S | Admite 6S; en espera |

**Control de bajo nivel:** un microcontrolador entre Jetson y motores, con seguridad y parada. El
Jetson solo tiene un controlador CAN. Con 12 motores a 500 Hz y 1 Mbit/s hacen falta **3 buses**
(cálculo). Teensy 4.1 (36 € S) tiene 3, y es el primer candidato. Se decide al elegir motor.

**Refrigeración:** reservar ventilador de 40 × 40 × 10 mm y disipador de unos 63 × 40 × 18 mm. Ningún
disipador pasivo publica su capacidad a 15-25 W.

## 7. Electrónica y batería

**Espacio disponible:** el torso tiene 120 × 170 × 140 mm, unos 2,9 L, y debe alojar además un motor
y los hombros.

| Elemento | Tamaño de referencia [mm] | Masa | Lugar |
|---|---|---|---|
| Jetson y placa | 87 × 52 × 44, con disipador | unos 200 g (E) | Torso, con entrada y salida de aire |
| Batería | 146 × 53 × 48 (6S 5 Ah) | 700-800 g S | Parte baja del torso, para bajar el CdM |
| Control y distribución | unos 100 × 60 × 30 (E) | unos 250 g (E) | Torso |
| IMU | BMI088 u otra | pocos gramos | Cerca del centro de la pelvis |

**Batería: sin decidir.** Consumo estimado al andar: unos 130 W (E), con 55 W de marcha, 25 W de
Jetson y 50 W de motores sosteniendo. 100 Wh darían unos 45 minutos (E).

| Opción | Tensión | Pro | Contra |
|---|---|---|---|
| 6S | 18-25,2 V | Packs compactos; 97 Wh en 6 celdas P45B | Menos velocidad que a 48 V: hay que medirla |
| 12S | 36-50,4 V | Toda la velocidad del motor | Packs de 1,5 kg y 323 mm: no caben bien |

La masa presupuestada es 0,6 kg. **Seguridad mínima:** BMS, fusible principal, parada de emergencia
física, y carga y almacenamiento ignífugos. No encontré fuente oficial para requisitos españoles.

## 8. Fabricación

**Impresora:** Elegoo Centauri Carbon 2, 256 mm³, boquilla de acero a 350 °C. **Datos de tiendas:** la
web oficial no respondió. **No tiene cámara calefactada**, así que el PA-CF puede deformarse: hay que
probarlo.

| Material | Tracción XY / Z [MPa] | HDT 1,8 MPa | Uso propuesto |
|---|---|---|---|
| PLA | 39 / 35 | ? | Maquetas de ajuste |
| PETG | 34 / 23 | 62 °C S | Piezas sin carga, no cerca de motores |
| ASA | 52 / 35 | 92 °C | **Paneles exteriores** |
| PET-CF | 74 / 35 | 182 °C | **Estructura, primera opción** |
| PAHT-CF | 92 / 47 | 170 °C | Estructura exigente, si no se deforma |

Valores del fabricante en probetas de otra impresora. **La resistencia en Z es cerca de la mitad:**
orienta las piezas con la carga en el plano de capa. Metal donde importa: ejes, rodamientos y
insertos roscados.

## 9. Coste

| Concepto | A: mínimo | B: **recomendado** | C: prestaciones |
|---|---|---|---|
| Motores (S) | 2 351 € | **2 887 €** | 3 594 € |
| Jetson, placa y disipador | 500 € | 570 € | 570 € |
| Control, buses e IMU | 80 € | 100 € | 100 € |
| Batería | 120 € | 150 € | 250 € |
| Potencia, seguridad y cables | 120 € | 150 € | 200 € |
| Filamento, metal y tornillos | 350 € | 450 € | 600 € |
| **Total aproximado** | **3 500 €** | **4 300 €** | **5 300 €** |
| Masa de motores | 5,32 kg | 6,34 kg | 7,61 kg |

Solo la fila de motores viene de precios reales. **El resto son estimaciones mías.** No incluye
impresora (312-362 € S), envío, aduanas, repuestos ni imprevistos: conviene sumar un 15-20 %.

| Opción | Capacidad | Riesgos | Mejora posible |
|---|---|---|---|
| A | De pie y pasos cortos, no marcha sostenida | RS00 no llega al continuo; HTD-85H con potenciómetro | Cambiar rodilla y cadera |
| B | Objetivo completo, a falta de medir el calor | Rodilla al límite térmico | Mejor disipación |
| C | Más margen en rodilla y cadera pitch | 1,3 kg más; RS06 mide 88 mm | — |

**Prototipo de una articulación:** RS02 con placa de depuración, **186,46 € S**, más fuente de 48 V y
pieza impresa: unos 270 € (E). **Es el siguiente gasto recomendado**, solo con tu aprobación.

## 10. Proyecto de Autodesk Fusion

**Automatización:** sí, con la API de Python. Es adecuada, porque el diseño queda editable. Según
las fuentes consultadas, la licencia personal permite scripts, con 10 documentos editables y uso no
comercial. **Esas condiciones debes confirmarlas al registrarte.** Un agente no puede ejecutarla sin
ti: pulsas Ejecutar, y el script deja un informe que el repositorio comprueba.

Estructura creada por el script, la que propusiste:

```
00_Master_Skeleton   01_Pelvis   02_Left_Leg/...   03_Right_Leg/...   04_Waist
05_Torso   06_Left_Arm/...   07_Right_Arm/...   08_Hands   09_Neck   10_Head
11_Electronics   12_Battery   13_Exterior_Panels
```

**Exportadores:** ACDC4Robot, que exporta a MJCF, pide componentes planos, cuerpos sólidos y juntas
sin lazos cerrados. Tu estructura anidada no lo cumple tal cual. Propongo mantenerla ahora y decidir
al llegar a las juntas, porque también podemos actualizar `robot_spec.py` a mano con las masas de
CAD. Exportar CAD no crea un modelo físico válido.

## 11. Esqueleto paramétrico

Archivos en [cad/fusion/DannyMasterSkeleton](../cad/fusion/DannyMasterSkeleton):

| Archivo | Qué es |
|---|---|
| `DannyMasterSkeleton.py` | Script de Fusion |
| `skeleton_parameters.json` | Datos generados con `export_cad_parameters.py`; no se edita |
| `DannyMasterSkeleton.manifest` | Descripción del script |

**Qué crea:** 16 parámetros de usuario, los componentes, dos bocetos totalmente restringidos con cotas
ligadas a parámetros, y puntos de construcción con nombre en cada articulación. Después sube el muslo
10 mm, comprueba que la cadera sube, lo restaura y escribe un informe.

**Qué no crea todavía:** ejes de construcción, juntas, volúmenes ni soportes. Los ejes y topes están
en `joints` del JSON y entrarán con la pelvis y las piernas.

**Pasos en tu Mac:**

1. Registra la licencia personal en autodesk.com y descarga Fusion.
2. Abre Fusion y comprueba en Preferencias > General que el modelado es **Z arriba**, que es el
   valor por defecto. Si cambias algo, reinicia.
3. Ve a **Utilidades > Scripts y complementos**.
4. Pulsa **+ > Script o complemento desde dispositivo** y elige
   `~/Downloads/danny-robot/cad/fusion/DannyMasterSkeleton`.
5. Selecciónalo y pulsa **Ejecutar**.
6. Comprueba el resultado en la terminal:

```bash
cd ~/Downloads/danny-robot && .venv/bin/python simulation/scripts/export_cad_parameters.py --check
```

Los menús pueden tener otro nombre en español: la referencia oficial está en inglés.

**Validación hecha:** 19 pruebas con un sustituto de la API, que imita el cambio de signo del plano
XZ. Los puntos coinciden con el robot simulado y la altura da 700 mm. Los nombres de la API se
contrastaron con la referencia oficial de Autodesk. **La prueba real es tu ejecución.**

## 12. Riesgos y decisiones abiertas

| Riesgo | Por qué importa | Siguiente paso |
|---|---|---|
| **Calor en rodilla y cadera** | Continuo de 6-6,6 N·m frente a 6 N·m con disipador enorme | Medirlo con el prototipo |
| **Cintura de 3 ejes en un punto** | Tres motores de unos 44 mm no caben en 40 mm | Diseñar la cintura y llevar sus alturas reales al modelo |
| **Cadera y cuello** | Igual: yaw, roll y pitch se cruzan en un punto en el modelo | Diseñar pelvis y actualizar el modelo |
| **Masa de estructura** | 2,8 kg sin diseño, y es el dato más incierto | Medirla en CAD |
| **Tensión** | 6S da menos velocidad; 12S no cabe bien | Medir el RS02 a 24 V |
| **Isaac Sim con la 3080 Ti** | Mínimo publicado: RTX 4080 de 16 GB. Isaac Lab 3.0 con Newton sin renderizar debería funcionar, sin comprobar | Probarlo antes de la fase 4, o usar la nube |
| **Cargas sin medir** | No hay marcha dinámica ni levantarse del suelo | Repetir con la fase 4 |
| **Impresión** | Sin cámara calefactada, y Z débil | Probetas reales |
| **Datos sin confirmar** | Holgura y par de retroceso de RobStride; especificaciones de la impresora | Hoja de datos o medición |

**Decisiones que te corresponden:**

1. Aprobar el prototipo de una articulación, unos 270 €.
2. Presupuesto objetivo: A, B o C. Recomiendo B.
3. Aceptar que cintura, cadera y cuello crezcan algo.
4. Licencia de Fusion y de este repositorio.

## Fuentes

Datos con enlaces en el [anexo de componentes](phase-5.1-component-research.md).
