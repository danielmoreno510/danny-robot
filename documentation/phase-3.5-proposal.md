# Propuesta técnica: fase 3.5, humanoide completo

**Estado: APROBADA por el usuario.** Todavía no se ha implementado nada de este
documento. Todas las cifras son preliminares: son estimaciones de diseño, no
resultados de simulación.

**Actualización:** el [presupuesto con motores de catálogo](phase-3.5-mass-budget.md) dio
10,7 kg y otro reparto de masa, y el objetivo pasó a **11 kg**. Ese documento sustituye a
la sección 4 de esta propuesta.

## 1. Objetivo y por qué

Construir el Danny completo en simulación antes de entrenar la locomoción principal.

Hoy el modelo tiene 12 GDL y el torso es una masa. Una política entrenada así aprendería
a andar con otro robot: los brazos, la cabeza y la cintura mueven el centro de masas, y
sus inercias cambian cómo responde el cuerpo.

**Un matiz importante:** la fase 3.5 reduce mucho el riesgo de reentrenar, pero no lo
elimina. Las masas de esta propuesta cambiarán al elegir motores (etapa 31) y al pesar
el robot real (etapa 47). Por eso propongo dos medidas:

- Hacer un **presupuesto de masa** con motores de catálogo en la etapa 3.5.1, aunque la
  elección no sea definitiva.
- Entrenar con variaciones de masa (etapa 27), para que la política tolere esos cambios.

## 2. Configuración propuesta: 30 GDL

| Grupo | Articulaciones | GDL |
|---|---|---|
| Piernas | cadera yaw, roll, pitch; rodilla; tobillo pitch, roll | 6 × 2 = 12 |
| Cintura | yaw, pitch | 2 |
| Brazos | hombro pitch, roll, yaw; codo; muñeca yaw, pitch, roll | 7 × 2 = 14 |
| Cuello | yaw, pitch | 2 |
| Manos | rígidas, sin articular | 0 |
| **Total** | | **30** |

### Referencias reales

| Robot | Altura | Masa | GDL | Reparto |
|---|---|---|---|---|
| [ToddlerBot](https://arxiv.org/html/2502.00893v4) | sin confirmar | sin confirmar | 30 | 7 brazo, 6 pierna, 2 cuello, 2 cintura |
| [ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/introduction/) | 51 cm | 3,5 kg | 20 | — |
| [Berkeley Humanoid Lite](https://arxiv.org/pdf/2504.17249) | 0,8-1 m según la fuente | 16 kg | 22 | 6 pierna, 5 brazo |
| [Unitree G1](https://www.unitree.com/en/mobile/g1) | 1,27-1,32 m según la página | 35 kg | 23 a 43 | 6 pierna, 5 a 7 brazo, 1 a 3 cintura |

Datos de búsqueda web, sin comprobar en cada fuente primaria.

### Decisiones y alternativas

**¿7 GDL por brazo? Sí, en el modelo.** Con 7 ejes, la mano puede mantener posición y
orientación mientras el codo se mueve. Eso ayuda a abrir cajones y a agarrar en sitios
estrechos, que son objetivos de la fase 11. El coste real es añadir un motor al final
del brazo, donde más pesa.

Recomiendo 7 porque quitar un eje de la simulación es trivial (se bloquea la
articulación), y añadirlo después obliga a rehacer el brazo. Si la etapa 31 muestra que
el tercer eje de muñeca no cabe o pesa demasiado, se bloquea y quedan 28 GDL. Son 5 en
Berkeley Humanoid Lite y en el G1 básico, así que un brazo de 5 también es viable si
priorizas simplicidad y coste.

**Cintura de 2 ejes.** Yaw para girar el torso y equilibrar el giro de los pasos, y pitch
para inclinarse a recoger objetos. No incluyo roll: la cadera ya inclina de lado, y cada
motor de cintura carga con todo el tronco.

**Cuello de 2 ejes.** Basta para orientar las cámaras.

**Manos sin articular.** Un bloque con masa e inercia realistas y geometría de colisión.
La pinza llegará con la manipulación.

## 3. Dimensiones

Altura total de 70 cm. El centro de masas propuesto queda al 56 % de la altura.

| Punto | Altura de pie, pierna recta |
|---|---|
| Suelo a tobillo | 4 cm |
| Rodilla | 20 cm |
| Cadera | 36 cm |
| Pelvis | 41 cm |
| Cintura | 45 cm |
| Hombros | 59 cm |
| Base del cuello | 60,5 cm |
| Centro de la cabeza | 65,5 cm |
| Parte superior | 70 cm |

| Segmento | Medida |
|---|---|
| Muslo y tibia | 16 cm cada uno |
| Pie | 14 × 8 cm |
| Caderas | ±7 cm |
| Hombros | ±11 cm |
| Brazo y antebrazo | 13 y 12 cm |
| Mano | 5 cm |
| Cabeza | unos 9 cm |

Mantengo las piernas actuales para poder comparar con el bípedo. El brazo llega a media
altura del muslo, como en una persona.

**A revisar:** el pie de 14 × 8 cm se diseñó para 8 kg. Con más masa y el CdM cerca de 39
cm, habrá que medir si conviene ampliarlo.

## 4. Reparto de masa: 10 kg

El modelo actual tiene 8 kg, con 12 motores. Con 30 motores, batería y ordenador,
propongo **10 kg** como objetivo provisional. Extrapolando por escala cúbica, OP3 daría
9,1 kg a 70 cm, Berkeley Humanoid Lite 5,5-10,7 kg según la altura, y el G1 5,5 kg. Es un
valor razonable, no un dato de diseño.

| Parte | Masa [kg] | Qué incluye, en principio |
|---|---|---|
| Cabeza | 0,55 | cámaras, sensores, motor de cuello |
| Cuello | 0,10 | eslabón y motor |
| Torso superior | 3,00 | batería, ordenador, electrónica, motores de hombro |
| Cintura | 0,35 | eslabón y motor |
| Pelvis | 1,30 | motores de cintura y de cadera |
| Hombro × 2 | 0,12 × 2 | |
| Brazo × 2 | 0,25 × 2 | |
| Antebrazo × 2 | 0,18 × 2 | |
| Mano × 2 | 0,10 × 2 | |
| Muslo × 2 | 0,85 × 2 | |
| Tibia × 2 | 0,60 × 2 | |
| Pie × 2 | 0,25 × 2 | |
| **Total** | **10,00** | piernas 34 %, brazos 13 %, resto 53 % |

CdM estimado de pie: 39 cm, el 56 % de la altura, frente a 41,6 cm del modelo actual.
Es un cálculo a mano con los segmentos rectos, sin validar en simulación.

## 5. Articulaciones y actuadores

Rangos provisionales. Los de las piernas no cambian.

| Articulación | Rango |
|---|---|
| Cintura yaw | ±45° |
| Cintura pitch | −15° a 45° |
| Hombro pitch | −150° a 60° |
| Hombro roll | −10° a 120° (hacia fuera) |
| Hombro yaw | ±90° |
| Codo | 0° a 140° |
| Muñeca yaw | ±90° |
| Muñeca pitch y roll | ±45° |
| Cuello yaw | ±80° |
| Cuello pitch | −30° a 45° |

**Pares estimados.** Escalando lo medido en la marcha de 8 kg a 10 kg: cadera roll 7,4
N·m y rodilla 6,1 N·m. Con margen para aceleraciones y empujones, una primera idea de
clases de motor:

| Clase | Par de pico | Articulaciones |
|---|---|---|
| Grande | 20 N·m | cadera, rodilla |
| Media | 10 N·m | tobillo, cintura |
| Pequeña | 3 N·m | hombro, codo |
| Mínima | 1 N·m | muñeca, cuello |

Son estimaciones sin validar. Los pares de brazos se fijarán midiendo en simulación.

## 6. Arquitectura

**Generar el modelo desde una sola especificación.** Con 30 articulaciones y dos lados,
copiar XML a mano duplica definiciones. Propongo:

- `simulation/scripts/robot_spec.py`: dimensiones, masas, rangos y actuadores, en un
  solo sitio.
- `simulation/scripts/build_model.py`: construye el modelo con `MjSpec`, la API de
  MuJoCo, y refleja izquierda y derecha desde una única descripción.
- `simulation/models/danny_humanoid.xml`: el resultado, versionado y revisable.
- Una prueba comprueba que el XML versionado coincide con lo que genera la
  especificación.

**El modelo de 12 GDL se conserva** para comparar: pasa a llamarse
`danny_biped.xml`, y todos sus experimentos y pruebas siguen funcionando.

**Controladores.** `Biped` tiene hoy las 12 articulaciones escritas a fuego. Propongo
separar la lista de articulaciones del robot, y añadir una postura fija para el tren
superior. Así, en la etapa 3.5.7, el mismo controlador sirve para ambos modelos.

**Autocolisión.** Cápsulas y cajas simples para todo el cuerpo, con exclusiones
declaradas solo donde dos piezas se solapan a propósito.

## 7. Orden de desarrollo

Propongo este orden, con revisión al final de cada etapa. Cada una añade masa e inercia,
y la anterior debe seguir pasando.

| Etapa | Trabajo | Se valida con |
|---|---|---|
| 3.5.1 | Especificación y presupuesto de masa con motores de catálogo | Tu revisión de este documento |
| 3.5.2 | `robot_spec.py` y `build_model.py`, reproduciendo el bípedo actual | XML generado equivalente, con las 124 pruebas pasando |
| 3.5.3 | Torso y cintura, con cintura bloqueada primero | Masa, CdM, inercias; marcha |
| 3.5.4 | Brazos, en postura fija | Alcance, rangos, autocolisión; marcha |
| 3.5.5 | Manos | Masa e inercia; marcha |
| 3.5.6 | Cabeza y cuello | Masa, CdM; espacio para sensores |
| 3.5.7 | Modelo integrado: equilibrio y marcha con todo el cuerpo | Pruebas de la fase 3 sobre el modelo nuevo |
| 3.5.8 | Validación y comparación con el bípedo | Informe de la fase 3.5 |

Es tu lista, con un cambio: la **3.5.2** es nueva. Separa la construcción del modelo de
sus cambios físicos, para que un fallo posterior no se confunda con un error de
refactorización. Las etapas de tu lista pasan a ser 3.5.3 a 3.5.8.

**Por qué en este orden:** el torso primero, porque es lo que más mueve el CdM. Los
brazos se añaden fijos, para separar el efecto de su masa del de su movimiento.

## 8. Riesgos

| Riesgo | Consecuencia | Cómo se detecta |
|---|---|---|
| Más masa alta y CdM sin centrar | La marcha actual falla | Pruebas tras cada etapa |
| Peor sensibilidad | Ya era frágil, y no corrige con la IMU | Las mismas 37 medidas |
| Inercia de brazos | Giros y cambios de peso distintos | Marcha con y sin brazos |
| Empujones | Los límites 24-32 N dejarán de valer | Se vuelven a medir |
| Tiempo | 30 motores multiplican par y pruebas | Revisión por etapas |
| Presupuesto de masa optimista | 10 kg puede quedarse corto | Catálogo en 3.5.1 |

**Si la marcha no se reproduce** con el cuerpo completo, propongo no forzarla con ajustes
y volver a revisar si conviene usar la cintura y los brazos para equilibrar. Eso sería ya
trabajo de la fase 4.

## 9. Preparación de la fase 4

Se mantiene la lista del roadmap: límites realistas de par y velocidad, dinámica y
retardos de actuador, ruido de sensores, fricción articular, autocolisiones, variaciones
de dominio, observaciones de la IMU, y observaciones, acciones, recompensas y condiciones
de terminación bien definidas.

## 10. Decisiones aprobadas

1. **Masa:** 10 kg como objetivo provisional; después 11 kg, tras el presupuesto.
2. **Brazos:** 7 GDL, 30 en total.
3. **Cintura:** 2 ejes, yaw y pitch.
4. **Orden:** se añade la etapa 3.5.2.
5. **Modelo generado:** con `MjSpec`, y el XML generado se versiona.
