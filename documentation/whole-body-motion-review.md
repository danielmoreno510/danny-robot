# Revisión del cuerpo completo para movimiento natural

**Estado: APROBADA por el usuario.** Los cambios 1 y 2 de la sección 7 están hechos (E22 en el
handbook); los 3 y 4 siguen pendientes. Las secciones 3 a 6 describen el modelo **anterior**
a esos cambios, de 30 GDL. La fase 3.5 sigue abierta y la fase 4 no ha empezado.

Resultado de E22: el balanceo de ±20° a 1,5 Hz pasa de 1,95 a 1,09 N·m, y el brazo baja de ζ
1,6 a 0,63, aunque con un amortiguamiento que sigue siendo una estimación. **Corrección de la
auditoría:** ese ζ es solo el pasivo. Con el `kv` del actuador es 6,2, y el brazo no oscila solo.

## 1. Objetivo y regla de esta revisión

Danny debe moverse de forma fluida, coordinada y físicamente posible: brazos que
acompañan a la marcha, peso que se transfiere con suavidad, y cabeza y torso estables.
La física, la estabilidad y lo que los motores pueden hacer van siempre antes que el
aspecto: el objetivo no es una animación convincente, sino un controlador que pueda
funcionar en el robot real.

Las cifras de esta revisión son de tres clases:

- **Medido:** sale del simulador, con `danny_humanoid.xml`.
- **Calculado:** sale de una fórmula con datos medidos.
- **Estimado:** usa un valor externo o un escalado. Es una orden de magnitud.

## 2. Resumen de las seis respuestas

| Pregunta | Respuesta | Qué hacer |
|---|---|---|
| 1. ¿Brazos de 7 GDL? | **Sí, son adecuados** | Mantenerlos |
| 2. ¿Cintura y torso? | **Casi: falta el roll** | Añadir roll a la cintura, si lo apruebas |
| 3. ¿Piernas? | **6 GDL bastan, el pie es la duda** | Probar el pie antes de cambiarlo |
| 4. ¿Actuadores? | **Es el mayor límite del modelo actual** | Modelo por articulación |
| 5. ¿Cambios antes de la fase 4? | Uno obligatorio, y tres más | Sección 7 |
| 6. ¿Cómo entrenar? | Seguimiento de referencia y estilo adversarial | Sección 8 |

## 3. Brazos de 7 GDL

Son adecuados. Nada los obliga a estar fijos: `held_target` acepta cualquier trayectoria
en cada paso de control, y el modelo tiene las 30 articulaciones con motor.

Balanceo medido de pie, ±20° de hombro y en contrafase:

| Frecuencia | Par máximo | Velocidad máxima | Error | Giro de la pelvis |
|---|---|---|---|---|
| 0,5 Hz | 0,68 N·m | 1,10 rad/s | 0,2° | ±0,04° |
| 1,0 Hz | 1,22 N·m | 2,21 rad/s | 0,4° | ±0,11° |
| 1,5 Hz | 1,95 N·m | 3,34 rad/s | 0,7° | ±0,24° |
| 2,0 Hz | 2,88 N·m | 4,50 rad/s | 1,1° | ±0,43° |

El catálogo del XM540 da 7,3 N·m y 5,55 rad/s. El ritmo natural estimado para Danny es de
unos 1,6 ciclos de brazo por segundo (sección 6), así que hay margen, pero en velocidad es
justo: amplitudes mayores o ritmos más altos lo agotarían. La marcha E13 cumple los
criterios con este balanceo a 1 Hz, aunque esa marcha es lentísima y el balanceo no está
sincronizado con los pasos.

**Reserva:** esas cifras usan el modelo actual de motor, que es optimista (sección 6).

## 4. Cintura y torso

Yaw (±45°) y pitch (-15° a 45°) sobran para lo que hacen en la marcha. La pelvis y el
tórax giran en oposición al andar, y eso reduce el momento angular del tronco
([estudio sobre la coordinación tórax-pelvis](https://research.vu.nl/en/publications/axial-pelvis-range-of-motion-affects-thorax-pelvis-timing-during-/)).
El yaw de la cintura lo permite.

**Falta el roll.** Al andar, la pelvis se inclina de lado mientras el tórax se mantiene
casi vertical. Sin roll en la cintura, inclinar la pelvis inclina también el torso, y solo
el cuello podría compensarlo. Tampoco se podrían reproducir bien las referencias humanas,
ni agacharse de lado al alcanzar algo.

La propuesta aprobada descartó el roll porque «la cadera ya inclina de lado» y por ahorrar
un motor. Con el objetivo de movimiento natural, ese argumento ya no basta: la cadera
inclina todo el cuerpo a la vez, no el torso respecto a la pelvis.

**Coste estimado:** un RobStride 00 son 0,31 kg, y el margen que queda en el torso es de
0,30 kg. El total subiría a 11,01 kg, justo en el límite del objetivo. Eleva el modelo a 31
GDL, y mecánicamente es más complejo. El Unitree G1 ofrece de 1 a 3 ejes de cintura.

## 5. Piernas

Seis GDL por pierna es la configuración habitual, y los rangos cubren una marcha humana.
Los puntos débiles son otros:

- **Pie rígido y plano.** Sin dedos no hay apoyo talón-punta completo, y el despegue es
  más corto. No propongo aún un dedo activo: pesa y complica. Primero hay que medir si un
  pie con talón y punta redondeados, o un dedo pasivo, mejora la marcha aprendida. Esta
  duda no la resuelve ninguna búsqueda: las fuentes consultadas no la cubren.
- **Masa distal alta.** Tibia y pie suman el 49,2 % de la masa de cada pierna (medido),
  porque los motores de tobillo están en la tibia. En una persona es bastante menos
  (valor de memoria, sin fuente: en torno al 35-40 %). Cuesta más par y energía adelantar la pierna. Llevar
  esos motores hacia la rodilla con bielas ayudaría, y se decide en el diseño mecánico.
- **Piernas muy dobladas.** La marcha clásica usa 56° de rodilla. Es propio de ese
  controlador, no del modelo: una política aprendida puede estirar más la pierna.

## 6. Actuadores: el mayor límite actual

**Todas las articulaciones comparten los mismos parámetros**, desde la cadera hasta la
muñeca. Eso sirvió para validar la física, pero no sirve para aprender movimiento natural:
una política aprovecharía capacidades que el robot real no tiene.

| Parámetro en el modelo | Problema |
|---|---|
| ±20 N·m en todas | La muñeca real da 1,9 N·m: el simulador le permite 10 veces más |
| `kp` de 100 N·m/rad en todas | Brazo a 9,5 Hz y muñeca a 15,8 Hz (calculado): muy rígido |
| Curva de par-velocidad solo en empujones | La marcha y el balanceo no tienen límite de velocidad |
| `armature` de 0,01 kg·m² en todas | La inercia medida de la muñeca, 0,0101, es casi toda ficticia |
| Amortiguamiento de 0,5 N·m·s/rad en todas | El brazo no puede oscilar libremente |

**Sobre el último punto.** Con los datos medidos, el brazo como péndulo tiene una
frecuencia propia de 0,89 Hz y un amortiguamiento ζ de 1,6 (calculado): está
sobreamortiguado. Con 0,05 N·m·s/rad, ζ sería 0,16. La literatura consultada describe el
balanceo humano como pasivo en gran parte, estabilizado por los músculos; no he podido
atribuir ese dato a un artículo concreto, así que hay que confirmarlo. Con este modelo,
Danny tendría que generarlo todo con los motores.

**Motores de dos mundos.** Las piernas usan motores cuasi-directos de 10:1, y los hombros
servos de 152:1. Es de esperar que esos servos cuesten de mover desde fuera y que se
parezcan poco a un péndulo. Es una inferencia por la reducción: ninguna fuente lo confirma,
y hace falta medirlo en banco.

**Lo que sí alcanza, según el escalado.** Con semejanza dinámica, un robot de 0,36 m de
pierna frente a 0,90 m de una persona tiene tiempos ×0,63, velocidades angulares ×1,58 y
pares por kg ×0,4. Es estimado, y la literatura avisa de que ninguna normalización de
momentos es perfecta ([comparación de cinco métodos, en carrera](https://cris.maastrichtuniversity.nl/en/publications/a-comparison-of-five-methods-to-normalize-joint-moments-during-ru/)).

| Articulación | Humano en marcha | Danny, escalado | Catálogo: pico / nominal |
|---|---|---|---|
| Cadera | [1,35 N·m/kg](https://clinicalgaitanalysis.com/faq/moment.html) | 5,9 N·m | 17 / 6 N·m |
| Rodilla | [0,85 N·m/kg](https://clinicalgaitanalysis.com/faq/moment.html) | 3,7 N·m | 17 / 6 N·m |
| Tobillo | unos 1,5 N·m/kg, valor de memoria sin fuente | 6,6 N·m | 14 / 5 N·m |

Los picos caben, y el tobillo supera su par nominal solo en el despegue. Falta comprobar
el calor y el par medio. La velocidad tampoco parece un problema: el RobStride 00 da
[315 rpm en vacío y 260 con carga](https://rcdrone.top/products/robstride-00-qdd-14n-m-integrated-actuator-module-high-performance-robotics-joint-motor)
según una tienda, es decir, 33 y 27 rad/s.

**La distancia que queda es grande.** E13 anda a 0,009 m/s, con número de Froude 0,00002.
La marcha humana cómoda corresponde a unos 0,8 m/s en Danny (Froude 0,19). Es unas 90 veces
más rápido, y ningún resultado actual dice nada de ese régimen.

## 7. Cambios propuestos antes de la fase 4

| # | Cambio | Prioridad | Por qué |
|---|---|---|---|
| 1 | **Actuador por articulación**: par, velocidad, `kp`, `kv`, amortiguamiento y `armature` de cada motor en `robot_spec.py`, con curva de par-velocidad siempre activa | **Obligatorio** | Sin esto la política aprende sobre un robot imposible |
| 2 | **Roll de cintura**: pasar a 31 GDL | Recomendado | Coste bajo ahora, reentrenamiento después |
| 3 | **Prueba del pie**: comparar pie plano y pie redondeado en simulación | Recomendado, en la fase 4 | Decidir con datos, sin cambiar la mecánica aún |
| 4 | **Puntos de seguimiento**: sitios en pelvis, torso, cabeza, manos y pies, y una IMU en la cabeza | Pequeño | Los necesita el seguimiento de movimiento |

Sin cambiar: 7 GDL por brazo, 6 por pierna, cuello de 2 ejes y cabeza simple, que es solo
una esfera de 0,31 kg, sin cara ni actuadores.

Además, el cambio 1 obliga a reajustar el control clásico. Podrían bajar los límites de
empujón o romperse la marcha. Si pasa, se documenta y no se fuerza: la marcha clásica ya
no es el objetivo, solo una comprobación de que el modelo es sano.

**No propongo** ninguna animación elaborada a mano. Bastan el balanceo sinusoidal ya medido
y una prueba que lo repita.

## 8. Entrenamiento recomendado

Una recompensa solo por avanzar **no** produce marcha natural: hay que pedirla
explícitamente. Propongo dos técnicas, en este orden:

1. **Seguimiento de una referencia.** Primero, que una política siga un clip de marcha ya
   adaptado a Danny. Es la prueba de viabilidad: si no puede seguirlo, el problema está en
   el modelo o en los motores, y se ve antes de invertir más. Es el enfoque de
   [BeyondMimic](https://arxiv.org/pdf/2508.08241v3) y de
   [ASAP](https://arxiv.org/pdf/2502.01143), este último con resultados en un Unitree G1
   real. Solo he leído resúmenes de esos artículos, no los textos completos.
2. **Estilo adversarial con órdenes de velocidad.** Después, la política principal: un
   discriminador premia que el movimiento se parezca a los datos, sin copiar un clip, y la
   recompensa de tarea pide velocidad y giro. Permite combinar comportamientos y
   transiciones. Es la línea de las Adversarial Motion Priors (AMP), usada en el robot
   [Cosmo](https://arxiv.org/html/2509.05581v1) y, en una variante, en
   [HumanMimic](https://arxiv.org/html/2309.14225v4).

**Recompensa, en orden:** no caer y cumplir la orden; estilo o seguimiento; suavidad
(acción, aceleración y sacudida); energía. Los pesos se ajustan midiendo las métricas de la
sección 9, no mirando el vídeo.

**Datos.** AMASS y LAFAN1 son candidatos. **La licencia hay que comprobarla antes de usarlos**,
sobre todo porque el repositorio es público y aún no tiene licencia. No lo he verificado.

**Adaptar el movimiento a Danny.** Los datos humanos no se aplican directamente:

1. **Geometría:** escalar por longitud de pierna y corregir pies y manos con una
   optimización que respete los rangos articulares.
2. **Tiempo:** comprimir los clips ×0,63, para conservar la semejanza dinámica.
3. **Física:** comprobar que par y velocidad caben en los motores, y descartar o modificar
   lo que no.
4. **Comprobar en simulación:** la referencia válida es la que el robot físico puede seguir.

**Frecuencia de control.** Si los tiempos son ×0,63, los 50 Hz habituales en un robot grande
equivalen a unos 80 Hz en Danny. Hay que medirlo.

**Riesgo principal.** Estos métodos se han demostrado en robots grandes. No he encontrado una
demostración clara en un robot de 70 cm con servos de brazo de alta reducción. El
sim-to-real de la fase 8 podría ser más difícil de lo que sugieren esos artículos.

**Pendiente de tu decisión:** el simulador de entrenamiento depende del hardware de
entrenamiento, GPU local o en la nube.

## 9. Métricas de naturalidad

Se evalúan con magnitudes sin dimensiones, para poder comparar con personas. `L` es la
longitud de pierna (0,36 m) y `g` la gravedad.

| Métrica | ¿Sirve a 70 cm? | Cómo normalizar |
|---|---|---|
| Velocidad | Sí | Froude: `v² / (g·L)` |
| Longitud de paso | Sí | `/ L` |
| Cadencia | Sí | `× sqrt(L / g)` |
| Ángulos, inclinación de torso y cabeza | Sí, directa | Ninguna: ya son grados |
| Velocidad articular | Sí | `× sqrt(L / g)` |
| Aceleración articular | Sí | `× L / g` |
| Sacudida articular | Sí, entre versiones de Danny | `× (L / g)^1,5`; mejor una medida sin dimensiones (SPARC) |
| Variabilidad del paso | Sí, directa | Coeficiente de variación |
| Altura del pie libre | Con cuidado | `/ L`, y mínimo absoluto, porque el suelo no escala |
| Deslizamiento del pie | Con cuidado | `/ L` por paso |
| Coordinación brazo-pierna | Sí, directa | Fase relativa en grados |
| Error de seguimiento | Sí | Ángulos directos; posiciones `/ L` |
| Energía | Sí, pero no comparable con personas | Coste de transporte: `E / (m·g·d)` |
| Recuperación | Sí | Impulso: `J / (m·sqrt(g·L))` |

El par no se compara con personas sin normalizar, y el coste de transporte depende de las
pérdidas del motor, muy distintas de las de un músculo: sirve para comparar políticas de
Danny entre sí.

Los límites concretos están en el [roadmap](../ROADMAP.md#fase-4--aprendizaje-por-refuerzo-y-locomoción).
Son provisionales: se fijan definitivamente antes de cada hito, tras comprobar que son
físicamente posibles.

## 10. Decisiones

El usuario aprobó el actuador por articulación y el roll de cintura, antes de la etapa
3.5.7.
