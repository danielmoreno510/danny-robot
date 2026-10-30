# Auditoría de preparación para la fase 4

**Veredicto: todavía NO estamos preparados para entrenar.** El cuerpo físico está bien
construido y validado, pero faltan las etapas 3.5.7 y 3.5.8, y aún no existe nada de lo que
hace falta para entrenar: entorno, sensores de IMU, datos de movimiento ni métricas
programadas. La lista completa está en la sección 8.

Las cifras salen de `results/` y de `tests/test_readiness.py`. **Nota sobre los números:** el
encargo hablaba de 30 GDL y 168 pruebas. Tras aprobar el roll de cintura son **31 GDL**, y
el recuento vigente de pruebas está en [PROJECT_HANDOFF.md](../PROJECT_HANDOFF.md).

## 1. Los 31 GDL

**Bien definidos.** Se comprobó con el modelo compilado: las 31 articulaciones tienen eje,
tope, motor, sensor de posición y sensor de par, y el lado derecho refleja al izquierdo.

**Límites articulares: la mayoría son plausibles, dos no.** Comparación con rangos humanos
típicos, que varían según la fuente
([tabla 1](https://wikem.org/wiki/Joint_ROM_(Table)),
[tabla 2](https://pmc.ncbi.nlm.nih.gov/articles/PMC9218764/table/T1)). No todos los valores
humanos están confirmados: los marcados con «?» son de memoria.

| Articulación | Danny | Humano típico | Valoración |
|---|---|---|---|
| Cadera, flexión | 90° | 120-130° | Corto: limita sentarse y agacharse |
| **Cadera, extensión** | **90°** | **unos 15°** | **Excesivo: pose imposible para una persona** |
| Cadera, separación (roll) | ±30° | ? unos 45° y 30° | Aceptable |
| Cadera, giro (yaw) | ±40° | 35-45° | Bien |
| Rodilla | 0 a 130° | 135-140° | Bien |
| Tobillo, flexión plantar | 45° | 45-50° | Bien |
| **Tobillo, flexión dorsal** | **65°** | **unos 20°** | **Excesivo** |
| Tobillo, roll | ±30° | ? unos 35° y 15° | Aceptable |
| Cintura yaw / roll / pitch | ±45° / ±20° / -15 a 45° | ? unos 40° / 30° / 25 a 80° | Aceptable para un robot |
| Hombro, flexión | 150° | unos 180° | Corto, suficiente |
| Hombro, extensión | 60° | ? unos 60° | Bien |
| Hombro, separación | 120° | unos 180° | Corto, suficiente |
| Hombro, giro | ±90° | 0-90° | Bien |
| Codo | 0 a 140° | 140-145° | Bien |
| Muñeca: yaw, pitch y roll | ±90°, ±45°, ±45° | 80°, 60°, 20-50° | Aceptable |
| Cuello yaw / pitch | ±80° / -30 a 45° | ? unos 80° / 55-70° y 70-90° | Corto en pitch, suficiente |

**Importante:** los topes son mecánicos y no tienen por qué coincidir con los humanos. Pero
con la extensión de cadera y la flexión dorsal actuales, una política puede aprender posturas
que no parecen humanas. Hay dos caminos: estrechar esos rangos en el modelo o penalizarlos al
entrenar. Lo propongo en la sección 8.

## 2. Física del modelo

| Comprobación | Resultado |
|---|---|
| Gravedad | -9,81 m/s² en z; la caída libre coincide con el integrador exacto |
| Masas | 11,004 kg; ningún cuerpo sin masa |
| Inercias | Válidas en los 21 cuerpos (desigualdad triangular) |
| Colisiones | Las 21 geometrías colisionan; 5 exclusiones, todas por solape intencionado |
| Contacto | De pie, los sensores suman 107,95 N, igual que el peso |
| Topes articulares | Con 2 veces el par del motor contra el tope, se pasa 2,7° como máximo: son blandos |
| Límite de par | Pidiendo el máximo, ningún motor supera su límite |
| Paso e integrador | 2 ms, `implicitfast` |

**Limitaciones de esta física:**

- Todas las masas salen de un presupuesto, no de una báscula. Las geometrías de colisión
  son primitivas simples, sin diseño mecánico detrás.
- `armature` y amortiguamiento son estimaciones, y no hay fricción seca (`frictionloss` es 0).
- El contacto pie-suelo usa parámetros por defecto.
- Los sensores y motores son ideales: sin ruido, retardo ni holgura.

## 3. Pruebas y comprobaciones

Ejecutadas de verdad el 30 de octubre de 2026: **189 pruebas y 21 comprobaciones pasan**, y las
277 métricas del bípedo no cambian. Dos matices:

- **Una prueba que pasa no siempre exige algo.** Las 37 de realismo y varias del humanoide
  solo miden.
- **Solo validan el control clásico lento.** Ninguna dice nada de marcha natural.

## 4. El roadmap de la fase 4

Cubre tus nueve hitos con criterios medibles. La revisión encontró **un fallo que ya está
corregido**: el orden de la tabla ponía la locomoción antes de las referencias, y la estrategia
hace lo contrario. También faltaba exigir plausibilidad biomecánica. Ahora el roadmap fija el
orden de ejecución y pide posturas dentro de rangos humanos.

## 5. Estrategia y herramientas

Se combinan aprendizaje por refuerzo e imitación en una sola recompensa: tarea (no caer,
cumplir la orden), imitación (seguimiento o estilo adversarial) y regularización (suavidad y
energía). El detalle está en la sección 8 de la
[revisión](whole-body-motion-review.md). Orden: seguir un clip para demostrar viabilidad, y
después estilo adversarial con órdenes de velocidad.

| Necesidad | Elección provisional | Por qué | Alternativa |
|---|---|---|---|
| Simulador de entrenamiento | [MuJoCo Playground](https://arxiv.org/html/2502.08844v1) sobre MJX | Usa el mismo modelo validado; abierto; sus autores informan de paso directo a robots reales, incluidos humanoides | Isaac Lab: más integración con PyTorch, instalación más pesada y GPU NVIDIA obligatoria |
| Algoritmo base | PPO | Es el estándar en estos trabajos | — |
| Imitación | Seguimiento de referencia, y después estilo adversarial | Ver sección 8 de la revisión | Solo una de las dos |
| Adaptación de movimiento | [GMR](https://arxiv.org/abs/2510.02252v1) | Hecho para humanoides; [el código es MIT](https://github.com/YanjieZe/GMR/wiki) según la búsqueda, a confirmar | Optimización propia |
| Datos | AMASS o LAFAN1 | Usados por GMR | **Licencias sin verificar** |

**Sin verificar, y con riesgo:** que `danny_humanoid.xml` funcione en MJX, porque MJX no
admite todo lo que MuJoCo. Hay que probarlo antes de decidir. Tampoco he comprobado el estado de
mjlab, otra opción posible. El simulador depende además de tu decisión sobre la GPU: este Mac no
vale para entrenar a escala.

## 6. Métricas objetivas

Definidas aquí; **aún no están programadas**. `L` = 0,36 m. Los límites son provisionales: se
fijan antes de cada hito, tras comprobar que son físicamente posibles.

| Grupo | Métrica | Fórmula | Límite provisional |
|---|---|---|---|
| Naturalidad | Error respecto a la referencia | Media de `|q - q_ref|` | < 0,2 rad |
| | Posturas dentro de rango humano | % de tiempo | > 99 % |
| | Longitud de paso y cadencia normalizadas | `paso / L`, `cadencia × sqrt(L/g)` | Dentro del rango humano escalado |
| Estabilidad | Inclinación de torso | RMS del ángulo con la vertical | < 5° |
| | Estabilidad de cabeza | RMS del pitch y roll de la cabeza | < 5° |
| | Tiempo sin caer | s bajo ruido y retardo | ≥ 60 s |
| | Recuperación | Impulso máximo `J / (m·sqrt(g·L))` | Se fija en la etapa 26 |
| Coordinación | Brazo y pierna opuestos | Fase relativa | 0° ± 30° |
| | Pelvis y tórax | Fase relativa en yaw | En oposición, según velocidad |
| | Simetría | Diferencia izquierda-derecha en tiempo de paso | < 5 % |
| Suavidad | Sacudida articular | SPARC o sacudida adimensional | Mejor que sin imitación |
| | Variabilidad del paso | Coeficiente de variación del tiempo de paso | < 5 % |
| | Transiciones | Pico de sacudida en transición / en marcha estable | < 1,5 |
| | Deslizamiento del pie | Por paso, `/ L` | < 2 % |
| | Altura del pie libre | `/ L`, y mínimo absoluto | > 5 % |
| Energía | Coste de transporte | `∫ Σ |τ·ω| dt / (m·g·d)`, con pérdidas del motor si se conocen | Mejor que sin imitación |
| | Par eficaz | RMS por articulación / par nominal | ≤ 1 |
| | Saturación | % de tiempo al límite de par | < 1 % |

La energía sirve para comparar políticas de Danny, no con personas.

## 7. Qué es cinemático y qué está listo para control dinámico

| Componente | Clase | Comentario |
|---|---|---|
| `Pose`, `joint_targets`, `leg_ik`, `leg_reach` | **Solo cinemático** | Geometría, sin fuerzas |
| `check_kinematics.py`, `check_joint_limits` | **Solo cinemático** | Validación geométrica |
| `center_com_over`, reparto de peso | **Estático** | Centro de masas sin aceleraciones |
| Plan de `walk_biped.py` | **Estático** | Marcha con el CdM siempre sobre el apoyo; Froude 0,00002 |
| `StanceCompensator`, `GravityCompensator` | **Cuasi-estático** | Gravedad y apoyo, sin inercias |
| Feedforward de velocidad en `step_targets` | **Dinámico parcial** | Compensa el retraso del motor, sin dinámica del cuerpo |
| `Biped.step` y `limit_torque_speed` | **Dinámico, para el control clásico** | La curva de par-velocidad está aquí; el entorno de entrenamiento debe reutilizarla |
| `danny_humanoid.xml` y `robot_spec.py` | **Listo para control dinámico** | Masas, inercias, contacto y motores por articulación |
| Motores de posición de MuJoCo | **Listo** | Una política puede mandar objetivos a las 31 articulaciones |
| Empujones (`push_biped.py`) | **Dinámico, pasivo** | Mide dinámica, sin corregir nada |
| Retardo, ruido y fricción de `test_realism.py` | **Dinámico, solo en pruebas** | Hay que llevarlos al entorno |
| Realimentación con IMU | **No existe** | Sin giróscopo ni acelerómetro |
| Control dinámico de cuerpo completo o política | **No existe** | Es la fase 4 |

**Conclusión:** el modelo está preparado para control dinámico; el controlador actual no. El
control clásico es cinemático y cuasi-estático, y solo sirve como comprobación. No se reutiliza
para entrenar, salvo la curva de par-velocidad y el código de sensores.

## 8. Requisitos antes de entrenar

**Bloqueantes:**

| # | Requisito | Cómo se cierra |
|---|---|---|
| 1 | Etapa 3.5.7: equilibrio, marcha y sensibilidad en el humanoide | Repetir las medidas de realismo |
| 2 | Etapa 3.5.8: informe de la fase y auditoría independiente | Informe, y tu revisión |
| 3 | IMU: el modelo solo tiene orientación y posición ideales | Añadir giróscopo y acelerómetro |
| 4 | Velocidades articulares como sensor | Añadirlas |
| 5 | Postura inicial guardada (hoy no hay ninguna) | Añadir keyframe de pie |
| 6 | Rangos de cadera y tobillo | Decidir: estrechar o penalizar |
| 7 | Decidir GPU local o nube | **Decisión tuya** |
| 8 | Comprobar el modelo en MJX | Prueba de compilación y de velocidad |
| 9 | Entorno: observaciones, acciones, recompensas y terminaciones | Etapas 21 y 22 |
| 10 | Motor en el entorno: curva, retardo, ruido, fricción y variaciones | Etapa 21 |
| 11 | Datos con licencia válida y adaptados a Danny | Hitos C y D |
| 12 | Métricas de la sección 6 programadas | Módulo de evaluación |

Los números 9 a 12 son el arranque de la propia fase 4. **Los números 1 a 8 son los que deben
estar antes de empezar la fase.**

**Importantes, sin bloquear:**

- Licencia del repositorio: **decisión tuya**, necesaria antes de publicar datos o resultados.
- Puntos de seguimiento en pelvis, torso, cabeza, manos y pies, e IMU en la cabeza.
- Comparar el pie plano con uno redondeado.
- Medir un motor real: es la mayor fuente de error del modelo, aunque no impide empezar.
