# Robot Danny — Roadmap maestro

**Objetivo:** construir Danny, un robot humanoide autónomo de unos 70 cm, capaz de
caminar, mantener el equilibrio, manipular objetos, interpretar instrucciones
con inteligencia artificial y comunicarse a través de una aplicación móvil.

Este documento es el plan del proyecto completo: **12 fases y 76 etapas, más la fase
3.5**, desde
la primera simulación hasta el robot físico validado. El detalle técnico de lo
ya hecho está en el [handbook](docs/HANDBOOK.md).

**Fases, etapas y experimentos:**

- Las **etapas** (1 a 76) son las de este roadmap.
- Los **experimentos** (E1, E2…) son el trabajo concreto del [handbook](docs/HANDBOOK.md).
  Una etapa puede necesitar varios experimentos.

## Cuándo se completa una fase

Cada fase necesita tres condiciones:

1. **Implementación:** todas sus etapas están desarrolladas.
2. **Validación:** hay pruebas reproducibles que demuestran que funcionan.
3. **Documentación:** quedan registrados resultados, limitaciones y decisiones.

Así se evita descubrir errores estructurales cuando ya hay piezas fabricadas.

## Resumen

| Fase | Objetivo | Etapas | Estado |
|---|---|---|---|
| 1 | Preparación del entorno y fundamentos | 1–5 | ✅ Completada |
| 2 | Cinemática y articulaciones | 6–11 | ✅ Completada |
| 3 | Simulación física y control del bípedo | 12–20 | 🟡 CONDITIONAL PASS, ver [validación](documentation/phase-3-validation.md) |
| 3.5 | Humanoide completo en simulación | 3.5.1–3.5.8 | 🔄 Actual |
| 4 | Aprendizaje por refuerzo y locomoción | 21–28 | ⏳ Pendiente |
| 5 | Diseño mecánico del humanoide | 29–35 | ⏳ Pendiente |
| 6 | Fabricación de las piernas | 36–40 | ⏳ Pendiente |
| 7 | Electrónica y control físico | 41–46 | ⏳ Pendiente |
| 8 | Sim-to-Real y locomoción física | 47–52 | ⏳ Pendiente |
| 9 | Torso, brazos y manos | 53–59 | ⏳ Pendiente |
| 10 | Visión, navegación e inteligencia artificial | 60–66 | ⏳ Pendiente |
| 11 | Manipulación autónoma | 67–72 | ⏳ Pendiente |
| 12 | Aplicación móvil e integración final | 73–76 | ⏳ Pendiente |

## Fase 1 — Preparación del entorno y fundamentos

**Entregable:** entorno de simulación funcional.

| Etapa | Actividad | Estado | Experimentos |
|---|---|---|---|
| 1 | Instalar Homebrew y Python 3.12 | ✅ | — |
| 2 | Crear el repositorio y entorno virtual | ✅ | — |
| 3 | Instalar y configurar MuJoCo | ✅ | — |
| 4 | Simular un cuerpo rígido con gravedad | ✅ | E1 |
| 5 | Comprender cuerpos, masas, colisiones y articulaciones | ✅ | E1, E2 |

## Fase 2 — Cinemática y articulaciones

**Entregable:** modelo cinemático capaz de calcular las posiciones articulares
necesarias para situar los pies en coordenadas determinadas.

| Etapa | Actividad | Estado | Experimentos |
|---|---|---|---|
| 6 | Crear una articulación motorizada | ✅ | E2 |
| 7 | Construir una pierna con cadera y rodilla | ✅ | E3, E4 |
| 8 | Implementar control de posición articular | ✅ | E2 a E7 |
| 9 | Implementar cinemática directa | ✅ | La de MuJoCo; `check_kinematics.py` |
| 10 | Implementar cinemática inversa | ✅ | E10, E13: `leg_ik`, `joint_targets`; `check_kinematics.py` |
| 11 | Validar límites articulares, alcance y singularidades | ✅ | `walk_biped.py` comprueba alcance y rangos al planificar |

## Fase 3 — Simulación física y control del bípedo

**Objetivo:** que un bípedo simulado se mantenga de pie, transfiera su peso,
camine y responda a perturbaciones.

**Entregable:** controlador de locomoción clásico, validado con pruebas
reproducibles. Al terminar se hace una auditoría técnica.

| Etapa | Actividad | Estado | Experimentos |
|---|---|---|---|
| 12 | Construir el modelo completo de dos piernas | ✅ | E10, E16 |
| 13 | Configurar masas, inercias, articulaciones y contactos | ✅ | E16: `danny_biped.xml`, provisional hasta la etapa 31 |
| 14 | Mantener el equilibrio sobre ambos pies | ✅ | E10 |
| 15 | Transferir el peso entre las piernas | ✅ | E10, E11 |
| 16 | Mantener el equilibrio sobre un solo pie | ✅ | E10, E11 |
| 17 | Caminar mediante trayectorias cuasiestáticas | ✅ | E11, E12, E13 |
| 18 | Implementar compensación dinámica | 🟡 | E12, E15, E16: probada y retirada; no aporta con el cuerpo de Danny |
| 19 | Recuperar el equilibrio ante empujones | 🟡 | E14, E16: solo pasiva, sin realimentación; 24 N en x, 32 N en y |
| 20 | Ajustar dimensiones físicas y auditar toda la simulación | ✅ | E16 |

**Criterios para completar la fase.** El bípedo debe demostrar que puede:

| Criterio | Estado |
|---|---|
| Mantenerse de pie sobre ambos pies | ✅ |
| Mantenerse sobre cualquiera de los dos pies | ✅ |
| Transferir su peso de forma controlada | ✅ |
| Caminar cuatro pasos consecutivos | ✅ |
| Controlar el contacto de los pies con el suelo | ✅ En E13; a ×2 los pies se balancean |
| Responder a perturbaciones dentro de límites definidos | 🟡 Pasiva, con límites derivados de la medida |
| Respetar los límites de las articulaciones | 🟡 Medido en marcha; no se comprueba al planificar en E10 ni E14 |
| Registrar métricas de estabilidad y errores de control | ✅ `results/`, en reposo y en movimiento |

**Límites conocidos al cerrar la fase** (detalle en el handbook, E16):

- El control sigue su plan sin corregir con la IMU: un error de 1 cm en el CdM o un
  suelo inclinado 0,25° incumplen los criterios.
- Las comprobaciones usan estado sin ruido ni retardos.

Esto no contradice los criterios de la fase, pero pesa para el paso al robot real.
La fase 4 debe tratarlo: las etapas 26 y 27 cubren perturbaciones y variaciones del
modelo, e incluyen la inclinación del suelo y el reparto de masas.

**Decisión:** el paso de recuperación ante empujones no funcionó con control
clásico (E14). La etapa 19 se cierra con límites definidos sin dar un paso, y la
recuperación con un paso pasa a la etapa 26, con aprendizaje por refuerzo.

## Fase 3.5 — Humanoide completo en simulación

**Objetivo:** construir el Danny completo en simulación antes de entrenar la
locomoción principal: dos piernas, pelvis, cintura, torso, dos brazos, muñecas, manos
simplificadas, cuello y cabeza, en torno a 30 GDL sin contar dedos.

**Entregable:** modelo integrado y validado, con el equilibrio y la marcha cuasiestática
reproducidos sobre el cuerpo completo.

**Propuesta aprobada:** [documentation/phase-3.5-proposal.md](documentation/phase-3.5-proposal.md).

**Antes de empezar:**

| Tarea previa, de la auditoría | Estado |
|---|---|
| `leg_ik` informa de objetivos inalcanzables y fuera de rango | ✅ |
| Autocolisión entre piernas y con el torso | ✅ |
| Regresión de equilibrio y marcha | ✅ |
| Documentar los límites actuales | ✅ [known-limitations.md](documentation/known-limitations.md) |

| Etapa | Actividad | Estado |
|---|---|---|
| 3.5.1 | Especificaciones del humanoide y presupuesto de masa | ✅ [Presupuesto](documentation/phase-3.5-mass-budget.md): objetivo 11 kg |
| 3.5.2 | Generar el modelo desde una especificación única | ✅ `robot_spec.py`, `build_model.py`; modelo idéntico al anterior |
| 3.5.3 | Torso y cintura | ✅ E18, `danny_humanoid.xml` |
| 3.5.4 | Brazos | ✅ E19: 7 GDL por brazo, en postura fija; 28 GDL en total |
| 3.5.5 | Manos | ⏳ Siguiente |
| 3.5.6 | Cabeza y cuello | ⏳ |
| 3.5.7 | Integración: equilibrio y marcha con todo el cuerpo | ⏳ |
| 3.5.8 | Validación y documentación | ⏳ |

**Reglas:** descripción modular del robot, sin duplicar articulaciones, geometría ni
control. Se conservan los experimentos anteriores para comparar. Prioridad a la física,
la cinemática y el control, no al aspecto. Sin dedos articulados ni control de
manipulación en esta fase.

**La compensación dinámica y el equilibrio activo siguen abiertos**, y pueden
desarrollarse en la fase 4.

## Fase 4 — Aprendizaje por refuerzo y locomoción

**Objetivo:** enseñar al robot a caminar con aprendizaje por refuerzo, sobre el humanoide
completo de la fase 3.5.

**Antes de entrenar**, el entorno debe incluir: límites realistas de par y velocidad,
dinámica y retardos de actuador, ruido de sensores, fricción articular, autocolisiones,
variaciones de dominio, observaciones de la IMU, y observaciones, acciones, recompensas
y condiciones de terminación bien definidas.

**Entregable:** política neuronal capaz de caminar en simulación y superar
pruebas de estabilidad.

| Etapa | Actividad |
|---|---|
| 21 | Preparar el entorno de entrenamiento |
| 22 | Definir observaciones, acciones y recompensas |
| 23 | Entrenar una política de equilibrio |
| 24 | Entrenar locomoción hacia delante y atrás |
| 25 | Entrenar giros y desplazamientos laterales |
| 26 | Entrenar resistencia a perturbaciones, incluido el paso de recuperación |
| 27 | Aplicar domain randomization, incluidos suelo inclinado, reparto de masas y retardos |
| 28 | Evaluar, optimizar y exportar las políticas |

**Tecnologías:** MuJoCo, Python, PyTorch, algoritmos como PPO, Isaac Lab si
conviene, y GPU NVIDIA local o en la nube para entrenar.

**Observaciones previstas:** orientación del torso, velocidad angular, posiciones
y velocidades articulares, contactos de los pies y velocidad objetivo.
**Acciones:** objetivos articulares.

## Fase 5 — Diseño mecánico del humanoide

**Entregable:** diseño mecánico fabricable de la primera versión de Danny. No se
compran los actuadores definitivos hasta comprobar que cumplen los requisitos.

| Etapa | Actividad |
|---|---|
| 29 | Definir dimensiones y proporciones definitivas |
| 30 | Calcular masas, cargas y torques articulares |
| 31 | Seleccionar motores, reductores y encoders |
| 32 | Diseñar articulaciones y transmisiones |
| 33 | Modelar la estructura completa en CAD |
| 34 | Analizar resistencia, rigidez y disipación térmica |
| 35 | Validar el diseño y preparar los archivos de fabricación |

## Fase 6 — Fabricación de las piernas

**Entregable:** dos piernas físicas articuladas, listas para la electrónica. Se
usa impresión 3D en las piezas adecuadas y metal donde las cargas lo requieran.

| Etapa | Actividad |
|---|---|
| 36 | Calibrar la impresora 3D y validar materiales |
| 37 | Fabricar piezas estructurales de las piernas |
| 38 | Montar articulaciones, ejes y rodamientos |
| 39 | Instalar motores y transmisiones |
| 40 | Verificar resistencia, alineación y movimiento mecánico |

## Fase 7 — Electrónica y control físico

**Entregable:** sistema físico capaz de ejecutar movimientos articulares
controlados, primero sobre un soporte de pruebas. Incluye límites de corriente,
torque y temperatura, y una parada de emergencia.

| Etapa | Actividad |
|---|---|
| 41 | Diseñar alimentación eléctrica y protecciones |
| 42 | Integrar controladores de motores |
| 43 | Instalar encoders, IMU y sensores de contacto |
| 44 | Implementar firmware y comunicaciones |
| 45 | Implementar control articular y mecanismos de seguridad |
| 46 | Calibrar y probar las articulaciones individualmente |

**Arquitectura prevista:** ordenador principal → sistema de control →
microcontroladores → drivers de motores → actuadores, con sensores que
retroalimentan el movimiento.

## Fase 8 — Sim-to-Real y locomoción física

**Entregable:** Danny caminando físicamente sobre una superficie plana, sin
asistencia en las pruebas finales que superen los criterios de seguridad.

| Etapa | Actividad |
|---|---|
| 47 | Identificar parámetros físicos reales |
| 48 | Actualizar el modelo de simulación |
| 49 | Transferir el controlador clásico al hardware |
| 50 | Transferir y adaptar las políticas aprendidas |
| 51 | Realizar pruebas de equilibrio y caminata con protección |
| 52 | Optimizar estabilidad, consumo y recuperación |

**Progresión de pruebas:** robot suspendido o sujeto a una estructura de
seguridad, después apoyo, equilibrio y pasos individuales, y por último caminata
autónoma controlada.

## Fase 9 — Torso, brazos y manos

**Entregable:** humanoide completo capaz de coordinar brazos y piernas. Levantar
un objeto cambia el centro de masas y puede requerir compensar con las piernas.

| Etapa | Actividad |
|---|---|
| 53 | Diseñar el torso y la distribución de componentes |
| 54 | Diseñar hombros, brazos y codos |
| 55 | Diseñar muñecas |
| 56 | Diseñar manos y mecanismos de agarre |
| 57 | Fabricar e integrar las extremidades superiores |
| 58 | Implementar cinemática y control de brazos |
| 59 | Coordinar brazos, torso y equilibrio |

## Fase 10 — Visión, navegación e inteligencia artificial

**Entregable:** robot capaz de comprender instrucciones y desplazarse hacia
objetivos identificados. Ejemplo: «Danny, ve a la cocina y busca una botella de agua».

| Etapa | Actividad |
|---|---|
| 60 | Integrar cámaras y percepción de profundidad |
| 61 | Implementar detección y reconocimiento de objetos |
| 62 | Implementar estimación de posición y mapas |
| 63 | Desarrollar navegación autónoma |
| 64 | Integrar reconocimiento y síntesis de voz |
| 65 | Integrar modelos de lenguaje y planificación |
| 66 | Crear una arquitectura de ejecución de tareas con supervisión de seguridad |

## Fase 11 — Manipulación autónoma

**Entregable:** ejecución autónoma de un conjunto definido de tareas domésticas.
Ejemplo avanzado: «Danny, ve a mi habitación y tráeme unos calcetines del cajón».
Manipular ropa y otros objetos deformables puede requerir investigación adicional.

| Etapa | Actividad |
|---|---|
| 67 | Detectar y estimar la posición de objetos manipulables |
| 68 | Planificar movimientos de brazos y manos |
| 69 | Aprender y controlar agarres |
| 70 | Recoger, transportar y entregar objetos |
| 71 | Manipular puertas, cajones y objetos cotidianos |
| 72 | Coordinar navegación, manipulación y planificación de tareas |

## Fase 12 — Aplicación móvil e integración final

**Entregable:** Danny integrado con una aplicación móvil y una batería de pruebas de
aceptación. La aplicación permite ver las cámaras, consultar batería y estado,
enviar instrucciones, supervisar tareas, recibir alertas, consultar errores,
activar una parada segura y controlar funciones a distancia.

| Etapa | Actividad |
|---|---|
| 73 | Desarrollar backend y comunicaciones seguras |
| 74 | Crear la aplicación móvil con React Native |
| 75 | Integrar vídeo, telemetría, comandos y alertas |
| 76 | Ejecutar pruebas integrales de autonomía, fiabilidad y seguridad |

## Duración orientativa

Con unas 14 horas semanales. Son rangos preliminares que pueden solaparse y
dependen del hardware y de las iteraciones necesarias.

| Bloque | Estimación |
|---|---|
| Fases 1–3: fundamentos y simulación | 2–4 meses |
| Fase 4: aprendizaje por refuerzo | 3–6 meses |
| Fases 5–7: diseño, fabricación y electrónica | 6–12 meses |
| Fase 8: locomoción física | 3–9 meses |
| Fases 9–11: cuerpo, percepción y manipulación | 9–24+ meses |
| Fase 12: integración final | 2–5 meses |

En total, entre 25 y 60 meses o más.
