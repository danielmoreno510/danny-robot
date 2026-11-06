# Plan del proyecto

Fuente de verdad del estado de cada etapa. Una etapa termina con implementación, validación
reproducible y documentación. **Nada se compra sin aprobación del usuario.**

Requisitos y criterios: [documentation/research-and-requirements.md](documentation/research-and-requirements.md).

## Etapa 1: investigación y definición

| Paso | Trabajo | Estado |
|---|---|---|
| 1.1 | Referencias, requisitos y primera comparación de actuadores | Hecho |
| 1.2 | Fuentes de movimiento humano y sus licencias | Hecho: CMU y 100STYLE, elegidas por el usuario ([fuentes](documentation/research-and-requirements.md)) |
| 1.3 | Arquitectura y desglose de piezas: [40 cm](documentation/architecture-and-bom.md) | Hecho y aprobado |
| 1.4 | [Handbook](documentation/HANDBOOK.md) completo | Hecho |

**Criterio del handbook:** otra persona, con su Claude y solo esa documentación, puede seguir el proyecto.
Se comprueba con un agente sin contexto previo, que debe decir qué hacer a continuación y por qué.

**Puerta, pasada:** altura y arquitectura aprobadas con datos de ficha, y handbook probado. La etapa 1.2 se cerró
el 4 de noviembre de 2026.

## Etapa 2: diseño en Autodesk Fusion (actual)

| Paso | Trabajo | Estado |
|---|---|---|
| 2.1 | Probar la exportación Fusion → MJCF con un modelo simple | Hecho: modelo y exportación por scripts |
| 2.2 | Plano del servo, maqueta impresa, cadera y pierna | En curso: tercera iteración en CAD, decisiones aprobadas. Encaje y pesado, cuando lleguen impresora y servos: [diseño](documentation/lower-body-design.md) |
| 2.3 | Torso, electrónica, brazos, cabeza y recorrido de cables | En curso: robot completo en CAD; faltan conectores reales y carga radial: [diseño](documentation/upper-body-design.md) |
| 2.4 | Protección contra caídas y reserva para muñeca, pinza, cámara y voz | En curso: primera iteración de [paneles y protección](documentation/panels-and-protection.md) |
| 2.5 | Corregir los bloqueos de la [auditoría](documentation/design-audit.md) y añadir comprobaciones de montaje | En curso: 6 bloqueos corregidos; la segunda auditoría añade fallos abiertos, y parte depende de medir el servo |

**Puerta:** robot completo sin interferencias, montable, y R4 y R10 cumplidos en CAD.

## Etapa 3: modelo en MuJoCo

Fusion y MuJoCo trabajan en bucle: cada cambio del CAD actualiza masas e inercias.

| Paso | Trabajo | Estado |
|---|---|---|
| 3.1 | Modelo exportado del CAD, con el modelo BAM de `xgoduck_rl`, sin confirmar en nuestros servos | Hecho: [resultados](documentation/simulation-results.md) |
| 3.2 | Pares y velocidades en cuclillas, apoyo a un pie, marcha y levantarse | Hecho; levantarse del suelo se probó por búsqueda, sin robustez |
| 3.3 | Corregir el diseño y repetir hasta cumplir | En curso: con BAM cumple de pie y da pasos alternos con realimentación mínima; la marcha sostenida depende del aprendizaje |

**Puerta:** las trayectorias de medida de R1 y R2 son viables con los límites de ficha.

## Etapa 4: aprendizaje en mjlab

Sustituye a Isaac Lab por decisión del usuario.

| Paso | Trabajo | Estado |
|---|---|---|
| 4.1 | Preparar la RTX 3080 Ti con mjlab | Hecho: el amigo del usuario entrena con la [guía](documentation/training-guide.md) |
| 4.2 | Marcha con órdenes de velocidad | Hecho en simulación el 4 de noviembre (Fase 2, `fase2_f_rodilla`, ONNX en `policies/`); se reentrena tras medir el servo real. Historia: con el modelo anterior camina hacia delante a 0,25 m/s y gira en simulación (`marcha_reloj_1000`); con el modelo final camina hacia delante a 0,25 m/s con paso alterno (`fase1b_separacion_500`, error 0,032 m/s), tras penalizar los choques entre pies; desde cero no aprendió. Fase 2 (`fase2_a` a `fase2_c`, 10 000 iteraciones): no cumple; giro corto (0,28 de 0,5 rad/s), 88 % sin caer en la prueba de 5 min con empujones y rodilla saturada el 6,35 %. Tercer intento (recompensa de giro doble): `fase2_d_doble` (etapa 2) cumple los siete movimientos, pero el modelo final `fase2_e_doble` no pasa: giro derecha 0,158 rad/s, 82,4 % sin caer en la prueba de la guía y rodilla saturada el 13 % a 6 V (4,9 % a 7,4 V). Con los empujones de la guía (±0,12 m/s) y a 6,6 V, `fase2_d_doble` pasa caídas (99,6 %) y los siete movimientos, pero la rodilla satura el 8,2 % a 6,6 V; `fase2_e_doble` falla además el giro derecha (0,163). Cuarto intento: `fase2_f_rodilla` (castigo de par sobre el 80 % del bloqueo en rodillas y tobillos, 1500 iteraciones desde `fase2_d_doble`) **cumple la fase 2**: 100 % sin caer en la prueba de la guía y a 6,6 V, siete movimientos a 6,6 V (giro derecha 0,135 rad/s) y saturación de rodilla por debajo del 0,1 % a 6,6, 7,4 y 8,4 V (peor articulación 0,67 %). Fase 3 (agacharse): **aprobada el 5 de noviembre solo quieto** con `fase3_agacharse/model_11994.pt`, usado parado y con cambio desde la marcha de la fase 2 (100 % sin caer, velocidad ≤ 0,003 m/s, 99,6 % en 5 min, ninguna articulación sobre el 90 % del bloqueo más del 1,6 % del tiempo; ONNX en `policies/`, vídeo en `results/videos/`); caminar agachado, descartado por ahora (esa política estropea el giro de la fase 2, y `fase3_quieto` no siguió la orden de altura). Plan por fases en [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md) |
| 4.3 | Levantarse | Criterio escrito y tarea implementada el 5 de noviembre (Fase 4 en [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md)). Primera corrida boca abajo (`fase4_boca_abajo`, 3 137 iteraciones): **no cumple**, 0 % de pie, estancada desde la iteración 702: la política queda boca abajo del revés sobre la cabeza (pelvis 0,152 m, tronco a 147°), un óptimo local de la recompensa de altura, con `left_hip_pitch` empujando contra su tope (pedido -308°, real -100°); no es falta de par. Parada. Corrida 2 (`fase4_boca_abajo_ayuda`, recompensa de altura por tronco, objetivo acotado y ayuda vertical que se quita hasta la it. 2 500): se levanta el 100 % a 6,6 V, con tensión aleatoria y a 7,4 V, y cumple la saturación, pero el cambio a la marcha da 91 % a 6,6 V y 94,5 % aleatoria (se pide 95 %): no cumple por poco; corrida 3 (`fase4_pose`, pose de pie más penalizada, 5 600 iteraciones): no cumple, 0 % de pie (la política se queda a 0,141 m, justo bajo el umbral del castigo); parada. corrida 4 (`fase4_pose_sin_umbral`, pose sin umbral, peso -1,5, 1 500 iteraciones más): se levanta el 100 %, pero el cambio a la marcha da 91,0 % a 6,6 V y 93,75 % aleatoria, igual que la corrida 2: no cumple. Fase 4 no aprobada; tras el cambio caen el 6 % (hacia delante, 1,1 s después, igual en las corridas 2 y 4 y aunque se espere 10 s de pie): fallo de la política de marcha con orden cero desde la postura estirada, hipótesis sin comprobar |
| 4.4 | Imitación de movimiento humano y gestos | Pendiente |
| 4.5 | Si el hardware lo permite: trote y salto | Pendiente |
| 4.6 | Si el hardware lo permite: manipulación con visión | Pendiente |

**Puerta:** políticas robustas a aleatorización de dominio y holgura.

## Etapa 5: robot físico

| Paso | Trabajo | Estado |
|---|---|---|
| 5.1 | Banco de pruebas: medir par-velocidad, calor y holgura, e identificar con BAM | Comprado, sin recibir: 3 servos, FE-URT-2, fuente, calibrador y báscula. Faltan brazo de carga y programas de medida |
| 5.2 | Una pierna real sujeta, comparada con simulación | Pendiente |
| 5.3 | Programa del robot: bucle de 50 Hz, ONNX, paro de seguridad y batería | Pendiente |
| 5.4 | Montaje completo y calibración | Pendiente |
| 5.5 | De sujeto a libre: postura, marcha y levantarse | Pendiente |
| 5.6 | Si el hardware lo permite: conversación por voz en la nube, sin tocar el bucle de 50 Hz | Pendiente |
| 5.7 | Comprobar R1 a R11 | Pendiente |

**Por decisión del usuario, se diseña antes de comprar.** Los pasos 5.1 y 5.2 se hacen antes de comprar el
resto, y sus medidas pueden obligar a cambiar el diseño y a repetir las etapas 3 y 4.
