# Plan del proyecto

Fuente de verdad del estado de cada etapa. Una etapa termina con implementación, validación
reproducible y documentación. **Nada se compra sin aprobación del usuario.**

Requisitos y criterios: [documentation/research-and-requirements.md](documentation/research-and-requirements.md).

## Etapa 1: investigación y definición

| Paso | Trabajo | Estado |
|---|---|---|
| 1.1 | Referencias, requisitos y primera comparación de actuadores | Hecho |
| 1.2 | Fuentes de movimiento humano y sus licencias | Pendiente |
| 1.3 | Arquitectura y desglose de piezas: [40 cm](documentation/architecture-and-bom.md) | Hecho y aprobado |
| 1.4 | [Handbook](documentation/HANDBOOK.md) completo | Hecho |

**Criterio del handbook:** otra persona, con su Claude y solo esa documentación, puede seguir el proyecto.
Se comprueba con un agente sin contexto previo, que debe decir qué hacer a continuación y por qué.

**Puerta, pasada:** altura y arquitectura aprobadas con datos de ficha, y handbook probado. La etapa 1.2 sigue
en paralelo y no frena el diseño.

## Etapa 2: diseño en Autodesk Fusion (actual)

| Paso | Trabajo | Estado |
|---|---|---|
| 2.1 | Probar la exportación Fusion → MJCF con un modelo simple | Pendiente |
| 2.2 | Plano del servo, maqueta impresa, cadera y pierna | Pendiente |
| 2.3 | Torso, electrónica, brazos, cabeza y recorrido de cables | Pendiente |
| 2.4 | Protección contra caídas y reserva para muñeca, pinza, cámara y voz | Pendiente |

**Puerta:** robot completo sin interferencias, y R4 y R10 cumplidos en CAD.

## Etapa 3: modelo en MuJoCo

Fusion y MuJoCo trabajan en bucle: cada cambio del CAD actualiza masas e inercias.

| Paso | Trabajo | Estado |
|---|---|---|
| 3.1 | Modelo exportado del CAD, con el modelo BAM de `xgoduck_rl`, sin confirmar en nuestros servos | Pendiente |
| 3.2 | Pares y velocidades en cuclillas, apoyo a un pie, marcha y levantarse | Pendiente |
| 3.3 | Corregir el diseño y repetir hasta cumplir | Pendiente |

**Puerta:** las trayectorias de medida de R1 y R2 son viables con los límites de ficha.

## Etapa 4: aprendizaje en mjlab

Sustituye a Isaac Lab por decisión del usuario.

| Paso | Trabajo | Estado |
|---|---|---|
| 4.1 | Preparar la RTX 3080 Ti con mjlab | Pendiente |
| 4.2 | Marcha con órdenes de velocidad | Pendiente |
| 4.3 | Levantarse | Pendiente |
| 4.4 | Imitación de movimiento humano y gestos | Pendiente |
| 4.5 | Si el hardware lo permite: trote y salto | Pendiente |
| 4.6 | Si el hardware lo permite: manipulación con visión | Pendiente |

**Puerta:** políticas robustas a aleatorización de dominio y holgura.

## Etapa 5: robot físico

| Paso | Trabajo | Estado |
|---|---|---|
| 5.1 | Banco de pruebas: medir par-velocidad, calor y holgura, e identificar con BAM. **Pide aprobación de compra** | Pendiente |
| 5.2 | Una pierna real sujeta, comparada con simulación | Pendiente |
| 5.3 | Programa del robot: bucle de 50 Hz, ONNX, paro de seguridad y batería | Pendiente |
| 5.4 | Montaje completo y calibración | Pendiente |
| 5.5 | De sujeto a libre: postura, marcha y levantarse | Pendiente |
| 5.6 | Si el hardware lo permite: conversación por voz en la nube, sin tocar el bucle de 50 Hz | Pendiente |
| 5.7 | Comprobar R1 a R11 | Pendiente |

**Por decisión del usuario, se diseña antes de comprar.** Los pasos 5.1 y 5.2 se hacen antes de comprar el
resto, y sus medidas pueden obligar a cambiar el diseño y a repetir las etapas 3 y 4.
