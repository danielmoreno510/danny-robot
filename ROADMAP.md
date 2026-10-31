# Plan del proyecto

Fuente de verdad del estado de cada etapa. Una etapa termina con implementación, validación
reproducible y documentación. **Nada se compra sin aprobación del usuario.**

Requisitos y criterios: [documentation/research-and-requirements.md](documentation/research-and-requirements.md).

## Etapa 1: investigación y definición (actual)

| Paso | Trabajo | Estado |
|---|---|---|
| 1.1 | Referencias, requisitos y primera comparación de actuadores | Hecho, en revisión |
| 1.2 | Banco de pruebas: 2 HD-1910, adaptador de bus y fuente. **Pide aprobación de compra** | Pendiente |
| 1.3 | Medir par-velocidad, calor y holgura, e identificar con BAM | Pendiente |
| 1.4 | Fuentes de movimiento humano y sus licencias | Pendiente |
| 1.5 | Arquitectura: altura, GDL, servos, ordenador, batería y lista de materiales | Pendiente |

**Puerta:** el usuario aprueba la arquitectura con datos medidos.

## Etapa 2: diseño en Autodesk Fusion

| Paso | Trabajo | Estado |
|---|---|---|
| 2.1 | Probar la exportación Fusion → MJCF con un modelo simple | Pendiente |
| 2.2 | Pierna: diseño, impresión de prueba y rodamientos | Pendiente |
| 2.3 | Torso, brazos, cabeza y recorrido de cables | Pendiente |
| 2.4 | Protección contra caídas y reserva para muñeca, pinza y cámara | Pendiente |

**Puerta:** robot completo sin interferencias, y masa estimada dentro de R4.

## Etapa 3: modelo en MuJoCo

Fusion y MuJoCo trabajan en bucle: cada cambio del CAD actualiza masas e inercias.

| Paso | Trabajo | Estado |
|---|---|---|
| 3.1 | Modelo exportado del CAD, con actuadores identificados | Pendiente |
| 3.2 | Pares en marcha, en apoyo a un pie y al levantarse | Pendiente |
| 3.3 | Corregir el diseño y repetir hasta cumplir | Pendiente |

**Puerta:** los movimientos de R1 y R2 son viables con los límites medidos.

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
| 5.1 | Una pierna real sujeta, comparada con simulación | Pendiente |
| 5.2 | Programa del robot: bucle de 50 Hz, ONNX, paro de seguridad y batería | Pendiente |
| 5.3 | Montaje completo y calibración | Pendiente |
| 5.4 | De sujeto a libre: postura, marcha y levantarse | Pendiente |
| 5.5 | Comprobar R1 a R9 | Pendiente |

**El paso 5.1 se hace al terminar la etapa 3, sin esperar al resto:** puede cambiar el diseño.
