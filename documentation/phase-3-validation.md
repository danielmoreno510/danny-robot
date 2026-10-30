# Validación de la fase 3 — Robot Danny

**Clasificación final: CONDITIONAL PASS.** Listo para empezar la fase 4 con las
limitaciones de este documento, tras las correcciones de la sección D.

Este proyecto se llamó H1 y ahora es el robot Danny. La fase auditada es la 3 del
[roadmap](../ROADMAP.md).

**Cómo se obtuvieron los datos.** Todas las cifras salen de
`results/phase3_metrics.json`, generado por la suite automática de `tests/`, sin
visor. Se miden estados simulados, no comandos. Para repetirlo:

```bash
.venv/bin/python -m pytest tests
```

**Resultado de la ejecución:** 117 de 117 pruebas pasan en 4 min 30 s. **Ojo:** esa
cifra no significa que todo funcione. 37 pruebas de `test_realism.py` y algunas de
`test_disturbance.py` **miden sin exigir**: varias registran fallos de criterio, que
se detallan en la sección B7. Los límites de las pruebas se escribieron antes de
ejecutarlas, salvo donde se indica.

**No se han abierto las ventanas del visor** en esta auditoría.

---

## A. Capacidades implementadas

| Capacidad | Estado | Dónde |
|---|---|---|
| Modelo de 70 cm y 8 kg, 12 GDL, pelvis libre, torso sin brazos | Implementado | `danny.xml` |
| Cinemática inversa analítica de pierna y de bípedo, con giro | Implementado | `leg_ik`, `joint_targets` |
| Compensación estática de gravedad con uno o dos pies | Implementado | `StanceCompensator` |
| Feedforward de velocidad | Implementado | `Biped.step_targets` |
| Equilibrio estático sobre cada pie | Implementado | `balance_biped.py`, pruebas |
| Marcha estática, pasos de hasta 8 cm y giro de 45° | Implementado | `walk_biped.py` |
| Curva de par-velocidad | Implementado, solo en empujones | `Biped.limit_torque_speed` |
| Respuesta a empujones | **Solo pasiva**, por rigidez de los servos | `push_biped.py` |
| Compensación dinámica | **NOT IMPLEMENTED**: se probó y se retiró (commit `f24ed39`) | E15 |
| Equilibrio activo con realimentación | **NOT IMPLEMENTED** | — |
| Recuperación dando un paso | **NOT IMPLEMENTED**: el intento hacía saltar al robot (commit `6c083cc`) | E14 |
| Marcha dinámica | **NOT IMPLEMENTED** | — |

**Control en lazo abierto respecto al equilibrio.** La marcha sigue un plan fijo. Los
servos corrigen el error de cada articulación, y el compensador usa la postura
medida, pero **nadie cambia el plan según la inclinación**. La IMU existe en el modelo
y no la usa ningún controlador. La prueba `test_response_is_passive_not_feedback` lo
confirma: durante un empujón, el plan cambia 0,0.

---

## B. Resultados de las pruebas

### B1. Modelo físico

| Prueba | Medido | Unidad | Criterio | Estado |
|---|---|---|---|---|
| Masa total | 8,00 | kg | 8,0 ± 0,01 | PASS |
| Altura | 0,700 | m | 0,70 ± 0,005 | PASS |
| CdM: MuJoCo frente a suma manual masa-posición | 1.1e-16 | m | < 1e-9 | PASS |
| CdM lateral, simetría | 0.0e+00 | m | < 1e-9 | PASS |
| Inercias: desigualdad triangular, peor margen | 0,031 | — | > 0 | PASS |
| Inercia del muslo frente a fórmula de cápsula | 2.6e-16 | relativo | < 0,02 | PASS |
| Ejes y rangos de las 12 articulaciones | coinciden | — | convención | PASS |
| Gravedad y paso de simulación | 9,81 y 0,002 | m/s², s | valores fijados | PASS |
| Geoms con colisión | 3 | — | 3: suelo y pies | PASS, ver C7 |

### B2. Cinemática

Se escribió una cinemática directa analítica independiente, con matrices de rotación,
y se comparó con los cuerpos de MuJoCo, cada pierna por separado.

| Prueba | Medido | Unidad | Criterio | Estado |
|---|---|---|---|---|
| FK analítica frente a MuJoCo, pierna izq., 2000 posturas | 2.3e-16 | m | < 1e-9 | PASS |
| FK analítica frente a MuJoCo, pierna der., 2000 posturas | 2.3e-16 | m | < 1e-9 | PASS |
| Orientación del pie, izq. y der. | 3.1e-14 | ° | < 1e-6 | PASS |
| IK ida y vuelta, 2000 objetivos por pierna | 1.2e-16 | m | < 1e-9 | PASS |
| Pie plano tras IK | 1.4e-14 | ° | < 1e-6 | PASS |
| IK con pelvis y pies girados, 500 poses | 2.4e-16 | m | < 1e-6 | PASS |
| Pierna recta (singularidad): resultado finito | sin NaN | — | finito | PASS |
| Objetivo inalcanzable | 0,080 | m | documenta el riesgo | ver C5 |
| Objetivos de `leg_ik` fuera de rango articular | 43 % | — | documenta el riesgo | ver C5 |

### B3. Actuadores y control

Banco de pruebas: `danny.xml` con la pelvis fija en el aire.

| Prueba | Medido | Unidad | Criterio | Estado |
|---|---|---|---|---|
| Asentamiento al escalón de 10°, peor de las 12 | 0,264 | s | < 0,5 | PASS |
| Sobrepaso, peor de las 12 | 1,6 | % | < 20 | PASS |
| Error final, peor de las 12 | 0,0000 | ° | < 0,1 | PASS |
| Seguimiento de seno ±10° a 1 Hz | 0,39 | ° | < 2 | PASS |
| Seguimiento de seno ±10° a 2 Hz | 1,36 | ° | < 2 | PASS |
| Seguimiento de seno ±10° a 4 Hz | 4,33 | ° | sin límite | informativa |
| Saturación de par | 20,0 | N·m | = 20 | PASS |

Par y velocidad máximos en la marcha E13, en articulaciones izquierdas: hip_roll 5,9
N·m y 0,43 rad/s; rodilla 4,9 N·m y 0,54 rad/s; hip_pitch 1,7 N·m y 0,75 rad/s;
tobillos 1,0-1,2 N·m y 0,83 rad/s. Muy por debajo de los ±20 N·m provisionales.

### B4. Equilibrio

Estabilidad **estática**: CdM frente al polígono de apoyo. Estabilidad **dinámica**:
centro de presiones real, calculado con las fuerzas de contacto.

| Prueba | Medido | Unidad | Criterio | Estado |
|---|---|---|---|---|
| Dos pies, 5 s: inclinación | 0,002 | ° | < 2 | PASS |
| Dos pies: margen del CdM en el polígono | 70,0 | mm | > 15 | PASS |
| Dos pies: reparto de carga izq. | 0,500 | — | 0,50 ± 0,02 | PASS |
| Pie izq., 5 s: inclinación | 0,179 | ° | < 2 | PASS |
| Pie izq.: margen del CdM (estático) | 26,8 | mm | > 15 | PASS |
| Pie izq.: margen del centro de presiones (dinámico) | 26,7 | mm | > 5 | PASS |
| Pie der., 5 s: inclinación | 0,179 | ° | < 2 | PASS |
| Pie der.: margen del CdM (estático) | 26,8 | mm | > 15 | PASS |
| Pie der.: margen del centro de presiones (dinámico) | 26,7 | mm | > 5 | PASS |
| Transferencia de peso: mayor salto de fuerza por paso | 0,066 | N | < 10 | PASS |
| Orientación del torso de pie: desviación | 0,0005 | ° | < 0,05 | PASS |

### B5. Marcha

| Prueba | Medido | Unidad | Criterio | Estado |
|---|---|---|---|---|
| E13, 8 pasos y giro: pasos | 8 | — | >= 4 | PASS |
| E13, 8 pasos y giro: duración | 50,3 | s | — | informativa |
| E13, 8 pasos y giro: deslizamiento con pie cargado | 1,48 | mm | < 5 | PASS |
| E13, 8 pasos y giro: altura mínima del pie libre | 23,0 | mm | > 20 | PASS |
| E13, 8 pasos y giro: error del pie libre frente al plan | 1,94 | mm | < 15 | PASS |
| E13, 8 pasos y giro: impacto al aterrizar | 1,00 | × peso | < 2,5 | PASS |
| E13, 8 pasos y giro: inclinación del torso | 0,50 | ° | < 5 | PASS |
| E13, 8 pasos y giro: error articular en movimiento | 0,35 | ° | < 1 | PASS |
| E13, 8 pasos y giro: margen a los topes articulares | 14,4 | ° | > 0 | PASS |
| E13, 8 pasos y giro: par máximo | 5,89 | N·m | < 20 | PASS |
| Recta a ×2: pasos | 4 | — | >= 4 | PASS |
| Recta a ×2: duración | 19,0 | s | — | informativa |
| Recta a ×2: deslizamiento con pie cargado | 3,54 | mm | < 5 | PASS |
| Recta a ×2: altura mínima del pie libre | 22,7 | mm | > 20 | PASS |
| Recta a ×2: error del pie libre frente al plan | 3,67 | mm | < 15 | PASS |
| Recta a ×2: impacto al aterrizar | 1,01 | × peso | < 2,5 | PASS |
| Recta a ×2: inclinación del torso | 1,34 | ° | < 5 | PASS |
| Recta a ×2: error articular en movimiento | 0,73 | ° | < 1 | PASS |
| Recta a ×2: margen a los topes articulares | 14,4 | ° | > 0 | PASS |
| Recta a ×2: par máximo | 5,82 | N·m | < 20 | PASS |
| E12: colocación del pie | 2,60 | mm | < 5 | PASS |
| E13: colocación del pie | 0,78 | mm | < 5 | PASS |
| Margen del CdM en toda la marcha (estático) | 26,0 | mm | > 0 | PASS |
| Margen del centro de presiones (dinámico) | 17,8 | mm | > 0 | PASS |
| Apoyos alternados | 3 | cambios | >= 3 | PASS |
| Pasos de simulación sin ningún pie en el suelo | 0 | — | = 0 | PASS |

**Barrido de velocidad** (recta, 4 pasos de 8 cm, pausas de 0,25 s):

| Velocidad | Duración | Fallos de criterio |
|---|---|---|
| ×0.7 | 44,4 s | 0 |
| ×1.0 | 32,7 s | 0 |
| ×1.27 | 26,8 s | 0 |
| ×1.6 | 22,4 s | 0 |
| ×2.0 | 19,0 s | 0 |
| ×2.3 | 17,2 s | 0 |
| ×2.5 | 16,2 s | 6 |
| ×3.0 | cae | — |

### B6. Perturbaciones

Empujón de 0,1 s en la pelvis, de pie, con curva de par-velocidad de 20 rad/s. La
recuperación es **pasiva**.

| Dirección | Límite definido [N] | Máximo medido [N] | Recuperación [s] | Inclin. máx [°] | CdM desplazado [mm] | Error art. [°] | Pies movidos [mm] |
|---|---|---|---|---|---|---|---|
| Adelante | 24 | 26 | 1,52 | 6,6 | 43,4 | 1,05 | 0,8 |
| Atrás | 24 | 26 | 1,27 | 6,3 | 41,2 | 1,05 | 0,9 |
| Izquierda | 32 | 34 | 1,57 | 8,8 | 56,4 | 1,62 | 3,3 |
| Derecha | 32 | 34 | 1,57 | 8,8 | 56,4 | 1,62 | 3,3 |

Los límites definidos son lo medido menos unos 2 N: **detectan regresiones, no
demuestran un requisito**. Máximo físico estimado sin dar un paso: unos 28 N en x y 36
N en y.

Sobre un solo pie, máximo sin caer: 24 N adelante,
24 N atrás, 9 N hacia el pie libre y
16 N hacia el apoyo (simétrico con el otro pie).

Empujón de 0,1 s **andando**, en un instante fijo: 2 N, 0 fallos;
5 N, 0; 10 N, 2; 15 N,
3. No cae, pero incumple criterios desde 10 N. Solo se
probó un instante.

### B7. Sensibilidad a efectos reales

Marcha E13, sin que el controlador conozca el cambio. Entre paréntesis, el pie más
lejos de su sitio.

| Efecto | Valor | Resultado |
|---|---|---|
| Frecuencia de control | 250 Hz | ✅ pasa (0,8 mm) |
| | 100 Hz | ✅ pasa (1,0 mm) |
| | 50 Hz | ✅ pasa (1,3 mm) |
| | 25 Hz | ❌ 4 fallos (2,2 mm) |
| Retardo del mando | 10 ms | ✅ pasa (0,8 mm) |
| | 20 ms | ❌ 4 fallos (0,7 mm) |
| Retardo del motor (primer orden) | 10 ms | ✅ pasa (0,8 mm) |
| | 20 ms | ❌ 4 fallos (0,7 mm) |
| Ruido de encoder | 0,05° | ✅ pasa (0,8 mm) |
| | 0,1° | ❌ 2 fallos (1,4 mm) |
| | 0,5° | ❌ 19 fallos (3,5 mm) |
| | 1° | ❌ 64 fallos (12,6 mm) |
| Fricción seca articular | 0,5 N·m | ✅ pasa (2,1 mm) |
| | 1 N·m | ✅ pasa (4,8 mm) |
| Fricción del suelo | 0,3 | ✅ pasa (0,7 mm) |
| | 0,6 | ✅ pasa (1,2 mm) |
| | 1,5 | ❌ 10 fallos (1,6 mm) |
| Contacto más blando | ×0,5 | ✅ pasa (1,2 mm) |
| | ×2 | ❌ 35 fallos (2,4 mm) |
| | ×5 | ❌ cae |
| CdM de pelvis y torso | +0,5 cm | ✅ pasa (4,5 mm) |
| | +1 cm | ❌ 10 fallos (7,9 mm) |
| | +2 cm | ❌ 48 fallos (15,2 mm) |
| Suelo inclinado | 0,1° | ✅ pasa (3,0 mm) |
| | 0,25° | ❌ 10 fallos (6,7 mm) |
| | 0,5° | ❌ 33 fallos (12,9 mm) |
| | 1° | ❌ 62 fallos (25,0 mm) |
| Holgura mecánica | — | NOT TESTED |

Además, `run_checks.py` comprueba con la marcha E13: masas ±10 %, torso ±10 %,
fricción 0,6 y paso de 1 ms. Todas pasan.

### B8. Software

| Prueba | Medido | Criterio | Estado |
|---|---|---|---|
| Determinismo: dos ejecuciones iguales | diferencia 0 | = 0 | PASS |
| Una sola implementación de IK | 1 | = 1 | PASS |
| Imports entre scripts de análisis | 0 | = 0 | PASS, tras corregirlo |
| Constantes geométricas iguales al modelo | coinciden | iguales | PASS |
| Experimentos anteriores (`run_checks.py`) | 16 de 16 | todos | PASS |

---

## C. Problemas identificados

**Encontrados y corregidos en esta auditoría:**

1. **Amortiguamiento de tobillo mal calculado (defecto de diseño).** El `kv` se calculó
   con la inercia del pie colgando, no con el cuerpo que sostiene en apoyo: ζ ≈ 0,15, y
   el robot rebotaba sobre los tobillos. Con `kv` 8,5 (ζ ≈ 0,6) y los mismos empujones
   (22-31 N), la recuperación bajó de 1,9-2,7 s a 1,1-1,4 s, y el máximo adelante subió
   de 24 a 26 N. Las cifras de B6 son mayores porque los límites subieron después a 24 y
   32 N. Es un cambio de control, no de masas ni geometría.
2. **Dos scripts importaban de otro script de análisis**, en contra de las reglas del
   proyecto. Los nombres compartidos pasaron a `leg_control.py`.

**Abiertos:**

3. **Sin equilibrio con realimentación.** Es el problema principal: ver la sección A y
   B7. El robot no reacciona a estar inclinado.
4. **Muy sensible a errores que un robot real tendrá:** suelo inclinado 0,25°, CdM
   desplazado 1 cm y ruido de encoder de 0,1°. Esto último se debe a que basta una
   muestra puntual para incumplir el criterio de fuerza, así que el criterio puede ser
   demasiado instantáneo.
5. **`leg_ik` no avisa.** Recorta en silencio los objetivos inalcanzables (un error de
   8 cm sin aviso) y no aplica rangos articulares: el
   43 % de objetivos aleatorios alcanzables quedan fuera de rango. Solo
   `walk_biped.py` lo comprueba, y al planificar. `balance_biped.py` y `push_biped.py`
   no. Cerca de la pierna recta la IK sigue siendo finita, pero la rodilla cambia hasta
   15,9° con un cambio de alcance mínimo. El límite del 98 % evita esa zona.
6. **Fricción 1,5 sin explicar.** Con más agarre, la marcha da 10 fallos, en su mayoría
   margen del CdM de 19 mm frente a 20 mm. Los pies no deslizan. **No se ha encontrado
   la causa.**
7. **Solo chocan pies y suelo.** Las piernas pueden atravesarse entre sí y el torso no
   colisiona. Un entrenamiento por refuerzo podría aprovecharlo.
8. **Límites de empujón derivados de la medida**, no de un requisito.
9. **Rango articular:** en los experimentos sin planificador no se comprueba al
   planificar. La marcha sí, y en estas pruebas el margen medido en movimiento es de
   10,7° o más.
10. **Deuda de documentación:** los números de los experimentos 10 a 15 del handbook son
    del modelo de 60 cm. Está indicado allí, pero no son las cifras actuales.
11. **`Biped.step` aplica el control al ritmo de la simulación** (500 Hz). La prueba de
    otros ritmos usa una subclase de prueba, no el código del proyecto.
12. **Discrepancia con la primera revisión independiente:** informó caídas con 6 ms de
    retardo en un PD externo. Aquí, retardando el mando, pasa hasta 10 ms. Son montajes
    distintos y **no se ha aclarado** cuál se parece más a un servo real.

---

## D. Correcciones necesarias antes de continuar

Antes de **entrenar**, no antes de preparar el entorno de la fase 4:

1. **Activar colisiones entre piernas y con el torso** (C7). Sin esto, una política
   puede aprender movimientos imposibles.
2. **Añadir al entorno de entrenamiento** la curva de par-velocidad, retardo y ruido de
   sensores, con los valores de B7 como punto de partida.
3. **Fijar un límite de par y velocidad realista.** ±20 N·m y 20 rad/s son provisionales.
   Una política entrenada con par de sobra no se transferirá.
4. **Hacer que `leg_ik` avise** de objetivos inalcanzables o fuera de rango (C5).
5. **Decidir el papel de la IMU:** en la fase 4 debe ser una observación de la política.

---

## E. Mejoras aplazadas

- Equilibrio activo clásico y recuperación con un paso: la etapa 26 lo cubre con
  aprendizaje por refuerzo.
- Marcha dinámica.
- Empujones durante la marcha, en más instantes y direcciones.
- Pie con suela flexible y contacto más realista.
- Límites de empujón basados en un requisito.
- Modelar el torso con articulación y brazos (fase 9).

### Clasificación de las suposiciones de simulación

| Suposición | Clase |
|---|---|
| Geometría simple de pies y piernas | 1. Aceptable por ahora |
| Paso de simulación de 2 ms | 1. Aceptable por ahora |
| Fricción de suelo 1,0 | 1. Aceptable por ahora, con variaciones en la fase 4 |
| Torso sin articulación ni brazos | 1. Aceptable por ahora |
| Sin autocolisión de las piernas | 2. Antes del aprendizaje por refuerzo |
| Motores sin retardo ni curva de par-velocidad en la marcha | 2. Antes del aprendizaje por refuerzo |
| Sin ruido de sensores | 2. Antes del aprendizaje por refuerzo |
| Saturación de par ±20 N·m | 2. Antes del aprendizaje por refuerzo |
| Sin fricción articular | 2. Antes del aprendizaje por refuerzo |
| Reparto de masas e inercias | 3. Antes de construir |
| Motores, reductoras y `armature` | 3. Antes de construir |
| Holgura mecánica | 3. Antes de construir |
| Flexibilidad de la estructura | 3. Antes de construir |
| Parámetros de contacto pie-suelo | 4. Medir en hardware real |
| Fricción y retardo reales de los motores | 4. Medir en hardware real |
| Ruido y sesgo reales de sensores | 4. Medir en hardware real |

---

## F. Preparación para la fase 4

**El modelo es apto como base del entorno de entrenamiento:** es físicamente coherente
(masas, inercias y cinemática verificadas), estable, determinista y simétrico, con
control de hasta 50 Hz y retardos de hasta 10 ms tolerados.

**No es apto todavía para entrenar políticas transferibles** hasta aplicar la sección D.
Su mayor debilidad, no corregir la inclinación, es precisamente lo que debe aportar una
política aprendida, así que no bloquea empezar. Sí significa que **no hay un controlador
clásico robusto de referencia** con el que comparar.

---

## G. Evaluación final

**CONDITIONAL PASS: listo para la fase 4 con limitaciones documentadas.**

**Por qué no PASS:** tres capacidades de la lista de auditoría no existen (equilibrio
activo, compensación dinámica y recuperación con realimentación), y el robot incumple
los criterios con desviaciones pequeñas (suelo 0,25°, CdM 1 cm). La etapa 19 se cumple
solo con límites derivados de la medida, y de forma pasiva.

**Por qué no FAIL:** el modelo y la cinemática son correctos, la marcha de 4 pasos
funciona, medida en estados reales y sin saturar, y las limitaciones están medidas, son
reproducibles y no impiden empezar.

**Estado de los criterios de salida de la fase 3:**

| Criterio | Estado |
|---|---|
| De pie sobre ambos pies | PASS |
| Sobre cualquiera de los dos pies | PASS |
| Transferir el peso | PASS |
| Cuatro pasos consecutivos | PASS |
| Controlar el contacto con el suelo | PASS en E13; a ×2 los pies se balancean |
| Responder a perturbaciones dentro de límites definidos | CONDITIONAL: pasiva, límites derivados de la medida |
| Respetar los límites articulares | PASS en la marcha; NOT TESTED al planificar en E10 y E14 |
| Registrar métricas | PASS |

**NOT TESTED:** holgura mecánica, ventana del visor, otros sistemas operativos, IMU
(no se usa), empujones en otros instantes de la marcha, y cualquier comparación con
hardware real.
