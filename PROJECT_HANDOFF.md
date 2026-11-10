# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.

**Fecha:** 4 de noviembre de 2026. **Etapa actual:** 3, MuJoCo, con 2.2, 2.3 y 2.4 abiertas por lo físico. La etapa 1 está
cerrada.
**Siguiente paso:** seguir el [plan de la etapa 4](#plan-de-la-etapa-4-fases) por fases. Las fases 0 a 2 están aprobadas: la Fase 2,
movimiento completo, con `fase2_f_rodilla/model_8995.pt`, fusionada en `main` el 4 de noviembre (PR 1), con su ONNX en `policies/` y vídeos en
`results/videos/`. La fase 3, agacharse quieto, está aprobada con `fase3_agacharse/model_11994.pt`, usado solo parado y con ONNX en `policies/` (ver el plan); caminar agachado queda descartado por ahora. `marcha_reloj_1000` ya caminó hacia delante y giró con el modelo
anterior (ver [Entrenamiento en simulación](#entrenamiento-en-simulación-etapa-4-sin-terminar)). El código de `training/` está en la
rama `entrenamiento-danny`, que se actualiza con `origin/main` antes de cada fase. `main` está protegida: los colaboradores entran por pull
request aprobado por el usuario, que es quien cambia el diseño. La fase 4, levantarse (boca abajo y boca arriba), está **aprobada el 7 de noviembre con `fase4_rodillas/model_11497.pt`** (corrida 10; ver más abajo), con su ONNX en `policies/danny_fase4_levantarse_2026-11-07.onnx`. El robot completo ya está en CAD:
[piernas](documentation/lower-body-design.md), [tronco](documentation/upper-body-design.md) y
[paneles](documentation/panels-and-protection.md).

**Decisión del 4 de noviembre, fuentes de movimiento humano (1.2):** CMU y 100STYLE, que permiten publicar lo derivado
([sección 6](documentation/research-and-requirements.md)). La etapa 1 queda cerrada. LAFAN1 solo en local si hiciera falta.

**Decisión del 4 de noviembre, tensión mínima:** Danny deja de caminar a 6,6 V (3,3 V por celda) y el programa corta a 6,4 V. Se sigue
entrenando de 6 a 8,4 V, pero la saturación de la Fase 2 se mide a 6,6, 7,4 y 8,4 V fijos ([guía](documentation/training-guide.md), 4.2).
La prueba de 5 minutos usa los empujones de la guía, ±0,12 m/s. El tercer intento (`fase2_e_doble`, rama `entrenamiento-danny`)
se midió con ±0,20 m/s, más fuerte que la guía. Con ±0,12 m/s y a 6,6 V, `fase2_f_rodilla` aprueba la Fase 2. Que el paro a 6,6 V mantenga los 20 minutos de R5 no está medido.

**Auditoría del 2 de noviembre:** [design-audit.md](documentation/design-audit.md). Corregidos los 6 bloqueos de montaje, el tobillo a -45°,
la pose de reposo y la simulación con BAM. Quedan puntos importantes: no imprimas piezas definitivas ni compres rodamientos de 10 × 15 × 3.
El modelo cambió, así que hay que reentrenar. Los pasos de verificación están en `check_assembly.py` y `check_standing.py`, y este último sale con
código 1 porque, sin política, las tareas fallan con ganancia 5.

**Segunda auditoría:** al final de [design-audit.md](documentation/design-audit.md) están los fallos, la tornillería, las medidas C1 a C10 y la primera tanda en PLA.
Corregido lo que no depende del servo. Lo primero con los servos es medir C1 a C5, antes de meter ningún tornillo.

**Comprado, sin recibir:** 3 servos HD-1910, FE-URT-2, fuente de alimentación, calibrador digital, báscula de cocina,
impresora 3D y filamentos. Al llegar: medir conectores, muesca y rosca del servo, comprobar encaje y pesar piezas (2.2 y
2.3), y empezar el banco (5.1). Falta el brazo de carga, que se imprime, y los programas de medida. No se compra el resto
hasta pasar 5.1 y 5.2. Claude modela en Fusion por scripts, con `DannyBridge`.

## Cómo empezar

Lee en este orden:

1. Este archivo.
2. [documentation/HANDBOOK.md](documentation/HANDBOOK.md): cómo se hace cada etapa.
3. [CLAUDE.md](CLAUDE.md): reglas de trabajo.
4. [ROADMAP.md](ROADMAP.md): plan y estado.
5. [documentation/research-and-requirements.md](documentation/research-and-requirements.md):
   decisiones, requisitos, referencias y actuadores.
6. [documentation/architecture-and-bom.md](documentation/architecture-and-bom.md): articulaciones,
   medidas, piezas, precios y riesgos.

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
.venv/bin/python analysis/estimate_robot_budget.py
```

## Situación

El 31 de octubre de 2026 el usuario reinició el proyecto: los modelos de 70 cm y la propuesta de 40 cm con
STS3215 quedan en la etiqueta `archivo-danny-70cm`. Sirve de consulta, no como base. Ya no son válidos
sus requisitos, sus 31 GDL ni Isaac Lab.

Nueva dirección: humanoide con aspecto de Tesla Optimus en pequeño, hasta 1 500 €. Del XGO-Duck se toman el
servo y el método, no la forma. Altura aprobada: 40 cm, que da esas proporciones a cambio de menos margen de
par. Prioridad: caminar natural y levantarse. Saltar, correr y manipular se intentan si el diseño lo
permite. Entrenamiento con mjlab en la RTX 3080 Ti.

**Candidato a validar:** todo HD-1910, porque el STS3215 pesa más y es más lento. Los picos superan el par
nominal. El usuario decidió diseñar antes de comprar, así que el calor se medirá en la etapa 5.1, y hasta
entonces el diseño usa datos de ficha.

## Entrenamiento en simulación (etapa 4, sin terminar)

Código en `training/`, rama `entrenamiento-danny`. Tarea `Mjlab-Velocity-Flat-Danny`, PPO de RSL-RL. Corridas con
4096 entornos. Todas las evaluaciones son deterministas, sin empujones, 256 entornos y 20 s (`scripts/evaluate_walk.py`).

- **Lo que funciona:** Danny se mantiene de pie 20 s sin caerse (`results/training_walk_*.json`). La prueba de pie
  sin acción da centro de masas a 0,227 m (`results/training_stand_measurement.json`).
- **Lo que no funcionó, sin reloj de marcha:** no caminaba. El error de velocidad es 0,155 m/s en `rewardw_300` (escala de acción 0,25) y
  0,155 m/s en `actionscale_300` (escala 0,5), frente a 0,16 m/s de quedarse quieto. Mejorar la escala de acción no ayudó.
  Esas corridas usan las recompensas de mjlab, con las que quedarse quieto da el 91 % de la recompensa de velocidad.
- **Escalas de la guía** (`marcha_guia_3000`, parada en la iteración 1000): cae en 0,40 s, error 0,54 m/s
  (`results/training_walk_marcha_guia_1000_sin_empujones.json`). Todas las terminaciones son por caída (más de 70°).
  El punto de control sin entrenar se mantiene 17,4 s, así que lo que tumba a Danny es lo aprendido: la acción cruda
  media sube de 0,05 (iteración 0) a 0,63 (iteración 1000), con `ACTION_SCALE` 1,0 y sin recorte de acciones. El ruido
  inicial de la política (desviación 1,0) lo tumba en los primeros episodios. Cambiar la escala de acción al evaluar no lo
  arregla: 0,38 s con 1,0 y 0,5, 0,46 s con 0,25.
- **Cambios propios sobre la guía** (primer intento, sin base en la guía): penalización de la magnitud de la acción
  (`mdp.action_l2`, peso −0,05) y desviación inicial de la política 0,5. Con peso −0,01, 300 iteraciones: 0,66 s. Con
  −0,05, 300 iteraciones (`penalizacion_accion_005_300`): de pie el 97,7 % de los episodios, error 0,155 m/s, igual que
  quieto. Altura máxima de suela en 20 s: 40 mm de media (`results/training_peak_height_penalizacion_accion_005_300.json`,
  máximo de todo el episodio, no por paso).
- **Trampa de los pies** (`scripts/measure_gait.py`, orden fija de 0,25 m/s): con esa configuración Danny se queda sobre
  el pie izquierdo con el derecho a 20–25 mm, sin apoyos y sin avanzar (0,0007 m/s,
  `results/training_gait_penalizacion_accion_005_300.json`). El `air_time` de mjlab paga mientras el pie está en el aire, y
  `foot_swing_height` (−5) castiga cada aterrizaje, así que no apoyar nunca le sale a cuenta. Se probaron dos arreglos,
  300 iteraciones cada uno:
  - `aterrizaje_300`, `air_time` pagado al aterrizar (`mdp.feet_air_time_at_landing`, mismo total por paso que mjlab):
    1,37 apoyos por pie y segundo, pero de un solo pie, sin avanzar (`results/training_gait_aterrizaje_300.json`). Al final
    del entrenamiento: episodio 866 pasos, `peak_height_mean` 23,6 mm y `air_time` 0,80, que ya cumple la comprobación
    de las 1000.
  - `pasos_alternos_300`, pagado solo si aterriza el pie contrario al último: vuelve a quedarse sobre un pie, sin apoyos
    (`results/training_gait_pasos_alternos_300.json`), con `peak_height_mean` 12,8 mm. Se descartó.
  Ningún término premia el pie levantado: es una postura neutra. Lo que falla es que no aprende a avanzar en 300
  iteraciones. xgoduck_rl camina con los mismos valores (`tasks/__init__.py` de su repositorio), pero entrena hasta 50 000.
- **`marcha_aterrizaje_3000`**, la configuración de `aterrizaje_300`, parada en la iteración 1694 para dar paso a la
  siguiente. Pasó la comprobación de la guía a las 1000 (log de entrenamiento: episodio
  unos 970 pasos, `peak_height_mean` 23 mm, `air_time` 1,1). Aun así no camina: de pie el 100 %, error 0,158 m/s
  (`results/training_walk_marcha_aterrizaje_1000.json`) y, con orden fija de 0,25 m/s, avance de 0,0003 m/s, apoyo a un
  pie el 90 % del tiempo y ningún apoyo alterno (`results/training_gait_marcha_aterrizaje_1000.json`): sigue pisando con
  un solo pie. Esas dos medidas leen la velocidad en el origen de `base_link`.
- **Velocidad en el centro de masas:** el origen de `base_link` está en el suelo, a unos 23 cm del centro de masas.
  `track_linear_velocity` se sustituye por `mdp.track_com_linear_velocity`, que lee `root_com_lin_vel_b` (existe en
  mjlab 1.3.0). `evaluate_walk.py` y `measure_gait.py` miden también ahí, así que sus cifras no se comparan directamente
  con las anteriores. Hipótesis de la guía, sin confirmar.
- **`marcha_com_3000`** (parada en la iteración 1402 por el cambio de modelo de la auditoría), todos los valores de la guía más los tres cambios propios (`action_l2`, desviación
  inicial 0,5 y `air_time` al aterrizar) y la velocidad en el centro de masas. Comprobación de la guía a las 1000; criterio
  a las 3000 con `evaluate_walk.py` y `measure_gait.py`. Pasó la comprobación de la guía a las 1000 (log: episodio 923 pasos,
  `peak_height_mean` 27,8 mm, `air_time` 1,04, `track_linear_velocity` 0,89), pero tampoco camina: de pie el 100 %, error
  0,155 m/s (`results/training_walk_marcha_com_1000.json`); con orden fija de 0,25 m/s avanza 0,0003 m/s, con un solo pie
  apoyado el 89 % del tiempo y 0,2 % de apoyos alternos (`results/training_gait_marcha_com_1000.json`). En `model_800` se
  apoya en el izquierdo y sube y baja el derecho unas 3 veces por segundo (medida aparte, sin script en el repositorio).
- **Modelo de la auditoría (2 de noviembre):** `danny_training.xml` nuevo, tobillo de -45° a 40° (`stand` a -26,5°, 14,3° del
  límite blando de -40,75°), brazos en reposo a 8° de hombro y -17,2° de codo, e IMU en el sitio `imu` de `torso_1`. Los puntos de
  control anteriores no valen para este modelo.
- **Aterrizaje repetido:** se probó una penalización de -2 (`mdp.repeated_foot_landing`, ya quitada) contra el atajo de
  subir y bajar un solo pie. No lo arregló (ver `marcha_pies_kp5`).
- **`marcha_pies_kp5`** (modelo de la auditoría, `KP_FW` 5, aterrizaje repetido -2, parada a las 1000): no aprueba
  (`results/training_gait_marcha_pies_kp5_1000.json`). De pie el 100 %, pero avanza 0,001 m/s, solo el 3,6 % de los apoyos
  alternan y el pie izquierdo hace el 2 % de los apoyos. La penalización de aterrizaje repetido no cambió el atajo. Pico de par
  (`results/training_torque_marcha_pies_kp5_1000.json`): rodilla izquierda 0,97 N·m, rodilla derecha 0,85, tobillo de balanceo
  derecho 0,83, tobillo de cabeceo derecho 0,77; ninguna pasa el 90 % del par de parada a 6 V más del 0,2 % del tiempo.
  Con acción cero, 7 de 64 entornos caen en 5 s con este modelo (antes ninguno): `stand` ya no es estable sin política.
- **Reloj de marcha** (`gait_clock.py`, `mdp.gait_contact_phase`): fase de 0,8 s que solo avanza con orden de movimiento;
  seno y coseno en actor y crítico; recompensa de contacto en contrafase, peso 1,5, y con orden cero premia los dos pies
  apoyados. Sustituye a `air_time`, a la penalización de acción y a la de aterrizaje repetido, que se quitaron del código
  (el historial está en git). Lo comprueba `scripts/check_gait_clock.py`.
- **`marcha_reloj_1000`: camina.** Desde cero, 1000 iteraciones, `model_999.pt`, con el modelo de la auditoría, `KP_FW` 5 y
  la velocidad en el centro de masas. Con orden fija de 0,25 m/s, sin empujones, 256 entornos y 20 s
  (`results/training_gait_marcha_reloj_1000.json`): sin caer el 100 %, avance 0,248 m/s, error 0,020 m/s, apoyos alternos
  99,9 %, pie izquierdo el 48,9 % de los apoyos. Con órdenes aleatorias (`results/training_walk_marcha_reloj_1000.json`):
  de pie 99,2 % de los entornos, error 0,028 m/s. Altura máxima de suela 43 mm
  (`results/training_peak_height_marcha_reloj_1000.json`). Log de entrenamiento: episodio de 996 pasos y
  `peak_height_mean` de 19,7 mm. Con el punto de control de la iteración 350 ya caminaba igual (error 0,023 m/s). El usuario
  lo vio caminar hacia delante en el visor.
- **Giro de `marcha_reloj_1000`** (`results/training_gait_reloj_1000_*.json`, 256 entornos, 20 s, sin empujones, giro de ±0,5 rad/s,
  el máximo del entrenamiento): en el sitio gira a +0,40 y -0,40 rad/s y avanza 0,002 m/s; andando a 0,25 m/s gira a +0,42 y
  -0,37 rad/s y avanza 0,249 y 0,249 m/s. Error de giro de 0,12 a 0,14 rad/s, o sea que llega al 75-85 % de la orden. En los cuatro
  casos el 100 % no cae y el 99,8 % de los apoyos alternan. Avance con giro: sin caer y sin perder la marcha.
- **Pares de `marcha_reloj_1000`** (`results/training_torque_marcha_reloj_1000.json`, orden de 0,25 m/s): tobillo de cabeceo
  derecho 1,02 N·m, izquierdo 0,92, rodilla izquierda 0,90, derecha 0,88, cadera de balanceo izquierda 0,70; el resto, por
  debajo de 0,5. El 90 % del par de parada a 6 V (0,762 N·m) se supera el 7,5 % del tiempo en el tobillo de cabeceo derecho
  y el 3,5 % en el izquierdo. La guía pide no saturar ninguna articulación más del 5 %: el derecho no lo cumple a 6 V, aunque
  ese umbral se mide con el par de parada a 6 V y la aleatorización llega a 8,4 V (1,067 N·m). Sin revisar por tensión.
- **Aviso sobre las medidas de órdenes fijas anteriores:** `rel_forward_envs` de mjlab vale 0,2, así que el 20 % de los entornos
  recibe siempre (0,3; 0; 0), sin giro, y `measure_gait.py` y `measure_actuator_torque.py` no lo quitaban. Los JSON
  `training_gait_*` y `training_torque_*` anteriores al 2 de noviembre por la tarde (`aterrizaje_300`, `pasos_alternos_300`,
  `penalizacion_accion_005_300`, `marcha_aterrizaje_1000`, `marcha_com_1000`, `marcha_pies_kp5_1000`) llevan esa mezcla: su
  conclusión (no avanza, un solo pie) no cambia, pero las cifras de orden y de par no son de una orden de 0,25 m/s limpia. Las
  de `marcha_reloj_1000` se repitieron ya sin esos entornos.
- **Sin comprobar de `marcha_reloj_1000`:** no se probó marcha lateral ni los empujones;
  tampoco con armadura y holgura aleatorias. El 94 % del tiempo hay un solo pie apoyado, así que casi no hay fase con los
  dos pies en el suelo. Falta saber si el criterio de la guía (5 minutos simulados sin caer en el 95 % de las pruebas, con
  aleatorización completa) se cumple. La política necesita la fase del reloj como entrada: el robot real tendrá que generarla
  por firmware. Los puntos de control anteriores no cargan con la observación nueva.
- **Pares** (`results/training_torque_*.json`): en ambos casos la mayoría de articulaciones usa menos de 0,4 N·m. Los
  tobillos de cabeceo llegan a 0,94–0,98 N·m con escala 0,5 (el límite a 6 V es 0,762 N·m y a 8,4 V es 1,067 N·m).
  Solo pasan el 90 % del límite entre el 4 % y el 10 % del tiempo. La saturación de par no explica el fallo.
- **Seguimiento de consigna:** el error RMS de las articulaciones es 0,047 rad en `rewardw_300` y 0,096 rad en
  `actionscale_300`. Los tobillos de cabeceo son lo peor (0,35–0,43 rad con escala 0,5).
- **Prueba aislada del tobillo** (escalón de 0,3 rad, sin política, `scratch/diag_ankle_step.py`): el tobillo sube
  hacia la consigna pero se queda corto y oscila. No separa el actuador de la caída del cuerpo.
- **Pies:** las suelas se despegan entre 1 y 2 cm en el entorno 0, y hasta 5 cm en algún entorno de los 64.
  No hay marcha sostenida.

Lo que no se ha comprobado: si el modelo BAM sigue la consigna en un banco sin el cuerpo, si el contacto del pie
(cajas de 8,5 × 4,8 cm, fricción 0,8, pies a 6,4 cm entre centros) limita el paso, y si más iteraciones cambian algo.
Las conclusiones dependen del modelo de actuador (`params/hd1910_m6.json`, de xgoduck_rl) y no están confirmadas
en servos reales.

## Plan de la etapa 4 (fases)

Orden fijo, sin saltarse pruebas. El criterio se escribe antes de cada corrida larga. Si una fase falla, se para, se escribe lo
que se vio y se avisa; no se alargan las corridas para ver si se arreglan solas. Cada fase tiene una corrida nueva con nombre
claro y se conserva el punto de control que aprobó. Todo se reentrenará después de medir los servos reales, porque los
parámetros son de otro robot: lo aprendido no se instala directo en el robot físico. La fase del reloj irá en el programa de
50 Hz de la Radxa.

| Fase | Trabajo | Estado |
|---|---|---|
| 0 | Preparar: main fusionado, modelo comprobado, colisiones, masa, centro de masas y bloqueo de par | Hecho el 2 de noviembre (abajo) |
| 1 | Validar la marcha con el modelo final | Aprobada el 2 de noviembre con la fase 1b (abajo): desde cero no aprendió, y reentrenando desde la política antigua con penalización de choques sí |
| 2 | Movimiento completo (avanzar, retroceder, lateral, girar, quieto) con armadura, holgura y empujones | Aprobada el 4 de noviembre con `fase2_f_rodilla/model_8995.pt` (ver más abajo; PR 1 a main) |
| 3 | Agacharse: orden de altura del cuerpo, hasta 3 000 iteraciones | **Aprobada (agacharse quieto)** el 5 de noviembre con `fase3_agacharse/model_11994.pt`, usado solo parado; caminar agachado, descartado por ahora (ver más abajo) |
| 4 | Levantarse (4.3): política propia; boca abajo y boca arriba | **Aprobada el 7 de noviembre con `fase4_rodillas/model_11497.pt` (corrida 10)**, con una excepción de cola aceptada por el dueño (abajo) |
| 5 | Sentarse y tumbarse: política propia, hasta 5 000 iteraciones por acción; solo si la fase 4 aprueba | **Sentarse: aprobación RETIRADA por el dueño el 8 de noviembre** (se deja caer; la pelvis baja 17 cm en unos 0,4 s y el criterio de 0,6 m/s era demasiado flojo); se reentrena con un criterio nuevo. **Tumbarse (`fase5_tumbarse_desde_sentado/model_11497.pt`, corrida 3) NO cumple** el giro de la pelvis (3,3 % / 6,1 % de los robots, pide 100 %); la prueba que aprueba es la cadena sentarse + tumbarse. La fase 6 **no empieza** hasta que el dueño lo confirme |
| 6 | Movimiento natural (4.4): imitar CMU y 100STYLE, retargeteadas con GMR | Pendiente: fuentes elegidas el 4 de noviembre |

Fuera del plan, salvo que el usuario lo pida: correr, saltar y tomar objetos.

**Fase 0, hecha** (`scripts/check_training_setup.py` lo comprueba): `danny_training.xml` con 1,3006 kg, 13 `<pair>`, tobillo de -45° a 40° y
sitio `imu`; las 31 cajas de colisión con `condim` 3 y prioridad 1 (antes 1 y 0, y el suelo les ponía su fricción de 1,0); masa
total de 0,90 a 1,10 veces la nominal y centro de masas de hasta unos 12,6 mm por eje, con `shared_random=True` en todos los cuerpos.
El centro de masas pasa de 10 mm porque el desplazamiento se aplica en el marco de cada cuerpo y el 24 % de la masa está en eslabones
con el marco girado. La inercia no escala con la masa (`dr.body_mass`). Par de bloqueo medido: 0,739 N·m a 6 V y 0,912 N·m a 7,4 V
(a 8,4 V, 1,036 N·m por extrapolación, sin medir). Los scripts de medida aceptan `--vin` para fijar la tensión sin caída de carga.

**Fase 1, criterio (escrito antes de entrenar):** corrida `fase1_marcha_1000`, 1 000 iteraciones desde cero, 4096 entornos, `KP_FW` 5,

reloj de 0,8 s, `std` de velocidad 0,125 y la configuración actual de `env_cfgs.py`. Se evalúa `model_999.pt` con

`measure_gait.py --vin 7.4` (orden de 0,25 m/s, 256 robots, 20 s, sin empujones). Aprueba si, a 7,4 V: apoyos alternos de 0,9 o más,

error de velocidad de 0,08 m/s o menos y 95 % o más de los 256 robots sin caer. Se mide también a 6 y 8,4 V, con el pico de par por

articulación (`measure_actuator_torque.py --vin ...`). Si no aprueba, se compara con `marcha_reloj_1000` (modelo anterior, 1,29 kg,

sin autocolisiones ni cajas de condim 3) para decir si dependía de pesar menos o de atravesar piezas.

**Fase 1, resultado: no aprueba.** Corrida `fase1_marcha_1000` (el modelo final, desde cero, 4096 entornos). Se cayó con un error de
CUDA en la iteración 701 (`Warp CUDA error 700`, coincidió con un cierre forzado de visores; el controlador de NVIDIA registró
tres eventos 153; la causa no está probada) y se reanudó desde `model_700.pt` con 300 iteraciones más (`fase1_marcha_1000_reanudada`,
hasta la 1 000). Se evalúa `model_999.pt`, orden de 0,25 m/s, 256 robots, 20 s, sin empujones
(`results/training_gait_fase1_marcha_1000_*V.json`):

| Tensión | Sin caer | Avance | Error de velocidad | Apoyos alternos | Aterrizajes del pie izquierdo |
|---|---|---|---|---|---|
| 7,4 V | 100 % | 0,001 m/s | 0,250 m/s | 0,008 | 99,5 % |
| 6 V | 100 % | 0,000 m/s | 0,250 m/s | 0,003 | 99,8 % |
| 8,4 V | 100 % | 0,001 m/s | 0,250 m/s | 0,030 | 98,1 % |

Criterio: alternancia de 0,9 o más, error de 0,08 m/s o menos y 95 % sin caer. Solo cumple lo último. Danny se queda quieto y mueve
el pie izquierdo sin avanzar, como las corridas sin reloj. Log: recompensa de velocidad 0,93 desde la iteración 100 (con `marcha_reloj_1000`
llegaba a 2,2 hacia la 300) y `peak_height_mean` de 9 a 10 mm; la altura máxima de suela fue de 25 mm
(`results/training_peak_height_fase1_marcha_1000.json`). Pico de par a 7,4 V (`results/training_torque_fase1_marcha_1000_*V.json`):
hombro de balanceo 0,94 N·m, cadera de guiñada izquierda 0,93; a 6 V el pico es 0,76 y a 8,4 V, 1,06, es decir, el bloqueo en cada tensión.

**Por qué: comparación con `marcha_reloj_1000`, sin entrenar.** La política antigua (`model_999`, que caminaba con el modelo anterior) se
prueba en el modelo nuevo a 7,4 V con orden de 0,25 m/s (`results/training_gait_antigua_en_modelo_nuevo_*.json`, `measure_gait.py` con
`--nominal-mass` y `--drop-link-contacts`):

| Variante en el modelo nuevo | Sin caer |
|---|---|
| Completo (masa y centro de masas aleatorios, 13 contactos entre eslabones) | 1,6 % |
| Masa nominal, con los 13 contactos | 0 % |
| Masa nominal, sin los contactos de brazo | 0 % |
| Masa nominal, sin los de pie (3 pares) | 63 % |
| Masa nominal, sin los de brazo ni los de pie | 64 % |
| Masa nominal, sin ningún contacto | 100 % (error 0,019 m/s) |
| Masa aleatoria, sin ningún contacto | 65 % |

Conclusión: no dependía de pesar menos (con 1,30 kg y sin contactos camina igual); dependía de **atravesar piezas**. Los contactos del pie
izquierdo con el derecho tumban casi todo (de 0 % a 63 % al quitarlos), los del muslo, la tibia y la cadera entre sí aportan el resto,
y los de brazo no influyen por sí solos. La aleatorización nueva de masa y centro de masas cuesta otro 35 % (de 100 % a 65 %). Holguras en
la pose de pie (`mj_geomDistance`): suela con suela 16 mm (suelas de 85 × 48 mm con 64 mm entre centros), cadera con cadera 8,6 mm, muslo con
muslo 23 mm, tibia con tibia 23 mm. Con tan poco margen, cualquier balanceo lateral hace chocar los pies. La política antigua nunca aprendió a evitarlo
porque no existía ese contacto, y la nueva no lo descubrió en 1 000 iteraciones.

**Lo que no se comprobó de la fase 1:** si más iteraciones, otra semilla o una recompensa de separación entre pies lo resuelven (hay una sola
corrida con el modelo final); qué par de contacto concreto roza en cada paso; y si la causa del error de CUDA fue el cierre forzado.

**Fase 1b, criterio (escrito antes de entrenar; el usuario aprobó el intento):** corrida `fase1b_separacion_500`, 500 iteraciones
partiendo de `marcha_reloj_1000/model_999.pt` (la política que caminaba sin contactos), en el modelo final y con la aleatorización de la fase 0.
Dos términos nuevos en `env_cfgs.py`: `link_pair_contacts` (peso -1, un punto por cada contacto activo de los 13 `<pair>`; con la política
antigua da -0,59 por paso, y coincide con el recuento independiente de unos 0,57) y `feet_lateral_clearance` (peso -2, separación lateral
entre suelas por debajo de 10 mm; con la política antigua da -4,2 por paso; de pie hay 16 mm). Se prueba con
`scripts/check_feet_lateral_clearance.py`. El sensor de contactos de mjlab no detecta los choques entre eslabones con sub-árboles (mide 0
con una política que choca miles de veces), por eso el recuento lee los contactos de la simulación. Se evalúa el último punto de control con
`measure_gait.py --vin 7.4` y el mismo criterio de la fase 1: alternancia de 0,9 o más, error de 0,08 m/s o menos y 95 % o más sin caer, y además
reparto de aterrizajes entre 0,4 y 0,6 y un recuento de choques entre eslabones cercano a 0. Se mide también a 6 y 8,4 V, con el pico de par.
Si no aprueba, se para, se escribe lo visto y se pide decidir un cambio de diseño (separar los pies o estrechar las suelas, sin tocar `cad/`
desde aquí). No se alarga la corrida.

**Fase 1b, resultado: aprueba.** Corrida `fase1b_separacion_500` (500 iteraciones desde `marcha_reloj_1000/model_999.pt`, de la 1 000 a la
1 499), punto de control aprobado `model_1498.pt`, guardado fuera del repositorio en `D:\Claude\puntos_de_control\fase1b_separacion_500_model_1498.pt`
(5 MB, no va a git). Orden de 0,25 m/s, 256 robots, 20 s, sin empujones (`results/training_gait_fase1b_separacion_500_*V.json`):

| Tensión | Sin caer | Avance | Error de velocidad | Apoyos alternos | Aterrizajes del pie izquierdo |
|---|---|---|---|---|---|
| 7,4 V | 100 % | 0,263 m/s | 0,032 m/s | 0,993 | 48,9 % |
| 6 V | 100 % | 0,244 m/s | 0,031 m/s | 0,997 | 48,9 % |
| 8,4 V | 100 % | 0,266 m/s | 0,034 m/s | 0,993 | 49,2 % |

Cumple todo el criterio a las tres tensiones: alternancia de 0,9 o más, error de 0,08 m/s o menos, 95 % o más sin caer, y reparto entre pies de
0,4 a 0,6. Choques entre eslabones: 0,000 por paso con la política determinista (recuento propio) y `feet_lateral_clearance` de -0,04 por paso
(antes -4,2). En el log, al final: episodio de 987 pasos, recompensa de velocidad 2,12, `peak_height_mean` de 17,8 mm y el 1,3 % de los
entornos con algún choque (con aleatorización y empujones). Altura máxima de suela de 40 mm (`results/training_peak_height_fase1b_separacion_500.json`).

Pico de par (`results/training_torque_fase1b_separacion_500_*V.json`, bloqueo de 0,739 N·m a 6 V, 0,912 a 7,4 V y 1,036 a 8,4 V): los picos llegan
al bloqueo en las rodillas y los tobillos de cabeceo (0,76 / 0,94 / 1,07 N·m, hasta un 3 % por encima de la cifra de bloqueo). Por encima del 5 % del
tiempo sobre el 90 % del bloqueo: a 7,4 V y a 8,4 V, el tobillo de cabeceo derecho (9,9 % y 9,1 %; el izquierdo, 3,1 % y 2,3 %); a 6 V, las dos rodillas
(7,9 % y 7,5 %) y ese tobillo (8,1 %). La guía pide no saturar ninguna articulación más del 5 % del tiempo: **no se cumple todavía**, y es parte del criterio
de cierre de la fase 2, no de esta.

**Lo que no se comprobó de la fase 1b:** solo se probó hacia delante a 0,25 m/s (ni giro, ni lateral, ni marcha atrás, ni con empujones); el
recuento de choques se validó con la política antigua y contra un recuento independiente, pero `Metrics/link_contact_fraction` salió `nan` en mi prueba;
no se probó otra semilla; no hay vídeo (la grabación automática con `play --video` no terminó en 9 minutos). La política aprobada parte de
una antigua: la que se entrenó desde cero con el modelo final no aprendió a andar (`fase1_marcha_1000`), así que el entrenamiento desde cero
necesita estas penalizaciones desde el principio o un arranque como este.

**Fase 2, criterio (escrito antes de entrenar):** una sola política con avanzar, retroceder, paso lateral, girar y parar con orden cero. Parte de
`fase1b_separacion_500/model_1498.pt` y entrena 10 000 iteraciones en tres corridas seguidas, cada una reanudada de la anterior, con más
aleatorización en cada una (`DANNY_DR_STAGE` en `env_cfgs.py`; sin la variable y al evaluar vale la 3, la completa):

| Corrida | Iteraciones | Etapa | Armadura de los servos | Holgura máxima | Empujones (XY) |
|---|---|---|---|---|---|
| `fase2_a` | 3 000 | 1 | ±10 % | 0,15° | ±0,12 m/s |
| `fase2_b` | 3 000 | 2 | ±20 % | 0,30° | ±0,16 m/s |
| `fase2_c` | 4 000 | 3 | ±30 % | 0,50° | ±0,20 m/s |

La holgura es un juego mecánico sobre la posición objetivo (`actuator.apply_backlash`, comprobado con `scripts/check_backlash.py`): una aproximación,
no el modelo de un servo real. Órdenes: avance de -0,25 a 0,25 m/s, lateral de -0,1 a 0,1 m/s y giro de -0,5 a 0,5 rad/s (11° por paso de 0,4 s, por
debajo de los 20° por paso del límite real), con el 20 % de los entornos a orden cero y sin los entornos forzados a avanzar de mjlab. Controles al final de
cada corrida (3 000, 6 000 y 10 000 iteraciones) con `scripts/evaluate_movements.py` a 7,4 V, aleatorización completa y sin empujones: cada uno de
los siete movimientos (adelante, atrás, lateral a cada lado, giro a cada lado, quieto) cumple si el 95 % o más de 256 robots no cae en 20 s y el error
de velocidad es de 0,08 m/s o menos y el de giro de 0,15 rad/s o menos; quieto, además, con velocidad media de 0,03 m/s o menos y giro medio de
0,10 rad/s o menos. Los controles intermedios informan y no frenan la corrida, salvo que la política colapse (menos del 50 % sin caer en adelante).
La fase se aprueba con la corrida `fase2_c` si cumplen los siete movimientos y además la prueba de la guía: 0,25 m/s durante 300 s simulados
(`measure_gait.py --pushes --seconds 300`, sin fijar la tensión) sin caer en el 95 % de los robots, y sin saturar ninguna articulación más del 5 %
del tiempo (`measure_actuator_torque.py --pushes --seconds 300`, contra el bloqueo a 6 V). Si no aprueba, se para y se explica; no se alarga.
Si aprueba: ONNX, vídeos si caben, un PR a `main` y aviso al dueño.

## Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que las cifras de `results/actuator_comparison.json` sean de diseño: son de primer orden.

## Pendiente de decidir por el usuario

- Licencia del repositorio, que es público.

## Última verificación

`analysis/compare_actuators.py`, `analysis/estimate_robot_budget.py`, `analysis/inspect_model.py` con un modelo mínimo
y pyflakes, el 31 de octubre de 2026. El handbook se probó dos veces con agentes sin contexto: en ambas dijeron qué
toca y por qué, y se corrigieron sus hallazgos. La segunda reprodujo igual los JSON de `results/`. Tras esas
correcciones no se repitió la prueba. La ventana del visor no se ha visto. No hay simulador, pruebas ni CAD
en esta nueva etapa.

Etapa 2.1 superada el 31 de octubre de 2026 en Fusion 2706.1.17, en macOS: `BuildExportTest` y `ExportModel` dieron
`cad/export/export_test/`, que carga en MuJoCo con 0,7536 kg, un 0,00 % de diferencia con el CAD, y el eje en X,
como en el CAD. Ese modelo mantiene su punto de giro al mover la bisagra. El complemento `DannyBridge` arrancó solo
al reiniciar Fusion. No se ha comprobado: la ventana del visor, el botón de ACDC4Robot en Fusion, ni un modelo con
ocurrencias giradas o varias articulaciones. La vista de Fusion sigue con Y arriba, así que los modelos se ven
tumbados, aunque sus coordenadas sean correctas. La pieza de prueba no se guardó en Fusion.

Etapas 2.2 y 2.3 en curso: el diseño de Fusion tampoco se guarda, se regenera con los seis comandos de
`documentation/lower-body-design.md`. `check_robot.py` da cero fallos con 24 cuerpos y 23 articulaciones, y
falla con resultados vacíos, obsoletos o con choques. Esa comprobación no ve el montaje: tanto piernas como
tronco la pasaron sin poder montarse. Por eso una revisión independiente sondeó los STL en cada iteración. Nada se ha
impreso ni montado. Lee las limitaciones de esos dos documentos antes de seguir.

### Fase 2, control 1 (fase2_a, etapa 1, 3000 iteraciones, model_4497)

Medido a 7,4 V con la aleatorización completa (etapa 3) y sin empujones (`results/training_movements_fase2_a_4497.json`): los siete movimientos cumplen (sin caer 1,000; error lineal 0,026-0,065 m/s; error de giro 0,035-0,132 rad/s; quieto 0,0013). Giro en el sitio queda cerca del límite (0,132 y 0,118 frente a 0,15). No se ha medido aún con empujones, durante 5 min ni la saturación de par: eso es la prueba final. Sigue `fase2_b` (etapa 2, 3000 iteraciones).

### Fase 2, control 2 (fase2_b, etapa 2, 3000 iteraciones más, model_7496)

Mismas condiciones que el control 1 (`results/training_movements_fase2_b_7496.json`): seis de siete movimientos cumplen; **giro a la derecha no cumple** (error de giro 0,196 rad/s, límite 0,15) y giro a la izquierda queda justo en el límite (0,150). Sin caer 1,000 en todos; adelante, atrás, laterales y quieto cumplen. El control es informativo y no detiene la corrida; empeora el giro respecto al control 1 (0,132 y 0,118), probablemente por la aleatorización más dura de la etapa 2 (causa no medida). Sigue `fase2_c` (etapa 3, 4000 iteraciones, desde model_7496).

### Fase 2, resultado final (fase2_c, etapa 3, 4000 iteraciones más, model_11495): NO CUMPLE

Corridas: `fase2_a` (etapa 1, 1498 a 4497), `fase2_b` (etapa 2, hasta 7496), `fase2_c` (etapa 3, hasta 11495). Puntos de control en `D:\Claude\puntos_de_control\` (fuera de git). Se paró aquí, sin alargar la corrida, como pide el plan.

- **Movimientos** (`results/training_movements_fase2_c_11495.json`, 7,4 V, aleatorización completa, sin empujones): adelante, atrás, laterales y quieto cumplen. **Giro a ambos lados no cumple**: error de giro 0,219 y 0,232 rad/s (límite 0,15). Causa medida: la velocidad de giro lograda es 0,28 y 0,27 rad/s con orden de 0,5, y baja con cada etapa (control 1: 0,39 y 0,42; control 2: 0,36 y 0,31; control 3: 0,28 y 0,27). El giro queda corto, no inestable (sin caer 1,000).
- **Prueba de la guía** (0,25 m/s, 300 s, 256 robots, empujones, aleatorización completa; `results/training_gait_fase2_c_prueba_guia.json`): **sin caer 0,883, por debajo del 0,95 exigido**. Entre los que no caen: velocidad 0,249 m/s, error 0,037, paso alterno 0,994.
- **Par** (`results/training_torque_fase2_c_prueba_guia.json`, 64 robots, 300 s, empujones): **rodilla izquierda satura (más del 90 % del bloqueo) el 6,35 % del tiempo**, por encima del 5 % exigido; rodilla derecha 3,5 %, cadera roll izquierda 0,7 %, tobillo izquierdo 0,6 %. Par máximo 1,067 N·m (bloqueo a 8,4 V: 1,036 extrapolado).
- **No comprobado:** por qué cae el 11,7 % en 5 min (si es por empujones, por una tensión baja o por deriva con el tiempo; no se separó). No se probó si el giro mejora con una orden menor ni con más peso de seguimiento de giro. No hay vídeo ni ONNX ni PR a main: la fase no pasó.
- **Siguiente paso, decide el dueño:** (a) endurecer la recompensa de giro y reentrenar desde `fase2_a` (que cumplía los siete movimientos sin empujones), (b) bajar la orden de giro por debajo de 0,5 rad/s si 20° por paso lo permite, (c) revisar el par de rodilla, que es de diseño del robot (no se modifica `cad/`).

### Fase 2, segundo intento (opción 1 del dueño: recompensa de giro más exigente, desde fase2_a)

**Cambio:** `track_angular_velocity` pasa de peso 2 y std 0,71 rad/s a peso 3 y std 0,25 (`env_cfgs.py`). **Motivo medido:** con el valor por defecto un error de giro de 0,22 rad/s aún da el 91 % de la recompensa, y el giro quedó en 0,28 de 0,5 rad/s. Es una hipótesis: no se probó que sea la causa.

**Plan antes de ejecutar:** `fase2_d` (etapa 2, 3000 iteraciones, desde `fase2_a` model_4497), control; `fase2_e` (etapa 3, 4000 iteraciones), control y prueba final. Mismo criterio de la fase 2 que arriba. Los controles informan y no paran la corrida salvo colapso (menos del 50 % sin caer en avance).

**Lo que este cambio no aborda:** el 88,3 % sin caer en 5 min con empujones y la rodilla izquierda saturada el 6,35 % del tiempo. Si el giro se arregla y esos dos siguen fallando, la fase 2 seguirá sin cumplir y lo diré.

#### Segundo intento, control 1 (fase2_d, etapa 2, model_7496)

`results/training_movements_fase2_d_7496.json` (7,4 V, aleatorización completa, sin empujones): sin caer 1,000 en todo; cumplen adelante, lateral izquierda y quieto; **no cumplen atrás (error de giro 0,522 rad/s), lateral derecha (0,165) ni los dos giros (0,199 y 0,202)**. Peor que el primer intento en atrás y lateral (control 2 anterior: atrás 0,078). El giro medio sí mejoró (0,40 y -0,34 rad/s con orden de 0,5, frente a 0,36 y -0,31), pero el error de giro por paso sigue alto porque la velocidad de giro oscila durante la marcha: en atrás la media es 0,026 y la media del valor absoluto 0,522. Es decir, el criterio mide sobre todo la oscilación del paso, no el seguimiento medio. No se ha medido de dónde sale esa oscilación. Se lanza `fase2_e` (etapa 3) según el plan; el control es informativo.

#### Segundo intento: `fase2_e` parada y análisis del giro (GPU libre)

`fase2_e` (etapa 3) se paró por orden del dueño en la iteración ~8200 de 11 496 (último punto `model_8200.pt`, sin evaluar; no cuenta como resultado). Se cerró a la fuerza porque `train.exe` tenía procesos hijos y no respondía a un cierre normal.

**Medidas** (`results/training_yaw_oscillation_fase2.json`, `scripts/measure_yaw_oscillation.py`, 64 robots, 12 s, 7,4 V, sin empujones):
- La oscilación de la velocidad de giro es el ritmo del paso: frecuencias dominantes 2,5 y 1,25 Hz (ciclo de 0,8 s). Quieto no oscila (0,002).
- Adelante y atrás: casi todo el error por paso es esa oscilación (`fase2_a` adelante: 0,060 por paso, 0,019 promediado a 0,8 s).
- **El giro de `fase2_c` falla por giro medio corto, no por oscilación:** error por paso 0,220, promediado 0,217; giro medio 0,283 de 0,5 rad/s. Corrige la lectura anterior de este documento (control 1 del segundo intento), que atribuía el criterio sobre todo a la oscilación: eso vale para adelante y atrás, no para el giro de `fase2_c`.
- `fase2_d` (recompensa de giro más exigente) **duplicó la oscilación** (RMS adelante 0,11 a 0,28 rad/s; atrás 0,14 a 1,05) y subió el giro medio (0,405), pero el error por paso no baja del criterio.
- **El entorno duro no explica el giro corto:** evaluada con aleatorización de etapa 1 o de etapa 3, `fase2_a` da 0,388 y 0,389 rad/s, y `fase2_c` 0,282 y 0,281. Es lo que aprendió la política.
- `fase2_c` gira poco a todas las órdenes, no solo a la máxima (orden 0,2: 0,066; 0,3: 0,135; 0,4: 0,205; 0,5: 0,281 rad/s).
- **No medido:** por qué la política entrenada con etapas 2 y 3 aprende a girar menos que `fase2_a`. Candidatas sin probar: los empujones durante el entrenamiento, el peso de la penalización de oscilación angular o la combinación de ambos.

#### Medidas previas al tercer intento (sobre fase2_c, model_11495)

**Recompensa doble** (propuesta del dueño tras revisar b0dcf8e): se vuelve a `track_angular_velocity` por defecto (peso 2, std 0,71 rad/s) y se añade `track_angular_velocity_fine` (peso 1, std 0,25). **El cambio a std 0,25 con peso 3 solo empeoró atrás (0,078 a 0,522 rad/s) y el lateral derecho**, sin llevar el giro por paso al criterio. Ambos términos incluyen la velocidad angular en x e y (balanceo del tronco), como la función de mjlab: con std 0,25 eso penaliza mucho el balanceo. Es una lectura del código, no está medida como causa.

**Par por tensión** (`results/training_torque_fase2_c_{6.0,7.4,8.4}V.json`, 300 s, empujones, 64 robots, bloqueo de su propia tensión): la rodilla izquierda supera el 90 % del bloqueo el **5,7 % del tiempo a 6 V** (derecha 2,5 %), el 1,4 % a 7,4 V y nada a 8,4 V. El 6,35 % medido antes comparaba a todos con el bloqueo a 6 V y sobrestimaba. El criterio de saturación (menos del 5 %) falla solo a 6 V. Valores de simulación; el HD-1910 no está confirmado.

**Caídas en 5 minutos** (`results/training_gait_fase2_c_5min_*.json`, 256 robots, 0,25 m/s): sin empujones a 7,4 V, **100 % sin caer**; con empujones a 7,4 V, **91,8 %**; con empujones a 6 V, **85,2 %**. Las caídas vienen de los empujones, y la tensión baja las empeora. La prueba de la guía (empujones y tensión de 6 a 8,4 V) dio 88,3 %, entre esos dos casos.

Cambios de herramientas: `check_training_setup.py` usa ahora la cota exacta del centro de masas por entorno (el límite fijo de 13,5 mm fallaba al azar entre 12,1 y 13,6 mm); `evaluate_movements.py` imprime el giro medio y la media de su valor absoluto; la nota de `measure_actuator_torque.py` indica contra qué bloqueo compara.

#### Tercer intento, control 1 (fase2_d_doble, etapa 2, model_7496, 3000 iteraciones desde fase2_a model_4497)

`results/training_movements_fase2_d_doble_7496.json` (7,4 V, aleatorización completa de la etapa 3 de evaluación, sin empujones): **cumplen los siete movimientos**, sin caer 1,000 en todos. Error de giro por paso: adelante 0,070, atrás 0,073, lateral izquierda 0,030, lateral derecha 0,034, giro izquierda 0,125 (media 0,424), giro derecha 0,114 (media -0,421), quieto 0,001. Error lineal máximo 0,070 m/s (laterales). La recompensa doble corrige lo que empeoró con std 0,25 solo: atrás vuelve de 0,522 a 0,073. Es solo el control de la etapa 2 de entrenamiento; falta la etapa 3 (`fase2_e_doble`, 4000 iteraciones), el control final y la prueba de la guía. No se ha medido todavía con empujones ni a 6 V.

#### Tercer intento, resultado final (fase2_e_doble, etapa 3, model_11495, 4000 iteraciones desde fase2_d_doble model_7496): NO CUMPLE

**Criterio** (el de la fase 2): cada movimiento por separado con 256 robots, 20 s: >=95 % sin caer, error <0,08 m/s o <0,15 rad/s, quieto se queda quieto; prueba de la guía: 0,25 m/s, 5 min, >=95 % sin caer con aleatorización completa, y ninguna articulación saturada (>90 % del bloqueo) más del 5 % del tiempo.

**Movimientos** (`results/training_movements_fase2_e_doble_11495.json`, 7,4 V, sin empujones): sin caer 1,000 en los siete. Error de giro por paso: adelante 0,067, atrás 0,086, lateral izquierda 0,027, lateral derecha 0,033, giro izquierda 0,135 (media 0,385), **giro derecha 0,158 (media -0,350), no cumple por 0,008 rad/s**, quieto 0,002. Error lineal máximo 0,076 m/s (lateral izquierda). Los demás cumplen.

**Prueba de la guía** (`training_gait_fase2_e_doble_prueba_guia.json`, empujones, 6 a 8,4 V, 5 min, 256 robots): **82,4 % sin caer, no cumple** (criterio 95 %).

**Caídas separadas** (`training_gait_fase2_e_doble_5min_*.json`): sin empujones a 7,4 V 100 %; con empujones a 7,4 V 83,2 %; con empujones a 6 V 82,8 %. Las caídas vienen de los empujones. Peor que fase2_c (91,8 % y 85,2 %).

**Par** (`training_torque_fase2_e_doble_{6.0,7.4,8.4}V.json`, 300 s, empujones, 64 robots, bloqueo de su propia tensión): la rodilla izquierda supera el 90 % del bloqueo el **13,2 % del tiempo a 6 V** (derecha 10,1 %), **4,9 % a 7,4 V** (derecha 3,4 %) y 1,8 % a 8,4 V (derecha 1,6 %; tobillo derecho 1,2 %). **No cumple a 6 V**; a 7,4 y 8,4 V queda por debajo del 5 %, la izquierda a 7,4 V por muy poco. El par máximo pasa un poco del bloqueo en las tres tensiones (0,762 frente a 0,739 N·m a 6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V). Peor que fase2_c a 6 V (5,7 % y 2,5 %).

**Resultado:** la fase 2 no se aprueba. Fallan la prueba de la guía (82,4 %), la saturación de la rodilla a 6 V, y el giro a la derecha por 0,008 rad/s. La recompensa de giro doble resolvió el seguimiento de giro en la etapa 2 (fase2_d_doble cumplía los siete movimientos), pero la etapa 3 con empujones de 0,20 m/s empeoró las caídas y la saturación respecto a fase2_c. No hay ONNX, vídeos ni PR a main. La fase 3 (agacharse) no se empieza.

**No comprobado:** la causa de que la etapa 3 empeore las caídas (hipótesis sin medir: empujones de entrenamiento demasiado fuertes para la rodilla a baja tensión); si fase2_d_doble pasa la prueba de la guía (no se midió con empujones ni a 6 V); parámetros reales del HD-1910 (todo se reentrena tras medirlos); el reloj de marcha lo tendría que generar el firmware en la Radxa a 50 Hz. Con orden cero y acción cero (`stand`) caen 7 de 64 robots con el modelo anterior.

**Siguiente paso a decidir por el dueño:** (a) medir `fase2_d_doble` con la prueba de la guía y el par, para ver si la etapa 2 basta; (b) repetir la etapa 3 con empujones de 0,16 m/s; (c) pedir el criterio de saturación solo a 7,4 V, donde cumple, si el robot no va a funcionar a 6 V. Ninguno se lanza sin su decisión.

#### Medidas con los empujones de la guía y a 6,6 V (fase2_d_doble model_7496 y fase2_e_doble model_11495)

Tras la decisión del 4 de noviembre (tensión mínima de marcha 6,6 V, empujones de la guía de ±0,12 m/s), se repitió todo con la opción nueva `--push 0.12` de `measure_gait.py` y `measure_actuator_torque.py`. **La etapa 3 entrenaba y se medía con empujones de ±0,20 m/s, más fuertes que los de la guía; por eso el 82 % anterior no era la prueba de la guía.** Resultados en `results/training_*_push012_*.json` y `training_movements_*_6.6V.json`.

| | fase2_d_doble (etapa 2) | fase2_e_doble (etapa 3) |
|---|---|---|
| Prueba de la guía (5 min, 6 a 8,4 V, 256 robots) | 99,6 % sin caer | 100 % |
| 5 min a 6,6 V | 99,2 % | 100 % |
| 5 min a 7,4 V | 100 % | 100 % |
| Rodilla izq./der. sobre el 90 % del bloqueo a 6,6 V | **8,2 % / 7,5 %** | **8,4 % / 6,5 %** (tobillo der. 2,2 %) |
| Ídem a 7,4 V | 4,9 % / 3,8 % | 3,7 % / 2,8 % |
| Ídem a 8,4 V | 2,2 % / 1,8 % | 1,8 % / 1,8 % |
| Siete movimientos a 6,6 V, sin empujones | **cumplen los siete** (giro izq. 0,137, giro der. 0,121 rad/s) | cumplen seis; **giro der. 0,163, no cumple** (giro izq. 0,149) |

**Conclusión: ninguno cumple todo.** Los dos pasan la prueba de caídas con holgura. Los dos fallan la saturación de la rodilla a 6,6 V (más del 5 %); a 7,4 y 8,4 V cumplen. fase2_e_doble además falla el giro a la derecha. El par máximo supera el bloqueo en las tres tensiones (0,838 frente a 0,813 N·m a 6,6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V) en los dos modelos; el bloqueo a 8,4 V es extrapolado. Candidato más cercano a cerrar la fase 2: fase2_d_doble, que solo falla la rodilla a 6,6 V. No se ha reentrenado nada; la decisión es del dueño.

No comprobado: si una corrida corta más con penalización de par en la rodilla baja la saturación a 6,6 V sin perder el giro; los siete movimientos con empujones; el HD-1910 real. Los movimientos se midieron sin empujones, como en los controles anteriores.

#### Cuarto intento: `fase2_f_rodilla` (castigo de par en rodillas y tobillos), resultado final: CUMPLE

**Criterio escrito antes de lanzar (plan del dueño tras 2f36307):** desde `fase2_d_doble/model_7496`, ~1500 iteraciones con etapa 2 de aleatorización y empujones de ±0,12 m/s. Nuevo castigo `servo_torque_excess` = -peso · Σ max(0, |τ| − 0,8·τ_bloqueo(vin del entorno))², solo en rodillas y cabeceo de tobillos (no castiga el par normal, solo el exceso). Peso **−317**, elegido para que valga ≈7,5 % de la recompensa por paso al principio (0,00184 de castigo crudo con `model_7496`; 5 %, 7,5 % y 10 % darían 211, 317 y 422). `check_training_setup.py` comprueba que vale 0 hasta el 80 % del bloqueo a 6, 7,4 y 8,4 V, crece por encima y vale 0 de pie con acción cero. Para cumplir: los 7 movimientos a 6,6 V, la prueba de la guía (≥95 %), 5 min a 6,6 V, rodilla sobre el 90 % del bloqueo por debajo del 5 % a 6,6, 7,4 y 8,4 V, y la flexión de rodilla en apoyo comparada con fase2_d_doble.

**Corrida:** `fase2_f_rodilla`, 1500 iteraciones (7496 → 8995), `model_8995.pt`, sin caídas durante el entrenamiento (episodio de 1000 pasos); el castigo bajó de −0,277 a −0,030 por episodio. Opciones nuevas: `DANNY_PUSH_M_S` (fuerza del empujón en entrenamiento) y métrica `mean_knee_angle_stance_deg` en `measure_gait.py`.

| Medida (empujones ±0,12 m/s) | fase2_d_doble (7496) | fase2_f_rodilla (8995) |
|---|---|---|
| Prueba de la guía (5 min, 6 a 8,4 V) | 99,6 % sin caer | 100 % |
| 5 min a 6,6 V | 99,2 % | 100 % |
| Rodilla izq./der. sobre el 90 % del bloqueo a 6,6 V | 8,2 % / 7,5 % | **0,01 % / 0,04 %** |
| Ídem a 7,4 V | 4,9 % / 3,8 % | 0,01 % / 0,02 % |
| Ídem a 8,4 V | 2,2 % / 1,8 % | 0,01 % / 0,01 % |
| Peor articulación (cualquiera) a 6,6 / 7,4 / 8,4 V | rodilla izq. 8,2 / 4,9 / 2,2 % | cadera roll der. 0,67 / 0,27 / 0,04 % |
| Flexión de rodilla en apoyo a 6,6 V | 60,6° | 61,3° (sin enderezar la pierna) |
| Siete movimientos a 6,6 V, sin empujones | cumplen los siete | **cumplen los siete**: giro izq. 0,124, giro der. 0,135 rad/s (límite 0,15); adelante/atrás 0,036; lateral 0,067; quieto 0,0006 |

Resultados en `results/training_*_fase2_f_rodilla_*.json`. **Fase 2 cumple todos los criterios con `fase2_f_rodilla/model_8995.pt`.** Copia del punto de control en `D:\Claude\puntos_de_control\fase2_f_rodilla_model_8995.pt` (fuera del repositorio).

**No comprobado / límites:** el par máximo sigue 3 % por encima del bloqueo lineal en las tres tensiones (0,838 frente a 0,813 N·m a 6,6 V; 0,940 frente a 0,912 a 7,4 V; 1,067 frente a 1,036 a 8,4 V), igual que en fase2_d_doble: es un pico puntual del modelo BAM, no saturación sostenida; el bloqueo a 8,4 V es extrapolado. Los movimientos se miden sin empujones. La caída de tensión bajo carga (≤0,15 V) no entra en el castigo. El reloj de marcha exige firmware a 50 Hz en la Radxa. Los parámetros reales del HD-1910 no están confirmados: todo se reentrena tras medir el servo real. Error de importación circular en `measure_actuator_torque.py` (importar `actuator` antes que `evaluation`) corregido con el orden de imports; las tres pruebas de par se repitieron.

**Entregables de la fase 2 (con `fase2_f_rodilla/model_8995.pt`):**
- Política en ONNX: `policies/danny_fase2_movimiento_completo_2026-11-04.onnx` (837 KB). Entrada `obs` de 80 valores (`base_ang_vel`, `projected_gravity`, `joint_pos`, `joint_vel`, `actions`, `command`, `gait_clock`), salida `actions` de 23 valores. Lo exporta el propio ejecutor de mjlab al guardar. `scripts/verify_policy_onnx.py` lo compara con el punto de control en 500 pasos y 16 robots: diferencia máxima 1,5e-6 (límite 1e-4), en `results/training_onnx_fase2_f_rodilla.json`. Aviso: los metadatos del ONNX dicen rigidez 1, amortiguamiento 0 y escala de acción 1,0; son valores de relleno del modelo BAM (la escala 1,0 sí es la del entrenamiento, `ACTION_SCALE`), no ganancias de un PD real. El programa de la Radxa debe usar la lista `joint_names` y `default_joint_pos` de los metadatos y el reloj de marcha a 50 Hz.
- Vídeos de 8 s a 6,6 V, sin empujones, 0,4 MB cada uno: `results/videos/fase2_f_rodilla_{adelante,atras,lateral_izquierda,giro_izquierda,quieto}.mp4` (`scripts/record_walk_video.py`).
- No comprobado: el ONNX no se ha ejecutado en la Radxa ni a 50 Hz reales; los vídeos son de 320×240.

### Fase 3, agacharse: criterio escrito antes de entrenar (4 de noviembre)

**Lo que permite el cuerpo (cálculo estático, no entrenamiento).** Con los pies planos y el tronco recto, el centro de masas queda sobre los pies con cadera -21° y rodilla 47° (la pose de pie). Al agacharse, el tobillo debe valer -(cadera + rodilla) y su límite es -45°, así que la rodilla llega a unos 82° y **el centro de masas baja unos 22 mm** (de 0,212 a 0,190 m, un 10 %). El par estático de la rodilla pasa de 0,13 a 0,24 N·m, el 29 % del bloqueo a 6,6 V (0,813 N·m): el par no es el límite, el rango del tobillo sí. Bajar más exige levantar talones o inclinar el tronco, y eso no se intenta. **Si el dueño quiere un agachado más profundo, es un tema de diseño (rango del tobillo).** Cálculo con `danny_training.xml` y la masa nominal; la masa y el centro de masas aleatorizados lo mueven unos milímetros.

**Orden.** Una orden `height` en metros, de -0,02 a 0 m respecto a la altura de pie, que se suma a la altura objetivo del tronco; se resortea cada 4 a 8 s y el 20 % de los entornos recibe 0. La política la ve como una observación más (82 entradas en lugar de 80).

**Corrida.** `fase3_agacharse`, hasta 3 000 iteraciones desde `fase2_f_rodilla/model_8995.pt`, ampliando la primera capa con columnas a cero para las entradas nuevas (así empieza caminando como en la fase 2), etapa 2 de aleatorización y empujones de ±0,12 m/s. Se mantiene el castigo de par (-317). Se para, sin alargar, si el 50 % o más de los robots cae hacia delante.

**Aprueba** (256 robots, 20 s, 6,6 V fijos, sin empujones, salvo la última prueba) si:
1. Con orden de altura de 0, -0,01 y -0,02 m y orden de velocidad cero: 95 % o más sin caer, error medio de altura de 4 mm o menos y velocidad media de 0,03 m/s o menos.
2. Caminando a 0,25 m/s con altura -0,02 m: 95 % o más sin caer y error de velocidad de 0,08 m/s o menos.
3. Los siete movimientos de la fase 2 con altura 0 siguen cumpliéndose (no se pierde lo aprendido).
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, en el agachado quieto a -0,02 m y en la marcha agachada.
5. Prueba de la guía (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, altura aleatoria): 95 % o más sin caer.
Si no cumple, se para y se anota; no se amplía la corrida.

### Fase 3, agacharse: criterio escrito antes de entrenar (4 de noviembre)

**Cambio de criterio, 4 de noviembre, a propuesta del dueño** (la primera versión suponía el tronco recto y limitaba la bajada a 20 mm; con el tronco inclinado como una persona, el cuerpo baja más). Orden de altura de **0 a -0,05 m medida en la pelvis** (`pelvis_bottom`), con el tronco inclinado hacia delante en proporción a la orden: 20° a -0,05 m. La tarea `squat` de `analysis/danny_motion.py` baja la pelvis 45 mm respecto a su pose de pie (60 mm respecto al origen del modelo, 0,241 m) con el tronco a 0,35 rad. Ojo: respecto a la pose de pie de la política (pelvis a 0,228 m), -0,05 m son 5 mm más que lo comprobado allí, por eso se comprueba también a mano (siguiente párrafo).

**Cálculo estático previo** (`scripts/check_crouch_pose.py`, `results/training_crouch_static.json`; pies planos, centro de masas sobre las suelas, peso repartido por igual, 6,6 V, tronco inclinado 20° por cada 50 mm):

| Bajada de la pelvis | Inclinación | Cadera | Rodilla | Tobillo (límite -45°) | Par de rodilla (% del bloqueo a 6,6 V) |
|---|---|---|---|---|---|
| 0 | 0° | -20,9° | 47,4° | -26,6° | 0,13 N·m (16 %) |
| -0,025 m | 10° | -53,0° | 79,6° | -36,6° | 0,20 N·m (24 %) |
| -0,045 m | 18° | -75,7° | 96,9° | -39,2° | 0,21 N·m (26 %) |
| **-0,050 m** | **20°** | -81,3° | 100,5° | **-39,3°** | **0,21 N·m (26 %)** |
| -0,060 m | 24° | -92,4° | 107,2° | -38,8° | 0,21 N·m (26 %) |

A -0,05 m el tobillo queda dentro de su rango (5,7° de margen), la cadera (límite -100°) y la rodilla (130°) también, y el centro de masas cae sobre los pies (error de 0 mm, por construcción). Es estático: el par dinámico y la aleatorización de masa y centro de masas no están incluidos.

**Orden.** Una orden `height` en metros, de -0,05 a 0 respecto a la pelvis de pie (0,228 m), que la política ve como una observación más (81 entradas en lugar de 80; el crítico, 96 en lugar de 95). Se resortea cada 4 a 8 s y el 20 % de los entornos recibe 0. Con esa orden:
- la recompensa de postura se desplaza: la referencia de cadera, rodilla y tobillo es la de la tabla de arriba (interpolada);
- `upright` pide un cabeceo del tronco de 0,35 rad por cada 0,05 m (7 rad/m), en vez de tronco recto;
- recompensa de altura de la pelvis (peso 3, std 6 mm), **solo con orden de velocidad cero**: al caminar la pelvis baja sola unos 12 mm (medido con `fase2_f_rodilla`: -0,0127 m de pie, -0,0252 m a 0,25 m/s), así que caminando se exige la bajada relativa, no la absoluta.

**Corrida.** `fase3_agacharse`, hasta 3 000 iteraciones desde `fase2_f_rodilla/model_8995.pt`, ampliando la primera capa del actor y del crítico con una columna a cero (y su normalizador) para la entrada nueva, de modo que empieza caminando como en la fase 2; etapa 2 de aleatorización y empujones de ±0,12 m/s; se mantiene el castigo de par (-317). Se para, sin alargar, si el 50 % o más de los robots cae hacia delante.

**Aprueba** (256 robots, 20 s, 6,6 V fijos, sin empujones, salvo la última prueba) si:
1. Quieto (orden de velocidad cero) con altura 0, -0,025 y -0,05 m: 95 % o más sin caer, error medio de altura de la pelvis de 5 mm o menos y velocidad media de 0,03 m/s o menos.
2. Caminando a 0,25 m/s con altura -0,025 m: 95 % o más sin caer, error de velocidad de 0,08 m/s o menos y pelvis al menos 15 mm más baja que caminando con altura 0.
3. Los siete movimientos de la fase 2 con altura 0 siguen cumpliéndose (no se pierde lo aprendido).
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, en el agachado quieto a -0,05 m y en la marcha agachada a -0,025 m.
5. Prueba de la guía (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, altura aleatoria entre 0 y -0,05): 95 % o más sin caer.
Si no cumple, se para y se anota; no se amplía la corrida. Ejecución automática de la fase 3, sin esperar permisos, a petición del usuario.

### Fase 3, resultado: NO CUMPLE (`fase3_agacharse`, 3 000 iteraciones, 8995 → 11994, `model_11994.pt`)

Corrida del 4 al 5 de noviembre, de 23:05 a 02:00, etapa 2 de aleatorización y empujones de ±0,12 m/s, sin colapso (episodios de 987 a 1000 pasos). Se midió automáticamente con el criterio de arriba, a 6,6 V (`results/training_*fase3_agacharse*.json`). Punto de partida: `fase2_f_rodilla/model_8995.pt` con la primera capa ampliada (`scripts/extend_checkpoint_inputs.py`); con altura 0 y ese punto de partida, antes de entrenar, caminaba como en la fase 2 (error de velocidad 0,0375 m/s, 100 % sin caer).

| Prueba del criterio | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m | 100 % sin caer; error de altura 2,2 / 2,1 / 4,6 mm (límite 5 mm); velocidad media 0,002 / 0,002 / 0,001 m/s | **Cumple** |
| 2. Marcha a 0,25 m/s a -0,025 m | 100 % sin caer, error de velocidad 0,043 m/s; pero la pelvis baja **4,6 mm** respecto a caminar con altura 0 (0,2065 → 0,2019 m); se pedían 15 mm | **No cumple** |
| 3. Siete movimientos a altura 0 | cumplen 3 de 7: lateral izq. 0,104, lateral der. 0,137 y quieto. Fallan **adelante** (giro 0,159 rad/s), **atrás** (0,336), **giro izq.** (0,321) y **giro der.** (0,406) | **No cumple** |
| 4. Rodilla y tobillo sobre el 90 % del bloqueo, 6,6 V | 0 % en rodillas y tobillos, quieto a -0,05 m y marchando a -0,025 m (con empujones de 0,12). Peor articulación: codo izq. 1,5 % (quieto), cadera izq. 3,8 % (marcha) | **Cumple** |
| 5. Prueba de la guía (5 min, 6 a 8,4 V, ±0,12, altura aleatoria) | 100 % sin caer, error de velocidad 0,044 m/s | **Cumple** |

**Qué se ve.** Quieto, la política baja la pelvis hasta 50 mm con el tronco a 12,6° (se pedían 20°), con el par muy por debajo del límite. Lo que falla es la marcha: (a) caminando agachado no sigue la orden de altura, solo 4,6 mm de los 25 pedidos (la recompensa de altura solo actúa parado y la de postura, que sí actúa caminando, no bastó); (b) con altura 0 la marcha cambió respecto a la fase 2: la pelvis camina 8 mm más baja (0,2065 frente a 0,2149 m) y el tronco inclinado 9,8° (4,2° antes), y el seguimiento del giro se rompió (error de giro 0,16 a 0,41 rad/s caminando). La velocidad lineal se mantiene (0,038 m/s). La causa de (b) no está medida: la hipótesis es que la inclinación y la postura agachada, premiadas quieto, se filtraron a la marcha con altura 0 porque la política comparte pesos; no se probó.

**Lo que no se comprobó:** otras ponderaciones, un punto de control intermedio (solo se midió el último), ni si la regresión del giro aparece ya en las primeras iteraciones. Según el plan, no se alarga la corrida ni se reentrena sin decisión del dueño.

**Siguientes pasos posibles (decide el dueño):** (1) política de agacharse solo quieto, separada de la marcha: no estropea la fase 2 y lo que funciona (criterios 1, 4 y 5) ya cumple; (2) reentrenar con recompensa de altura también caminando (referida a la marcha de altura 0) y con las órdenes de giro de la fase 2 vigiladas en cada control; (3) mezclar con datos de la fase 2 (congelar la marcha con altura 0). La fase 2 sigue aprobada con `fase2_f_rodilla/model_8995.pt` (PR 1 a main); la fase 4 (levantarse) no depende de la 3.

### Fase 3 (segundo intento): agacharse solo quieto, criterio escrito antes de entrenar (5 de noviembre)

**Decisión del dueño, 5 de noviembre:** agacharse quieto como política aparte; caminar agachado queda **descartado por ahora**. Hipótesis del dueño para la regresión de `fase3_agacharse` (sin medir): el 80 % de los entornos caminaba con orden de altura y referencias de cuclillas estáticas, y solo el 20 % practicaba la marcha de la fase 2.

**Corrida.** `fase3_quieto`, hasta 1 500 iteraciones desde `fase2_f_rodilla/model_8995.pt` con la primera capa ampliada (el mismo punto de partida `fase3_inicio`), orden de velocidad siempre cero (`DANNY_STAND_ONLY=1`: todos los entornos reciben orden cero) y orden de altura de 0 a -0,05 m (20 % de ceros), etapa 2 de aleatorización, empujones de ±0,12 m/s, castigo de par -317 y las mismas recompensas de altura, postura e inclinación. Se para, sin alargar, si el 50 % o más de los robots cae. Los puntos de control se guardan cada 50 iteraciones.

**Aprueba** (256 robots, 6,6 V fijos, sin empujones salvo donde se indica) si:
1. Quieto con altura 0, -0,025 y -0,05 m, 20 s: 95 % o más sin caer, error medio de altura de la pelvis de 5 mm o menos y velocidad media de 0,03 m/s o menos.
4. Rodilla y tobillo por encima del 90 % del bloqueo menos del 5 % del tiempo a 6,6 V, quieto a -0,05 m con empujones de ±0,12 m/s.
5. Prueba de la guía quieta (5 min, 6 a 8,4 V, empujones de ±0,12 m/s, orden de velocidad cero, altura aleatoria entre 0 y -0,05): 95 % o más sin caer.
6. **Cambio de política (nueva).** De pie con la política de la fase 2 y orden cero, a los 3 s se cambia a esta con orden cero y altura 0, -0,025 o -0,05 m: 95 % o más sin caer en los 5 s siguientes y velocidad media de 0,03 m/s o menos entre los 2 y los 5 s tras el cambio (`scripts/measure_policy_switch.py`; la política de la fase 2 es `fase3_inicio`, su equivalente con la entrada de altura a cero). El error de altura se anota, no decide.
Las pruebas 2 y 3 de la fase 3 anterior (marcha agachada y siete movimientos con la política nueva) ya no aplican: la marcha es la de la fase 2, sin cambios.
Si cumple, la fase 3 se anota aprobada (agacharse quieto). Si no, se para y se anota; no se amplía la corrida. Después se escribe el criterio de la fase 4 antes de entrenar. Ejecución automática, a petición del dueño.

### Fase 3 (segundo intento), resultado: NO CUMPLE (`fase3_quieto`, 1 500 iteraciones, 8995 → 10494, `model_10494.pt`)

Corrida del 5 de noviembre (empezó a las 10:40), `DANNY_STAND_ONLY=1`, etapa 2, empujones de ±0,12 m/s, sin colapso (episodios de 1 000 pasos, ninguna caída en el entrenamiento). Medido automáticamente a 6,6 V (`results/training_*fase3_quieto*.json`).

| Prueba | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m, 20 s | 100 % sin caer, velocidad 0,0002–0,0003 m/s; **error de altura de la pelvis 31,8 / 7,5 / 16,6 mm** (límite 5 mm). La pelvis queda en 0,196 / 0,196 / 0,195 m para las tres órdenes (objetivos 0,228 / 0,203 / 0,178) | **No cumple** |
| 4. Rodilla y tobillo sobre el 90 % del bloqueo, quieto a -0,05 m con empujones | 0 % en rodillas y tobillos (rodilla izq. 0,48 N·m máx., der. 0,54; tobillo 0,38 / 0,34) | Cumple |
| 5. Prueba de la guía quieta (5 min, 6 a 8,4 V, ±0,12, altura aleatoria) | 100 % sin caer, velocidad 0,0007 m/s | Cumple |
| 6. Cambio de la política de la fase 2 a esta (orden 0, 5 s tras el cambio) | 100 % sin caer en las tres alturas, velocidad 0,0005 m/s; error de altura 34 / 9,9 / 14,1 mm (se anota, no decide) | Cumple |

**Qué se ve.** La política se queda quieta y estable, pero **no obedece la orden de altura**: la pelvis queda en unos 0,195 m (33 mm por debajo de la de pie) y el tronco casi recto (0 a 0,9°, se pedía hasta 20°) para cualquier orden. Es un punto medio fijo, no un seguimiento. Los puntos de control intermedios (`results/training_gait_fase3_quieto_diag_*.json`, quieto a 0 y -0,05 m) dan lo mismo: pelvis a 0,198 / 0,197 (iteración 9000, solo 5 iteraciones después de reanudar), 0,205 / 0,205 (9250), 0,199 / 0,199 (9500) y 0,198 / 0,197 m (10000). Es decir, **la orden de altura nunca se siguió**, y la pelvis ya estaba baja a las 5 iteraciones, cuando la política de la fase 2 de pie la tenía en 0,2275 m (`results/training_switch_fase2_sin_cambio.json`, error 0,5 mm a altura 0).

**Hallazgo aparte: `right_hip_yaw` está al tope de par el 99,97 % del tiempo** (0,8135 N·m, el bloqueo a 6,6 V) en el agachado quieto a -0,05 m con empujones; el izquierdo, 0 %. El criterio 4 solo cubre rodilla y tobillo, así que no decide, pero esa pose no es válida para un robot real (el servo estaría bloqueado todo el tiempo). La causa no se midió; la hipótesis es que la cadera derecha empuja contra un límite de articulación.

**Lo que no se comprobó:** por qué la pelvis cae tan rápido en las primeras iteraciones (hipótesis sin medir: sin el estado del optimizador, las primeras actualizaciones son grandes y el castigo de la rodilla y la postura de cuclillas atraen a un mínimo local fijo); si con más iteraciones, otro peso de la recompensa de altura o empezando con la entrada de altura con pesos no nulos sí la seguiría; la pose de `right_hip_yaw`. Según el plan, no se alarga la corrida ni se reentrena sin decisión del dueño.

**Contraste con `fase3_agacharse`**, que sí seguía la orden de altura quieto (error 2,2 / 2,1 / 4,6 mm) pero con las órdenes de velocidad de la fase 2 mezcladas. Lo que cambia entre ambas corridas es solo `DANNY_STAND_ONLY=1`, 1 500 en lugar de 3 000 iteraciones y que no hay caminata; no se sabe cuál de las tres causa la diferencia.

**Estado de la fase 3:** sigue **no aprobada**. Opciones, decide el dueño: (a) usar `fase3_agacharse/model_11994.pt` solo para agacharse quieto: cumple 1, 4 y 5 quieto, pero cambia la marcha de la fase 2, así que habría que usarlo solo parado y cambiar de política con la prueba 6 (no medida con ese punto de control); (b) repetir `fase3_quieto` con una corrección que se decida después de medir la causa; (c) dejar el agacharse para después de medir el servo real. La fase 4 (levantarse) no depende de la 3 y no se ha empezado: su criterio no está escrito.

### Fase 3, aprobada (agacharse quieto) con `fase3_agacharse/model_11994.pt`, usado solo parado (5 de noviembre)

**Decisión del dueño, 5 de noviembre:** opción (a). El modelo se usa solo parado y se cambia a él desde la marcha de la fase 2 con orden de velocidad cero. Caminar agachado queda **descartado por ahora**, y las pruebas 2 y 3 de la fase 3 original no aplican. La marcha sigue siendo `fase2_f_rodilla/model_8995.pt`.

| Prueba (modelo `model_11994`, 6,6 V) | Resultado | |
|---|---|---|
| 1. Quieto a 0 / -0,025 / -0,05 m, 20 s (ya medido) | 100 % sin caer, error de altura 2,2 / 2,1 / 4,6 mm, velocidad ≤ 0,002 m/s | Cumple |
| 4. Saturación (>90 % del bloqueo), quieto a -0,05 m con empujones ±0,12, 120 s, **todas las articulaciones** | Máximo: codo izquierdo 1,54 %; rodillas, tobillos y las dos caderas en giro, 0 % (`results/training_torque_fase3_agacharse_quieto_h-0.05_6.6V.json`, 64 robots) | Cumple |
| 5. Prueba de la guía **quieta** (5 min, 6 a 8,4 V, ±0,12, orden de velocidad cero, altura aleatoria; `training_gait_fase3_agacharse_quieto_push012_prueba_guia.json`) | 99,6 % sin caer (255 de 256), velocidad media 0,0046 m/s | Cumple |
| 6. Cambio de política: de pie con la de la fase 2, a los 3 s a esta, 5 s después (`training_switch_fase3_agacharse.json`) | 100 % sin caer en las tres alturas; velocidad 0,0027 / 0,0025 / 0,0015 m/s (límite 0,03); error de altura 0,9 / 1,4 / 5,3 mm | Cumple |

La prueba 5 de antes se había medido caminando a 0,25 m/s; esta es la versión quieta, medida hoy. El ONNX está en `policies/danny_fase3_agacharse_quieto_2026-11-05.onnx` (81 entradas → 23 acciones; la entrada nueva, la última, es la bajada de la pelvis en m, de 0 a -0,05). Coincide con el punto de control (`results/training_onnx_fase3_agacharse.json`, diferencia máxima 2,4e-6). Vídeo: `results/videos/fase3_agacharse_agacharse.mp4` (0,62 MB; la orden baja de 0 a -0,05 m en 4 s, se mantiene 3 s y vuelve a 0). Se ve con `scripts/record_crouch_video.py`.

**Lo que muestra el vídeo y los números no ocultan:** en el vídeo (un solo robot) la pelvis llega a 0,188 m con la orden de -0,05 m (objetivo 0,178), 10 mm por encima; la media de 256 robots es de 4,6 mm de error. Al subir tarda más de 1 s en llegar a 0,224 m (de pie, 0,228). **No se ha comprobado:** la transición contraria (de esta política a la marcha de la fase 2), ni el cambio con empujones, ni la rodilla y el tobillo de la política con 256 robots en la prueba de saturación (se midieron 64). El tronco se inclina 12,6° a -0,05 m, no los 20° del criterio original; el criterio de aceptación no lo exigía.

### Diagnóstico de la caída de la pelvis al empezar (sin entrenar, 5 de noviembre)

Pregunta: `fase3_quieto` tenía la pelvis a 0,198 m en `model_9000`, 5 iteraciones después de empezar. ¿Por qué? Se compararon el normalizador y los pesos y se midieron los puntos tempranos de `fase3_agacharse` (`results/training_normalizador_fase3.json`, `results/training_gait_fase3_agacharse_diag_*.json`).

| Punto de control | Pelvis con orden 0 / -0,05 m | Sin caer |
|---|---|---|
| `fase3_inicio` (política de la fase 2, sin entrenar) | 0,2275 m / sin seguir la orden | 100 % |
| `fase3_quieto` 9000 (a las 5 iteraciones) | 0,198 / 0,197 | 98 % / 98 % |
| `fase3_quieto` 10000 | 0,198 / 0,197 | 100 % |
| `fase3_agacharse` 9000 (a las 5 iteraciones) | 0,200 / 0,200 | 54 % / 61 % |
| `fase3_agacharse` 9050 | 0,202 / 0,202 | 100 % |
| `fase3_agacharse` 9500 | 0,2266 / 0,2265 (error 1,4 / 48,5 mm) | 100 % |

- **El normalizador no es la causa.** `_mean`, `_std` y `count` casi no cambian en las primeras 5 iteraciones (diferencia máxima 0,09 % de la desviación; `count` pasa de 884,7 a 885,3 millones y sigue creciendo). La entrada nueva (altura) empieza con media 0 y desviación 1 y se queda ahí.
- **El salto ocurre igual en las dos corridas** (`fase3_quieto` y `fase3_agacharse`), así que no lo causa `DANNY_STAND_ONLY`. En ambas, en 5 iteraciones los pesos cambian mucho: primera capa 3,5 %, última 9 % (norma relativa).
- **No se encontró la causa del salto.** Hipótesis, sin probar: al ampliar la capa se descarta el estado del optimizador, y las primeras actualizaciones de un Adam nuevo son grandes. Para comprobarlo habría que entrenar 5 iteraciones con otro ritmo de aprendizaje o conservando el optimizador; no se hizo.
- **Por qué una se recupera y otra no, tampoco se encontró.** `fase3_agacharse` vuelve a 0,2266 m a las 500 iteraciones y acaba siguiendo la orden; `fase3_quieto` se queda en 0,198 m. Datos de acompañamiento: al final, la columna de la altura en la primera capa pesa 29,7 en `agacharse` y 8,4 en `quieto`, y la desviación media de la acción es 0,51 y 0,29. Podría ser que sin marcha no haya señal suficiente, pero no se probó.

**Qué hacer en las fases 4 y 5 (precaución, no regla probada):** si una corrida arranca de un punto de control con una capa ampliada o un optimizador nuevo, medir la política a las 5 y a las 50 iteraciones (altura de la pelvis, caídas) antes de dejarla correr, y parar si la política de partida se rompe. Eso cuesta unos minutos.

**Regla nueva del criterio de saturación (5 de noviembre, a petición del dueño):** desde ahora el criterio de «más del 90 % del bloqueo menos del 5 % del tiempo» cubre **todas** las articulaciones, no solo rodillas y tobillos. Motivo: en `fase3_quieto`, `right_hip_yaw` estuvo al tope el 99,97 % del tiempo y el criterio de entonces no lo veía.

**Estado de la fase 3:** **aprobada (agacharse quieto)** con `fase3_agacharse/model_11994.pt`, usado solo parado; caminar agachado, descartado por ahora.

### Fase 4, levantarse: criterio escrito antes de implementar y entrenar (5 de noviembre)

**Qué se sabe del cuerpo (no se ha calculado para esta fase; viene de la etapa 3).** Una búsqueda sin realimentación (`results/get_up_prone.json`) logra ponerse de pie boca abajo en simulación, con el modelo nominal (altura del centro de masas 0,217 m, inclinación 0,3°), pero **solo en el caso nominal: falla con 6 V, con suelo de 0,5 y de 1,2, y con ±10 % de masa** (altura final 0,04 m, tumbado). El gesto necesita la cadera a -100° (su límite) y el tobillo a -45°. Boca arriba, `roll_over.json` da la vuelta a boca abajo primero. Por tanto, **no está demostrado que se pueda con 6,6 V ni con aleatorización**; la fase puede acabar diciendo que no, y entonces el cambio de diseño (cadera, par) lo decide el dueño. Lo que no hay que hacer: copiar la secuencia de la búsqueda como objetivo (la guía dice que no lo es).

**Tarea nueva** (hay que implementarla; no existe en `training/`): `Mjlab-GetUp-Danny`. El robot empieza tumbado (boca abajo o boca arriba según la corrida) con guiñada aleatoria, articulaciones con ruido pequeño y brazos en la pose de reposo; la observación es la del entrenamiento actual sin el reloj de marcha ni la orden de altura. Recompensas: altura de la pelvis y del tronco, tronco vertical, pies apoyados, mantenerse de pie, y castigos de par excedido (como en la fase 2), cambio de acción, límites de articulación y choques. Sin imitar las secuencias de referencia. Aleatorización de la etapa 3 completa en la prueba; en el entrenamiento, etapa 2 al principio.

**Se levanta** un intento si, antes de 20 s, durante 3 s seguidos se cumple a la vez: `pelvis_bottom` a 0,20 m o más (de pie vale 0,228 m), tronco inclinado 15° o menos, los dos pies en el suelo, velocidad media de 0,05 m/s o menos y ningún eslabón distinto de los pies tocando el suelo. Los 0,20 m son una estimación (88 % de la altura de pie); si se cambian, se dice.

**Corridas** (nombres claros, se conserva el punto de control que aprueba):
1. `fase4_boca_abajo`: desde cero, hasta 5 000 iteraciones, 4096 entornos, empezando siempre boca abajo.
2. `fase4_boca_arriba`: solo si la 1 aprueba; arranca del punto de control aprobado de la 1, con la mitad de los entornos boca abajo y la mitad boca arriba (para no olvidar), hasta 5 000 iteraciones más (10 000 en total, como en el plan).
Si una corrida arranca de un punto de control con capa ampliada u optimizador nuevo, se mide a las 5 y a las 50 iteraciones (altura, caídas) antes de dejarla correr (precaución del diagnóstico anterior).

**Comprobaciones durante la corrida 1** (informan; paran la corrida si fallan, sin alargarla): a las 1 000 iteraciones, al menos el 5 % de los entornos tiene la pelvis a 0,15 m o más **con el tronco a 45° o menos** (`Metrics/getup_up_fraction`; cambio del 5 de noviembre: la comprobación original, pelvis a 0,10 m en algún momento, la pasaba el 99 % tumbado, y la de 0,15 m sin tronco la pasaba una postura invertida); a las 3 000, al menos el 5 % se levanta del todo según la definición de arriba. Si no, se para y se explica por qué (puede significar rediseñar la cadera, y eso lo decide el dueño).

**Aprueba la corrida 1** (boca abajo; 256 robots, 20 s, sin empujones, aleatorización completa de la etapa 3): se levanta el **80 % o más** de los intentos (8 de cada 10)
 a. a 6,6 V fijos (decide);
 b. con la tensión aleatoria de 6 a 8,4 V (decide);
 c. a 7,4 V fijos (informa: dice si un fallo en a es de par).
**Aprueba la corrida 2** (la fase 4 entera): lo mismo, boca abajo y boca arriba por separado, a 6,6 V y con la tensión aleatoria. Una sola política para los dos casos.

**Saturación (todas las articulaciones, regla de hoy):** mientras se mantiene de pie los 3 s, ninguna articulación pasa del 90 % del bloqueo más del 5 % del tiempo. **Mientras se levanta, solo se mide** (máximo y fracción por articulación): subir del suelo exige picos altos y no hay medida previa para fijar un umbral; tras la primera corrida se propone uno con números y se anota el cambio.

**Añadido 1 (dueño, 5 de noviembre): prueba de cambio a la marcha.** Tras levantarse (3 s estable), se cambia a la política de la fase 2 (`fase2_f_rodilla/model_8995.pt`, la de `fase3_inicio` con la altura ignorada) con orden cero: **95 % o más sin caer en los 5 s siguientes, a 6,6 V** (decide; es parte de aprobar las corridas 1 y 2). Se mide con un script como `measure_policy_switch.py`, partiendo de los entornos que se levantaron. Además se mide **el cambio contrario de la fase 3** (de agacharse con `fase3_agacharse/model_11994.pt` a la marcha de la fase 2, orden cero, 6,6 V, tres alturas), que no se había comprobado; informa, no decide la fase 3, que ya está aprobada.

**Añadido 2 (dueño, 5 de noviembre): saturación por tiempo seguido.** Mientras se levanta, además del máximo y la fracción por articulación, se anota **el tiempo seguido más largo al tope de par** (más del 90 % del bloqueo) por articulación. Un pico corto es normal; un servo bloqueado varios segundos (como `right_hip_yaw` en `fase3_quieto`) no vale en el robot real. Con esos números se propone el umbral tras la corrida 1 y se anota el cambio.

**Si no cumple:** se para, se anota lo visto (qué caso falla, a qué tensión, qué articulación satura) y no se alarga la corrida. La fase 5 (sentarse y tumbarse) no empieza si la 4 no aprueba, y la fase 6 no empieza.

**Cambio contrario de la fase 3, medido el 5 de noviembre** (`results/training_switch_fase3_agacharse_a_marcha.json`; `fase3_agacharse/model_11994.pt` 3 s y luego la marcha de la fase 2 con orden cero, 256 robots, 6,6 V, sin empujones, `measure_policy_switch.py --reverse`): **100 % sin caer** en las tres alturas (0, -0,025 y -0,05 m) y velocidad media de 0,0008, 0,0012 y 0,0018 m/s entre los 2 y 5 s tras el cambio. El error de altura de ese JSON no vale aquí: la marcha de la fase 2 ignora la orden de altura. No medido: el cambio con empujones.

**Implementación (5 de noviembre):** `training/src/danny_training/getup.py` (tarea `Mjlab-GetUp-Danny`, registrada en `tasks.py`; observación del actor de 78 entradas: la de la fase 3 sin reloj de marcha ni altura), `training/scripts/measure_getup.py` (mide el criterio, el cambio a la marcha y la saturación con tiempo seguido; `evaluation.py` admite ahora otra tarea). Inicio: boca abajo = tronco girado 1,5 rad, pelvis a 0,045 m, articulaciones en cero ±0,05 rad, guiñada aleatoria (como `analysis/search_get_up.py`); boca arriba = -1,5 rad. Recompensas (primer intento): altura de la pelvis (peso 4, lineal), tronco vertical (2), pies apoyados con la pelvis sobre 0,15 m (1), quieto con la pelvis sobre 0,20 m (2), pose de pie (-1), límites de articulación (-1), cambio de acción (-0,1), choques entre eslabones (-1) y castigo de par sobre el 80 % del bloqueo en todas las articulaciones **solo con la pelvis sobre 0,20 m** (-317). Sin terminación por caída, episodios de 20 s, sin empujones, etapa 2 de aleatorización. Los pesos son un primer intento sin medir.

**Cómo se verá:** `training/scripts/diagnose_getup_posture.py` mide la postura y graba tres vídeos por punto de control en `results/videos/<nombre>_robot<i>.mp4` (desde `training/`: `uv run python scripts/diagnose_getup_posture.py --checkpoint <modelo.pt> --name <nombre> --vin 6.6`). Un vídeo por caso con la política que sí se levante (`record_getup_video.py`) queda por escribir.

### Fase 4, corrida 1 (`fase4_boca_abajo`): NO CUMPLE, parada en la iteración 3137 (5 de noviembre)

Corrida de `Mjlab-GetUp-Danny` desde cero, 4096 entornos, etapa 2, sin empujones, parada a mano tras la comprobación de las 3 000 iteraciones (punto de control guardado: `puntos_de_control/fase4_boca_abajo_model_3100.pt`, fuera del repositorio).

- **Comprobación de las 3 000 iteraciones: falla.** Robots de pie según el criterio (`Metrics/getup_stood_fraction`): **0 %** durante toda la corrida; el mínimo pedido era el 5 %. Pasan de 0,10 m en algún momento el 99 %, así que esa comprobación de las 1 000 iteraciones no discrimina (ya era el 100 % en el punto 50).
- **Estancada desde la iteración 702:** recompensa de altura de pelvis 2,63 de 4 (unos 0,15 m de altura media, estimado) hasta la 3 137 (2,65); recompensa media 62 a 64.
- **Medida a 6,6 V** (`results/training_getup_fase4_boca_abajo_it3000.json`, 256 robots, 20 s, aleatorización completa): se levanta el **0 %**. **`left_hip_pitch` está al tope de par (más del 90 % del bloqueo) el 98,6 % del tiempo, 19,7 s seguidos, con picos de 1,03 veces el bloqueo**; ninguna otra articulación pasa del 0,2 %. El cambio a la marcha no se pudo medir: nadie se levantó (esa parte del script sigue sin ejercitarse).
- **Qué hace la política (medido sin entrenar, 5 de noviembre; punto de control 3 100, 256 robots, 6,6 V, últimos 10 s de 20; `results/training_getup_postura_fase4_boca_abajo_it3000.json`, `scripts/diagnose_getup_posture.py`):** se pone **boca abajo del revés, sobre la cabeza**: pelvis a 0,152 m (percentil 10 y 90: 0,152) con el tronco a **147°** de la vertical (0° es de pie), y solo tocan el suelo la cabeza, el antebrazo derecho y el pie izquierdo (100 % del tiempo, en los 256 robots, casi idénticos entre sí). El vídeo (`results/videos/fase4_boca_abajo_it3000_robot0.mp4` a `robot2.mp4`) lo muestra: piernas en el aire. Es un óptimo local de la recompensa de altura de pelvis (lineal, no mira hacia dónde apunta el tronco): da 4 x 0,152 / 0,228 = 2,67, lo que se vio (2,63 a 2,65). Con la comprobación nueva (pelvis 0,15 m y tronco 45°) este punto da **0 %**.
- **`left_hip_pitch`: la política empuja contra el tope, no es falta de par.** Ángulo pedido **-308°** (la acción no está acotada: -5,4 rad), ángulo real **-100,3°** fijo, con límite blando -94° y duro unos -100°; par al 100 % del bloqueo todo el tiempo. Ese par no es una medida de lo que pide el gesto: el servo empuja contra el tope mecánico. Solo la izquierda porque la postura es asimétrica: se apoya en el pie izquierdo y el antebrazo derecho; la cadera derecha queda a -27° (límite -94°) con el 20 % del bloqueo. Otras articulaciones junto a su límite durante >90 % del tiempo: `left_knee` (inferior), `left_ankle_roll`, `left_shoulder_roll`, `right_hip_yaw`, `right_ankle_pitch`, `right_hip_roll` (96 %), pero con par bajo (menos del 22 % del bloqueo). Por qué no lo castiga nada: el castigo de par (-317) solo actúa con la pelvis sobre 0,20 m y la pelvis nunca llega; `dof_pos_limits` mira el ángulo real, que ya está en el tope, no el pedido.
- **Causa: óptimo local de la recompensa** (hipótesis (a) confirmada); la hipótesis (b), par insuficiente, **no es lo que se ve** en esta corrida (no hay falta de par: hay un tope empujado). No descarta que el par a 6,6 V sea insuficiente más adelante; no se ha podido medir (siguiente punto).
- **El par que exige el gesto de `get_up_prone.json` no se puede medir con ese gesto:** repetido hoy en el modelo actual (`scripts/measure_getup_gesture_torque.py`, `results/training_getup_gesto_par.json`), **no se pone de pie ni sin límite de par** (altura del centro de masa 0,042 m, tronco a 1,60 rad, a 6,0, 6,6, 7,4 y 8,4 V e ilimitado) y la hoja `get_up_prone.json` se calculó el 31 de octubre; el modelo cambió el 2 de noviembre (servo BAM, auditoría). El «nominal estable» y el «falló a 6 V» de ese archivo son anteriores a ese cambio y no valen. Con el par ilimitado, el control de posición pedía hasta 3,7 N·m en las caderas de balanceo, lo que indica que el gesto, tal cual, no es una referencia útil en el modelo nuevo. Para medir el par hay que **volver a buscar un gesto** con `analysis/search_get_up.py` (no se toca `analysis/`; ejecutarlo sobrescribe `results/get_up_prone.json`) o sacar la referencia de otra parte; no se hizo sin decisión.
- **No se alarga la corrida ni se cambian las recompensas sin decisión del dueño** (regla del criterio). La fase 5 y la 6 no empiezan. Por lo medido, el siguiente intento no es un cambio de cadera sino de aprendizaje: una recompensa que no se cumpla boca abajo del revés (p. ej. altura de pelvis multiplicada por lo vertical que está el tronco, o castigo por tocar el suelo con la cabeza), acotar la acción de la cadera, y la ayuda vertical con currículo (o empezar desde posturas intermedias) que propuso el dueño. A decidir.

### Fase 4, corrida 2 (`fase4_boca_abajo_ayuda`): criterio escrito antes de entrenar (5 de noviembre)

Autorizada por el dueño tras el diagnóstico de la corrida 1; el dueño pidió que se siga sola y solo se pare si se bloquea. Misma tarea, desde cero, 4096 entornos, 5 000 iteraciones, etapa 2, sin empujones, boca abajo, con **tres cambios** respecto a la corrida 1 (`getup.py`):
1. **Altura de pelvis por (1 + cos del tronco) / 2**: del revés cobra casi nada (147° da 0,08).
2. **Objetivo de posición acotado al rango blando** de cada articulación (la corrida 1 pidió -308° en una cadera de -94°).
3. **Ayuda vertical** (`DANNY_GETUP_ASSIST=1`, solo al entrenar; medidas y vídeos la dejan apagada): fuerza hacia arriba en `base_link` del 80 % del peso (1,30 kg, estimado de la etapa 3) hasta la iteración 500, lineal hasta 0 en la 2 500 y 0 después.
Pesos de recompensa, resto de la tarea y definición de «se levanta» sin cambios.

**Comprobaciones** (paran la corrida si fallan, sin alargarla; `Metrics/getup_up_fraction` = pelvis 0,15 m y tronco 45° o menos, `Metrics/getup_stood_fraction` = definición de pie): it. 1 000 (ayuda 60 %): `up` 5 % o más; it. 2 000 (ayuda 20 %): `up` 20 % o más; it. 3 000 (ayuda ya apagada desde la 2 500): `stood` 5 % o más. Se mira además la postura con `diagnose_getup_posture.py` en cada parada, para no repetir el óptimo local.
**Aprueba** (`measure_getup.py`, ayuda apagada, 256 robots, 20 s, 6,6 V decide, tensión aleatoria decide, 7,4 V informa): 80 % o más de los intentos se levanta, más el cambio a la marcha de la fase 2 con 95 % o más sin caer en 5 s. La saturación mientras se levanta se mide y, con esos números, se propone el umbral.
**Si falla:** se para, se documenta (con postura y vídeo) y, con la autorización de seguir solo, se prueba un cambio de diseño distinto y diagnosticado en una corrida nueva con su criterio escrito antes, hasta un máximo de **3 corridas** en total en esta pasada; después se para.

**Corrida 2, medida intermedia en la iteración 3 000** (`results/training_getup_fase4_boca_abajo_ayuda_it3000.json` y `training_getup_postura_fase4_boca_abajo_ayuda_it3000.json`; 256 robots, 6,6 V, ayuda apagada). Las tres comprobaciones pasan (it. 1 000: `up` 97 %; it. 2 000: 96 %; it. 3 000: `stood` 94 % en el entrenamiento). **Se levanta el 100 %**, a los 3,9 s de mediana (3 s de ventana incluidos); la racha más larga al tope de par mientras se levanta es de **0,28 s** (`left_hip_pitch`), un pico corto. Pero **no aprueba**:
- **Cambio a la marcha: 75,4 % sin caer** (se pide 95 %); la pelvis tras el cambio queda a 0,173 m de media (de pie, 0,228). La marcha de la fase 2 no puede tomar el mando.
- **Saturación de pie: `right_hip_roll` al 94 %** del tiempo sobre el 90 % del bloqueo (se pide 5 % como máximo).
- **Causa, medida**: de pie, pelvis a 0,225 m y tronco a 2,4°, pero con una postura de topes: cintura a -54° (límite), cuello, codos y hombros en sus límites, piernas abiertas con los dos `hip_roll` e `ankle_roll` contra su límite y par medio del 58 % en `hip_roll`. La recompensa de pose de pie (peso -1 sobre el **error medio** de 23 articulaciones) pesaba casi nada.

**Corrida 3 (`fase4_pose`), criterio escrito antes de entrenar:** cambio único de diseño: `pose_when_up` pasa a la **suma** de errores cuadráticos con peso **-2** (unas 150 veces más fuerte que antes para un error parecido). 1 500 iteraciones desde el final de la corrida 2 (`model_4999.pt`), sin ayuda, etapa 2, 4096 entornos (`/d/Claude/fase4_pose.sh`). Última de las 3 corridas de esta pasada. Mismo criterio de aprobación que la corrida 1 (80 % se levanta a 6,6 V y con tensión aleatoria; 7,4 V informa; cambio a la marcha 95 %; saturación de pie 5 %), y además que ninguna articulación de pie pase del 90 % del bloqueo más del 5 % del tiempo. Comprobación a las 500 iteraciones: `stood` 50 % o más (si cae por debajo, la pose penaliza demasiado el levantarse; se para y se documenta).

**Corrida 2, final (`fase4_boca_abajo_ayuda/model_4999.pt`, copia en `puntos_de_control/`), medida el 6 de noviembre** (`results/training_getup_fase4_boca_abajo_ayuda_final_6.6V.json`, `_aleatoria.json`, `_7.4V.json`; 256 robots boca abajo, 20 s, ayuda apagada, sin empujones): **se levanta el 100 %** en las tres tensiones (mediana 3,8 a 3,9 s con los 3 s de ventana). **Cambio a la marcha de la fase 2 con orden cero, sin caer en 5 s: 91,0 % a 6,6 V (se pide 95 %: NO CUMPLE), 94,5 % con tensión aleatoria (NO CUMPLE), 96,1 % a 7,4 V (informa).** Saturación de pie: la peor articulación pasa del 90 % del bloqueo el 0,7 % del tiempo a 6,6 V (2,7 % aleatoria): cumple el 5 %.
**Saturación mientras se levanta (números para el umbral que se pidió):** tiempo seguido más largo al tope de par, el máximo entre robots: `left_hip_pitch` 0,24 s (6,6 V), 0,26 s (aleatoria), 0,20 s (7,4 V); el resto de articulaciones, 0,14 s o menos (`waist_yaw` 0,20 s en la aleatoria); percentil 95 de la peor: 0,18 s. Pico de par 1,03 veces el bloqueo a 6,6 V y 1,29 con la tensión aleatoria (a tensiones altas). **Umbral propuesto: ninguna articulación más de 0,5 s seguidos al tope de par mientras se levanta** (el doble del máximo medido, 0,26 s; un servo bloqueado varios segundos, como `right_hip_yaw` en `fase3_quieto`, queda muy por encima). Se aplica desde la corrida 3.
Falla por poco solo el cambio a la marcha; la postura de pie de la corrida 2 es la de topes diagnosticada arriba (a las 3 000), y su cambio a la marcha mejoró de 75,4 % a 91 % con más entrenamiento. Se lanza la corrida 3 (`fase4_pose`), con el criterio escrito antes (arriba) y el umbral de 0,5 s añadido a su aprobación.

### Fase 4, corrida 3 (`fase4_pose`): NO CUMPLE, parada en la iteración 5 600 (6 de noviembre)

1 500 iteraciones previstas desde `fase4_boca_abajo_ayuda/model_4999.pt`, sin ayuda, con `pose_when_up` como suma de errores cuadráticos y peso -2. **La comprobación de las 500 iteraciones falla**: robots de pie 0 % (se pedía 50 % o más). Historia (`/d/Claude/fase4_pose.log`): de pie 45,7 % en la it. 5 010, 22 % en la 5 025 y **0 % desde la 5 050**; recompensa media de -21 a +69 (la de la corrida 2 era 140). Parada en la 5 600 (copia `puntos_de_control/fase4_pose_model_5600.pt`).
**Medida** (`results/training_getup_fase4_pose_parada.json`, `training_getup_postura_fase4_pose_parada.json`, 6,6 V, 256 robots): se levanta el **0 %**. Postura: pelvis a **0,141 m** (percentil 10 a 90: 0,128 a 0,145), tronco casi vertical (6° de media), apoyada en el pie derecho, el pie y la espinilla izquierdos y a veces el muslo y el antebrazo izquierdos.
**Causa, medida:** el castigo de pose solo actúa con la pelvis sobre 0,15 m, y con peso -2 sobre la suma cuesta más que lo que se gana por subir. La política se queda justo debajo de 0,15 m (0,141) donde no se castiga: un precipicio en la recompensa. Una pose con peso alto exige quitar el umbral o suavizarlo, y subirlo poco a poco; no se probó.
**Código:** `getup.py` vuelve a la pose de la corrida 2 (media, peso -1); el cambio de la corrida 3 queda solo aquí descrito.

### Fase 4, estado al cerrar la pasada autónoma (6 de noviembre): NO APROBADA

Tres corridas hechas (máximo de la pasada). La mejor es la **corrida 2** (`fase4_boca_abajo_ayuda/model_4999.pt`): se levanta el 100 % boca abajo a 6,6 V, con tensión aleatoria y a 7,4 V; saturación de pie y racha al tope mientras se levanta cumplen; falla **solo el cambio a la marcha: 91,0 % a 6,6 V y 94,5 % aleatoria (se pide 95 %)**, 96,1 % a 7,4 V. Su postura de pie es de topes (cintura a -54°, brazos y piernas contra sus límites a las 3 000 iteraciones; no se midió en la 4 999). **No comprobado:** la postura final de la corrida 2; boca arriba; el cambio a la marcha con empujones; el robot real. La fase 5 y la 6 no empiezan. Siguientes pasos posibles, para decidir: (1) quitar el umbral de 0,15 m del castigo de pose y subirlo poco a poco desde el modelo 4 999; (2) añadir como parte de la recompensa de pie el estado que necesita la marcha (pose de la fase 2) con peso suave y una corrida corta; (3) aceptar el 91 % como paso intermedio y medir antes la causa de las caídas tras el cambio (inclinación, qué articulación).

### Fase 4, corrida 4 (`fase4_pose_sin_umbral`), criterio escrito antes de entrenar (6 de noviembre)

Decisión del usuario: quitar el umbral de 0,15 m del castigo de pose. Cambio único: `pose_when_up` sin umbral de altura (siempre activo, así no hay escalón en la recompensa), **media** de errores cuadráticos con peso **-1,5** (corrida 2: -1 con umbral; corrida 3: suma con peso -2, es decir unas 150 veces más fuerte). 1 500 iteraciones desde `fase4_boca_abajo_ayuda/model_4999.pt`, sin ayuda, etapa 2, 4096 entornos (`/d/Claude/fase4_pose_sin_umbral.sh`). Aprobación: la de la corrida 3 (80 % se levanta a 6,6 V y aleatoria; cambio a la marcha 95 %; saturación de pie 5 %; ninguna articulación más de 0,5 s seguidas al tope mientras se levanta). Comprobación a las 500 iteraciones: `stood` 80 % o más (la corrida 2 daba 100 %); si baja del 50 % se para y se documenta. Es la cuarta corrida de la fase: el tope de 3 de la pasada autónoma se levantó por decisión del usuario.

### Fase 4, corrida 4 (`fase4_pose_sin_umbral`): NO CUMPLE el cambio a la marcha (6 de noviembre)

1 500 iteraciones (hasta la 6 498, `puntos_de_control/fase4_pose_sin_umbral_model_6498.pt`), sin colapso: a las 500 iteraciones `stood` 96,5 % (criterio 80 %). Medido con `measure_getup.py`, 256 robots boca abajo, 20 s (`results/training_getup_fase4_pose_sin_umbral_{6.6V,7.4V,aleatoria}.json`):

| Tensión | Se levanta | Mediana | Cambio a la marcha sin caer en 5 s (se pide 95 %) | Racha más larga al tope mientras se levanta |
|---|---|---|---|---|
| 6,6 V | 100 % | 3,9 s | 91,0 % (no cumple) | 0,22 s (`left_hip_pitch`) |
| Aleatoria 6 a 8,4 V | 100 % | 3,88 s | 93,75 % (no cumple) | 0,20 s |
| 7,4 V (informa) | 100 % | 3,82 s | 94,5 % | 0,20 s |

Saturación de pie: 1,3 % como mucho (`left_hip_pitch`, 6,6 V). Cumple levantarse, saturación y el umbral de 0,5 s; falla solo el cambio a la marcha, igual que la corrida 2 (91,0 / 94,5 / 96,1 %): quitar el umbral y penalizar más la pose **no lo mejoró** (a 6,6 V igual, en las otras dos algo peor). Conclusión con números: la hipótesis «la postura de topes explica las caídas tras el cambio» no se sostiene; castigar la pose no es la palanca. **No comprobado:** la postura final de esta corrida (no se corrió `diagnose_getup_posture.py`) ni por qué caen los robots (inclinación, articulación, instante). Siguiente paso propuesto, sin entrenar: medir esas caídas (tiempo hasta caer, dirección, articulación) en el modelo 6 498 y en el 4 999. Código: `getup.py` queda con la pose sin umbral y peso -1,5 (corrida 4); `fase4_boca_abajo_ayuda/model_4999.pt` sigue siendo la referencia más simple. Fase 4 no aprobada.

### Fase 4: por qué caen tras el cambio a la marcha (medido sin entrenar, 6 de noviembre)

`training/scripts/diagnose_getup_switch_falls.py` repite el cambio (orden cero) con 512 robots a 6,6 V y guarda `results/training_getup_caidas_<name>.json`. Resultados:

| Modelo | Caen | Sin caer | Mediana hasta caer | Dirección |
|---|---|---|---|---|
| Corrida 2 (`model_4999`) | 32 de 512 | 93,75 % | 1,12 s | 29 hacia delante, 3 a la izquierda |
| Corrida 4 (`model_6498`) | 32 de 512 | 93,75 % | 1,14 s | 30 hacia delante, 2 a la izquierda |
| Corrida 4, 6 s de pie antes del cambio | 35 de 512 | 93,2 % | 1,08 s | 27 delante, 8 izquierda |
| Corrida 4, 10 s de pie antes del cambio | 31 de 512 | 94,0 % | 1,12 s | 24 delante, 7 izquierda |

(Con 256 robots el mismo modelo daba 91,0 %: el ruido de muestreo es de unos ±2 puntos; con 512, 93,75 %.)

- **Es un fallo de la política de marcha con orden cero, no de la política de levantarse ni de la postura de topes:** las corridas 2 y 4 dan lo mismo, y esperar 6 o 10 s de pie antes del cambio no lo arregla (quedaría en 93 a 94 %). Los que caen lo hacen siempre hacia delante (más del 80 %), 0,8 a 1,8 s después del cambio; los que no caen se quedan a un máximo de 10,5° de inclinación (percentil 95).
- **Diferencias al cambiar (caen menos sus medias en desviaciones típicas):** pelvis más alta (+0,6 a +0,9 σ), algo más de velocidad (+0,8 σ), rodillas y caderas algo menos flexionadas (rodilla derecha 4,0° frente a 6,0°; cadera 11,5° frente a 13,5°). Diferencias de 1 a 2 σ con ángulos de 2°; indican que los robots que caen se quedan un poco más estirados, no una postura distinta.
- **Hipótesis (no comprobada):** la política de marcha de la fase 2 arranca de una postura ligeramente más flexionada que la de pie de la política de levantarse y, con orden cero desde la postura estirada, se inclina hacia delante. **No comprobado:** que flexionar más las rodillas al cambiar lo arregle; el cambio con la orden de marcha real; el cambio desde la política de agacharse de la fase 3.
- **Siguiente paso sin entrenar:** probar en el cambio un periodo de transición (p. ej. mezclar las dos acciones en 0,5 s) o una pose de pie objetivo más flexionada en la política de levantarse, y medir con este mismo script. La fase 4 sigue sin aprobarse (93,75 % frente a 95 %, mejor que la corrida 2 original solo por la muestra).

**Transición suave entre políticas (probada, empeora):** `--blend-s` mezcla linealmente la acción de la política de levantarse con la de marcha durante N segundos tras el cambio (modelo `fase4_pose_sin_umbral/model_6498.pt`, 512 robots, `results/training_getup_caidas_*mezcla*.json`). Cambio seco: 93,75 % sin caer. Mezcla de 0,5 s: **3,5 %** sin caer a 6,6 V y 2,9 % con tensión aleatoria; mezcla de 1,0 s: **0 %** (los 512 caen, 94 % hacia delante, mediana 0,88 s). La mezcla de acciones de dos políticas distintas da objetivos que ninguna de las dos espera y es peor que el cambio seco; descartada. Sigue sin explicar por qué caen el 6 % con el cambio seco. **Siguiente paso posible:** pedir en el entrenamiento de levantarse una pose de pie más flexionada (rodillas y caderas como las de los robots que no caen: rodilla ~6°, cadera ~13,5°) o ajustar el reloj/observación extra del cambio; no se hace sin decisión.

### Fase 4: prueba de la postura con rampa de objetivos (revisión externa, 6 de noviembre)

**Revisión recibida:** en el cambio todos los robots (caigan o no) están en una postura de topes, lejos de la pose de pie de la fase 2. **Medido en la corrida 4** (`diagnose_getup_posture.py`, `results/training_getup_postura_fase4_pose_sin_umbral_6.6V.json`, 256 robots, 6,6 V, últimos 10 s; pelvis 0,229 m, tronco 1,9°): al límite más del 95 % del tiempo `right_knee` 5,4° (fase 2: 47,4°), `right_hip_pitch` 14,5° (fase 2: -20,9°), `left_hip_pitch` 12,0°, `waist_yaw` -52,9° (fase 2: 0°; límite -54°), `right_hip_yaw` 13,8° (límite 12,7°), codos y `right_shoulder_roll` en el tope, `neck_pitch` -25,2° (límite -26,5°). Objetivo y posición difieren hasta 25° (p. ej. `right_hip_roll`: objetivo 12,5°, real -13,7°): la política se sostiene empujando contra los topes. El castigo de pose (media, peso -1,5) es demasiado débil para evitarlo: un error de 40° en una articulación de 23 cuesta solo 0,02. La postura queda confirmada como muy distinta de la de la fase 2.

**Prueba sin entrenar** (`diagnose_getup_switch_falls.py --ramp-s N`): al cumplirse los 3 s de pie, los objetivos de los servos van en línea recta hasta la pose de pie de la fase 2 en N s (acción = (objetivo - defecto)/escala) y después manda la política de marcha; 512 robots, 6,6 V, modelo `fase4_pose_sin_umbral/model_6498.pt`. Se pedía 98 % o más sin caer:

| Rampa | Parte de | Sin caer | Mediana hasta caer | Dirección |
|---|---|---|---|---|
| 1,5 s | posición medida | 0 % | 0,70 s | 511 hacia delante |
| 4 s | posición medida | 0 % | 0,72 s | 512 hacia delante |
| 8 s | posición medida | 0 % | 0,72 s | 512 hacia delante |
| 1,5 s | último objetivo de la política | 5,5 % | 1,42 s | 167 delante, 316 atrás |
| 4 s | último objetivo de la política | 0,2 % | 1,86 s | 236 delante, 274 atrás |

**Resultado: no se cumple (0 a 5,5 % frente a 98 %), y la prueba no confirma ni descarta la postura:** los robots caen **durante la rampa**, antes de dar el mando a la política de marcha (a los 0,7 s con 8 s de rampa; a mitad de camino partiendo del objetivo). Tardar más no cambia el momento de la caída partiendo de la posición medida: lo que tira al robot es quitar de golpe el empuje contra los topes. Ir en línea recta en articulaciones desde la postura de topes hasta la de la fase 2 no es una trayectoria estable (cintura -53° a 0°, cadera +12° a -21° y rodilla 5° a 47° a la vez). Por la regla fijada (si no da 98 %, la hipótesis cae) la causa «postura» **no queda confirmada con esta prueba**; la corrida 5 propuesta (recompensa positiva exp(-error²/σ²) respecto a la pose de pie, multiplicada por la altura por tronco, peso ~2; comprobación a las 500 iteraciones: `stood` 80 % o más y piernas y cintura a 10° o menos de la pose de pie, estimación del revisor) **no se lanza sin decisión**. Lo que sí es cierto: la postura de la corrida 4 es de topes y la de la fase 2 es muy distinta; que eso cause el 6 % de caídas sigue sin probarse.

### Fase 4, corrida 5 (`fase4_pose_positiva`), criterio escrito antes de entrenar (6 de noviembre)

Decisión del usuario. Cambio único de diseño: `pose_when_up` (castigo, corridas 2 a 4) se sustituye por `pose_stand`, una recompensa **positiva** exp(-suma de errores² / std²) respecto a la pose de pie de la fase 2, multiplicada por la recompensa de altura por tronco (`pelvis_height_progress`): sin escalón y solo cobra de pie y vertical. **Peso 2** (como `stand_still`), **std² = 1 rad²** (elección mía, sin medir con más valores: la postura de topes de la corrida 4 suma unos 2 rad² y cobra exp(-2) = 0,13; la pose de pie cobra 1). 1 500 iteraciones desde `fase4_pose_sin_umbral/model_6498.pt`, sin ayuda, etapa 2, 4096 entornos (`/d/Claude/fase4_pose_positiva.sh`). Como los pesos y el std son un primer intento, si la comprobación falla se para y se documenta (no se ajusta dentro de la misma corrida).

**Comprobación a las 500 iteraciones:** `stood` 80 % o más, y la postura de pie (`diagnose_getup_posture.py`) con las articulaciones de las piernas y la cintura a **10° o menos** de la pose de pie (estimación del revisor). **Aprobación, sin cambios:** 95 % o más sin caer tras el cambio a la marcha a 6,6 V y con tensión aleatoria (512 robots para reducir el ruido de ±2 puntos medido), 80 % o más se levanta, saturación de pie 5 %, ninguna articulación más de 0,5 s seguidas al tope mientras se levanta. La postura al cambiar se mide como dato informativo.

### Fase 4, corrida 5 (`fase4_pose_positiva`): NO CUMPLE la comprobación de postura, parada en la iteración 7 045 (6 de noviembre)

Parada a las 547 iteraciones de la corrida (punto de control medido: `model_7000.pt`, `results/training_getup_postura_fase4_pose_positiva_it500.json`, 256 robots, 6,6 V, últimos 10 s). **`stood` se mantiene (95,9 % a 96,1 %, criterio 80 %), pero la postura no se mueve:** pelvis 0,230 m, tronco 2,1°, y las mismas articulaciones pegadas al límite que en la corrida 4: rodillas 6,7° y 4,9° (pose de pie: 47,4°), caderas 12,9° y 13,5° (-20,9°), `waist_yaw` -52,9° (0°), `right_hip_yaw` 13,8° (límite 12,7°). Errores respecto a la pose de pie de piernas y cintura: de 9 a 53°, ninguno a 10° o menos (se pedía todas). La recompensa `pose_stand` queda en 0,024 por paso (de un máximo de 2): **el std² = 1 rad² que elegí es demasiado estrecho**; la suma de errores cuadráticos de la postura de topes es de unos 4 rad² (mi estimación de 2 estaba mal), así que exp(-4) = 0,02 y la pendiente donde está la política es casi nula. No es un fallo de la política sino de la escala que puse. Por la regla de la corrida («si falla, se para y se documenta, no se ajusta dentro de la misma corrida») queda parada. El código de `getup.py` conserva `pose_stand` con std 1. **No comprobado:** que un std mayor (p. ej. std² = 4 rad², que da exp(-1) = 0,37 en la postura de topes y pendiente útil), o reducir el error por articulación en vez de en suma, mueva la postura; ni el cambio a la marcha con este modelo (no se midió).

### Fase 4, corrida 6 (`fase4_pose_positiva_std4`), criterio escrito antes de entrenar (6 de noviembre)

Decisión del usuario. Cambio único respecto a la corrida 5: **std² = 4 rad²** en `pose_stand` (std 2 en el código; antes 1). La postura de topes (suma de errores de unos 4 rad²) cobra exp(-1) = 0,37 por la altura y tronco, y la pose de pie cobra 1, así que hay pendiente donde está la política. Peso 2 sin cambios. 1 500 iteraciones desde `fase4_pose_sin_umbral/model_6498.pt` (no desde la corrida 5), sin ayuda, etapa 2, 4096 entornos (`/d/Claude/fase4_pose_positiva_std4.sh`). Misma comprobación a las 500 iteraciones (`stood` 80 % o más; piernas y cintura a 10° o menos de la pose de pie, estimación del revisor) y misma aprobación que la corrida 5. Si falla, se para y se documenta; con ella son seis corridas en la fase 4 y el relevo debe decir si seguir con esta línea.

### Fase 4, corrida 6 (`fase4_pose_positiva_std4`): NO CUMPLE la comprobación de postura, parada en la iteración 7 075 (6 de noviembre) — causa encontrada

Con std² = 4 rad²: a las 500 iteraciones `stood` 93,2 % (criterio 80 %), pero la postura no se mueve (`results/training_getup_postura_fase4_pose_positiva_std4_it500.json`): rodillas 10,9° y 5,9° (pose de pie 47,4°), caderas 11,8° y 14,9° (-20,9°), `waist_yaw` -53,2° (0°), caderas y tobillos en roll al tope. **Corrección a lo que anoté al ver 0,70 en `pose_stand`:** ese valor no indicaba movimiento; es lo que da la misma postura de topes con el std nuevo (suma de errores ~4 rad², exp(-1) por el peso 2).

**Causa medida** (`training/scripts/measure_getup_raw_actions.py`, `model_7000.pt`, acción cruda media de los últimos 10 s, objetivo = defecto + acción, escala 1): la salida de la política está muy por fuera de los límites y el recorte a los límites (`soft_limit_clip`, factor 0,9) la corta, de modo que **no hay gradiente** para sacar esas articulaciones de su tope por mucho que la recompensa de pose lo pida. Objetivos sin recorte frente al límite: `right_elbow` 3 026° (límite -124°/-7°), `left_shoulder_roll` -1 113°, `left_elbow` 811°, `right_hip_yaw` 780° (límite 13°), `left_hip_roll` -643°, `left_shoulder_pitch` 532°, `right_shoulder_roll` 518°, `right_hip_pitch` 464° (límite 14°), `right_ankle_roll` 365°, `left_hip_pitch` 333°, `right_knee` -308° (límite 7°), `waist_yaw` -248° (límite 54°), `neck_pitch` -178°. Solo se salvan las articulaciones del medio del rango (`left_knee` 22°, tobillos de pitch, hombros en yaw, cuello en yaw). Es el mismo síntoma de la corrida 1 (`left_hip_pitch` pedido -308° con límite -100°), que el recorte no corrigió: solo ocultó el valor pedido. Esto explica por qué las corridas 3, 4, 5 y 6 no mueven la postura con ninguna recompensa de pose: la recompensa no puede mover una salida que está saturada del recorte.

**Siguiente paso propuesto (corrida 7, sin lanzar):** castigar la acción cruda fuera del rango (exceso sobre el límite recortado, en L1 para que el gradiente no se anule) y/o reiniciar la desviación típica de las acciones; con ello las articulaciones salen del tope y la recompensa de pose empieza a actuar. **No comprobado:** que alcance en 1 500 iteraciones (las acciones están a 4 a 50 rad del rango), el peso del castigo (sin medir), y que la postura resultante mejore el cambio a la marcha.

### Acciones crudas fuera de rango en otras políticas (medido sin entrenar, 6 de noviembre)

`measure_getup_raw_actions.py` (128 robots, 6,6 V, 20 s, últimos 10 s; objetivo = defecto + acción cruda, rango = `soft_limit_clip`, exceso L1 en rad sumado sobre articulaciones; `results/training_acciones_crudas_*.json`):

| Política | Exceso total | Articulaciones fuera de rango más de la mitad del tiempo |
|---|---|---|
| Levantarse, `fase4_pose_sin_umbral/model_6498` | **190,9 rad** | 16 de 23 (`right_elbow` 50,8 rad de exceso medio) |
| Fase 2 (`fase3_inicio/model_8995`, pesos de `fase2_f_rodilla` con una entrada extra) | 0,089 | ninguna (tobillos pitch fuera el 32 % y 24 % del tiempo, hasta 50°) |
| Fase 3 agacharse (`fase3_agacharse/model_11994`, aprobada) | 0,62 | ninguna (picos de hasta 233° en `left_elbow`) |
| Fase 3 quieto (`fase3_quieto/model_10494`, descartada) | 1,38 | `right_hip_yaw` (99,5 % del tiempo, 0,83 rad de exceso medio, hasta 88°), `left_ankle_pitch` 85 %, `right_ankle_pitch` 88 % |

Las políticas aprobadas de las fases 2 y 3 no tienen el problema; la de levantarse sí, y `fase3_quieto` (descartada) confirma lo que ya apuntaba `right_hip_yaw`. `fase2_f_rodilla_model_8995.pt` no carga en el entorno actual (80 entradas frente a 81).

### Fase 4, corrida 7 (`fase4_accion_rango`), criterio escrito antes de entrenar (6 de noviembre)

Decisión del usuario, con la propuesta de la revisión (con PPO el problema es que, si la media está muy fuera, toda la muestra se recorta al mismo valor y la ventaja no distingue ninguna acción). Cambios: nuevo castigo `action_out_of_range` = **L1 del exceso de la acción cruda sobre el rango recortado, sumado sobre todas las articulaciones**, y se mantiene `pose_stand` con std 2 (std² = 4) y peso 2. **Peso: -0,035.** Cálculo: la política de partida (`model_6498`) suma 190,9 rad de exceso; el objetivo era que al principio costara del orden de lo que cobra estar de pie (unos 6 a 8 por paso: `pelvis_height` 3,9 + `upright` 1,9 + `stand_still` 1,9 + `feet_down` 1,0 en el registro de la corrida 5); 7 / 190,9 = 0,0367, redondeado a 0,035, es decir -6,7 por paso al principio. 1 500 iteraciones desde `fase4_pose_sin_umbral/model_6498.pt`, sin ayuda, etapa 2, 4096 entornos (`/d/Claude/fase4_accion_rango.sh`).

**Comprobación a las 200 iteraciones:** el exceso total de la acción cruda (el registro `Episode_Reward/action_out_of_range` dividido por 0,035) debe haber bajado al menos a la mitad (de 6,7 a 3,35 o menos en el registro) y `stood` mantenerse en 80 % o más. Si no, se para y se propone empezar desde cero con la receta de la corrida 2 (ayuda vertical) más este castigo desde el principio. **A las 500 iteraciones:** la postura (piernas y cintura a 10° o menos de la pose de pie) y ninguna articulación con objetivo sin recortar fuera de su rango (`measure_getup_raw_actions.py`). **Aprobación de la fase 4, sin cambios:** 95 % tras el cambio a la marcha a 6,6 V y con tensión aleatoria.

### Fase 4, corrida 7 (`fase4_accion_rango`): NO CUMPLE la comprobación de las 200 iteraciones, parada en la iteración 6 700 (6 de noviembre)

Registro (`/d/Claude/fase4_accion_rango.log`, media por paso de los episodios terminados en cada iteración; de 6 498 a 6 700):

| Iteración | `stood` | `risen` | castigo de rango (peso incluido) | `pelvis_height` | `pose_stand` |
|---|---|---|---|---|---|
| 6 518 | 12,6 % | 98,8 % | -1,17 | 1,03 | 0,11 |
| 6 538 | 0,7 % | 98,7 % | -1,78 (máximo) | 1,93 | 0,19 |
| 6 598 | 0 % | 98,0 % | -0,26 | 2,11 | 0,11 |
| 6 698 | 0 % | 98,5 % | -0,09 | 2,17 | 0,17 |

- **El castigo funciona:** el exceso de la acción cruda cae un 95 % desde su máximo registrado (-1,78 a -0,09 en 160 iteraciones). Mi estimación de partida de -6,7 por paso no se reprodujo en el registro (el máximo medido fue -1,78); el cálculo del peso con 190,9 rad era de la política en pie y los episodios del registro empiezan tumbados, así que el peso era más suave de lo que anoté.
- **Pero `stood` cae a 0 %** y no se recupera (criterio: 80 % o más a las 200 iteraciones): la política sigue levantando la pelvis (`risen` 98 %) pero ya no se queda de pie (`getup_up` 0 %, `pelvis_height` 2,17 de unos 3,9 que daba de pie, es decir, pelvis a menos de 0,13 m). Lectura (no comprobada): al sacar las articulaciones de los topes se pierde la postura de equilibrio que la política tenía sostenida contra ellos, y no sabe sostenerse de pie sin ellos; la recompensa de altura no se recupera en 200 iteraciones.
- **Por la regla escrita** (si falla, se para y se propone empezar desde cero con la receta de la corrida 2 más este castigo), la propuesta es: **corrida 8 desde cero** con `DANNY_GETUP_ASSIST=1` (ayuda vertical, etapa 2, 5 000 iteraciones, `fase4_boca_abajo_ayuda`) más `action_out_of_range` (-0,035; con pesos nuevos medidos al inicio) y `pose_stand` (std 2, peso 2) **desde la primera iteración**, para que la política nunca aprenda a apoyarse en los topes. Coste: unas 3 horas. **No comprobado:** que con esa receta se levante (la corrida 2 sí lo hizo, sin esos términos) ni que la postura final sea la de la fase 2.

### Fase 4, corrida 8 (`fase4_accion_rango_ayuda`), criterio escrito antes de entrenar (6 de noviembre)

Decisión del usuario. **Desde cero** (sin punto de partida) con la receta de la corrida 2: ayuda vertical (`DANNY_GETUP_ASSIST=1`, se quita hasta la iteración 2 500), etapa 2 de aleatorización, 4096 entornos, **5 000 iteraciones** (`/d/Claude/fase4_accion_rango_ayuda.sh`). Diferencia con la corrida 2: desde la primera iteración están activos `action_out_of_range` (L1 del exceso de la acción cruda, peso -0,035; la política inicial sale casi sin exceso, así que el peso no se recalcula) y `pose_stand` (std 2, peso 2), que sustituye al castigo de pose débil. Objetivo: que la política nunca aprenda a sostenerse contra los topes (corridas 1 a 7) y que la postura de pie sea la de la fase 2.

**Comprobaciones:** a las 1 000 iteraciones, `getup_up` (pelvis a 0,15 m o más y tronco a 45° o menos) 5 % o más (la de la corrida 2, ver arriba), y si cae por debajo del 1 % o `risen` baja del 50 %, se para y se documenta. A las 3 000 iteraciones (ayuda ya casi quitada), `stood` 50 % o más y, con `measure_getup_raw_actions.py`, exceso total de la acción cruda de 5 rad o menos. **Aprobación al terminar, sin cambios:** 80 % o más se levanta a 6,6 V y con tensión aleatoria; 95 % o más sin caer tras el cambio a la marcha (512 robots); saturación de pie 5 %; ninguna articulación más de 0,5 s seguidas al tope mientras se levanta. Además, como medidas informativas pedidas por la revisión: piernas y cintura a 10° o menos de la pose de pie, y ninguna articulación con objetivo sin recortar fuera de su rango.

### Fase 4, corrida 8 (`fase4_accion_rango_ayuda`): CUMPLE el criterio escrito, boca abajo (6 de noviembre)

5 000 iteraciones desde cero (`/d/Claude/fase4_accion_rango_ayuda.sh`, unas 3 horas), receta de la corrida 2 más `action_out_of_range` (-0,035) y `pose_stand` (std 2, peso 2) desde la primera iteración. Punto de control `fase4_accion_rango_ayuda/model_4999.pt` (copia en `puntos_de_control/fase4_accion_rango_ayuda_model_4999.pt`; ONNX `danny_fase4_levantarse_2026-11-06.onnx`, borrado el 7 de noviembre y sustituido por el de la corrida 10). Medido sin ayuda con `measure_getup.py`, 512 robots boca abajo, 20 s (`results/training_getup_fase4_accion_rango_ayuda_final_{6.6V,7.4V,aleatoria}.json`):

| Tensión | Se levanta | Mediana | Cambio a la marcha sin caer en 5 s (se pide 95 %) | Racha más larga al tope mientras se levanta | Saturación de pie (se pide 5 %) |
|---|---|---|---|---|---|
| 6,6 V (decide) | 100 % | 3,78 s | **99,0 %** | 0,14 s (`left_elbow`) | 0,7 % (`left_ankle_pitch`) |
| Aleatoria 6 a 8,4 V (decide) | 100 % | 3,78 s | **97,85 %** | 0,14 s | 1,3 % |
| 7,4 V (informa) | 100 % | 3,70 s | 97,85 % | 0,12 s | 1,3 % |

Se cumplen los cuatro criterios: se levanta (80 % o más), cambio a la marcha (95 % o más a 6,6 V y aleatoria), saturación de pie (5 % o menos) y rachas al tope por debajo de los 0,5 s. Además, `diagnose_getup_switch_falls.py` (512 robots, 6,6 V, `results/training_getup_caidas_fase4_accion_rango_ayuda_final_6.6V.json`) da 0 caídas (la diferencia con el 99,0 % es el ruido de muestreo de unos ±2 puntos, medido antes). Frente a la corrida 4, que daba 91,0 % y 93,75 %, el cambio mejora unos 6 puntos: era la postura de topes, como proponía la revisión.

**Medidas informativas pedidas por la revisión** (`results/training_getup_postura_fase4_accion_rango_ayuda_final.json`, 256 robots, 6,6 V, últimos 10 s): pelvis 0,2235 m, tronco 1,9°. Errores respecto a la pose de pie de la fase 2: `waist_yaw` 0,0°, rodillas -9,7° y -0,8°, `right_hip_pitch` -1,5°, tobillos de pitch -4,8° y 2,3°, pero **seis articulaciones de piernas quedan a más de 10°**: `left_hip_pitch` 13,9°, `left_ankle_roll` -12,3°, `left_hip_yaw` -11,3°, `right_hip_roll` -11,3°, `right_hip_yaw` 11,1°, `left_hip_roll` 11,0° (y `right_ankle_roll` 10,6°). **Ninguna articulación con objetivo sin recortar fuera de rango: no se cumple** (`results/training_acciones_crudas_fase4_accion_rango_ayuda_final.json`): exceso total 1,76 rad (era 190,9 en la corrida 4), y siguen fuera de rango más de la mitad del tiempo `left_hip_yaw`, `right_hip_yaw`, `left_hip_roll`, `right_hip_roll`, `left_ankle_pitch` y `left_ankle_roll`; los `hip_yaw` están al límite el 98 % y el 95 % del tiempo. Es poco exceso (la acción sale unos 0,3 a 0,8 rad), pero en un robot real, un servo apretado contra un tope se calienta; queda como riesgo anotado, sin medir el par que hace.

**No comprobado:** empezar boca arriba (solo se midió boca abajo), el cambio a la marcha con empujones, el cambio desde otras posturas, el ONNX, el robot real, ni la fricción o el calor de los servos pegados al tope de cadera en yaw. Las fases 5 y 6 no se empiezan sin tu visto bueno. Vídeos: `results/videos/fase4_accion_rango_ayuda_final_robot{0,1}.mp4`.

### Fase 4: la política de la corrida 8 boca arriba (medido sin entrenar, 6 de noviembre)

`measure_getup.py --prone-fraction 0` (todos los robots empiezan boca arriba, tronco -1,5 rad; 512 robots, 20 s, `results/training_getup_fase4_accion_rango_ayuda_boca_arriba_{6.6V,7.4V,aleatoria}.json`), `fase4_accion_rango_ayuda/model_4999.pt`: **se levanta el 0 % a 6,6 V, con tensión aleatoria y a 7,4 V; ningún robot llega ni a 0,10 m de pelvis** (`fraction_ever_pelvis_above_0.10m` 0 %; sin robots que se levanten no hay cambio a la marcha que medir). Era lo esperable: la política solo se entrenó boca abajo. El criterio de la fase 4 entera (línea de «Aprueba la corrida 2»: boca abajo y boca arriba por separado, a 6,6 V y con tensión aleatoria, una sola política) **no se cumple**: la fase 4 está aprobada boca abajo, no entera. Camino previsto en el plan: `fase4_boca_arriba` desde `model_4999.pt`, mitad de los entornos boca abajo y mitad boca arriba (para no olvidar), hasta 5 000 iteraciones más. **No comprobado:** si hace falta la ayuda vertical otra vez (boca arriba la altura de la pelvis parte del suelo y la ayuda empujó bien boca abajo), el tiempo que tarda, y si lo aprendido boca abajo se conserva.

### Fase 4, corrida 9 (`fase4_boca_arriba`), criterio escrito antes de entrenar (6 de noviembre)

Subido a git antes (commit `6383b03`). Decisión del usuario. Desde `fase4_accion_rango_ayuda/model_4999.pt` (reanudando), **mitad boca abajo y mitad boca arriba** (`DANNY_PRONE_FRACTION=0.5`, para no olvidar lo aprendido), **con la ayuda vertical** (`DANNY_GETUP_ASSIST=1`: el contador de pasos se reinicia al reanudar, así que la ayuda vuelve a 0,8 y se quita otra vez hasta la iteración 2 500 de esta corrida; boca arriba la pelvis parte del suelo y la ayuda fue lo que funcionó boca abajo), etapa 2, 4096 entornos, **5 000 iteraciones** (`/d/Claude/fase4_boca_arriba.sh`), con `action_out_of_range` (-0,035) y `pose_stand` (std 2, peso 2) activos como en la corrida 8. Como arranca de un punto de control con otra distribución de inicio, se mide a las 5 y 50 iteraciones (`getup_up`, `stood`) antes de dejarla correr.

**Comprobaciones:** a las 1 000 iteraciones, `getup_up` de 55 % o más (la mitad boca abajo se conserva, y algo boca arriba); si cae por debajo del 40 % o hay un hundimiento como el de la corrida 7, se para y se documenta. A las 3 000 (ayuda ya casi quitada), `stood` 70 % o más. **Aprobación de la fase 4 entera (sin cambios):** con `measure_getup.py` (512 robots, 20 s), boca abajo (`--prone-fraction 1`) y boca arriba (`--prone-fraction 0`) por separado, a 6,6 V y con tensión aleatoria: 80 % o más se levanta; 95 % o más sin caer tras el cambio a la marcha; saturación de pie 5 % o menos; ninguna racha al tope de más de 0,5 s mientras se levanta. Informativo: la postura de pie (piernas y cintura a 10° o menos de la pose de la fase 2) y el exceso de la acción cruda, como en la corrida 8.

**Corrección a la corrida 9 (6 de noviembre, 2 minutos después de lanzarla):** al reanudar desde un punto de control el contador de pasos del entorno continúa desde la iteración del punto de control (5 000), así que la ayuda vertical, contada desde el principio del entrenamiento, ya estaba en 0 (`getup_assist_fraction` 0,0000 en la iteración 5 011) y la corrida no tenía ayuda, contra lo escrito arriba. Se paró, se borró su carpeta y se corrigió `assist_lift` para contar la ayuda desde el primer paso de la corrida. En las 11 primeras iteraciones sin ayuda, la mitad boca arriba no se levantó (`getup_up` 47,6 %, es decir, solo la mitad boca abajo): consistente con que boca arriba sin ayuda no arranca, pero sin más datos no se concluye nada. La corrida 9 se relanza con la ayuda de verdad; el criterio no cambia. (Las corridas 3 y 7, reanudadas sin ayuda, no se ven afectadas.)


**Corrida 9, resultado final (7 de noviembre): NO CUMPLE por la cola de robots sueltos boca arriba.** Entrenamiento completo (9 999 iteraciones, `stood` 94,2 % con la mitad boca arriba sin ayuda). Medida con `model_9998.pt`, 512 robots, `results/training_getup_fase4_final_boca_{abajo,arriba}_{6.6V,aleatoria,7.4V}.json`:

| Caso | Se levanta | Sin caer tras el cambio | Saturación de pie (máx.) | Racha más larga al tope mientras se levanta |
|---|---|---|---|---|
| Boca abajo, 6,6 V | 100 % | 100 % | 0 % | 0,10 s |
| Boca abajo, aleatoria | 100 % | 100 % | 0 % | 0,08 s |
| Boca arriba, 6,6 V | 100 % | 99,8 % | 3,3 % | **0,52 s** (`right_knee`) |
| Boca arriba, aleatoria | 100 % | 100 % | 1,3 % | 0,36 s |
| Boca arriba, 7,4 V (informativo) | 100 % | 99,8 % | 2,0 % | 0,70 s |

Con 512 robots solo falla boca arriba a 6,6 V la racha (0,52 s, un robot, 0,02 s sobre el límite; percentil 95: 0,12 s). Repetida la misma medida con otra muestra de 1 024 robots (`..._boca_arriba_6.6V_1024.json`): se levanta el 100 %, sin caer 99,9 %, pero la racha máxima sube a **1,34 s** y la saturación de pie máxima a **13,3 %**, las dos en `right_knee` y de un solo robot (percentil 95 de la racha: 0,14 s). Es decir, la media está bien y hay una cola rara de robots, boca arriba, con la rodilla derecha al tope; como los dos criterios están escritos sobre el máximo entre robots, **no se puede dar por aprobada**. No se cambia el límite (es decisión del dueño).

Medidas informativas pedidas (256 robots, 6,6 V, `training_getup_postura_fase4_final_*.json`, `training_acciones_crudas_fase4_final_*.json`): la postura de pie sigue sin ser la de la fase 2 (cadera izquierda a 16° y derecha a 7-8° de la pose de pie; la prueba pedía 10° o menos); la acción cruda sigue fuera de rango en cadera (guiñada y balanceo) y tobillo izquierdo, con un exceso total de 2,2 rad boca abajo y 2,4 boca arriba (hasta 54° de exceso), así que «ninguna articulación fuera de rango» tampoco se cumple. Aun así el cambio a la marcha no cae (99,8-100 %).

**No comprobado:** empujones durante el cambio; verificación del ONNX (el script no admite esta tarea); el robot real; si el servo de la cadera, pegado a su tope, se calentaría. Fase 5 y 6 **no empiezan** (la 5 exige la 4 aprobada).


### Fase 4, corrida 10 (`fase4_rodillas`), criterio escrito antes de entrenar (7 de noviembre)

Decisión del dueño: ajuste corto con castigo de par en rodillas, como en la fase 2. **Cambio único:** recompensa `servo_torque_excess_knees` (`getup.py`): cuadrado del par de las dos rodillas por encima del 80 % del bloqueo de la tensión de cada robot, peso -317 (el de la fase 2, `fase2_f_rodilla`), **siempre activa** (sin umbral de altura, para no crear un escalón de recompensa). La penalización de par en todas las articulaciones con la pelvis sobre 0,20 m sigue como estaba. **Corrida:** 1 500 iteraciones desde `fase4_boca_arriba/model_9998.pt`, 4096 entornos, etapa 2, mitad boca abajo y mitad boca arriba, sin ayuda (`/d/Claude/fase4_rodillas.sh`).

**Comprobación a las 500 iteraciones:** `stood` 80 % o más en el entrenamiento (la corrida 9 daba 94 %); si baja del 50 % se para, se anota y no se sigue.

**Aprobación, el criterio de la fase 4 sin cambios**, medida con **1 024 robots** (la muestra que dio la cola; con 512 la cola se ve menos), boca abajo y boca arriba por separado, a 6,6 V y con tensión aleatoria: 80 % o más se levanta; 95 % o más sin caer a los 5 s del cambio a la marcha con orden cero; saturación de pie 5 % o menos (máximo entre robots); ninguna articulación más de 0,5 s seguidos al tope mientras se levanta (máximo entre robots). 7,4 V, postura de pie (10° de la pose de la fase 2) y exceso de acción cruda se miden solo como datos informativos. Si no cumple, se para y se anota qué falla; no se alarga la corrida.


**Corrida 10, resultado (7 de noviembre): NO CUMPLE; el castigo de rodillas no quitó la cola.** 1 500 iteraciones terminadas sin errores (`stood` 95 % a las 500 y al final). Medida con `model_11497.pt`, **1 024 robots**, `results/training_getup_fase4_rodillas_boca_{abajo,arriba}_{6.6V,aleatoria,7.4V}.json`. En los seis casos medidos se levanta el 100 % y no cae ninguno tras el cambio; cumplen los cinco salvo «boca arriba, 6,6 V» (el de 7,4 V es informativo), que falla:

| Caso (1 024 robots) | Racha más larga al tope (máx. entre robots) | Saturación de pie (máx. entre robots) |
|---|---|---|
| Boca abajo 6,6 V / aleatoria / 7,4 V | 0,12 / 0,26 / 0,20 s | 0,7 % |
| Boca arriba 6,6 V | **1,08 s** (`right_knee`) | **17,3 %** (`right_knee`) |
| Boca arriba aleatoria | 0,36 s | 0,7 % |
| Boca arriba 7,4 V (informativo) | 0,64 s | 2,7 % |

Comparado con la corrida 9 en el mismo caso y con el mismo número de robots: media de saturación de la rodilla derecha mientras se levanta 3,6 % (antes 6,3 %), la izquierda 1,2 % (antes 6,4 %), percentil 95 de la racha 0,06 s (antes 0,14 s); pero el máximo entre robots sigue siendo de un robot suelto (1,08 s frente a 1,34 s; pie 17,3 % frente a 13,3 %). El castigo redujo la saturación típica a la mitad o menos y no cambió la cola: un robot concreto de la muestra, boca arriba y a 6,6 V, sigue con la rodilla derecha al tope. **No se sabe por qué ese robot**: no se ha mirado qué parámetros sorteados tiene (masa, centro de masas, fricción, servo). Postura y acción cruda siguen como en la corrida 9 (informativas): exceso total 2,1 rad boca abajo y 2,3 boca arriba, cadera derecha a 29-31° (pose de la fase 2: -21°). Fase 4 sin aprobar; fase 5 y 6 sin empezar.


**Diagnóstico de los robots atípicos de la corrida 10 (7 de noviembre, sin entrenar).** Script nuevo `training/scripts/diagnose_getup_outlier_robots.py` (`results/training_getup_atipicos_fase4_rodillas_boca_arriba_6.6V.json`): 4 096 robots boca arriba a 6,6 V, política de levantarse sin cambio a la marcha, atípico = una rodilla más de 0,5 s seguidos al 90 % del bloqueo; se comparan sus parámetros sorteados (masas, centros de masa, inercias, fricciones, armadura, holgura, tensión, posición inicial) con los del resto. Para ejecutarlo, desde `training/`: `.venv/Scripts/python.exe scripts/diagnose_getup_outlier_robots.py --checkpoint logs/rsl_rl/danny_walk/2026-11-07_08-24-49_fase4_rodillas/model_11497.pt --name fase4_rodillas_boca_arriba_6.6V`.

Resultado: **12 de 4 096 robots (0,29 %) con la rodilla derecha al tope más de 0,5 s; la izquierda, 0**. Rachas de esos 12: 0,52 a 6,98 s (mediana 0,68 s; dos de 3,4 y 7,0 s); ninguno está al tope en los últimos 10 s. **Ningún parámetro sorteado lo explica**: la mayor diferencia es de 0,9 desviaciones típicas (la fricción de articulación, algo más alta, `friction_scale` 1,14 frente a 1,00) y las posiciones iniciales no aparecen entre las primeras. Es decir, no es un robot con un servo o una masa extremos, sino una fracción pequeña de arranques (0,3 %) que atraviesan una trayectoria distinta (probablemente un pie o una pierna atascados al levantarse; no se ha mirado). Consecuencia para el criterio, con números: con una fracción del 0,29 %, el máximo entre robots pasa de 0,5 s en cualquier muestra de unos cientos de robots con probabilidad apreciable (512 robots: 1,5 atípicos esperados; 1 024: 3), así que «ninguna articulación más de 0,5 s en ningún robot» no es un criterio que una política con esa cola pueda pasar de forma fiable. **No comprobado:** qué hace físicamente el robot en esos casos (vídeo/trayectoria), si boca abajo existe una cola equivalente más pequeña, y si más entrenamiento la reduce.


### Cambio del criterio de la fase 4 por decisión del dueño (7 de noviembre)

**Cambio del criterio escrito.** Donde decía «ninguna articulación más de 0,5 s seguidos al tope mientras se levanta» (y «saturación de pie 5 % o menos») como **máximo entre robots**, ahora dice: **el 99 % de los robots o más, sin pasar de 0,5 s seguidos al tope mientras se levanta, y el 99 % o más con saturación de pie de 5 % o menos**, siempre medido con **1 024 robots o más**. **Motivo:** el máximo entre robots depende del tamaño de la muestra. El diagnóstico de la corrida 10 (`results/training_getup_atipicos_fase4_rodillas_boca_arriba_6.6V.json`) da 12 de 4 096 robots (0,29 %) con la rodilla derecha al tope más de 0,5 s boca arriba a 6,6 V, sin parámetro sorteado que lo explique; con una cola así, el máximo supera 0,5 s en casi cualquier muestra de unos cientos de robots (la corrida 9 falló por 0,02 s con 512 y la 10 falla con 1 024). El resto del criterio (80 % se levanta, 95 % sin caer a los 5 s del cambio a la marcha, boca abajo y boca arriba por separado, a 6,6 V y con tensión aleatoria) no cambia.

**Dos condiciones añadidas antes de aprobar la fase 4:**
1. **Ningún robot más de 2 s seguidos al tope** (mientras se levanta). Si siguen los robots de 7 s y de 3,4 s, no aprueba todavía. Se graban en vídeo y se explica qué pasa (se traba la pierna con algo, choca con el otro pie, se queda a medio levantar) para decidir si hace falta otro ajuste.
2. **Medir la misma cola boca abajo** con `diagnose_getup_outlier_robots.py`, para saber si también existe ahí.

### Requisito para la etapa 5 (robot real): protección del servo

Pedido por el dueño el 7 de noviembre, **independiente de lo que pase en simulación**: si un servo pasa **más de 1 s seguido cerca de su bloqueo** (par sobre el 90 % del bloqueo de su tensión), el programa de la Radxa **relaja esa articulación o se para** (par cero o parada del movimiento, a decidir con el hardware real). Protege al servo de calentamiento y de bloqueo aunque la política falle. Hay que implementarlo en el programa de 50 Hz y probarlo con una carga que bloquee el servo a propósito antes de dejar al robot moverse solo. Se mide con la corriente o la temperatura reales cuando existan (ahora no hay medida real).


**Resultado de las dos condiciones (corrida 10, `model_11497.pt`, 4 096 robots, 6,6 V; segunda pasada, los robots no son los de la primera).** Datos en `results/training_getup_atipicos_fase4_rodillas_boca_{arriba,abajo}_6.6V.json`; vídeos de los cuatro con la racha más larga en `results/videos/fase4_rodillas_boca_arriba_6.6V_robot{3589,487,841,2702}_right_knee_*.mp4` (20 s, cámara del entorno; el número es el del robot en esta pasada).

- **Boca abajo: la cola no existe** (0 de 4 096 robots con una rodilla más de 0,5 s al tope). Condición 2 cumplida.
- **Boca arriba: 14 de 4 096 (0,34 %), todos la rodilla derecha.** Rachas: 4,6, 3,6, 3,2 y 3,2 s los cuatro mayores; el resto de 0,5 a 1,0 s. **Condición 1 (ningún robot más de 2 s) no se cumple:** 4 robots la superan (el máximo de la pasada anterior era de 7,0 s; en esta, 4,6 s). Con el criterio del 99 %, boca arriba daría 99,66 % (cumple); pero no aprueba por la condición 1.
- **Qué pasa (medido y visto en los fotogramas del robot 3589 a 0,5, 1,3, 2,5, 4,0, 5,8 y 7,0 s):** no se traba ni choca. Ya está de pie a los 1,3 s (pelvis 0,21 m, tronco 9 a 13° de inclinación, ningún eslabón no-pie en el suelo, contactos entre los dos pies 0, autocontactos casi 0), en una postura **asimétrica con un pie adelantado**; la rodilla derecha, flexionada unos 72 a 76° (la de la pose de pie son 47°), carga más peso y está al tope de par desde que se pone de pie (arranque entre 1,26 y 1,32 s en los 14) hasta que reordena las piernas, entre 1 y 5 s después, y se estabiliza. Es un modo de levantarse concreto, no un robot con parámetros raros (el diagnóstico anterior no halló ningún parámetro sorteado asociado). **No comprobado:** por qué algunos arranques acaban en esa postura y otros no; si los otros 3 vídeos (487, 841, 2702) hacen lo mismo (los números de la tabla coinciden: pelvis, tilt, contactos); si un cambio de entrenamiento lo elimina.
- **Nota de medida:** en la primera versión del script la rodilla de la que se daba el ángulo estaba mal etiquetada (`left_knee_deg`); es la derecha (`right_knee_deg`), y está corregido en el script y en el JSON.

**Fase 4: sigue sin aprobar** (condición 1). La fase 5 y 6 no empiezan. Opciones para decidir con estos datos: (a) un ajuste que penalice estar mucho tiempo al tope de par con una rodilla (por ejemplo, más peso al castigo de par de rodillas o un castigo creciente con el tiempo seguido al tope), midiendo después con 4 096 robots; (b) pedir una postura de pie simétrica al terminar de levantarse (castigar la diferencia de altura o de ángulo entre los pies/rodillas) para evitar el pie adelantado; (c) relajar la condición 1 (decisión del dueño).


### Fase 4, corrida 11 (`fase4_simetria`), criterio escrito antes de entrenar (7 de noviembre)

Decisión del dueño: probar una postura de pie simétrica para quitar el pie adelantado de los robots atípicos. **Medido antes:** la asimetría no es solo de los atípicos; la postura de pie típica de la corrida 9 tiene la cadera izquierda a -5° y la derecha a -29° (24° de diferencia; con la misma convención de signo, la pose de pie de la fase 2 es -20,9° en las dos). **Cambio único sobre la corrida 10 (que se conserva entera, incluido el castigo de par en rodillas):** recompensa `pose_symmetry` (`getup.py`): exp(-suma, sobre cadera, rodilla y tobillo de cabeceo, de (izquierda - derecha)² / std²) por `pelvis_height_progress`, con **std 0,5 rad (std² 0,25) y peso 2,0**. Positiva y multiplicada por el avance de altura y tronco, como `pose_stand`, para no crear escalón. Con la asimetría típica (suma de unos 0,5 rad²) cobra 0,14 de 1 (0,27 por paso); simétrica, 1 (2,0 por paso): un empuje del orden de `pose_stand` y `stand_still`. **Corrida:** 1 500 iteraciones desde `fase4_rodillas/model_11497.pt`, 4096 entornos, etapa 2, mitad boca abajo y mitad boca arriba, sin ayuda (`/d/Claude/fase4_simetria.sh`).

**Comprobación a las 500 iteraciones:** `stood` 80 % o más en el entrenamiento (la corrida 10 daba 95 %); si baja del 50 % se para, se anota y no se sigue.

**Aprobación (criterio de la fase 4 vigente, con el cambio del dueño de hoy):** medido con **4 096 robots** para ver la cola (el criterio pide 1 024 o más), boca abajo y boca arriba por separado, a 6,6 V y con tensión aleatoria: 80 % o más se levanta; 95 % o más sin caer a los 5 s del cambio a la marcha con orden cero; el 99 % de los robots o más sin pasar de 0,5 s seguidos al tope mientras se levanta; el 99 % o más con saturación de pie de 5 % o menos; **ningún robot más de 2 s seguidos al tope**. Informativo: 7,4 V; postura de pie (diferencia izquierda-derecha de cadera, rodilla y tobillo, y distancia a la pose de la fase 2); exceso de acción cruda; vídeos y estado de los atípicos que queden (`diagnose_getup_outlier_robots.py`). Si no cumple, se para y se anota qué falla; no se alarga la corrida.


**Corrida 11, resultado (7 de noviembre): NO CUMPLE; la simetría quita la cola pero rompe el cambio a la marcha.** 1 500 iteraciones sin errores (`stood` 94 % a las 500 y al final, `pose_symmetry` 1,8 de 2,0). Medida con `model_12996.pt`, 4 096 robots (1 024 a 7,4 V), `results/training_getup_fase4_simetria_boca_{abajo,arriba}_{6.6V,aleatoria,7.4V}.json`:

| Caso | Se levanta | **Sin caer tras el cambio** (pide 95 %) | Racha más larga al tope | Saturación de pie (máx.) |
|---|---|---|---|---|
| Boca abajo 6,6 V | 100 % | **90,8 %** | 0,12 s | 2,7 % |
| Boca abajo aleatoria | 100 % | **90,8 %** | 0,14 s | 0,7 % |
| Boca arriba 6,6 V | 100 % | **88,2 %** | 0,86 s (`right_knee`) | 6,0 % (un robot) |
| Boca arriba aleatoria | 100 % | **91,0 %** | 0,34 s | 1,3 % |
| Boca abajo / arriba 7,4 V (informativo) | 100 % | 85,8 % / 92,5 % | 0,06 / 0,26 s | |

- **Lo que sí consigue:** la postura de pie es simétrica (cadera izquierda -17,9° y derecha -18,6° boca abajo; -19,7° y -20,2° boca arriba; la pose de la fase 2 es -20,9°; la corrida 9 daba -5° y -29°) y **la cola desaparece**: atípicos con una rodilla más de 0,5 s al tope, 0 de 4 096 boca abajo y 1 de 4 096 boca arriba (0,56 s; la corrida 10 daba 14 con rachas de hasta 4,6 s). Se cumplen el 99 % y «ningún robot más de 2 s».
- **Lo que rompe:** el cambio a la marcha cae entre el 8 y el 12 % (la corrida 10 caía el 0 al 0,2 %). Diagnóstico de caídas (`diagnose_getup_switch_falls.py`, 1 024 robots boca abajo, 6,6 V, `results/training_getup_caidas_fase4_simetria_6.6V.json`): caen 55, **todos hacia delante**, entre 1,6 y 5,0 s después del cambio (mediana 2,9 s). Ninguna articulación lo explica (la mayor diferencia entre los que caen y los que no es de 0,9 desviaciones típicas); la velocidad del tronco al cambiar es 0,7 desviaciones típicas mayor en los que caen. Es decir, ya **no es la postura simétrica o asimétrica**: la política de la fase 2 recibe una postura que no sostiene y se va hacia delante. Esa medida con 1 024 robots dio 94,6 % sin caer y la de 4 096, 90,8 %: la diferencia entre muestras (3,8 puntos) es mayor que el ±2 que se había medido antes con 512.
- **Informativas:** exceso de acción cruda 2,7 rad boca abajo y 2,8 boca arriba (corrida 10: 2,1 y 2,3), es decir, algo peor.
- **No comprobado:** por qué la postura simétrica hace caer hacia delante a la política de la fase 2 (posición del centro de masas al cambiar); si un peso menor de simetría da un compromiso; empujones; ONNX de la tarea; robot real.

**Fase 4 sigue sin aprobar.** Las dos mejores corridas hasta ahora fallan en cosas opuestas: la 10 cumple el cambio a la marcha (99,8 a 100 %) y falla la cola de rodilla (4 robots de más de 2 s boca arriba); la 11 cumple la cola y falla el cambio (88 a 91 %). Fase 5 y 6 sin empezar.


### Fase 4, corrida 12 (`fase4_simetria_suave`), criterio escrito antes de entrenar (7 de noviembre)

Decisión del dueño: probar un peso menor de la simetría, desde la corrida 10. **Medido antes:** con peso 2,0 (corrida 11) la postura queda simétrica y la cola desaparece, pero el cambio a la marcha cae el 8 a 12 %; con peso 0 (corrida 10) el cambio cae el 0 a 0,2 % y hay cola (14 de 4 096, 4 robots de más de 2 s). **Cambio único sobre la corrida 11:** `pose_symmetry` con **peso 0,5** (era 2,0; std 0,5 rad igual) y arranque desde **`fase4_rodillas/model_11497.pt`** (la corrida 10, sin simetría), no desde la 11. Con la asimetría típica de la corrida 10 (suma de unos 0,5 rad², 24° de diferencia de cadera) cobra 0,07 por paso y simétrica 0,5. Es una hipótesis de punto intermedio: no se sabe si existe. 1 500 iteraciones, 4096 entornos, etapa 2, mitad y mitad, sin ayuda (`/d/Claude/fase4_simetria_suave.sh`).

**Comprobación a las 500 iteraciones:** `stood` 80 % o más; si baja del 50 %, se para.

**Aprobación: la misma de la corrida 11** (4 096 robots, boca abajo y boca arriba por separado, a 6,6 V y con tensión aleatoria; se levanta 80 % o más; **95 % o más sin caer tras el cambio**; 99 % de los robots sin pasar de 0,5 s al tope al levantarse; 99 % con saturación de pie de 5 % o menos; **ningún robot más de 2 s seguidos al tope**), con 7,4 V, postura, acción cruda y atípicos como informativos. Si el cambio a la marcha cae más del 5 %, o hay robots de más de 2 s, no aprueba y se para. Una medida de 1 024 robots no basta: entre 1 024 y 4 096 hubo 3,8 puntos de diferencia en el cambio a la marcha.


**Corrida 12, resultado (7 de noviembre): NO CUMPLE; con peso 0,5 el cambio a la marcha también cae.** 1 500 iteraciones sin errores (`stood` 94 % a las 500 y al final, `pose_symmetry` 0,44 de 0,5). Medida con `model_12996.pt`, 4 096 robots (1 024 a 7,4 V), `results/training_getup_fase4_simetria_suave_boca_*.json`:

| Caso | Se levanta | **Sin caer tras el cambio** (pide 95 %) | Racha más larga al tope | Saturación de pie (máx.) |
|---|---|---|---|---|
| Boca abajo 6,6 V | 100 % | **90,1 %** | 0,42 s | 1,3 % |
| Boca abajo aleatoria | 99,98 % | **91,2 %** | 0,28 s | 2,0 % |
| Boca arriba 6,6 V | 100 % | **93,5 %** | 0,50 s | 12,7 % (un robot) |
| Boca arriba aleatoria | 99,98 % | 95,0 % | 0,54 s | 0,7 % |
| Boca abajo / arriba 7,4 V (informativo) | 100 % | 92,9 % / 98,2 % | 0,08 / 0,26 s | |

Postura (256 robots, 6,6 V): cadera izquierda/derecha -15,9° / -19,5° boca abajo y -18,2° / -21,3° boca arriba (con peso 2,0: -17,9°/-18,6° y -19,7°/-20,2°; peso 0: -5°/-29°). Atípicos de rodilla con la racha de más de 0,5 s: 0 de 4 096 boca abajo y 2 de 4 096 boca arriba (0,62 y 0,52 s; ninguno de 2 s). Acción cruda: exceso 2,8 rad boca abajo y 2,9 boca arriba.

**El cambio cae entre el 5 y el 10 %, igual que con peso 2,0** (diagnóstico de caídas, 2 048 robots boca abajo, 6,6 V: 150 de 2 048 sin caer 92,7 %, todos hacia delante, entre 1,5 y 4,7 s, mediana 2,8 s). Cualquier peso de simetría probado (0,5 y 2,0) rompe el cambio a la marcha; **no hay punto intermedio entre 0 y 0,5**, o al menos no se ha encontrado. **Prueba de espera (medición, sin entrenar):** esperar más de pie antes de cambiar no lo arregla: con 5 s de pie antes del cambio caen 148 de 2 048 (7,2 %) y con 8 s, 132 (6,4 %); con los 3 s del criterio, 7,3 %. No es un problema de estabilizarse. **Hipótesis (no comprobada):** la política de la fase 2 sostiene mejor la postura asimétrica de la corrida 10 que la simétrica, quizá por dónde queda el centro de masas respecto al punto de apoyo, o porque esta postura no es una que viera al entrenar.

**Balance de la fase 4 (corridas 9 a 12), todo con 4 096 robots salvo la 9:** la corrida 10 cumple el cambio a la marcha y falla la cola; las 11 y 12 cumplen la cola y fallan el cambio. Fase 4 sin aprobar; fase 5 y 6 sin empezar.


### Fase 4 APROBADA (7 de noviembre): corrida 10, con una excepción aceptada

**Decisión del dueño, 7 de noviembre:** se aprueba la fase 4 con **`fase4_rodillas/model_11497.pt`** (corrida 10; copia en `/d/Claude/puntos_de_control/fase4_rodillas_model_11497.pt`, fuera del repo). Es la política de levantarse desde el suelo, boca abajo y boca arriba, con una sola red.

**Cambio del criterio escrito, decidido por el dueño:** la condición «ningún robot más de 2 s seguidos al tope de par mientras se levanta» se **relaja** para esta corrida. Queda anotada la cola: boca arriba a 6,6 V, 14 de 4 096 robots (0,34 %) pasan más de 0,5 s con la rodilla derecha al tope, y 4 de ellos (0,1 %) entre 3,2 y 4,6 s, en una postura de pie con un pie adelantado que deshacen en unos 1 a 5 s; boca abajo no hay cola (0 de 4 096). **Motivo:** las corridas 11 y 12 (simetría de piernas, peso 2,0 y 0,5) quitan la cola pero hacen que el cambio a la marcha caiga entre el 5 y el 12 %, y esperar más de pie antes de cambiar no lo arregla (6 a 7 % de caídas esperando 5 u 8 s). **Compensación obligatoria:** el requisito de la etapa 5 (robot real) de relajar o parar una articulación que pase más de 1 s cerca de su bloqueo protege al servo en esos casos.

**Cifras de la aprobación** (`model_11497.pt`, `results/training_getup_fase4_rodillas_boca_{abajo,arriba}_{6.6V,aleatoria,7.4V}.json`, 1 024 robots; la cola, con 4 096 en `training_getup_atipicos_fase4_rodillas_boca_*.json`): se levanta el 100 % boca abajo y boca arriba a 6,6 V, con tensión aleatoria y a 7,4 V; sin caer tras el cambio a la marcha, 100 % en los seis casos; saturación de pie por debajo del 5 % en todos menos un robot (17,3 %, rodilla derecha, boca arriba a 6,6 V); racha al tope de 0,12 a 0,26 s boca abajo y de 0,36 a 1,08 s boca arriba como máximo entre robots. Tiempo mediano en ponerse de pie, 3,6 s boca abajo y 4,4 a 4,7 s boca arriba. Corridas posteriores que no se adoptan: 11 (`fase4_simetria`, cambio 88 a 91 %) y 12 (`fase4_simetria_suave`, 90 a 95 %). Su código (recompensa `pose_symmetry`) se quitó de `getup.py`; la receta está descrita arriba y sus resultados en `results/training_getup_fase4_simetria*.json`.

**ONNX:** `policies/danny_fase4_levantarse_2026-11-07.onnx` (78 entradas, 23 salidas; sin el reloj de marcha ni la orden de altura). **Verificado** contra el punto de control con `scripts/verify_policy_onnx.py --task Mjlab-GetUp-Danny` (500 pasos, 16 robots): diferencia máxima 3,1e-6 (límite 1e-4), `results/training_onnx_fase4_rodillas.json`. Se sustituyó el ONNX de la corrida 8.

**Cómo usarla:** el robot tumbado ejecuta esta política hasta cumplir el criterio de «se levanta» (3 s seguidos con pelvis a 0,20 m o más, tronco a 15° o menos, los dos pies en el suelo y velocidad media de 0,05 m/s o menos); en ese instante cambia a la política de la fase 2 con orden cero (el reloj de marcha parado, `[0, 1]`, y la altura 0). **No comprobado:** empujones durante el cambio, otras posturas iniciales distintas de las dos del entrenamiento (prone o supine con pitch ±1,5 rad), el robot real, y los servos pegados al tope en la guiñada de cadera (posible calor).


## Fase 5: sentarse y tumbarse (empezada el 7 de noviembre)

El dueño aprobó la fase 4 y pidió seguir con la 5 (la 6 no empieza). **Decisiones de diseño (mías, a revisar por el dueño):** dos acciones, cada una con **política propia**: (a) **sentarse** desde de pie hasta quedar sentado en el suelo, quieto; (b) **tumbarse** desde sentado hasta quedar boca arriba, quieto. Se hace primero (a) (tarea `Mjlab-SitDown-Danny`, `training/src/danny_training/sit.py`) y luego (b). Hasta 5 000 iteraciones por acción, desde cero, 4096 entornos, etapa 2 de aleatorización, sin ayuda. Las dos políticas no llevan orden: arrancan en su estado inicial y hacen su acción (como la de levantarse; 78 entradas y 23 salidas).

**Medido antes de escribir el criterio** (`scripts/check_sit_pose.py`, `results/training_sit_pose.json`): sentado en el suelo con las piernas hacia delante (cadera de cabeceo -80 a -90°, rodilla de 10° a 40°) y los brazos colgando, el robot **se cae hacia atrás hasta quedar boca arriba**, pase lo que pase con la pose de las piernas (cuatro candidatas, 64 robots, 6,6 V; pelvis a 0,03 m y tronco a 90° al final); sobre los talones, igual. Por tanto «sentado» **no es una pose pasiva**: la política tiene que apoyarse con los brazos o con los pies. Por eso el sentado permite apoyar manos, brazos, piernas y pies, y prohíbe tocar el suelo con tronco, cuello y cabeza. La altura de la pelvis sentado **no se ha medido** (el robot terminó tumbado): `SEATED_PELVIS_M` = 0,07 m es una **estimación** que se ajusta con lo que aprenda la política. Altura de pie: 0,228 m.

### Fase 5, sentarse, corrida 1 (`fase5_sentarse`), criterio escrito antes de entrenar (7 de noviembre)

**Tarea:** empieza de pie en la pose de la fase 2 (altura 0,228 m, órdenes cero), episodio de 10 s, termina si el tronco pasa de 70°. **Recompensas** (`sit.py`): `sit_progress` (peso 4: bajada de la pelvis de 0 a 1 entre 0,228 y 0,05 m, por (1 + cos del tronco) / 2, lineal para que haya pendiente desde el principio y sin escalones), `seated_still` (peso 2: exp(-velocidad² / 0,3²) por el avance, quieto solo cobra abajo), `descent_speed_excess` (-5: cuadrado de lo que la pelvis baja a más de 0,4 m/s), `forbidden_floor_contacts` (-1 por contacto de tronco, cuello o cabeza con el suelo), y las de las corridas de levantarse que sirvieron: `action_out_of_range` (-0,035), `dof_pos_limits` (-1), `action_rate_l2` (-0,1), `link_pair_contacts` (-1) y castigo de par al 80 % del bloqueo en todas las articulaciones (-317). 5 000 iteraciones desde cero (`/d/Claude/fase5_sentarse.sh`).

**Comprobaciones:** a las 1 000 iteraciones, `sit_low_fraction` (pelvis a 0,15 m o menos) del 50 % o más; a las 2 500, `sit_seated_fraction` del 30 % o más. Si no, se para, se anota y no se alarga.

**Aprobación** (script de medida `scripts/measure_sit.py`, por escribir mientras entrena; 4 096 robots, 6,6 V y tensión aleatoria de 6 a 8,4 V, sin empujones): **se sienta el 95 % o más** (pelvis a 0,07 m o menos, tronco a 35° o menos, velocidad media de 0,05 m/s o menos, sin tronco, cuello ni cabeza tocando el suelo, 3 s seguidos, en 10 s); **suave:** el 95 % o más con la bajada de la pelvis a 0,6 m/s o menos; **no cae:** el 98 % o más sin que el tronco pase de 70° en ningún momento; saturación (par sobre el 90 % del bloqueo): el 99 % o más de los robots sin pasar de 0,5 s seguidos mientras baja y con saturación 5 % o menos estando sentado, y ningún robot más de 2 s seguidos. 7,4 V solo informativo. Si no cumple, se para y se anota qué falla. **Fuera de esta corrida:** levantarse desde sentado con la política de la fase 4 (no entrenada para ello) y la política de tumbarse.


**Corrida 1 de sentarse, resultado intermedio (7 de noviembre).** El entrenamiento se paró solo en la iteración 2 092 de 5 000 (el proceso murió al cerrarse la sesión anterior; sin error en el registro). Se midió el último punto de control, `fase5_sentarse/model_2050.pt`, con `scripts/measure_sit.py` (4 096 robots a 6,6 V y con tensión aleatoria, 1 024 a 7,4 V; `results/training_sentarse_fase5_sentarse_{6.6V,aleatoria,7.4V}.json`):

| Medida | 6,6 V | Aleatoria | 7,4 V (informativo) | Pide |
|---|---|---|---|---|
| Se sienta | 100 % | 100 % | 100 % | 95 % o más |
| Sin pasar de 70° de tronco | 100 % | 100 % | 100 % | 98 % o más |
| Saturación (racha de 0,5 s o menos mientras baja; sentado 5 % o menos) | 100 % / 100 % | 100 % / 100 % | 100 % / 100 % | 99 % o más |
| **Bajada de la pelvis a 0,6 m/s o menos** | **1,2 %** | **0,9 %** | 0,6 % | **95 % o más** |

Velocidad máxima de bajada: mediana 0,76 m/s y percentil 95 0,87 m/s. Tiempo mediano en sentarse, 3,4 s. **Postura final** (el mismo valor en el 80 % o más de los robots): pelvis a 0,056 m (el umbral de 0,07 m estimado vale), tronco a 1,3°, tocan el suelo solo muslos y espinillas (sentado con las piernas estiradas hacia delante, sin apoyar los brazos; lo medí después con `measure_sit_final_pose.py`: antes lo llamé «sobre los talones» por error), sin tronco ni cabeza en el suelo. **No cumple solo la bajada suave:** cae demasiado deprisa (0,76 m/s en vez de 0,6 m/s o menos). Causa clara: el castigo de `descent_speed_excess` valía -5, de modo que una bajada de 0,76 m/s (exceso 0,36 m/s) costaba unos 0,65 por paso, frente a unos 6 por paso de recompensa.

### Fase 5, sentarse, corrida 2 (`fase5_sentarse_suave`), criterio escrito antes de entrenar

**Cambio único:** `descent_speed_excess` de -5 a **-100** (una bajada de 0,76 m/s pasa a costar unos 13 por paso mientras dura; sigue siendo un castigo liso del exceso sobre 0,4 m/s, sin escalón). 1 500 iteraciones desde `fase5_sentarse/model_2050.pt`, 4096 entornos, etapa 2 (`/d/Claude/fase5_sentarse_suave.sh`). **Comprobación a las 500 iteraciones:** sentado en el entrenamiento (`sit_seated_fraction`) del 80 % o más; si baja del 50 %, se para. **Aprobación:** la de la corrida 1, sin cambios (se sienta 95 % o más; bajada a 0,6 m/s o menos 95 % o más; sin pasar de 70° 98 % o más; saturación 99 % o más; 4 096 robots a 6,6 V y con tensión aleatoria).


### Fase 5, sentarse: corrida 2 (`fase5_sentarse_suave`), aprobación RETIRADA el 8 de noviembre

**APROBACIÓN RETIRADA por el dueño el 8 de noviembre, tras ver los vídeos:** el robot se deja caer, no baja controlado. La pelvis baja 17 cm en unos 0,4 s (unos 0,4 m/s de media, 0,5 a 0,65 m/s de pico) y el criterio de bajada a 0,6 m/s o menos era demasiado flojo. `model_3549.pt` y su ONNX (`policies/danny_fase5_sentarse_2026-11-07.onnx`) siguen en el repositorio como punto de partida, pero **no están aprobados** y no deben usarse como la política de sentarse. Criterio nuevo propuesto por el dueño (pendiente de ajustar con el cálculo de par estático antes de escribirlo): bajada de la pelvis de 1 s o más, pelvis a 0,3 m/s o menos en el 95 %, tronco a 3 rad/s o menos en cualquier eje en el 99 %, brazos a 20° o menos de la pose de reposo al terminar. El texto de abajo es el histórico de la aprobación anterior.

(Histórico, 7 y 8 de noviembre) APROBADO

**Aprobado por el dueño el 8 de noviembre** con `fase5_sentarse_suave/model_3549.pt` (4 096 robots a 6,6 V y con tensión aleatoria: se sienta el 100 %, bajada a 0,6 m/s o menos el 97,5 % y el 97,6 %, no cae el 100 %, saturación 100 %). **Diferencia máxima del ONNX frente a la política:** 1,19e-6 en la acción (500 pasos, 16 robots; el máximo permitido es 1e-4), `results/training_onnx_fase5_sentarse_suave.json`. **Vídeos a 6,6 V** (1 024 robots, `measure_sit.py --videos`, `results/videos/`): robot típico (bajada de 0,50 m/s, `..._tipico_robot124_bajada0.50ms.mp4`) y el de la bajada más rápida (0,65 m/s, `..._bajada_mas_rapida_robot437_bajada0.65ms.mp4`). **Aviso tras ver los fotogramas (la aprobación se revisa si el dueño lo ve raro):** no baja de forma lenta y controlada: se **deja caer hacia atrás** en unos 0,5 s, con el tronco inclinado hacia atrás en el aire y los brazos lanzados hacia arriba, y queda sentado con las piernas estiradas; el robot típico acaba con un brazo levantado. Es coherente con la bajada de 0,5 m/s medida y con que la pose sentada no se sostiene sola (brazos, cintura y cuello pegados a sus topes).

**Corrección de la medida de bajada (decidida por mí, a revisar por el dueño):** `measure_sit.py` contaba como bajada la caída del robot al nacer (entre 1 y 5 cm sobre el suelo; pico de unos 1 m/s a los 0,06 s con la pelvis a 0,237 m, por encima de la altura de pie, 0,228 m). Ahora la bajada cuenta solo con la pelvis a 1 cm o más por debajo de la altura de pie. El criterio escrito (0,6 m/s o menos en el 95 % o más) no cambia; sí cambia cómo se mide. La medida anterior daba 57 % (corrida 2) y 1,2 % (corrida 1); la corregida da, para la corrida 2, 97,5 %. El perfil de la bajada real de la política (mediana de 512 robots, 6,6 V): de 0,255 a 0,057 m en unos 0,4 s, a 0,42 a 0,51 m/s, y quieto desde los 0,6 s.

**Resultado** (`fase5_sentarse_suave/model_3549.pt`, 3 549 iteraciones: 2 050 de la corrida 1 y 1 500 de la 2, dentro del tope de 5 000; `results/training_sentarse_fase5_sentarse_suave_{6.6V,aleatoria,7.4V}.json`; 4 096 robots a 6,6 V y con tensión aleatoria, 1 024 a 7,4 V):

| Medida | 6,6 V | Aleatoria | 7,4 V (informativo) | Pide |
|---|---|---|---|---|
| Se sienta | 100 % | 100 % | 100 % | 95 % o más |
| Bajada de la pelvis a 0,6 m/s o menos | 97,5 % | 97,6 % | 98,3 % | 95 % o más |
| Sin pasar de 70° de tronco | 100 % | 100 % | 100 % | 98 % o más |
| Saturación: racha de 0,5 s o menos mientras baja / sentado 5 % o menos | 100 % / 100 % | 100 % / 100 % | 100 % / 100 % | 99 % o más |

Velocidad máxima de bajada: mediana 0,50 m/s y percentil 95 de 0,59 m/s. Tiempo mediano en quedar sentado, 3,5 s (3 s de ventana incluidos). **Postura final** (`scripts/measure_sit_final_pose.py`, `results/training_sentarse_pose_final.json`, igual en todos los robots: percentiles 10 y 90 a menos de 4°): pelvis a 0,056 m, tronco a 1,4°, **sentado con las piernas estiradas hacia delante** (cadera de cabeceo -91°, rodilla 7°, tobillo -32°; muslos y espinillas en el suelo), sin tronco, cuello ni cabeza en el suelo. **Observación para el dueño:** la política deja la cintura, los hombros, los codos y el cuello **cerca de sus topes** (cintura de giro 53° de ±54°, hombro izquierdo en balanceo 108° de 113°, codos a -118° y -112° de -123°, cuello de giro 57° de ±63°), porque nada en la tarea les pide una pose; es el mismo fallo de postura que se vio en la fase 4. La saturación de par medida es 0 y no incumple el criterio, pero con el robot real conviene una recompensa de pose para brazos, cintura y cuello, o un ajuste corto al reentrenar con el servo real. Ningún robot más de 2 s al tope.

**ONNX:** `policies/danny_fase5_sentarse_2026-11-07.onnx` (78 entradas, 23 salidas), **verificado** con `scripts/verify_policy_onnx.py --task Mjlab-SitDown-Danny`: diferencia máxima 1,2e-6 (límite 1e-4), `results/training_onnx_fase5_sentarse_suave.json`. Punto de control copiado a `/d/Claude/puntos_de_control/fase5_sentarse_suave_model_3549.pt`. El entrenamiento de la corrida 1 se paró solo en la iteración 2 092 (se cerró la sesión); la corrida 2 se reanudó desde el punto de control 2 050.

**No comprobado:** empujones durante la sentada, otras posturas iniciales que no sean la de pie de la fase 2, levantarse desde sentado (la política de la fase 4 solo se entrenó desde tumbado), el robot real, y si los servos de brazos, cintura y cuello pegados a sus topes calientan o se bloquean (la saturación medida es 0).


### Fase 5, tumbarse, corrida 1 (`fase5_tumbarse`), criterio escrito antes de entrenar (7 de noviembre)

**Tarea** (`training/src/danny_training/lie.py`, `Mjlab-LieDown-Danny`): empieza **sentado con las piernas estiradas** (la pose final de la política de sentarse: cadera de cabeceo -91°, rodilla 7°, tobillo -32°, el resto en la pose de pie, ruido de 0,05 rad, guiñada aleatoria; **no** copia los brazos, cintura y cuello pegados a sus topes de esa política, así que encadenar sentarse y tumbarse con la política real no está probado); órdenes cero; 10 s; sin terminación por caída. **Medido antes:** con acción cero el robot se cae solo hacia atrás y queda boca arriba y quieto en menos de 1 s (pelvis y tronco a 0,03 m, eje x del tronco hacia arriba), así que tumbarse en sí es trivial; lo que se pide es **hacerlo sin dejarse caer**. El giro de cabeceo del tronco en esa caída pasiva tiene un pico de **10,6 rad/s** (mediana de 256 robots; percentil 5 a 95: 9,3 a 11,6 rad/s). **Recompensas:** `lie_progress` (peso 4: cuánto apunta el eje x del tronco hacia arriba, de 0 a 1, menos la inclinación lateral pasada de 0,3 para que tumbarse de lado no valga), `lying_still` (peso 2: exp(-(velocidad lineal² + angular²) / 0,3²) por el avance), `pitch_rate_excess` (-20: cuadrado de lo que el tronco gira en cabeceo por encima de **2,0 rad/s**), y las de sentarse que sirvieron: `action_out_of_range` (-0,035), `dof_pos_limits` (-1), `action_rate_l2` (-0,1), `link_pair_contacts` (-1) y castigo de par al 80 % del bloqueo (-317). 5 000 iteraciones desde cero (`/d/Claude/fase5_tumbarse.sh`).

**Comprobación a las 2 500 iteraciones:** `lie_lying_fraction` (boca arriba, inclinación lateral de 0,3 o menos, velocidad lineal de 0,05 m/s o menos y angular de 0,1 rad/s o menos) del 80 % o más en el entrenamiento, y medida con `measure_lie.py` en 256 robots, el pico de cabeceo con mediana por debajo de 5 rad/s (la caída pasiva da 10,6). Si no, se para, se anota y no se alarga.

**Aprobación** (`scripts/measure_lie.py`, por escribir mientras entrena; 4 096 robots, 6,6 V y tensión aleatoria, sin empujones): **queda tumbado** el 95 % o más (boca arriba con el eje x del tronco a -0,9 o menos, inclinación lateral de 0,3 o menos, velocidad lineal de 0,05 m/s o menos y angular de 0,1 rad/s o menos, 3 s seguidos, en 10 s); **suave:** el 95 % o más con el pico de giro de cabeceo del tronco de 2,0 rad/s o menos (**2,0 es una estimación mía**: la bajada de 90° en 0,8 s; se ajusta con lo que dé la política si es físicamente inalcanzable, y entonces se anota como cambio del criterio); saturación (par sobre el 90 % del bloqueo): el 99 % o más de los robots sin pasar de 0,5 s seguidos mientras se tumba y con 5 % o menos estando tumbado, y ningún robot más de 2 s. 7,4 V informativo. **Fuera de esta corrida:** levantarse desde tumbado con la política de la fase 4 después de tumbarse (la 4 se entrenó con el robot a 1,5 rad de inclinación, no con este estado), encadenar con la política de sentarse y los empujones.

### Fase 5, tumbarse, corridas 1 y 2 y diagnóstico del pico (8 de noviembre)

**Corrida 1** (`fase5_tumbarse`, 5 000 iteraciones desde cero, `model_4999`): queda tumbado el **100 %** (6,6 V, tensión aleatoria y 7,4 V), saturación **100 %** de los robots sin pasar de 0,5 s al tope y 100 % con 5 % o menos estando tumbado; pero con pico de cabeceo ≤ 2,0 rad/s solo el **21,8 %** (6,6 V), **18,3 %** (aleatoria) y 11,9 % (7,4 V, informativo); mediana del pico 2,27 / 2,34 / 2,50 rad/s (`results/training_tumbarse_fase5_tumbarse_*.json`). **Corrida 2** (`fase5_tumbarse_suave`, un solo cambio: `pitch_rate_excess` de -20 a -200, 1 500 iteraciones desde `model_4999`; el contador sigue en 6 498, por eso el checkpoint es `model_6498.pt`): tumbado 100 %, saturación 100 %, pico ≤ 2,0 rad/s **50,7 %** (6,6 V), **48,2 %** (aleatoria), 37,5 % (7,4 V); mediana 1,99 / 2,03 / 2,20 rad/s. Total 6 500 iteraciones: **pasa de las 5 000 que fijé como tope**, así que se paró. **No cumple el criterio de suavidad (95 %).** Cifras de esta corrida: `results/training_tumbarse_fase5_tumbarse_suave_{6.6V,aleatoria,7.4V}.json`.

**Diagnóstico del pico** (`scripts/diagnose_lie_pitch_peak.py`, sin entrenar, 4 096 robots, `model_6498`, 6,6 V y aleatoria; `results/training_tumbarse_pico_fase5_tumbarse_suave_{6.6V,aleatoria}.json`, vídeo de los 2 robots con el pico más alto en `results/videos/fase5_tumbarse_suave_6.6V_robot3084_*.mp4` y `..._robot3238_*.mp4`). Cifras a 6,6 V (aleatoria casi igual):
- **Cuándo:** el pico de cabeceo cae en el paso 1 (0,02 s) en el **100 %** de los robots (el arranque, primer medio segundo), **95,2 % «antes»** del primer contacto del tronco con el suelo, 4,8 % «golpe», **0 % rebote**. El primer contacto de tronco, cuello o cabeza con el suelo es a los **0,42 s** (mediana). Si se mide el cabeceo después del primer medio segundo, el **99,98 %** cumple 2,0 rad/s. Es decir: el pico del criterio es un **transitorio de arranque**, no el golpe ni la bajada. **Causa del transitorio: no determinada** (el cabeceo vale 1,5 rad/s de mediana ya en el primer paso con el robot en reposo; lo más probable es la primera acción y el apoyo inicial, **no medido**).
- **Hallazgo más importante:** el criterio solo mira el eje de cabeceo, y el robot **se tumba girando en otros ejes**. El mayor giro del episodio es de **guiñada de la pelvis en el 100 % de los robots, 15,5 rad/s de mediana** (percentil 95: 17,1; máximo 18,6), y el de balanceo 8,3 rad/s de mediana; el de cabeceo es 2,0. Todo el movimiento dura unos 0,45 s: el eje x del tronco apunta ya hacia arriba (gravedad x ≤ -0,96) a los 0,36 s y a los 0,5 s está quieto. Los ejes son los de la pelvis (x balanceo, y cabeceo, z guiñada). Con los giros por encima, el castigo de cabeceo -200 no cambió nada del movimiento real: el pico de cabeceo del criterio está en el arranque y el robot cae dando una vuelta de lado. Interpretación (mía, no medida paso a paso): rueda sobre un costado y luego gira hasta quedar de espaldas.
- **Velocidad al primer contacto con el suelo:** tronco (`torso_1`) **0,57 m/s de mediana**, percentil 95 0,72, máximo 0,90 (bajada: 0,54 / 0,68 / 0,86). Cabeza: solo 5 de 4 096 robots la tocan (0,78 m/s). Cuello: ninguno. A tensión aleatoria: tronco 0,56 / 0,72 / 0,89 m/s.
- **Criterios alternativos medidos (fracción de robots, 6,6 V):** cabeceo solo antes del contacto 52,6 %; sin la ventana del golpe 52,6 %; sin el primer medio segundo 99,98 %. Ninguno es un buen criterio: el cabeceo no es el giro que importa.
- **No comprobado:** la causa del transitorio de arranque, qué parte del giro de guiñada es del robot entero y cuál de la pelvis, empujones, otras posturas iniciales, encadenar con la política de sentarse y el robot real.

**Decisión pendiente del dueño** (no se entrena más hasta que decida): si tumbarse debe medirse con el **giro total** de la pelvis (norma, no solo cabeceo) y/o con la **velocidad de impacto del tronco** (hoy 0,57 m/s de mediana); con qué límite; y si se reentrena con ese castigo (el castigo actual solo sobre cabeceo lo esquiva girando en otros ejes). Cambiar el criterio escrito es decisión suya.

### Fase 5, tumbarse: cambio del criterio de suavidad (8 de noviembre) y resultado con `model_6498`

**Cambio del criterio escrito, decidido por el dueño el 8 de noviembre.** Se sustituye «pico de giro de cabeceo del tronco de 2,0 rad/s o menos en el 95 %» por: (1) el **95 % o más** con el tronco tocando el suelo a **0,8 m/s o menos** (había: mediana 0,57, percentil 95 0,72; 0,8 m/s equivale a dejarlo caer desde unos 3 cm); (2) el **99 % o más** sin que la cabeza toque el suelo; (3) **ningún giro de la pelvis de más de 3 rad/s en ningún eje** una vez pasado el reinicio. Siempre con 4 096 robots, a 6,6 V y con tensión aleatoria; el resto del criterio no cambia (tumbado 95 %, saturación 99 % con 5 % tumbado y ninguno más de 2 s). **Motivo:** el pico de cabeceo salía del paso 1, al soltar al robot en el reinicio, y no de lo que hace la política; lo que importa para el robot real es la velocidad del impacto. Velocidad del tronco al primer contacto = máximo de los 3 pasos previos, contacto incluido (`danny_training/lie_measure.py`).

**Reinicio arreglado** (`lie.py`, `reset_seated`): el reinicio anterior dejaba **los 256 robots penetrando el suelo** en el paso 0 (distancia mínima de contacto mediana -2,7 cm, mínima -2,8 cm; pies, espinillas, tobillos y sobre todo los antebrazos). Ahora la pelvis se sube hasta que el punto más bajo queda a 1 mm del suelo: en el paso 0, **0 robots penetran** (no hay ningún contacto con el suelo). Esto cambia el entorno en el que se entrenó la política (se entrenó con el reinicio penetrando).

**Medida con `model_6498`** (`scripts/measure_lie.py`, 4 096 robots, `results/training_tumbarse_criterio_nuevo*_fase5_tumbarse_suave_*.json`). Con 10 pasos sujetando la pose antes de contar (mi elección, antes de ver nada), a 6,6 V / tensión aleatoria:
- Tumbado 100 % / 100 % (**pasa**); tronco a 0,8 m/s o menos al tocar el suelo **97,9 % / 96,8 %** (**pasa**; mediana 0,57, percentil 95 0,73 / 0,76 m/s); cabeza sin tocar **93,6 % / 92,4 %** (**no pasa**, pide 99 %; la cabeza toca a 1,15 m/s de mediana); saturación 100 % en las tres condiciones (**pasa**); giro de la pelvis de 3 rad/s o menos en todos los ejes: **0 % de los robots** (**no pasa**).
- **El giro de guiñada de 15 rad/s NO era del paso 1:** su pico es de **14,1 rad/s de mediana a los 0,38 s** (percentil 95 15,7; máximo 18,3), de guiñada de la pelvis en el 100 % de los robots, y el balanceo llega a 7,4 rad/s; el cabeceo, 2,4. Sigue después del reinicio, así que **el arreglo no lo quita**. Vídeo del robot 206 (guiñada de 18,27 rad/s a los 0,32 s): `results/videos/criterio_nuevo_fase5_tumbarse_suave_6.6V_robot206_giro18.27_t0.32s_guinada.mp4`.
- **Hallazgo sobre el «quieto unos pasos»:** la pose sentada no se sostiene sola (se cae hacia atrás sin política). Tras 10 pasos sujetándola, los robots **no están quietos** (velocidad mediana 0,15 m/s, máxima 0,47; el eje x del tronco ya a -0,15) y la cabeza toca en el 6,4 % de los robots. Variantes medidas a 6,6 V para separar el efecto (**no elegí la mejor a posteriori; las tres se anotan**): **0 pasos** (empieza apoyado y quieto, velocidad 0): tronco 99,95 %, cabeza 99,98 %, giro de la pelvis 0 %; **3 pasos** (velocidad mediana 0,06 m/s): tronco 99,93 %, cabeza 99,88 %, giro 0 %. Es decir: **las condiciones 1 y 2 se cumplen con 0 y 3 pasos y fallan con 10**; la 3 (giro de la pelvis, 14,9 rad/s de mediana a los 0,36 s) falla en las tres. Depende de cuánto se deje caer sola la pose antes de que actúe la política; el criterio no fija cuántos pasos.
- **Conclusión: tumbarse NO queda aprobado** (falla el giro de la pelvis en todos los casos, y la cabeza con 10 pasos). Un giro de cabeceo puro de 90° en 0,8 s da unos 2 rad/s, así que el límite de 3 rad/s es alcanzable; es la política la que se tumba retorciéndose (guiñada y balanceo), un movimiento que ningún castigo de cabeceo vigilaba.

**Cadena sentarse + tumbarse** (`scripts/measure_sit_lie_chain.py`, 4 096 robots; sentarse 10 s con su política y, sin reinicio, tumbarse 10 s con la suya desde donde queda el robot; `results/training_cadena_sentarse_tumbarse_{6.6V,aleatoria}.json`). **No pasa.** Sentarse pasa todos sus criterios (100 % sentado, bajada 97,7 %, saturación 100 %). Tumbarse desde ahí: tumbado 100 % y saturación 100 %, pero tronco a 0,8 m/s o menos **88,9 % / 91,1 %** (pide 95 %), cabeza sin tocar **82,2 % / 82,7 %** (pide 99 %; la cabeza toca a 1,16 m/s de mediana) y giro de la pelvis 0 % (12,1 rad/s de mediana de pico a 0,52 s). Es peor que con el inicio limpio porque la política de tumbarse se entrenó sin los brazos, la cintura y el cuello donde los deja la de sentarse.

**No comprobado:** empujones, otras posturas iniciales, levantarse después de tumbarse con la política de la fase 4, el robot real, ni una política de tumbarse reentrenada con el criterio nuevo (el dueño pidió no reentrenar). **Pendiente de decisión del dueño:** reentrenar tumbarse con castigo del giro total y del impacto de cabeza, empezando de la pose real con que acaba sentarse; la fase 6 no empieza hasta que lo confirme.

### Fase 5, tumbarse, corrida 3 (`fase5_tumbarse_desde_sentado`), criterio escrito antes de entrenar (8 de noviembre)

**Orden del dueño:** reentrenar tumbarse con castigo, porque se deja caer y no baja de forma controlada. **Causa medida** (corridas 1 y 2): el castigo solo vigilaba el cabeceo de la pelvis y la política se tumba retorciéndose (guiñada de 14 rad/s y balanceo de 7 rad/s de pico), y desde la pose real con que acaba sentarse la cabeza toca el suelo en el 18 % de los robots.

**Cambios de la corrida 3** (varios a la vez, porque es un reentrenamiento nuevo con el criterio nuevo; la corrida 2 no se toca): (1) **inicio** = uno de los 4 096 estados reales con que acaba la política de sentarse (`scripts/export_sit_end_states.py` → `training/data/sit_end_states.pt`: todos quedaron sentados y quietos; con brazos, cintura y cuello donde los deja esa política: p. ej. cintura 0,93 rad y cuello 1,01 rad de mediana), guiñada aleatoria, ruido de 0,02 rad, apoyado sin penetrar; el inicio limpio sigue disponible con `DANNY_LIE_START=clean` (así se midieron las corridas 1 y 2); (2) se quita `pitch_rate_excess` y se ponen **`pelvis_rate_excess`** (peso -2: suma en los 3 ejes del cuadrado de lo que el giro pasa de **2,0 rad/s**, recortado a 5; el criterio es 3 rad/s), **`trunk_speed_excess`** (peso -20: cuadrado de lo que la velocidad del tronco pasa de **0,5 m/s**, recortado a 1; el criterio es 0,8 m/s al impacto) y **`head_floor_contacts`** (peso -2 por contacto de cuello o cabeza con el suelo); el resto igual (`lie_progress` 4, `lying_still` 2, `action_out_of_range`, `dof_pos_limits`, `action_rate_l2`, `link_pair_contacts`, par al 80 %). Los pesos son estimaciones mías para que el castigo sea comparable al avance (4 + 2 por paso); se ajustan solo con lo medido. Parte de `fase5_tumbarse_suave/model_6498.pt`, hasta **5 000 iteraciones nuevas** (el tope de 5 000 de las corridas 1 y 2 ya está gastado; este es nuevo, por orden del dueño), `DANNY_DR_STAGE=2`.

**Comprobación a las 1 500 iteraciones** (`measure_sit_lie_chain.py`, 1 024 robots, 6,6 V, sobre el último punto de control): tumbado 95 % o más, mediana del pico de giro de la pelvis **por debajo de 8 rad/s** (ahora 12 a 14), y cabeza sin tocar el 90 % o más (ahora 82 %). Si no, se para, se anota y no se alarga.

**Aprobación:** la **cadena** (`measure_sit_lie_chain.py`: sentarse con su política y, sin reinicio, tumbarse con la nueva) cumple **todo** lo escrito, con 4 096 robots, a 6,6 V y con tensión aleatoria: sentarse (se sienta 95 %, bajada a 0,6 m/s o menos 95 %, no cae 98 %, saturación 99 % y ninguno más de 2 s) y tumbarse (tumbado 95 %, tronco a 0,8 m/s o menos al tocar el suelo 95 %, cabeza sin tocar 99 %, ningún giro de la pelvis de más de 3 rad/s en ningún eje, saturación 99 % con 5 % tumbado y ninguno más de 2 s). 7,4 V informativo. Si no cumple, se anota qué falla y no se cambia el criterio sin el dueño. **Fuera de esta corrida:** empujones, otras posturas iniciales, levantarse después de tumbarse y el robot real.

### Fase 5, tumbarse, corrida 3: resultado (8 de noviembre) — NO cumple el giro de la pelvis

Entrenamiento terminado (5 000 iteraciones nuevas, `fase5_tumbarse_desde_sentado/model_11497.pt`, no copiado a `puntos_de_control` porque no está aprobado). **Cadena sentarse + tumbarse, 4 096 robots** (`results/training_cadena_sentarse_tumbarse_model_11497_{6.6V,aleatoria}.json`): sentarse pasa todo; tumbarse: tumbado 100 %, tronco a 0,8 m/s o menos 100 %, cabeza sin tocar 99,98 % / 100 %, saturación 100 %; **giro de la pelvis de 3 rad/s o menos en todos los ejes: 3,3 % (6,6 V) y 6,1 % (aleatoria), pedido 100 % → la cadena NO pasa.** Falla solo el cabeceo: pico de mediana 3,53 / 3,57 rad/s (máximo 4,6 / 5,2); guiñada 2,0 y balanceo 2,2. Antes (`model_6498`): cabeza 82 %, tronco 89 %, giro de 12 rad/s. Pico a los 0,06 s de mediana. Medidas intermedias: 1 500 iteraciones (`..._comprobacion_1500_6.6V.json`, 6,6 %) y `model_9950` (`..._final_*.json`, 0,8 % / 1,6 %; medido por error en vez del último). **No comprobado:** si el pico de cabeceo a los 0,06 s es del cambio de política en la cadena y no de la caída; empujones, el robot real. Criterio sin cambiar; la decisión (seguir entrenando, analizar el pico o relajar el límite) es del dueño. La fase 6 no empieza hasta que lo confirme. Sin commit.

**Medida del pico de cabeceo (8 de noviembre, `model_11497`, 4 096 robots, 6,6 V, `results/training_tumbarse_model_11497_inicio_sentado_6.6V.json` y `training_tumbarse_pico_model_11497_inicio_sentado_6.6V.json`).** Sin cadena, con el inicio de sentado del propio entorno de tumbarse: pico de cabeceo de la pelvis 3,0 rad/s de mediana (p95 3,7, máximo 4,6), cumple el 50,8 % de los robots; balanceo 2,2 y guiñada 2,1 (cumplen). El pico ocurre a los **0,04 s (paso 2)** y el tronco toca el suelo a los 0,6 s con 0,27 m/s de mediana (máximo 0,41): **el pico no es del impacto, es del arranque.** En la cadena el pico sube a 3,5 rad/s, es decir, el cambio de política añade unos 0,5 rad/s. Aparte, cuello y cabeza giran en guiñada a unos 12 rad/s, pero el criterio solo mide la pelvis. **No comprobado:** cuánto del pico es la pose sentada soltándose sola (no se sostiene) y cuánto la primera acción de la política (falta medir con acción cero).

### Fase 5, tumbarse: de dónde sale el pico de cabeceo, velocidad de las articulaciones y par en cuclillas (8 de noviembre)

**Modelo de tumbarse de la corrida 3:** `fase5_tumbarse_desde_sentado/model_11497.pt` (es el reentrenamiento; copia en `puntos_de_control/fase5_tumbarse_desde_sentado_model_11497.pt`, ONNX `policies/danny_fase5_tumbarse_desde_sentado_2026-11-08.onnx`, diferencia máxima 1,3e-6, `results/training_onnx_fase5_tumbarse_desde_sentado.json`). **NO está aprobado**: la cadena falla el giro de la pelvis. Se sube a git como resultado de la corrida, no como política a usar.

**Pico de cabeceo, sin política, con la pose sentada como objetivo** (`scripts/measure_sit_pose_hold.py`, 4 096 robots, 6,6 V, inicio de tumbarse, objetivo de cada articulación fijo en la pose inicial; `results/training_sentarse_pose_sujeta_6.6V.json`): pico de cabeceo de la pelvis 2,1 rad/s de mediana (p95 2,8, máx 3,5) en los pasos 0 a 2, luego 0,28 rad/s o menos; después el robot queda quieto (velocidad 0, eje x del tronco -0,07). Cumple el 97,7 % de los robots. **La pose se sostiene sola con el servo sujetándola**; ese transitorio de 2 rad/s es el del reinicio (el robot se suelta a 1 mm del suelo), no de que la pose se caiga. Balanceo 0,3 y guiñada 0,6 rad/s.

**Pico en la cadena, hora y eje** (`measure_sit_lie_chain.py`, `results/training_cadena_sentarse_tumbarse_pico_model_11497_{6.6V,aleatoria}.json`): al cambiar de política el robot está **completamente quieto** (giro de la pelvis de 0,000 rad/s los 5 pasos previos). El cabeceo de la mediana sube 0,15 / 1,31 / 2,71 / **3,52** / 2,78 / 2,21 rad/s en los pasos 0 a 5 tras el cambio: el pico es de **cabeceo en el 100 % de los robots, a los 0,06 s (paso 3)** del cambio; el balanceo llega a 2,0 y la guiñada a 1,3 en esos primeros pasos. Pasado el primer medio segundo el cabeceo es de 2,3 rad/s de mediana (p95 2,5). Es decir, con el robot quieto, **el pico de 3,5 rad/s lo provoca la política en sus primeros 3 pasos**; la pose sostenida sola no lo explica (2,1 en el arranque del reinicio, que no es el mismo estado). **No comprobado:** sujetar la pose sentada fija dentro de la cadena (sin reinicio) para separar del todo la parte de la política.

**Velocidad de las articulaciones frente al servo** (cadena, 6,6 V; `joint_speed` en los mismos JSON). Velocidad en vacío: modelo 10,26 rad/s (`vin * 0,97 / kt`, `kt` = 0,6238), servo real 10,6 rad/s (11,9 a 7,4 V, escalado; hoja del servo). **El modelo sí limita la velocidad**: tiene fuerza contraelectromotriz (par cero a la velocidad en vacío y frenado de `kt²/R` = 0,079 N·m·s/rad por encima). **Pero no la limita del todo, y su límite explícito `max_velocity` = 100 rad/s no hace nada**: una articulación movida por algo externo pasa de la velocidad en vacío. Cuello: guiñada 7,5 a 8,5 rad/s de pico mediano (máx 9,2) y cabeceo 5 a 6 (máx 6,7), ningún robot por encima del vacío: **el 12 rad/s que vi antes era el giro del cuerpo del cuello y la cabeza, no de la articulación**. Articulaciones por encima del vacío del servo a 6,6 V: rodilla izquierda al sentarse (mediana de pico 10,4, máx 12,4 rad/s, el 40,4 % de los robots), rodilla derecha (4,0 %, máx 11,3) y el hombro izquierdo (balanceo) al tumbarse (máx 11,0, 21,5 %); con tensión aleatoria (hasta 8,4 V) la rodilla llega a 13,4 y el hombro a 13,3. Son excesos de hasta el 17 % sobre el vacío del servo, no de órdenes de magnitud. **No comprobado:** si el servo real, con su límite de velocidad del firmware, se comportaría igual; el brazo y el cuello van a 8 a 10 rad/s, cerca del límite, durante toda la caída.

**Par estático en cuclillas** (`scripts/check_squat_torque.py`, `danny_training/static_pose.py`, `results/training_cuclillas_par_estatico.json`; pies planos, centro de masas sobre las suelas, 6,6 V, peso repartido entre los dos pies, barriendo la inclinación del tronco; 80 % del bloqueo = 0,65 N·m): pelvis 0,228 m → 11 % del bloqueo (inclinación 10°); 0,200 m → 20 % (20°); 0,190 / 0,180 / 0,170 m → 21 / 23 / 24 %; **0,160 m → 27 %** (rodilla 27 %, cadera 21 %, tobillo 8 %). **0,150, 0,100 y 0,070 m: ninguna pose con pies planos dentro del rango de las articulaciones** (la cadera pasa de -100° o el tobillo de -45°). El límite es el rango, no el par: la bajada por debajo de unos 0,16 m tiene que apoyar las manos atrás o sentarse sobre la pelvis. Estático; no incluye el par dinámico ni apoyar las manos. La política de agacharse de la fase 3 solo bajaba a 0,178 m (orden de -0,05 m) y no saturaba (`results/training_torque_fase3_agacharse_quieto_h-0.05_6.6V.json`).

**Decisiones del dueño (8 de noviembre) y pendiente:** sentarse queda sin aprobar (arriba). Criterio nuevo de sentarse propuesto por el dueño (bajada de 1 s o más, pelvis a 0,3 m/s o menos en el 95 %, tronco a 3 rad/s o menos en el 99 % en cualquier eje, brazos a 20° o menos de reposo; resto igual), **sin escribir como definitivo hasta ajustarlo con el cálculo de par** (17 cm en 1 s son 0,17 m/s de media, por debajo del 0,3 pedido; falta decidir cómo se baja por debajo de 0,16 m, donde con los pies planos no hay pose). Tumbarse se reentrena después, partiendo de donde termine el nuevo sentarse; la prueba que aprueba es la cadena. La fase 6 no empieza. Criterio sin cambiar.

### Fase 5, sentarse (segundo intento): criterio escrito antes de entrenar (8 de noviembre)

**Origen:** el dueño retiró la aprobación de sentarse tras ver los vídeos (se deja caer: la pelvis baja 17 cm en unos 0,4 s) y propuso este criterio, que se escribe con las condiciones de abajo. Cálculo previo hecho (arriba, «Par estático en cuclillas»): con los pies planos el par sobra hasta 0,16 m de pelvis (27 % del bloqueo a 6,6 V, límite 80 %) y por debajo de 0,15 m no hay pose de pies planos dentro del rango de las articulaciones, así que **el último tramo de la bajada (de unos 0,16 m a 0,07 m) tiene que apoyar las manos atrás** (decisión del dueño, 8 de noviembre) o inclinarse; el apoyo de las manos no se ha calculado y lo decide el entrenamiento con la saturación de par incluida.

**Se mide** con `scripts/measure_sit.py` sobre 4 096 robots, a **6,6 V y con tensión aleatoria** (7,4 V informativo), aleatorización completa de la etapa 3, sin empujones, 10 s, y se graba un vídeo del caso típico y del peor. **Pasa si cumple todo:**
1. **Se sienta** el 95 % o más (definición de siempre: pelvis a 0,07 m o menos, tronco a 35° o menos, ni tronco, cuello ni cabeza en el suelo, velocidad media de 0,05 m/s o menos, 3 s seguidos).
2. **La bajada dura 1 s o más** en el 95 % o más: tiempo desde que la pelvis baja de 0,218 m (la altura de pie menos 1 cm) hasta que llega a 0,07 m. *Porcentaje elegido por mí; el dueño no lo fijó.*
3. **La pelvis baja a 0,3 m/s o menos** en el 95 % o más (máximo de la velocidad vertical de bajada, medida como hasta ahora desde 0,218 m). Sustituye al límite de 0,6 m/s.
4. **El tronco gira a 3 rad/s o menos en cualquier eje** (balanceo, cabeceo y guiñada de la pelvis) en el 99 % o más, **desde que empieza a bajar** (pelvis bajo 0,218 m) hasta el final, para no contar la caída del robot al nacer.
5. **Al terminar, los brazos quedan a 20° o menos de la pose de reposo** en el 95 % o más: las ocho articulaciones de los brazos, cada una a 20° o menos de su valor en la pose de pie (brazos a los lados: hombro de cabeceo 0, balanceo ±0,14 rad, guiñada 0, codo -0,3 rad), en el último paso de los 10 s. *Porcentaje elegido por mí.*
6. **No cae** (tronco nunca a más de 70°) el 98 % o más.
7. **Saturación** como hasta ahora: ninguna articulación sobre el 90 % del bloqueo más de 0,5 s seguido bajando en el 99 %, sobre el 90 % estando sentado más del 5 % del tiempo en el 99 %, y ninguna más de 2 s seguidos en el 100 %.

**No se añade como criterio** (informativo): la velocidad de cada articulación frente a la del servo (la rodilla izquierda pasaba del vacío en el 40 % de los robots con `model_3549`); si el dueño lo quiere como criterio, se escribe aparte. **Corrida:** punto de partida y número de iteraciones sin fijar todavía (hasta 5 000 iteraciones nuevas, tope de la fase); se escribirá aquí antes de entrenar. Después se reentrena tumbarse partiendo de donde termine el nuevo sentarse, y la prueba que aprueba es la cadena. **La medida de partida** (`model_3549` con este criterio) se hace antes de entrenar, para ver cuánto falla cada punto.

**Medida de partida: `model_3549` con el criterio nuevo** (`measure_sit.py`, 4 096 robots, `results/training_sentarse_base_criterio_nuevo_{6.6V,aleatoria}.json`; mismas cifras en las dos tensiones). **No cumple.** Pasan: se sienta 100 %, no cae 100 %, saturación 100 %. Fallan, todos al 0 %: **bajada de 1 s o más** (dura 0,36 s de mediana, p5 0,34, p95 0,38); **pelvis a 0,3 m/s o menos** (pico de 0,50 m/s de mediana, p95 0,59); **tronco a 3 rad/s o menos** (pico de balanceo 4,9, cabeceo 3,5 y guiñada 2,7 rad/s de mediana; máximo 7,1 de balanceo); **brazos a 20° o menos del reposo** (desviación máxima de 101° de mediana al terminar, es decir, los deja lanzados hacia arriba). El criterio es entonces exigente en cuatro puntos a la vez; hay que bajar la velocidad de bajada a casi la mitad y alargarla a casi el triple. Sin commit de esta medida.

**Tercera corrida de sentarse, 9 de noviembre: no cumple, se para (la anterior a esta decisión).** `fase5_sentarse_lenta` (desde cero, 5 000 iteraciones previstas, penalizaciones de giro de tronco y de brazos fuera de reposo) se paró a la iteración 3 800 de 5 000 porque la recompensa llevaba plana (40,8-41,1) y `sit_seated_fraction` en 0 %. Medida del último punto de control (`model_3800`, copiado a `puntos_de_control/fase5_sentarse_lenta_model_3800.pt`; `scripts/measure_sit.py`, 1 024 robots, 6,6 V; `results/training_sentarse_lenta_model_3800_6.6V.json`): **no cumple**. Se queda con la **pelvis a 0,0905 m** (p10 0,0904, p90 0,0907; sentarse exige 0,07), tronco a 21°, con **pies y los dos antebrazos tocando el suelo** en el 100 % de los robots, y los brazos a **55,5° de mediana (máximo 58°) de su reposo**. Pasan: tronco a 3 rad/s o menos (pico mediano 2,1 balanceo, 2,2 cabeceo, 1,8 guiñada), no cae, saturación en movimiento. Fallan: se sienta 0 %, bajada de 1 s o más (no llega), pelvis a 0,3 m/s (75 %; pico mediano 0,285 m/s), brazos a 20° del reposo (0 %), saturación sentado. **Causa probable (no demostrada):** 0,0905 m es 0,5 mm por encima del tope de 0,09 m a partir del cual castigaba `arms_off_rest_when_seated`; la política se quedó justo fuera de la zona penalizada con los brazos apoyados. Ese término se retira (ver abajo).

**Cómo se sienta (decisión del dueño, 9 de noviembre):** baja en cuclillas despacio, se echa hacia atrás, apoya las manos detrás y baja la pelvis frenando con los brazos. Motivo: por debajo de 0,16 m no hay pose posible con los pies planos (`results/training_cuclillas_par_estatico.json`).

**Cálculo previo con el modelo** (`scripts/check_sit_hands.py`, `StaticPose.solve_hands` en `static_pose.py`; `results/training_sentarse_manos_par_estatico_edge.json`; estático, las dos manos simétricas con balanceo de hombro 0, las cajas del antebrazo como mano, peso de las manos repartido por igual, 6,6 V, límite 80 % del bloqueo = 0,65 N·m; pasos de 1 cm de pelvis y de 5° de inclinación; articulaciones dentro de su rango):
- **Altura de la pelvis a la que las manos llegan al suelo: 0,13 m** (la más alta con pose posible; tu estimación era 0,10-0,12). El hombro está 8,5 cm sobre la pelvis, como suponías. Solo existe pose con **el tronco echado 65-70° hacia atrás**, con el hombro en su tope de 45° hacia atrás, la cadera en su tope de extensión (+16 a +20°), el tobillo en su tope (+40°) y la rodilla de 123 a 130°. **Con los pies planos no hay ninguna pose** (barrido completo, `results/training_sentarse_manos_par_estatico_flat.json`: pelvis de 0,20 a 0,06 m cada 1 cm, tronco de -80° a 0° cada 5°, manos al 30, 40 y 50 % del peso: ninguna pose en ningún caso), solo con el pie apoyado en el borde de la suela. Es una pose en los límites de varias articulaciones, casi sin margen.
- **Par con las manos aguantando el 30-50 % del peso** (fracción del bloqueo, límite 0,8): a 0,13 m con 40 % en las manos (inclinación -70°): rodilla 49 %, cadera 31 %, tobillo 22 %, **hombro 16 %, codo 7 %**; con 30 %: rodilla 59 %. A 0,12 m con 40 % (-65°): rodilla 52 %, cadera 34 %, tobillo 24 %, hombro 20 %, codo 12 %; con 30 %: rodilla 61 %. **Todo bajo el 80 %, y hombros y codos con mucho margen**; el que más carga es la rodilla. Con el 50 % no hay pose a 0,13 m ni a 0,12 m (el centro de masas queda fuera). Por debajo de 0,09 m no hay pose con el pie en punta o talón: ahí la pelvis ya se apoya en el suelo y el estático no la cubre.
- **Caída entre 0,16 m y el contacto de las manos (0,13 m): 3 cm.** En caída libre llegarían a **0,77 m/s**; para tocar a 0,4 m/s o menos la caída libre admite solo **8 mm**. Es decir: **de 0,16 m a 0,13 m el frenado tiene que ser activo (piernas), no dejarse caer**; el criterio de 0,3 m/s en ese tramo es el que lo exige. El par dinámico del frenado no está calculado.
- **Con qué apoyo de pie salen las poses válidas: todas con el talón** (el vértice más bajo de la suela es el trasero, 1,5 a 2 cm detrás del centro de la suela; ninguna con la punta). Hay estas 6 poses en todo el barrido (`..._edge.json`; tabla de `scripts/describe_sit_hands_poses.py` en `results/training_sentarse_manos_poses_validas.json`; 6,6 V; fracciones del bloqueo, límite 0,8):

| Pelvis | Manos | Tronco atrás | Hombro | Codo | Rodilla | Cadera | Tobillo |
|---|---|---|---|---|---|---|---|
| 0,13 m | 30 % | 70° | 11 % | 5 % | 59 % | 36 % | 26 % |
| 0,13 m | 40 % | 70° | 16 % | 7 % | 49 % | 31 % | 22 % |
| 0,12 m | 30 % | 70° | 15 % | 7 % | 61 % | 39 % | 27 % |
| 0,12 m | 30 % | 65° | 14 % | 9 % | 61 % | 37 % | 27 % |
| 0,12 m | 40 % | 65° | 20 % | 12 % | 52 % | 34 % | 24 % |
| 0,06 m | 50 % | 0° | 36 % | 21 % | 33 % | 39 % | 22 % |

  **Todas bajo el 80 %** (el mayor es la rodilla al 61 %; hombros 11-36 % y codos 5-21 %). **Las dos de arriba (0,13 y 0,12 m) piden el tronco a 65-70° hacia atrás, más de 60°**, con el tobillo en su tope (+40°) y, salvo una, la cadera o el hombro también en el suyo. **De 0,11 m a 0,07 m no hay ninguna pose estática** (el pie en el borde de la suela no sostiene al robot ahí: ya se apoya la pelvis) y a 0,06 m solo con el tronco derecho (0°) y el 50 % del peso en las manos. **Este cálculo no cubre un tronco final de 30-50°** (lo que el dueño espera sentado): no hay pose estática a 0,07 m con esa inclinación; lo resolvería el entrenamiento con la pelvis ya apoyada, sin cálculo detrás.
- **Cambio del criterio escrito, decisión del dueño, 9 de noviembre (tras revisar el commit 4531934): el límite de «no cae» pasa de 70° a 80°**, en el punto 7 del criterio y en el corte del episodio del entrenamiento (`fell_over` de mjlab, 70° heredado; ahora `FALL_TILT_RAD` en `sit.py`; `measure_sit.py` usa 80° por defecto con `--fall-tilt-deg`). **Motivo:** el barrido muestra que apoyar las manos pide el tronco a 65-70° hacia atrás, y con el corte en 70° la política no puede explorar ese movimiento (margen de 0 a 5°). **«Se sienta» no cambia:** sigue pidiendo el tronco a 35° o menos al terminar; el dueño espera 30-50° sentado con las manos atrás, y un robot que se quede apoyado a 40-50° **no cuenta como sentado** con esa definición: si es lo que sale, se manda el vídeo y el dueño decide. Solo cambia la corrida de sentarse: caminar, levantarse y tumbarse conservan su criterio y su corte.

**Criterio (propuesta del dueño, ajustada con esos números; sustituye al del 8 de noviembre, que se retiró el 9):** se mide con `scripts/measure_sit.py` sobre 4 096 robots, a 6,6 V y con tensión aleatoria, aleatorización completa de la etapa 3, sin empujones, 10 s, y vídeo del caso típico y del peor (`--videos`). Pasa si cumple todo:
1. **Se sienta** el 95 % o más (definición de siempre: pelvis a 0,07 m o menos, tronco a 35° o menos, ni tronco, cuello ni cabeza en el suelo, velocidad media de 0,05 m/s o menos, 3 s seguidos). **Estar apoyado en las manos se permite:** manos y brazos pueden tocar el suelo.
2. **La bajada completa, desde que la pelvis baja de 0,218 m (pie menos 1 cm) hasta que llega a 0,07 m, dura 0,6 s o más** en el 95 % o más. **Cambio del criterio escrito, decisión del dueño, 10 de noviembre: 0,6 s en vez de 1 s.** Motivo: el segundo no tenía en cuenta el tamaño; para un mismo movimiento el tiempo crece con la raíz de la altura. Danny mide 0,4 m y una persona 1,7 m: factor √(0,4/1,7) = 0,49; si una persona tarda de 1,2 a 2 s, Danny tardaría de 0,6 a 1,0 s (1,2 × 0,49 = 0,58; 2 × 0,49 = 0,97). **Corrección anterior del criterio escrito, error del dueño, 9 de noviembre (de 0,218-0,16 m a la bajada completa):** antes decía «de 0,218 a 0,16 m». Motivo: 1 s para esos primeros 6 cm equivale a 0,06 m/s, demasiado lento y nada natural; la propuesta pensaba en la bajada entera. (Se mide desde 0,218 m porque el robot nace entre 1 y 5 cm por encima de la altura de pie.) El tiempo de 0,218 a 0,16 m se sigue guardando como informativo.
3. **La pelvis baja a 0,3 m/s o menos de 0,218 a 0,16 m** (sin cambios) en el 95 % o más.
4. **Las manos tocan el suelo antes que la pelvis** en el 95 % o más (un robot cuya pelvis no llega a tocar el suelo cumple esto sin más: `model_3549` lo cumple así, su pelvis nunca toca).
5. **La pelvis toca el suelo a 0,4 m/s o menos** en el 95 % o más. **Medida corregida el 9 de noviembre** (antes se cumplía sin medir nada si la pelvis no tocaba el suelo, y en `model_3549` y `model_7700` no lo toca en ningún robot): se mide la velocidad vertical de bajada (en el paso anterior) **en el primer contacto con el suelo de la pelvis, los muslos o las espinillas** (las espinillas se añaden el 10 de noviembre por decisión del dueño: el 9,6 % de `model_10699` con tensión aleatoria que no tocaba con los muslos se sienta sobre las espinillas y ninguno baja a más de 0,312 m/s; no es un golpe) y **en el instante en que la pelvis baja de 0,07 m**; vale si las dos son de 0,4 m/s o menos, y un robot en el que alguna no ocurre no cumple.
6. **El tronco gira a 3 rad/s o menos en cualquier eje** en el 99 % o más, desde que empieza a bajar.
7. **No cae** el 98 % o más (límite de inclinación: **80°**, cambio del dueño del 9 de noviembre en este criterio; antes 70°. El contacto de las manos pide 65-70° según el cálculo de arriba).
8. **Saturación** como hasta ahora: ninguna articulación sobre el 90 % del bloqueo más de 0,5 s seguido bajando en el 99 %, sobre el 90 % estando sentado más del 5 % del tiempo en el 99 %, y ninguna más de 2 s seguidos en el 100 %.

Se quita el criterio de brazos a 20° del reposo (apoyar las manos lo contradice); se sigue guardando como informativo. Los porcentajes del 95 % de los puntos 2 a 5 son los que se propusieron; el 99 % del giro del tronco también. **No comprobado:** la medida del contacto de manos y pelvis con 4 096 robots (el script se probó con 256 robots en `model_3549`: corre y da cifras razonables, pero no se ha comprobado a mano el cuerpo que `measure_sit.py` toma como pelvis, el del sitio `pelvis_bottom`).

**Recompensa de la corrida** (`training/src/danny_training/sit.py`): se mantienen `descent_speed_excess` con umbral **0,2 m/s** (criterio 0,3) y `trunk_rate_excess` (peso -1, umbral 2 rad/s, criterio 3); se **retira** `arms_off_rest_when_seated`; se **añade** `hands_on_floor_when_low` (peso +1): vale 1 cuando algún antebrazo toca el suelo y la pelvis está a **0,13 m o menos** (por encima las manos no llegan, así que no puede agacharse sobre las manos desde arriba). Los umbrales con margen y los pesos son **estimaciones mías, sin medir**. **Corrida:** parte de `fase5_sentarse_suave/model_3549.pt` (decisión del dueño, 9 de noviembre, que cambia la de «desde cero» tomada antes de ver los resultados), 4 096 robots, `DANNY_DR_STAGE=2`, hasta 5 000 iteraciones. **Corrida 4, `fase5_sentarse_manos` (9 de noviembre): no cumple, parada en la iteración ~7700.** Desde `model_3549` con corte de «no cae» a 80°; recompensa plana desde la 4 740 (63). `model_7700` (copiado a `puntos_de_control/fase5_sentarse_manos_model_7700.pt`; 4 096 robots, `results/training_sentarse_sentarse_manos_model_7700_{6.6V,aleatoria}.json`, vídeos en `results/videos/`): se sienta 100 %, manos antes que la pelvis 100 % (pelvis a 0,128 m), giro del tronco 99,85 %, no cae 100 %, saturación bien; **fallan** la bajada completa de 1 s o más (0,6 s de mediana, 0 %) y la velocidad de 0,218 a 0,16 m (pico mediano 0,31 m/s; 18,9 % a 6,6 V y 16,1 % aleatoria). Termina con el tronco a 1,2° y la pelvis a 0,0566 m, sentado sobre muslos y espinillas y un antebrazo. Con la medida del punto 5 corregida (`..._punto5_6.6V.json`): 99,15 % (primer contacto de pelvis o muslos: mediana 0,084 m/s, máximo 0,253; la pelvis baja de 0,07 m a 0,181 m/s de mediana, máximo 0,234; el 0,85 % restante no toca con los muslos). Al quedar sentado tocan el suelo las dos espinillas (100 %), los muslos (88-92 %) y un antebrazo (100 %). **Causa probable (no demostrada):** el umbral de castigo de bajada era 0,2 m/s, muy por encima de los ~0,15 m/s que pide 1 s para 15 cm.

**Corrida 5, `fase5_sentarse_manos_lenta` (9 de noviembre; decisión del dueño): un solo cambio, `MAX_DESCENT_SPEED_M_S` de 0,2 a 0,15 m/s** (castigo de bajada a partir de 0,15 m/s en todo el tramo hasta 0,07 m). Parte de `model_7700` de la corrida 4, 4 096 robots, `DANNY_DR_STAGE=2`, hasta 3 000 iteraciones nuevas (el contador sigue desde la 7700: llega a la 10700). Comprobación a las 1 000 iteraciones (iteración ~8700): `sit_seated_fraction` de 80 % o más; si cae del 50 %, o se queda colgado sobre las manos a unos 0,09 m como `model_3800`, se para y se mide. Al terminar, se mide con 4 096 robots a 6,6 V y aleatoria, con vídeos del típico y del peor.

**Resultado de la corrida 5 (9-10 de noviembre): no cumple.** Terminó en la iteración 10699 (`puntos_de_control/fase5_sentarse_manos_lenta_model_10699.pt`). Medida con 4 096 robots (`results/training_sentarse_sentarse_manos_lenta_model_10699_{6.6V,aleatoria}.json`, vídeos del típico y del peor en `results/videos/`; a mitad, `model_8650` con 1 024 robots: `..._lenta_a_mitad_model_8650.json`). **Falla la bajada completa de 1 s o más: 0 % en las dos tensiones, 0,70 s de mediana (p5 0,66, p95 0,72)**; de 0,218 a 0,16 m tarda 0,24 s. Punto 3 (0,3 m/s o menos de 0,218 a 0,16 m): 98,2 % a 6,6 V (pico mediano 0,273 m/s) pero **89,7 % con tensión aleatoria**. Punto 5 (medida corregida): 97,4 % a 6,6 V y **90,4 % con tensión aleatoria** (el 9,6 % no toca con los muslos); primer contacto de pelvis o muslos a 0,02 m/s de mediana (máximo 0,237), pelvis bajando de 0,07 m a 0,14 m/s (máximo 0,237; 0,312 con tensión aleatoria). Pasan: se sienta 100 %, manos antes que la pelvis 100 % (a ~0,13 m), giro del tronco 100 % / 99,98 %, no cae (80°) 100 %, saturación. Termina con el tronco a 0,7° y la pelvis a 0,0565 m, sobre espinillas, muslos (71-86 %) y un antebrazo; pico de tronco en la bajada ~35° (máximo 38,8°). **Causa probable (no demostrada):** el castigo de 0,15 m/s solo penaliza el exceso y no premia tardar; para 1 s hacen falta ~0,15 m/s de media en todo el recorrido. **Propuesta, pendiente del dueño (cambio de recompensa, no de criterio):** premiar la lentitud en el tramo de 0,218 a 0,07 m, partiendo de `model_10699`. **No lanzada.**

**Punto 3 por tensión (10 de noviembre), `model_10699` con los criterios nuevos (bajada de 0,6 s, punto 5 con espinillas), 4 096 robots** (`results/training_sentarse_sentarse_manos_lenta_model_10699_{6.0V,6.6V_nuevo_criterio,8.4V}.json`): a **6,0 V y a 6,6 V cumple todo** (punto 3: 99,8 % y 98,1 %; bajada completa 0,70 s con el 100 % sobre 0,6 s; punto 5 100 %). A **8,4 V falla solo el punto 3: 55,1 %** (pico mediano 0,298 m/s, p95 0,335; bajada completa 0,68 s). **Los que fallan con tensión aleatoria son los de tensión alta.** El punto 5 con espinillas da 100 % en las tres tensiones (contacto máximo 0,295 m/s; pelvis bajando de 0,07 m, máximo 0,268).

**Corrida 6, `fase5_sentarse_manos_afinado` (10 de noviembre; decisión del dueño): afinado corto desde `model_10699`, un solo cambio: peso de `descent_speed_excess` de -100 a -200.** No se premia la lentitud (con 0,6 s ya no hace falta). Hasta 1 500 iteraciones nuevas (la 10699 a la 12199); `D:\Claude\fase5_sentarse_manos_afinado.sh`. Después: medir con 4 096 robots a 6,6 V y con tensión aleatoria; si cumple, sentarse queda aprobado y se reentrena tumbarse desde donde termine este sentarse.

**Resultado de la corrida 6 (10 de noviembre): `model_12198` CUMPLE TODO y sentarse queda aprobado** (criterio con bajada completa de 0,6 s, punto 5 con espinillas, no cae a 80°). Afinado desde `model_10699` con el peso de `descent_speed_excess` en -200, 1 500 iteraciones (10699 a 12198). Copia en `puntos_de_control/fase5_sentarse_manos_afinado_model_12198.pt`. Medida con 4 096 robots (`results/training_sentarse_sentarse_manos_afinado_model_12198_{6.6V,aleatoria}.json`; vídeos del típico y del peor en `results/videos/`), a 6,6 V / con tensión aleatoria: se sienta 100 % / 100 %; bajada completa de 0,6 s o más 100 % / 100 % (0,82 s de mediana, p5 0,78, p95 0,86); pelvis a 0,3 m/s o menos de 0,218 a 0,16 m 99,98 % / 99,83 % (pico mediano 0,227 / 0,229 m/s, p95 0,248 / 0,254); manos antes que la pelvis 100 % / 100 % (a unos 0,13 m, máximo 0,136); contacto de pelvis, muslos o espinillas a 0,4 m/s o menos y pelvis bajando de 0,07 m a 0,4 m/s o menos 100 % / 100 % (primer contacto máximo 0,219 / 0,233 m/s; pelvis bajando de 0,07 m, máximo 0,213 / 0,219 m/s); giro del tronco a 3 rad/s o menos 99,83 % / 99,66 % (exige 99 %); no cae (80°) 100 % / 100 % (pico de tronco 36° de mediana, máximo 39,0 / 39,4°); saturación 100 % en los tres puntos. Termina con la pelvis a 0,0565 m y el tronco a 0,8° (p90 1,4°), sentado sobre las espinillas (100 %), los muslos (64-75 % a 6,6 V; 51-58 % aleatoria) y un antebrazo (100 %). **No comprobado:** que `measure_sit.py` tome el cuerpo correcto como pelvis; empujones; la cadena sentarse + tumbarse; servo real. **Siguiente:** reentrenar tumbarse desde donde termina este sentarse; la prueba que aprueba es la cadena. La fase 6 no empieza hasta que el dueño lo confirme.
