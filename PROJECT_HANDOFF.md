# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.

**Fecha:** 2 de noviembre de 2026. **Etapa actual:** 3, MuJoCo, con 2.2, 2.3 y 2.4 abiertas por lo físico. La etapa 1 está
cerrada salvo la 1.2, que va en paralelo.
**Siguiente paso:** ampliar la marcha en simulación con el reloj (`marcha_reloj_1000`, etapa 4.2): ya camina hacia delante a
0,25 m/s con paso alterno (ver [Entrenamiento en simulación](#entrenamiento-en-simulación-etapa-4-sin-terminar)). Faltan giro,
marcha lateral, empujones, la aleatorización de armadura y holgura, y decidir cómo obtiene el robot real la fase del reloj.
El código de `training/` está en la rama `entrenamiento-danny`. `main` está protegida: los colaboradores entran por pull
request aprobado por el usuario, que es quien cambia el diseño. Levantarse es posible en simulación, pero aún frágil. El
robot completo ya está en CAD:
[piernas](documentation/lower-body-design.md), [tronco](documentation/upper-body-design.md) y
[paneles](documentation/panels-and-protection.md).

**Auditoría del 2 de noviembre:** [design-audit.md](documentation/design-audit.md). Corregidos los 6 bloqueos de montaje, el tobillo a -45°,
la pose de reposo y la simulación con BAM. Quedan puntos importantes: no imprimas piezas definitivas ni compres rodamientos de 10 × 15 × 3.
El modelo cambió, así que hay que reentrenar. Los pasos de verificación están en `check_assembly.py` y `check_standing.py`, y este último sale con
código 1 porque, sin política, las tareas fallan con ganancia 5.

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
  (`results/training_gait_marcha_reloj_1000.json`): sin caer el 100 %, avance 0,257 m/s, error 0,020 m/s, apoyos alternos
  99,9 %, pie izquierdo el 48,9 % de los apoyos. Con órdenes aleatorias (`results/training_walk_marcha_reloj_1000.json`):
  de pie 99,2 % de los entornos, error 0,028 m/s. Altura máxima de suela 43 mm
  (`results/training_peak_height_marcha_reloj_1000.json`). Log de entrenamiento: episodio de 996 pasos y
  `peak_height_mean` de 19,7 mm. Con el punto de control de la iteración 350 ya caminaba igual (error 0,023 m/s). El usuario
  lo vio caminar hacia delante en el visor.
- **Pares de `marcha_reloj_1000`** (`results/training_torque_marcha_reloj_1000.json`, orden de 0,25 m/s): tobillo de cabeceo
  derecho 1,07 N·m, izquierdo 1,02, rodilla derecha 0,94, izquierda 0,90, cadera de balanceo izquierda 0,74; el resto, por
  debajo de 0,55. El 90 % del par de parada a 6 V (0,762 N·m) se supera el 8,1 % del tiempo en el tobillo de cabeceo derecho
  y el 4,4 % en el izquierdo. La guía pide no saturar ninguna articulación más del 5 %: el derecho no lo cumple a 6 V, aunque
  ese umbral se mide con el par de parada a 6 V y la aleatorización llega a 8,4 V (1,067 N·m). Sin revisar por tensión.
- **Sin comprobar de `marcha_reloj_1000`:** solo se probó hacia delante a 0,25 m/s, sin giro, sin marcha lateral y sin empujones;
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

## Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que las cifras de `results/actuator_comparison.json` sean de diseño: son de primer orden.

## Pendiente de decidir por el usuario

- Licencia del repositorio, que es público.
- Fuentes de movimiento humano (1.2): sin ellas no empieza la etapa 4.4.

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
