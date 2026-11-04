# Guía de entrenamiento: etapa 4

Para quien entrena a Danny en una computadora con GPU NVIDIA, y para su Claude. Con este documento y el repositorio basta.

**Danny:** humanoide de 40 cm y 1,30 kg (E), con 23 motores HD-1910. Todavía **no existe físicamente**: solo el CAD y la
simulación. Entrenarlo aquí es lo que dirá si puede caminar y levantarse.

## Lo que tiene que hacer la persona

1. Instala [Claude Code](https://claude.com/claude-code), `git`, [uv](https://docs.astral.sh/uv/) y los controladores de NVIDIA.
2. Clona el repositorio:

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
```

3. Abre Claude Code dentro de la carpeta `danny-robot` y pégale este mensaje:

```text
Lee CLAUDE.md, PROJECT_HANDOFF.md y documentation/training-guide.md. Entrena a Danny con mjlab siguiendo esa guía, por orden,
empezando por comprobar la GPU. Dime qué vas a hacer antes de cada entrenamiento largo y avísame de cada resultado.
```

Para devolver el resultado por GitHub hace falta permiso: ver [Devolver el resultado](#devolver-el-resultado).

## Lo que hay que saber antes de empezar

- **No está probado:** esta guía se escribió en un Mac sin GPU, y mjlab no se ha ejecutado con Danny. El modelo sí carga y se
  sostiene en MuJoCo. Espera ajustes.
- **Referencia:** [`xgoduck_rl`](https://github.com/LuwuDynamics/xgoduck_rl), Apache 2.0, entrena un robot con este mismo
  servo. Tiene marcha, levantarse, modelo BAM y exportación ONNX. Es la mejor base: se adapta su código a Danny y se conserva
  su aviso de licencia. **Sus piezas 3D no se copian**, solo el código.
- **Versiones:** `xgoduck_rl` fija `mjlab==1.3.0`, `torch==2.9.1` y Python 3.12 por BAM. La última de mjlab es 1.6.0, que usa
  MuJoCo 3.16. Empieza con las versiones que funcionan en la referencia, y actualiza solo si hace falta.
- **Entorno separado:** el entrenamiento va en su propia carpeta `training/`, con su `uv`. No toques el `.venv` del proyecto.
- **No uses Isaac Lab:** el proyecto eligió mjlab.

## Qué se entrega ya

| Archivo | Qué es |
|---|---|
| `cad/export/danny/danny_training.xml` | Modelo para entrenar: base libre, suelo, cajas de colisión, 23 actuadores de posición y postura `stand`. Lo genera `analysis/export_training_model.py` |
| `cad/export/danny/meshes/` | Mallas |
| `analysis/hd1910_bam_parameters.json` | Parámetros del servo, de `xgoduck_rl`, sin confirmar en nuestros servos |
| `documentation/HANDBOOK.md`, secciones 4 y 9 | Ejes, orden de articulaciones, rangos y objetivos de la etapa |
| `documentation/simulation-results.md` | Pares, velocidades y lo aprendido |

**No edites a mano** `danny_training.xml` ni el CAD. Las mallas son visuales: las colisiones son cajas simples, y las de los
eslabones cortos no existen. Si la política se aprovecha de eso, añade colisiones en `analysis/danny_sim.py` y regenera.

## Datos del robot

| Dato | Valor |
|---|---|
| Control | 50 Hz |
| Articulaciones | 23, en el orden de la sección 4 del handbook. Solo se añadirán 4 de manos, más adelante |
| Altura del centro de masas de pie | 0,212 m |
| Par de bloqueo | 0,912 N·m a 7,4 V y 0,739 N·m a 6 V según el modelo; 1,47 N·m según ficha. Usa el modelo |
| Velocidad en vacío | 11,5 rad/s a 7,4 V |
| Batería | 2S: de 8,4 V a unos 6 V bajo carga. Danny deja de caminar a 6,6 V, y el programa corta a 6,4 V |
| Sensores en el robot real | IMU BNO085 en el torso, y posición y velocidad de cada servo. La política no puede usar nada más |

## Orden de trabajo

### 4.1 Preparar

1. Comprueba `nvidia-smi` y que torch ve la GPU.
2. Crea `training/` a partir de `xgoduck_rl`, con su licencia y un archivo que diga de dónde viene.
3. Registra Danny con `danny_training.xml`, y sus actuadores BAM con `analysis/hd1910_bam_parameters.json`. Comprueba el orden de articulaciones,
   los signos y que no sea un copia y pega de los 15 servos del pato.
4. Reescribe lo que dependa de la forma del pato: pies, cuerpos, escalas de recompensa, alturas. Danny es más alto, con brazos y cintura.
5. Mide y escribe en `results/` la altura de pie y las velocidades reales. No heredes nada del pato.
6. Corre un entrenamiento corto, de unos minutos, para confirmar que aprende algo antes de lanzar uno largo.

**Terminado cuando:** un entrenamiento corto termina sin errores y se ve con `play`.

### 4.2 Marcha

Órdenes de velocidad sobre suelo plano, con los brazos acompañando.

**Aleatorización obligatoria:** tensión de 6 a 8,4 V, masa ±10 %, centro de masas ±10 mm, rozamiento del suelo de 0,4 a 1,2,
rozamiento y armadura del servo ±30 %, retardo de 10 a 30 ms, holgura de hasta 0,5° y empujones.

**Terminado cuando:** con aleatorización, camina a 0,25 m/s durante 5 minutos simulados sin caer en al menos el 95 % de las
pruebas, con los empujones de la tabla de abajo (±0,12 m/s). Sin superar la curva del motor, y sin saturar ninguna articulación más
del 5 % del tiempo a 6,6 V, 7,4 V y 8,4 V fijos. Se entrena de 6 a 8,4 V para cubrir la caída de tensión bajo carga, pero Danny
no camina por debajo de 6,6 V: el criterio de par se mide desde ahí. Decisión del usuario del 4 de noviembre de 2026.

**Primer intento, 1 de noviembre de 2026: no camina.** Los valores por defecto de mjlab están pensados para un G1 de 1,3 m. Con
ellos, quedarse quieto da el 91 % de la recompensa de velocidad, frente al 36 % en `xgoduck_rl`, que camina con este mismo
servo (`results/walk_incentive.json`). Además, el 96 % de los episodios acabó en caída, a los 5 s de media.

| Término | Primer intento | `xgoduck_rl` | Punto de partida para Danny (E, sin entrenar) |
|---|---|---|---|
| `track_linear_velocity` | std 0,5; peso 4 | std 0,25; peso 3 | std 0,125; peso 3 |
| `air_time` | peso 0 | peso 3, de 0,125 a 0,3 s, umbral 0,01 | peso 3, de 0,125 a 0,35 s, umbral 0,01 |
| `target_height` de los pies | 0,1 m | 0,02 m | 0,025 m |
| `foot_swing_height` | -0,25 | -5 | -5 |
| Empujones | ±0,5 m/s, con giro y vertical, cada 1-3 s | ±0,2 m/s en XY, cada 3-6 s | ±0,12 m/s en XY, cada 3-6 s |
| `base_lin_vel` | En el actor, con ruido de ±0,5 m/s | Solo en el crítico | Solo en el crítico: el robot real no la mide |
| Ruido del actor | Velocidad angular ±0,2; gravedad ±0,05; velocidad de articulación ±1,5 | ±0,06; ±0,03; ±0,25 | Como `xgoduck_rl` |
| Escala de acción | 0,5 | 1,0 | 1,0 |
| Peso de `pose` | 0,5 | 1,3 | 1,0 |
| Iteraciones | 300 | Hasta 50 000 | 3 000, unas 3 h |

**La velocidad debe medirse en el tronco.** El origen de `base_link` está en el suelo, a unos 23 cm del centro de masas, porque
sale así del CAD. `track_linear_velocity` y `evaluate_walk.py` leen `root_link_lin_vel_b` justo ahí, así que inclinarse sobre
los tobillos casi no cuenta como moverse, y cada balanceo del cuerpo añade un error falso de 0,23 m/s por rad/s. Usa
`root_com_lin_vel_b`, que mide en el centro de masas de `base_link`. Es una hipótesis sin entrenar, aunque la corrección es barata.

**El modelo cambió el 2 de noviembre:** tobillo a -45°, brazos en reposo a 8° con el codo a -17°, y sitio `imu` en `torso_1`, donde va la
BNO085. Vuelve a generar `training/` con `danny_training.xml` y reinicia: una política anterior no sirve. Con -30°, `stand` quedaba a 0,04° del
límite blando de -26,5°, así que apoyarse sobre el pie adelantado se penalizaba.

**Antes de cada corrida:** `analysis/check_training_model.py` comprueba el modelo, y `analysis/check_walk_feasibility.py` muestra que con ganancia 5 y la
física del entrenamiento da pasos alternos con zancada de 98 mm. Si Danny no camina, mira primero el aprendizaje. Mantén `kp_fw` 5.

**Cajas de colisión:** el modelo trae fricción 0,35, prioridad 1 y 13 contactos entre eslabones. En `DANNY_COLLISION`, usa `condim=3` y `priority=1` también para
`.*_box_\d+`, o el suelo les pondrá 1,0. Aleatoriza la masa en todos los cuerpos con `shared_random=True`, para que sea ±10 % de verdad.

**Resultado del 2 de noviembre: con un reloj de marcha camina.** Con las escalas de arriba y `kp_fw` 5, Danny seguía apoyado en un pie y sin avanzar, con `air_time` y con una penalización de aterrizaje repetido. Con un reloj de marcha (fase de 0,8 s que solo avanza con orden de movimiento, seno y coseno en la observación, y una recompensa de contacto en contrafase de peso 1,5) caminó hacia delante a 0,248 m/s en 1000 iteraciones, y gira hasta 0,4 rad/s, con error de 0,020 m/s y apoyos alternos al 99,9 % (`results/training_gait_marcha_reloj_1000.json`). Se probó hacia delante y con giro, sin empujones ni marcha lateral. Detalles en `PROJECT_HANDOFF.md`.

**El error de posición no indica un fallo.** El servo funciona como un muelle blando: a 7,4 V da 1,03 N·m/rad y satura con
0,88 rad (`results/walk_incentive.json`). Así que el par sale de ese error, y 0,8 N·m exigen unos 0,8 rad.

**Comprobación a las 1 000 iteraciones:** media de episodio por encima de 600 pasos, `peak_height_mean` de 15 mm o más y
`air_time` distinto de cero. Si no se cumple, para y revisa antes de seguir.

**Criterio a las 3 000:** sin empujones, el error de velocidad baja de 0,155 m/s, que es estar quieto, a 0,08 m/s o menos, y se
ve un paso alterno. Si se cumple, añade la aleatorización que falta, armadura y holgura, y los empujones hasta donde aguante.

### 4.3 Levantarse

Desde boca abajo y boca arriba, con la misma aleatorización.

**Terminado cuando:** se levanta en 8 de cada 10 intentos, en los dos casos, y termina estable 3 s.

**Es lo más incierto.** Una búsqueda sin realimentación lo logró, pero muy frágil. Lo que se aprendió:

- Boca abajo, las manos deben estar delante de la cabeza. Si empujan junto a la cadera, se voltea.
- El hombro no pasa por detrás: los brazos barren por el costado.
- Boca arriba, conviene darse la vuelta primero.
- La cadera llega a -100°, y no se puede ampliar sin rediseñar. El tobillo llega a -45°.

Las secuencias están en `results/get_up_prone.json` y `results/roll_over.json`, y se ven con `analysis/view_simulation.py`.
Pueden servir de referencia, pero no son el objetivo: tienen un aspecto poco natural.

**Si no lo consigue tras un esfuerzo razonable:** para y explica por qué. Puede significar rediseñar la cadera, y eso lo decide
el dueño del proyecto.

### 4.4 Movimiento natural

**Fuentes elegidas en la etapa 1.2: CMU y 100STYLE** (sección 6 de `research-and-requirements.md`), retargeteadas con GMR. LAFAN1 y
AMASS no se publican. Aquí es donde deja de moverse raro.

## Reglas

Son las de [CLAUDE.md](../CLAUDE.md). Las más importantes para ti:

- Responde en español.
- Fija los criterios antes de entrenar y di si los cambias.
- Mide antes de cambiar, y di lo que no has comprobado, también lo que falla.
- No modifiques CAD, requisitos ni rangos sin avisar: si un límite físico estorba, repórtalo.
- No compres nada.
- Commit y push solo cuando la persona lo pida.

## Devolver el resultado

**Sí, por GitHub.** Hay que dar permiso primero: el dueño añade a la persona como colaborador en *Settings > Collaborators*. Si
no, se trabaja en un fork y se abre un pull request.

Trabaja en una rama `entrenamiento-danny`, nunca en `main`: está protegida, y solo entra por pull request aprobado por el dueño.
El diseño, en `cad/` y `analysis/`, lo cambia el dueño: si el entrenamiento necesita algo, pídeselo. Sube:

| Qué | Dónde |
|---|---|
| Código de `training/` | En la rama |
| Política final en ONNX, por tarea | `policies/<tarea>_<fecha>.onnx`, con normalización incluida |
| Configuración exacta y commit del entrenamiento | `policies/<tarea>_<fecha>.json` |
| Resultados de cada criterio | `results/training_<tarea>.json` |
| Vídeo corto de cada tarea | `results/videos/`, si cada uno pesa menos de 25 MB |

**No subas** `logs/` ni todos los puntos de control: GitHub rechaza archivos de más de 100 MB. Guarda el mejor `.pt` en la
computadora, o en una release de GitHub, por si hay que seguir entrenando.

Añade una sección en `PROJECT_HANDOFF.md` con qué se hizo, qué cumple, qué no y qué falta. Actualiza `ROADMAP.md`.

## Qué pasa después

La política no se instala todavía: no hay robot. Primero se mide el servo real, en la etapa 5.1, y con esos datos se reentrena.
En la etapa 5.3, el programa del robot carga el ONNX en la Radxa Zero 3W y lo ejecuta a 50 Hz.
