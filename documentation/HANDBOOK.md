# Handbook de Robot Danny

Manual para trabajar en el proyecto. Con este documento, una persona nueva y su Claude deben poder seguir
sin preguntar nada más. Si algo falta, se añade aquí.

**Cómo usarlo:** [PROJECT_HANDOFF.md](../PROJECT_HANDOFF.md) dice dónde estamos, [ROADMAP.md](../ROADMAP.md)
dice el estado de cada etapa, y este handbook dice cómo se hace cada cosa.

| Si quieres saber... | Mira |
|---|---|
| Qué toca ahora | [PROJECT_HANDOFF.md](../PROJECT_HANDOFF.md) |
| Estado de cada etapa | [ROADMAP.md](../ROADMAP.md), la fuente de verdad |
| Cómo se trabaja | Este documento y [CLAUDE.md](../CLAUDE.md) |
| Piezas, medidas y precios | [architecture-and-bom.md](architecture-and-bom.md) |
| Requisitos y por qué se decidió cada cosa | [research-and-requirements.md](research-and-requirements.md) |
| Cifras exactas | `results/`, generados por `analysis/` |

Lee en este orden: `PROJECT_HANDOFF.md`, este handbook y `ROADMAP.md`. Si dos documentos se contradicen, manda el ROADMAP para el estado y `results/` para las cifras. Corrige el
otro documento.

## 1. Qué es Danny

Un humanoide de 40 cm (395 mm), autónomo y con aspecto de Tesla Optimus, impreso en 3D, con servos de bus y
movimiento aprendido en simulación. Coste objetivo: hasta 1 500 €.

**Referencia:** el XGO-Duck aporta el servo y el método de entrenamiento, no la forma. No se copian sus
piezas, que no tienen licencia final.

**Prioridad cuando dos objetivos chocan:** primero caminar natural y levantarse. Trotar, saltar, manipular y
conversar van después, y solo si el diseño lo permite. Se reservan espacio, masa y bus para las manos, y sitio
para micrófono y altavoz. Con manos, el coste estimado es de 1 533 €: supera el tope y necesitará mejor precio o
aprobación.

**Principio de diseño:** no se sacrifican física, estabilidad ni viabilidad de los motores para que se vea
humano, ni se invierte en animaciones a mano. El movimiento natural sale del aprendizaje con referencias
humanas.

## 2. Primer día

Necesitas Python 3.12 y `git`. Para cada etapa, además:

| Etapa | Herramienta | Quién |
|---|---|---|
| 2 | Autodesk Fusion y [ACDC4Robot 1.1.1](https://github.com/ACDC4Robot/Fusion360/releases/tag/v1.1.1). Comprueba que tu licencia de Fusion permite complementos | Humano para instalar y activar. Claude modela por scripts con `DannyBridge`, sección 7 |
| 3 | MuJoCo, por `requirements.txt`, en cualquier equipo | Claude y humano |
| 4 | GPU NVIDIA, CUDA, Linux o WSL, y `uv` | Equipo con la RTX 3080 Ti. Sin GPU, mjlab solo evalúa |
| 2 y 5 | Impresora cerrada multimaterial. En la etapa 2, material de prueba; en la 5, PETG-CF, TPU y soldador | Humano |

```bash
git clone https://github.com/danielmoreno510/danny-robot.git
cd danny-robot
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analysis/compare_actuators.py
.venv/bin/python analysis/estimate_robot_budget.py
```

Los dos scripts deben terminar sin error. Ahora dile a tu Claude:

> Lee PROJECT_HANDOFF.md y documentation/HANDBOOK.md, y dime qué toca hacer y qué necesitas de mí.

El proyecto anterior, con robots de 70 cm, está en la etiqueta `archivo-danny-70cm`. Solo es consulta: sus
requisitos, sus 31 GDL e Isaac Lab ya no valen.

## 3. Cómo se trabaja

Estas reglas son las de [CLAUDE.md](../CLAUDE.md). Las más importantes:

- **Criterios antes de ejecutar.** Se fijan antes y se calcula si son físicamente posibles. Si cambian, se dice
  y se documenta.
- **Medir antes de cambiar.** Un arreglo que no explica el fallo no se deja.
- **Decir lo que no se ha comprobado**, y documentar también lo que falla.
- **Las cifras salen de `results/`**, nunca de memoria. Las estimaciones se marcan.
- **Nada se compra sin aprobación del dueño del proyecto.**
- **Commit y push solo cuando se pidan.** En equipo: una rama por trabajo y revisión antes de unir a `main`.
- **Cada cambio actualiza la documentación** en el mismo trabajo. Sin menciones obsoletas.
- **Quitar lo que no se usa.** Git guarda el historial.

**Decisiones:** las toma Daniel Moreno, dueño del proyecto, si afectan a coste, forma, requisitos o licencia. Se
le consulta por una incidencia en GitHub o por el canal que te haya dado. Las técnicas
reversibles las toma quien trabaja, y las deja escritas.

**Al terminar una etapa:** implementación, validación reproducible, documentación, revisión independiente de lo
hecho, y relevo actualizado en `PROJECT_HANDOFF.md`.

**Etiquetas de datos:** **V** fabricante o proyecto; **S** tienda; **E** estimación; **?** desconocido.

### Añadir un análisis

1. Script en `analysis/`, con nombre verbo + objeto, que escriba su resultado en `results/`.
2. Un script no importa de otro: el código compartido va en un módulo.
3. Ejecutarlo y pasar `pyflakes`.
4. Documentar las cifras citando `results/`.

## 4. Convenciones

- **Unidades:** SI en código; milímetros, gramos y euros en el desglose, que lo indica.
- **Ejes (REP-103):** X adelante, Y a la izquierda, Z arriba. Ángulos positivos según la regla de la mano
  derecha sobre el eje de la articulación.
- **Pose cero:** de pie, recto, brazos colgando, palmas hacia dentro.
- **Nombres:** descriptivos, sin `_v2`. Modelos por lo que contienen y scripts verbo + objeto.
- **Lados:** prefijo `left_` o `right_`, siempre el lado del robot, no el de quien lo mira.

### Articulaciones

Este orden se usa en CAD, simulación, política y servos. No se cambia sin actualizar todo.

| ID | Articulación | Cadena de bus |
|---|---:|---|
| 1-6 | `left_hip_yaw`, `left_hip_roll`, `left_hip_pitch`, `left_knee`, `left_ankle_pitch`, `left_ankle_roll` | Pierna izquierda |
| 7-12 | `right_hip_yaw`, `right_hip_roll`, `right_hip_pitch`, `right_knee`, `right_ankle_pitch`, `right_ankle_roll` | Pierna derecha |
| 13 | `waist_yaw` | Superior |
| 14-17 | `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw`, `left_elbow` | Superior |
| 18-21 | `right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`, `right_elbow` | Superior |
| 22-23 | `neck_yaw`, `neck_pitch` | Superior |
| 24-27 | `left_wrist`, `left_gripper`, `right_wrist`, `right_gripper` | Superior, más adelante |

Las articulaciones sin motor, como la holgura simulada, llevan el prefijo `passive_`.

## 5. Datos que se usan al diseñar

Detalle en [architecture-and-bom.md](architecture-and-bom.md).

| Dato | Valor |
|---|---|
| Altura | 395 mm: tobillo 35, tibia 82, muslo 83, cadera 48, torso 88, holgura 4, cabeza 55 |
| Hombros | 133,8 mm; torso de 73 mm de ancho interior |
| Caderas | ±29 mm |
| Pie | 85 × 48 mm |
| Masa | Máximo 1,8 kg sin manos y 1,9 kg con manos; 1,29 kg (E) en CAD, con cables y tornillos estimados |
| Servo HD-1910-C001 | 34 × 20 × 23 mm, 29 mm con ejes, 21 g, eje doble, 7,4 V |
| Par del servo | Bloqueo 1,47 N·m y nominal 0,36 N·m a 7,4 V; el continuo real es desconocido |
| Velocidad del servo | 11,9 rad/s en vacío a 7,4 V |
| Protección del servo | Corta a 80 °C |

### Rangos de articulación

Objetivo provisional (E), inspirado en una persona. Son criterio de la etapa 2: si uno no se alcanza, para y avisa
con la causa. Cada rango se comprueba solo, desde la pose de partida de `SweepJointRanges`: con el brazo colgando no hay
sitio para que una pierna se abra, igual que en una persona. Detalle en [upper-body-design.md](upper-body-design.md).

En pose cero, los ejes son iguales en ambos lados: pitch sobre +Y, roll sobre +X y yaw sobre +Z. Por eso, pitch
positivo lleva pierna y brazo atrás, y roll positivo lleva el lado izquierdo afuera y el derecho adentro.

| Articulación | Eje | Mínimo | Máximo | Qué es |
|---|---|---:|---:|---|
| `hip_pitch` | +Y | -100° | 20° | Negativo: pierna adelante |
| `left_hip_roll` | +X | -15° | 35° | Positivo: afuera. Hacia dentro, con el otro pie apartado |
| `right_hip_roll` | +X | -35° | 15° | Negativo: afuera |
| `left_hip_yaw` | +Z | -15° | 30° | Positivo: punta afuera. El otro pie limita hacia dentro |
| `right_hip_yaw` | +Z | -30° | 15° | Negativo: punta afuera |
| `knee` | +Y | 0° | 130° | Positivo: dobla |
| `ankle_pitch` | +Y | -45° | 40° | Positivo: punta abajo |
| `ankle_roll` | +X | -20° | 20° | |
| `waist_yaw` | +Z | -60° | 60° | |
| `shoulder_pitch` | +Y | -170° | 45° | Negativo: brazo adelante |
| `left_shoulder_roll` | +X | -10° | 120° | Positivo: afuera. Con el brazo abajo, la pelvis limita cerca de 0° |
| `right_shoulder_roll` | +X | -120° | 10° | Negativo: afuera. Igual |
| `shoulder_yaw` | +Z | -80° | 80° | |
| `elbow` | +Y | -130° | 0° | Negativo: dobla hacia adelante |
| `neck_yaw` | +Z | -70° | 70° | |
| `neck_pitch` | +Y | -30° | 40° | Positivo: mira abajo |

## 6. Etapa 1: investigación y definición

Terminada salvo la 1.2, que va en paralelo y no frena la etapa 2. Para esa etapa:

**Objetivo:** elegir fuentes de movimiento humano para imitar.

**Entrega:** sección en `research-and-requirements.md` con las fuentes, su licencia y si permiten un repositorio
público. Candidatas sin revisar: AMASS y LAFAN1.

**Terminada cuando:** hay al menos una fuente con licencia compatible. Se investiga en paralelo: no frena el
diseño.

## 7. Etapa 2: diseño en Autodesk Fusion

**Objetivo:** un robot completo y sin interferencias, dentro de la masa, exportable a MuJoCo.

**Reglas del CAD:**

- Las medidas viven en constantes al principio de cada script de `cad/fusion/`: se cambian ahí y se regenera. Decisión
  de la etapa 2.2, que sustituye a los parámetros de usuario de Fusion, porque el script ya es reproducible.
- Un componente por eslabón rígido, nombrado en inglés como su función: `left_thigh`, `left_shin`. La pelvis se
  llama `base_link`, porque el exportador pide ese nombre o el de un eslabón fijo.
- Una articulación de Fusion por cada una de la sección 4, con el mismo nombre, eje y signo que la tabla de rangos.
- **Material con densidad real en cada cuerpo,** porque de ahí salen masas e inercias. Los servos, con su masa
  medida o de ficha, aunque se modelen como caja.
- Geometría de colisión simple por separado: los pies importan más que nada.
- Holguras para impresión: probarlas con una pieza pequeña antes de diseñar el resto.
- Prever: paso de cables por las articulaciones y herramienta para llegar a cada tornillo. El lado libre del eje
  doble usa la bocina libre del servo, sin rodamiento añadido: sin comprobar con un servo real.
- `cad/fusion/` guarda los scripts, `cad/export/` el MJCF y sus mallas, y `cad/print/` los STL para imprimir.

**Modelar con Claude:** el complemento `cad/fusion/DannyBridge` ejecuta en Fusion los scripts de `cad/fusion/`,
cada uno en su carpeta con una función `build(app)`. Se instala una vez en cada equipo:

```bash
ln -s "$PWD/cad/fusion/DannyBridge" ~/Library/Application\ Support/Autodesk/Autodesk\ Fusion\ 360/API/AddIns/DannyBridge
```

Reinicia Fusion: su manifiesto hace que arranque solo. Si no responde, abre Scripts y complementos (Mayús+S),
pestaña Complementos, selecciona `DannyBridge` y pulsa Ejecutar. Después, para probarlo y lanzar la pieza de la
etapa 2.1:

```bash
.venv/bin/python analysis/run_fusion_script.py ReportFusionState
.venv/bin/python analysis/run_fusion_script.py BuildExportTest
```

El comando muestra lo que devuelve el script y deja la vista en `cad/fusion/bridge_view.png`.

**Exportar:** descomprime `ACDC4Robot-1.1.1.zip` en esa misma carpeta `AddIns`, de modo que quede
`AddIns/ACDC4Robot/`. `ExportModel` usa su código sin abrir diálogos y escribe en `cad/export/<export_name>/`:

```bash
.venv/bin/python analysis/run_fusion_script.py ExportModel
```

Reglas del exportador:

- El diseño necesita el atributo `danny` / `export_name`, porque Fusion no deja renombrar el componente raíz. Ver
  `BuildExportTest`.
- En una articulación, la primera ocurrencia es la hija y la segunda el padre.
- La estructura debe ser plana, sin ocurrencias anidadas, con un componente fijo llamado `base_link`.
- Los cuerpos de MuJoCo toman el nombre de la ocurrencia: `arm_link` sale como `arm_link_1`.
- El MJCF no trae colisiones, actuadores, sensores ni suelo: van en una capa aparte.

**Orientación:** los scripts escriben coordenadas REP-103, con Z arriba, sea cual sea la preferencia de Fusion. Con
Y arriba el modelo es correcto, pero se ve tumbado. Fíate de `camera_up` en `ReportFusionState`: `z_up` dio
`true` sin documento abierto y `false` con él.

**Pasos:** las órdenes y su estado están en [lower-body-design.md](lower-body-design.md) y
[upper-body-design.md](upper-body-design.md). `BuildRobot` modela el robot completo, y
`analysis/estimate_fall_energy.py` escribe la estimación de caída.

| Paso | Qué hacer | Terminado cuando |
|---|---|---|
| 2.1 | Hecho. Instalar ACDC4Robot 1.1.1. Modelar dos eslabones y una articulación, exportar a MJCF y abrirlo en MuJoCo | El modelo carga, la masa difiere menos de un 1 % y el eje menos de 1° del CAD |
| 2.2 | Conseguir el plano del servo (sección 8 de la arquitectura) e imprimir la [maqueta](https://makerworld.com/en/models/3502393-feetech-1910-servo-mock-1-1-dummy) con material de prueba, o usar un servo real. Modelar cadera y pierna por script. La impresora y los servos del usuario vienen en camino | Rangos de la sección 5 sin choques, encaje comprobado con la maqueta y pieza pesada |
| 2.3 | Torso, brazos, cabeza y cables. Empezar por la electrónica: el torso va sin holgura | Electrónica con IMU, regulador e interruptor, y servos, caben con espacio para manos y voz, y hay salida real para cada conector |
| 2.4 | Primera iteración hecha: [paneles y protección](panels-and-protection.md). Afinar la forma con el dueño del proyecto | Robot completo sin interferencias, y R4 y R10 cumplidos en CAD |

Para comprobar el modelo exportado sin ventana, con la masa total del CAD en kg, y para verlo:

```bash
.venv/bin/python analysis/inspect_model.py cad/export/danny/danny.xml --expected-mass 1.2006
```

```bash
.venv/bin/python -m mujoco.viewer --mjcf=cad/export/danny/danny.xml
```

El primero sale con código 1 si el modelo no carga o la masa difiere más de un 1 %, y muestra cada eje para
compararlo con el CAD. En macOS, este visor va con `python`: con `mjpython` falla. Di siempre si llegaste a ver la
ventana. En Windows, las órdenes usan `.venv\Scripts\python`.

**Densidad:** en Fusion, asigna al cuerpo impreso una densidad efectiva, no la del material macizo. En piezas
gruesas, parte de 0,7 g/cm³ (E). Las paredes finas salen macizas: pelvis y piernas usan 1,25 g/cm³ (E). Corrígela
pesando una pieza impresa en 2.2: de ahí depende R4.

**No se compra nada en esta etapa.** Los rodamientos y el filamento final llegan en la 5. Si necesitas material
de prueba y no lo tienes, pide aprobación.

**Si la cadera no cabe:** es la zona que más limita la altura. Prueba en este orden: cambiar la colocación de
los servos, cambiar el orden de ejes y reducir holguras. Si aun así hay que cambiar tramos, se puede dentro de
R10 dejándolo escrito. Fuera de R10, decide el dueño del proyecto.

## 8. Etapa 3: modelo en MuJoCo

**Objetivo:** confirmar que el diseño puede hacer lo que se le pedirá, antes de entrenar y de imprimir.

Trabaja en bucle con Fusion: cada cambio del CAD se exporta de nuevo. **El MJCF exportado no se edita a mano**:
los ajustes de simulación van en un script o archivo aparte, para que sobrevivan a cada exportación.

| Paso | Qué hacer | Terminado cuando |
|---|---|---|
| 3.1 | Importar el MJCF y añadir actuadores con el modelo BAM del HD-1910 de `xgoduck_rl`, marcado como no confirmado | Se sostiene de pie con una postura estable 3 s |
| 3.2 | Medir par y velocidad en cuclillas, apoyo a un pie, marcha y levantarse | Tabla por articulación en `results/` |
| 3.3 | Corregir el diseño y repetir | Picos de 0,61 N·m o menos a 7,4 V y 0,49 N·m a 6 V, y velocidades por debajo de las de vacío |

Ambos pares son el bloqueo del modelo BAM entre el margen de 1,5, y son provisionales hasta medir el servo. Revisa también la
curva par-velocidad, y usa `analysis/danny_servo.py`: los actuadores rígidos dieron por buenas pruebas que el servo no sostiene.

**Estado y comandos:** [simulation-results.md](simulation-results.md). Scripts: `analysis/danny_sim.py`, `check_standing.py` y
`measure_joint_loads.py`.

**De dónde sale el movimiento en 3.2:** aún no hay políticas. Se usan posturas cuasi estáticas y trayectorias
paramétricas sencillas con cinemática inversa. Son herramientas de medida, no animaciones, y no se conservan.
Las cifras reales llegan en la etapa 4: si una política supera los límites, se vuelve a esta etapa.

**Comprobar antes de seguir:** que la postura sea estable midiendo la inclinación, no solo la altura. Las alturas
de referencia se miden en el modelo actual, nunca se heredan.

## 9. Etapa 4: aprendizaje en mjlab

**Objetivo:** políticas que funcionen en el robot real, no solo en el visor.

**Partida:** adaptar `xgoduck_rl` (Apache 2.0, conservar su aviso de licencia y atribución), derivado de
`microduck_rl`. El XGO-Duck tiene 15 servos, y su política mueve 14: sin brazos y con otro cuello. Hay que
adaptarlo a 23.

**Lectura obligatoria antes de entrenar:** `AGENTS.md` de
[microduck_rl](https://github.com/pollen-robotics/microduck_rl/blob/main/AGENTS.md). Recoge lo aprendido sobre
transferencia a real y recompensas.

**Preparación:**

```bash
git clone https://github.com/mujocolab/mjlab.git
cd mjlab
uv run demo
```

Comandos habituales en un repositorio de tareas como `xgoduck_rl`:

```bash
uv sync
uv run list-envs
uv run train <TAREA> --env.scene.num-envs 64 --agent.max_iterations 5
uv run train <TAREA> --env.scene.num-envs 4096
uv run play <TAREA>
```

**Antes de un entrenamiento largo, siempre la prueba de 64 entornos y 5 iteraciones.** Si la RTX 3080 Ti no
llega a 4 096 entornos, bajar el número y medir: esa cifra viene de los proyectos de referencia, sin probar
en esta tarjeta.

**Requisitos del entrenamiento:**

- Control a 50 Hz, como en la referencia.
- Aleatorización de masa, centro de masas, fricción, retardo, tensión de batería y holgura. Debe no acumularse
  entre reinicios.
- Tensión variable de 6 a 8,4 V. El par se juzga desde 6,6 V, donde Danny deja de caminar.
- Ruido en lo que el robot mide, no en el mando.
- Exportar a ONNX con el script del proyecto, que incluye la normalización.

| Paso | Qué hacer | Terminado cuando (provisional) |
|---|---|---|
| 4.1 | Preparar el equipo y entrenar la demo de mjlab | Entrena sin errores en la 3080 Ti |
| 4.2 | Marcha con órdenes de velocidad | 0,25 m/s, y el 95 % de 1 000 episodios aleatorizados sin caer |
| 4.3 | Levantarse | Boca arriba y boca abajo, 8 de 10 con aleatorización |
| 4.4 | Imitación y gestos | Saludo y baile sin caer, y marcha con braceo y giro de torso |
| 4.5 | Trote y salto, si procede | Criterios R7 y R8 |
| 4.6 | Manipulación, si procede | Criterio R9 |

Cada política se comprueba antes en MuJoCo por CPU con actuadores BAM. Revisa siempre que las penalizaciones
sumen negativo en los registros: un signo cambiado premia lo que se quería evitar.

## 10. Etapa 5: robot físico

**Orden obligatorio y por qué:** aquí aparece el mayor riesgo del proyecto. Se compra por partes, siempre con
aprobación, y **no se compra el robot entero hasta pasar 5.1 y 5.2**.

| Paso | Compra | Qué hacer | Terminado cuando (provisional) |
|---|---|---|---|
| 5.1 | Banco, ya comprado: 3 servos, FE-URT-2 y fuente. Falta brazo impreso y pesas caseras | Curva par-velocidad, holgura e identificación BAM con carga conocida | Modelo propio; 5 minutos a par de apoyo a un pie por debajo de 70 °C |
| 5.2 | 3 servos más, hasta 6, y piezas de una pierna | Pierna sujeta, comparar con simulación | Respuesta real parecida a la simulada, criterio numérico por fijar antes |
| 5.3 | Electrónica | Programa: bucle de 50 Hz, ONNX, paro de seguridad, dejar de caminar a 6,6 V y corte a 6,4 V | Bucle estable 10 minutos sin retrasos |
| 5.4 | Resto | Montaje, IDs y calibración | Todas las articulaciones responden |
| 5.5 | - | Del robot colgado al libre: postura, marcha y levantarse | R1 y R2 |
| 5.6 | Micrófono y altavoz, si procede | Voz en la nube, en un proceso aparte del bucle de 50 Hz | Criterio R11, y el bucle sigue estable |
| 5.7 | - | Comprobar todos los requisitos | Informe con R1 a R11, también lo que falle |

Si 5.1 falla, se vuelve a la etapa 2: reducir masa o reforzar las articulaciones que fallen, y repetir 3 y 4.

**Reutilizable:** el programa `xgoduck_runtime_arduino` sirve de consulta para ajustes del servo y protocolo.
Corre en UNO Q, no en nuestra Radxa.

**Configurar servos:** uno a uno, con FE-URT-2. Vienen con ID 1 y a 1 Mbit/s. Evita bloquearlos al probar, y
activa las protecciones de corriente, que vienen desactivadas.

## 11. Seguridad

- **Batería:** solo paquetes con soldadura por puntos y BMS. Nunca soldar celdas directamente. Revisar tras
  caídas fuertes. Cargar vigilada y sobre superficie no inflamable. A 6,6 V, Danny deja de caminar y avisa, y se sienta
  cuando tenga esa política. A 6,4 V,
  el programa corta, porque el regulador pide 6 V.
- **Tensión del servo:** máximo 8,4 V. Nunca conectar servos a 12 V.
- **Pruebas:** robot colgado o sujeto hasta que la marcha sea fiable, con paro accesible en la mano.
- **Primero sin carga:** cada política nueva, primero con el robot al aire y par bajo.
- **Calor:** vigilar la temperatura que informa cada servo. Corta a 80 °C.

## 12. Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que algo funcione en real porque funciona en simulación.
- Que un script de Fusion o una ventana de simulador se hayan ejecutado, si no se ha hecho.
- Que las cifras de `results/` sean de diseño: hasta tener CAD, son de primer orden.

## 13. Glosario

| Término | Significado |
|---|---|
| GDL | Grado de libertad: una articulación motorizada |
| CAD | Modelo 3D de diseño, en Fusion |
| MJCF | Formato de robot de MuJoCo |
| MuJoCo | Simulador físico |
| mjlab | Entorno de entrenamiento paralelo en GPU sobre MuJoCo Warp |
| Política | Red neuronal que decide los movimientos |
| PPO | Algoritmo de aprendizaje por refuerzo |
| ONNX | Formato para llevar la política al robot |
| BAM | Modelo de actuador que representa mejor la fricción del servo |
| Sim-to-real | Lograr que lo aprendido en simulación funcione en real |
| Aleatorización de dominio | Variar la física al entrenar para ganar robustez |
| Par de bloqueo | Par máximo con el servo parado; no es sostenible |
| Par nominal | Par que el fabricante indica para uso normal |
| IMU | Sensor de giro y aceleración |
| BMS | Protección electrónica de la batería |
| BOM | Lista de materiales |

## 14. Mantener este documento

Se actualiza con cada cambio de método, convención, herramienta o criterio. Las cifras viven en `results/` y los
estados en el ROADMAP: aquí solo lo necesario para trabajar. **Criterio de calidad:** un agente sin contexto debe
poder decir qué toca y por qué leyendo solo el repositorio.
