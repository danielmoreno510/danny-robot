# Guía de entrenamiento: etapa 4

Para quien entrena a Danny en una computadora con GPU NVIDIA, y para su Claude. Con este documento y el repositorio basta.

**Danny:** humanoide de 40 cm y 1,29 kg (E), con 23 motores HD-1910. Todavía **no existe físicamente**: solo el CAD y la
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
| Par de bloqueo | 0,94 N·m a 7,4 V y 0,76 N·m a 6 V según el modelo; 1,47 N·m según ficha. Usa el modelo |
| Velocidad en vacío | 11,9 rad/s a 7,4 V |
| Batería | 2S: de 8,4 V a unos 6 V |
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
pruebas. Sin superar la curva del motor, y sin saturar ninguna articulación más del 5 % del tiempo.

### 4.3 Levantarse

Desde boca abajo y boca arriba, con la misma aleatorización.

**Terminado cuando:** se levanta en 8 de cada 10 intentos, en los dos casos, y termina estable 3 s.

**Es lo más incierto.** Una búsqueda sin realimentación lo logró, pero muy frágil. Lo que se aprendió:

- Boca abajo, las manos deben estar delante de la cabeza. Si empujan junto a la cadera, se voltea.
- El hombro no pasa por detrás: los brazos barren por el costado.
- Boca arriba, conviene darse la vuelta primero.
- La cadera llega a -100° y el tobillo a -30°, y no se puede ampliar sin rediseñar. Están medidos en el CAD.

Las secuencias están en `results/get_up_prone.json` y `results/roll_over.json`, y se ven con `analysis/view_simulation.py`.
Pueden servir de referencia, pero no son el objetivo: tienen un aspecto poco natural.

**Si no lo consigue tras un esfuerzo razonable:** para y explica por qué. Puede significar rediseñar la cadera, y eso lo decide
el dueño del proyecto.

### 4.4 Movimiento natural

**Espera a que estén elegidas las fuentes de movimiento humano** de la etapa 1.2. Aquí es donde deja de moverse raro.

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

Trabaja en una rama `entrenamiento-danny`, nunca en `main`. Sube:

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
