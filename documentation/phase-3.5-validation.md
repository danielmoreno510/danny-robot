# Validación de la fase 3.5 — Robot Danny

**Clasificación final: CONDITIONAL PASS, revisada y aceptada por el usuario.** El humanoide de 31 GDL y
11 kg está construido y cumple los criterios de la fase 3, con las condiciones de la sección 4.
**Todavía no se puede entrenar:** falta lo de la sección 6.

**Cómo se obtuvieron los datos.** Las cifras salen de `results/`, generado sin visor. Para
repetirlo:

```bash
.venv/bin/python simulation/scripts/run_checks.py
.venv/bin/python -m pytest tests
```

**Resultado de la ejecución limpia:** 27 de 27 comprobaciones, 236 de 236 pruebas en 19 min y
pyflakes sin avisos. **Ojo:** las pruebas de realismo y varias de medida registran fallos sin
exigir que pasen. Las cifras reales están en `results/phase3_metrics.json`.

**No se ha abierto ninguna ventana del visor.** Ese camino se probó con una ventana falsa.

## 1. Qué se construyó

| Parte | Resultado |
|---|---|
| Modelo | 31 GDL, 11,004 kg, 70 cm, generado desde `robot_spec.py` |
| Piernas | 6 GDL × 2; extensión de cadera a 30° (el bípedo sigue en 90°) |
| Cintura | Yaw, roll y pitch |
| Brazos | 7 GDL × 2 |
| Manos y cabeza | Rígidas y simples, sin dedos ni cara, por decisión tuya |
| Motores | Cada articulación con su motor de catálogo, su par y su curva de velocidad |
| Sensores | Posición, velocidad y par por articulación; giróscopo y acelerómetro |
| Postura inicial | Keyframe `stand`, con la rodilla a 30° |

## 2. Comparación con el bípedo

Mismo controlador y mismos criterios. El humanoide lleva 3 kg más y motores reales, en vez de los
20 N·m genéricos del bípedo.

| Medida | Bípedo (8 kg) | Humanoide (11 kg) |
|---|---|---|
| E11, marcha: fallos | 0 | 0 |
| E11: inclinación máxima | 0,68° | 0,91° |
| E11: par máximo | 6,2 N·m | 8,5 N·m |
| E11: error en movimiento | 0,43° | 0,52° |
| E12, marcha rápida: duración | 26,6 s | 28,3 s, con 0,35 s de espera y no 0,25 s |
| E12: inclinación máxima | 1,06° | 1,44° |
| E13, con giro: inclinación máxima | 0,50° | 0,58° |
| E14, empujón máximo adelante | 26 N | 48 N |
| E14, atrás | 26 N | 49 N |
| E14, de lado | 34 N | 59 N |
| Empujón a un pie, adelante | 24 N | 44 N |
| Empujón andando | pasa 5 N | pasa 5 N |
| Auditoría de masa | pasa ±10 % | pasa +10 % y -5 %; **incumple -10 %** |

Que aguante empujones mayores es coherente con tener más masa sobre el mismo pie, aunque no se ha
aislado esa causa. La sensibilidad
completa, con sus 36 medidas, está en E23 del handbook y es casi igual en los dos.

## 3. Auditoría independiente del código

La hizo una revisión independiente, sin conocer mis conclusiones, en una copia limpia. **No
encontró ningún defecto crítico ni de física, y ninguna cifra de aceptación cambia.** Encontró
pruebas que afirmaban más de lo que medían.

**Corregido:**

| Hallazgo | Corrección |
|---|---|
| El ζ 0,63 del brazo ignoraba el `kv` del actuador | Ahora se registran los dos: pasivo 0,63 y con `kv` 6,2 |
| Un par recortado por la curva de velocidad no contaba como saturación | Cuenta, con una prueba directa |
| «2 veces el par contra el tope» era en realidad 1 vez neta | El motor ya no se opone: 2,87°, límite 3° |
| `center_com_over` podía fallar sin avisar | Ahora lanza un error |
| `--turn` negativo no hacía nada | Ahora da error |
| La prueba de imports entre scripts no veía `import x` | Ya lo ve |
| Pruebas sin valor o con nombre engañoso | Dos eliminadas y dos renombradas, y una más estricta |
| `pyflakes` no estaba instalado | En `requirements.txt` |

**Documentado sin cambiar**, en [limitaciones](known-limitations.md):

- El «ruido de encoder» se suma al mando, no a la medida. **Las cifras de ruido son optimistas.**
- El margen del CdM en la marcha se mide al final de cada fase, no en continuo. Medido en continuo
  en E13: 25,9 mm frente a 26,7 mm.
- `check_acceptance` solo sigue las piernas. Error máximo fuera de ellas: 1,2° en el hombro.
- Equilibrio y empujones no miran las autocolisiones. Medidas: ninguna.
- En las piernas, un ángulo positivo de roll o yaw significa lo contrario en cada lado. Importa
  para aumentar datos por simetría en la fase 4.
- `--view` no abre la ventana si el robot cae antes.
- Reiniciar sin keyframe deja las plantas 1 cm bajo el suelo. Usa `stand`.

**Sin cerrar:** el exceso de los empujones sobre la estimación probablemente tiene explicación. La
estimación de 43 N trata el empujón como instantáneo. La auditoría calculó que un cuerpo rígido
integrado durante los 0,1 s vuelca a 53,7 N, y 48 N queda entre ambos modelos. Es plausible, pero
no hay prueba en el repositorio y no se analizó el bípedo.

## 4. Condiciones del CONDITIONAL PASS

1. **Error de masa:** con -10 %, un pie queda a 5,1 mm, y el límite es de 5 mm.
2. **Marcha lenta y agachada:** 0,009 m/s con rodillas a 56°, y flexión dorsal de hasta 50°. Es
   una comprobación física, no una marcha natural.
3. **Sin equilibrio activo ni compensación dinámica.** Los empujones se aguantan de forma pasiva.
4. **Armature y amortiguamiento son estimaciones.** Faltan datos del STS3215 y la velocidad del
   RobStride 00.
5. **`kv` del brazo sobreamortiguado:** con ζ 6,2 el brazo no oscila libremente. La fase 4 debe
   decidirlo antes de buscar un balanceo natural.
6. **Fricción de suelo 1,5:** 38 fallos, sin explicar. En el bípedo, 10.
7. **Solo probado en macOS con Apple Silicon.** Holgura mecánica: NOT TESTED.

## 5. Lo que ya no bloquea

Sensores, postura inicial y rangos de cadera están hechos. La flexión dorsal se penalizará en la
recompensa, porque la marcha clásica la necesita.

## 6. ¿Se puede entrenar?

**Todavía no.** El cuerpo físico está listo. Falta:

1. Elegir GPU local o nube.
2. Comprobar que el modelo funciona en MJX. Sin hacer.
3. Verificar las licencias de AMASS y LAFAN1, y elegir licencia del repositorio.

Lo demás (entorno, datos adaptados y métricas) es el arranque de la propia fase 4. Ver la
[auditoría de preparación](phase-4-readiness-audit.md).
