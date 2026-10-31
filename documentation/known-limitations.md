# Limitaciones conocidas

Estado actual del robot Danny. Salvo que se indique, las cifras son del bípedo
(`danny_biped.xml`, 70 cm, 8 kg, 12 GDL); el humanoide en construcción tiene su propia
sección. Las cifras salen de `results/phase3_metrics.json`, y el detalle está en la
[validación de la fase 3](phase-3-validation.md). Este documento se mantiene al día.

## Lo que NO está implementado

- **Equilibrio activo con realimentación.** Ningún controlador usa la inclinación ni
  la IMU para corregir la postura.
- **Compensación dinámica.** Se probó y se retiró, porque no aportaba con este cuerpo
  (commit `f24ed39`). No debe contarse como implementada.
- **Recuperación dando un paso.** Con control clásico, el robot saltaba en vez de dar
  un paso (commit `6c083cc`). Pasa a la etapa 26.
- **Marcha dinámica.** El centro de masas se mantiene siempre dentro del apoyo.

Estas capacidades siguen abiertas y pueden desarrollarse en la fase 4.

## Equilibrio pasivo

El robot sigue un plan fijo. Ante un empujón, el plan no cambia: lo sostiene la
rigidez de los servos. Medido, de pie y con empujón de 0,1 s en la pelvis:

| Condición | Máximo |
|---|---|
| Dos pies, adelante y atrás | 26 N |
| Dos pies, de lado | 34 N |
| Un pie, adelante y atrás | 24 N |
| Un pie, hacia el pie libre | 9 N |
| Andando, en un solo instante probado | 5 N sin incumplir criterios |

Los límites de `push_biped.py` (24 y 32 N) son lo medido menos 2 N. Detectan
regresiones, pero no demuestran que el robot cumpla un requisito.

## Sensibilidad a perturbaciones y errores de modelo

La marcha incumple sus criterios con desviaciones pequeñas, que un robot real tendrá:

| Efecto | Pasa | Incumple |
|---|---|---|
| Suelo inclinado | 0,1° | 0,25° |
| Centro de masas de pelvis y torso desplazado | 0,5 cm | 1 cm |
| Ruido de encoder | 0,05° | 0,1° |
| Contacto más blando | ×0,5 | ×2 (y cae con ×5) |
| Empujón andando | 5 N | 10 N |

Sin explicar: con fricción 1,5 (más agarre que el valor base) da 10 fallos de margen
del centro de masas, y los pies no deslizan.

## Modelo de actuadores

Los motores están idealizados:

- Son un PD perfecto: `τ = kp·(ctrl − q) − kv·q̇`, sin dinámica eléctrica.
- Saturan en ±20 N·m, un valor provisional, muy por encima de los 5,9 N·m que pide la
  marcha.
- La curva de par-velocidad (20 rad/s, provisional) se aplica solo en los empujones.
- No tienen fricción articular, holgura mecánica, calor ni límite de batería.
- El control va a 500 Hz sobre el estado exacto del simulador.
- `armature` (0,01 kg·m²) es una estimación, no un dato de motor.

La marcha sí tolera, en simulación: control de 50 Hz, 10 ms de retardo, 10 ms de
retardo de motor y fricción articular de 1 N·m. Con 20 ms de cualquiera de los dos
retardos, incumple.

## Humanoide en construcción

`danny_humanoid.xml` (11 kg, 31 GDL), tras E23:

- Cabeza simple y manos rígidas, sin dedos.
- Sensibilidad casi igual que la del bípedo: suelo 0,25°, 1 cm de CdM o 20 ms de retardo
  incumplen. Detalle en E23.
- Con masa real un 10 % menor, un pie queda a 5,1 mm de su sitio (límite: 5 mm). Con -5 % cumple.
- La marcha clásica es lentísima (Froude 0,00002): no dice nada de una marcha natural.
- Los brazos se han balanceado a ±20°, sin sincronizar con los pasos.
- Cada articulación tiene su motor, pero **`armature` y amortiguamiento son estimaciones
  sin medir**, y el `kp` sale de una regla. Los datos del STS3215 están sin confirmar. Sigue
  sin modelarse si los servos de 152:1 se dejan mover desde fuera.
- Con ±20° de roll de cintura hay contactos reales de brazos y torso con muslos y pelvis.
- El margen de masa del torso está agotado.
- El pie es plano y rígido.
- La marcha rápida necesita 0,35 s de espera entre pasos: con 0,3 s incumple por muy poco.
- El plan de la marcha no conoce la cintura. Se mide que tolera ±30° de yaw y 30° de
  pitch, a 0,25 Hz y sin carga, pero nada la usa para equilibrar.
- Los límites de empujón (46 a 57 N) son lo medido menos 2 N. El máximo en x, 48 N, supera
  los 43 N de la estimación instantánea. Probable causa: integrado durante el empujón, un cuerpo
  rígido volcaría a 53,7 N (cálculo de la auditoría, sin prueba en el repositorio).
- El brazo no oscila libremente: el amortiguamiento pasivo da ζ 0,63, pero con el `kv` del
  actuador sube a 6,2.
- La curva de par-velocidad no ha llegado a recortar en E10 a E14: no está probada en
  movimiento rápido.
- En las piernas, un ángulo positivo de roll o yaw significa lo contrario en cada lado. En los
  brazos significa lo mismo.
- Con `mj_resetData` sin keyframe, las plantas quedan 1 cm bajo el suelo. Usa `stand`.

## Modelo físico

- En el bípedo no hay brazos, cabeza ni cintura: el torso es una masa provisional.
- El reparto de masas no está medido.
- El contacto pie-suelo usa parámetros por defecto, no medidos.
- La planta es rígida.

## Pruebas

- Las 73 pruebas de `test_realism.py` miden sin exigir: que pasen no significa que el
  robot aguante ese efecto.
- El «ruido de encoder» se suma al mando. El compensador y `kv` usan el estado exacto, así que
  esas cifras son optimistas. La fase 4 debe modelarlo en la medida.
- El margen del CdM de la marcha se mide al final de cada fase, no en continuo.
- `check_acceptance` solo sigue las piernas. Equilibrio y empujones no miran autocolisiones.
- `--view` no abre la ventana si el robot cae antes.
- `test_response_is_passive_not_feedback` documenta un plan constante: no puede fallar.
- Las pruebas de XML idéntico solo se han comprobado en macOS: el keyframe sale de una iteración
  numérica.
- Holgura mecánica: NOT TESTED.
- Ventana del visor: no se ha abierto en ninguna auditoría.
- Solo probado en macOS con Apple Silicon.

## Danny compacto de 40 cm

- Masa y reparto estimados, y parámetros del servo de 7,4 V sobre servos de 12 V.
- Sin curva par-velocidad, sin controlador de marcha y sin colisiones reales.
- `compact_spec.py` repite parte de `build_model.py` hasta decidir el robot principal.
