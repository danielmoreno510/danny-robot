# Paneles y protección

Etapa 2.4, segunda iteración en CAD. Una revisión independiente sondeó la primera; sus correcciones no se revisaron de nuevo. Fuente de las medidas: `cad/fusion/BuildRobot/BuildRobot.py`. El aspecto sigue siendo
mecánico: es la primera capa, no el acabado de Optimus.

## Criterios de aceptación

Fijados antes de modelar. `analysis/check_robot.py` comprueba los automáticos.

| Criterio | Límite | Estado |
|---|---|---|
| Masa de paneles y protecciones | 100 g o menos | Cumple: 22 g (E) |
| Rangos, pose cero y poses combinadas | Sin interferencias | Cumple |
| R4 | 1,8 kg o menos | Cumple: 1,290 kg (E), con cables y tornillos estimados |
| R10 | Altura, pierna y hombros | Cumple: 395 mm, 50,6 % y 33,9 % |
| Ventilación, interruptor y salidas de cable | Sin tapar | Por diseño: panel del pecho con ranura inferior para el aire, muescas laterales para los cables y ventana del interruptor |
| Caída desde de pie | Estimada con números | Hecho: `results/fall_estimate.json` |

La masa baja respecto a la cifra anterior porque ya no suma 100 g de paneles estimados, sino los 22 g modelados.

## Qué se añadió

| Pieza | Dónde | Por qué |
|---|---|---|
| `torso_chest_panel` | Pecho, 74 × 62 × 6 mm, con facetas | Da la forma de pecho y afina el abdomen |
| `base_link_panel` | Delante de la pelvis | Cierra el frente de la pelvis |
| `*_thigh_panel`, `*_shin_panel` | Frente de muslo y tibia | Cubre la ventana de cada eslabón |
| `head_visor_panel` | Cara | Enmarca la cámara. Pensado para imprimirse en negro |
| `*_pad` | Pecho, espalda y rodillas, en TPU de 3 mm | Puntos de golpe en una caída |
| Rodamiento de 16 × 22 × 4 mm | Alrededor de la bocina de cintura | Toma parte de la carga lateral del torso, como hace XGO-Duck |

Sin paneles en brazos y hombros: hay 0,5 mm libres junto a la pelvis y los hombros están casi en el límite de R10. Quedan
unos 78 g libres del presupuesto de paneles.

## Montaje

Los paneles van al final, con tornillos M2 autorroscantes desde fuera. El visor y las almohadillas se pegan. El panel del
muslo sitúa su borde inferior con una lengüeta. El rodamiento se apoya en la pelvis superior, con holgura en su asiento, y
se pega cuando el torso ya está atornillado a la bocina. Así queda centrado en la bocina y no fuerza el eje del servo.

## Caída desde de pie

Cálculo de primer orden con 1,29 kg y el centro de masas a 224 mm: 2,8 J en un golpe. No se ha simulado ni medido.

| Qué frena el golpe | Recorrido (E) | Fuerza | Aceleración |
|---|---:|---:|---:|
| Panel rígido | 1 mm | 5 661 N | 447 g |
| Almohadilla y panel | 3 mm | 1 887 N | 149 g |
| El brazo se dobla primero | 15 mm | 377 N | 30 g |
| Además ceden los servos | 40 mm | 142 N | 11 g |

**Conclusión:** almohadillas y paneles no bastan por sí solos: con 3 mm, la fuerza supera a los tornillos del panel, estimados
en 600 N. La protección real debe venir de frenar con recorrido: doblar brazos y dejar que cedan los servos al detectar la
caída. Es una tarea de la etapa 4, y de la política de seguridad en la 5.3.

## Limitaciones conocidas

- **Aspecto:** falta forma en brazos, hombros y tronco inferior, que sigue siendo una caja.
- **Almohadillas:** 3 mm de TPU 95A comprimen muy poco.
- **Rodamiento:** ajuste sin comprobar, porque el diámetro real de la bocina no está medido. Agarra 2,3 de sus 4 mm.
  La pelvis solo lleva dos tornillos superiores a la cintura, más los cuatro inferiores. La espiga tiene 0,8 mm de pared, y
  el refuerzo del asiento solo 0,45 mm junto al alma.
- **Sin ensayo:** ningún panel se ha impreso ni golpeado, y el dato de 150 N por tornillo es una estimación.
