# Investigación y requisitos

**Etapa 1, en curso.** Nada está comprado ni diseñado. Las cifras de comparación salen de
`analysis/compare_actuators.py` y `results/actuator_comparison.json`.

**Etiquetas:** **V**, dato del fabricante o del proyecto. **S**, tienda. **E**, estimación. **?**,
desconocido.

## 1. Decisiones del usuario (31 de octubre de 2026)

| Tema | Decisión |
|---|---|
| Referencia | El XGO-Duck inspira, no se copia: cada pieza se elige por lo que necesita |
| Tamaño | 32-36 cm, compacto |
| Presupuesto | Hasta 1 500 € con repuestos, sin impresora |
| Ordenador | Pequeño, tipo Raspberry Pi 5, con cámara. Modelo por elegir |
| Entrenamiento | mjlab en la RTX 3080 Ti, en lugar de Isaac Lab |
| Impresora | Cerrada multimaterial |
| Prioridad | Caminar natural y levantarse. El resto, si el diseño lo permite |

## 2. Requisitos y criterios

Se comprueban en el robot real. Pueden cambiar con justificación escrita.

| Id | Requisito | Criterio | Nivel |
|---|---|---|---|
| R1 | Caminar natural | 0,25 m/s en interior, 5 minutos sin caer, con braceo y giro de torso | Obligatorio |
| R2 | Levantarse | Desde boca arriba y boca abajo, 8 de 10 intentos | Obligatorio |
| R3 | Gestos | Saludo y baile de cuerpo entero sin caer | Obligatorio |
| R4 | Masa | 1,6 kg o menos, medidos | Obligatorio |
| R5 | Autonomía | 20 minutos caminando | Obligatorio |
| R6 | Coste | 1 500 € o menos | Obligatorio |
| R7 | Trotar | Pasos rápidos a 0,4 m/s; fase de vuelo si se logra | Deseable |
| R8 | Saltar | Ambos pies a 1 cm del suelo y aterrizar de pie | Deseable |
| R9 | Manipular | Coger con visión un objeto de 50 g de una mesa | Deseable |

Se reserva desde el diseño espacio, masa y bus para R9: muñeca, pinza y cámara. Los límites térmicos
se fijan al medir el servo, en la etapa 1.3.

## 3. Referencias

| | XGO-Duck | Microduck | Open Duck Mini | ToddlerBot | Zeroth Bot |
|---|---|---|---|---|---|
| Altura | ? | 25 cm V | 42 cm V | 56 cm V | 48 cm (tabla de ToddlerBot) |
| Masa | ? | Menos de 800 g V | 2,1 kg de modelo V | 3,4 kg V | 3,6 kg (tabla de ToddlerBot) |
| GDL | 15 V | 15 V | 14 V | 30 V | 16 (tabla) |
| Servo | HD-1910-C001 V | XL330 V | STS3215 V | Dynamixel V | Feetech, modelo ? |
| Brazos | No | No | No | Sí | Sí |
| Ordenador | UNO Q V | RK3566 V | Pi Zero 2W V | Jetson Orin NX V | ? |
| Precio | Unos 400 $ | 399 $ V | 398 € V | Unos 6 000 $ V | 350 $ a 1,4 k$, en conflicto |
| Entrenamiento | mjlab, PPO, BAM V | mjlab, PPO, BAM V | MuJoCo Playground V | MJX, Brax V | K-Sim V |

**Conclusiones:**

- **Receta probada para este tamaño:** mjlab, PPO, 50 Hz, modelos de actuador BAM, aleatorización de
  dominio, simulación de holgura y exportación a ONNX. Microduck indica 1-2 horas para una marcha
  usable con 4 096 entornos.
- **Reutilizable:** el código de `xgoduck_rl` es Apache 2.0 e incluye un modelo BAM del servo
  (`1910_m6.json`). `microduck_rl` trae además una guía de recetas de entrenamiento.
- **No reutilizable sin permiso:** las piezas de XGO-Duck, sin licencia final según su `LICENSING.md`.
  Diseñamos las nuestras.
- **Sin precedente:** no he encontrado ningún humanoide con brazos que camine con HD-1910. Es el mayor
  riesgo del proyecto.

## 4. Comparación de actuadores

| Servo | Tensión | Bloqueo | Nominal | Vacío | Masa | Precio |
|---|---|---|---|---|---|---|
| Feetech HD-1910-C001 | 7,4 V (máx. 8,4) | 1,47 N·m V | 0,36 N·m V | 11,9 rad/s V | 21 g V | 29,95-32,95 € S |
| STS3215 C018 | 12 V | 2,94 N·m S | 0,98 N·m S | 4,7 rad/s S | 55 g S | 26,51 € S |
| STS3215 C044 | 7,4 V | 1,6 o 2,7 N·m, en conflicto S | 0,88 N·m S | 9,0 rad/s S | 55 g S | 21,89-23,57 € S |

El HD-1910 tiene reducción 1:320, motor sin núcleo, holgura hasta 0,5° y encoder de 12 bits (V). Las
fichas del HD-1910 discrepan en el par: 10 kg·cm a 6 V en la wiki de Feetech, y 12 kg·cm a 6 V y 15 kg·cm
a 7,4 V en la ficha del proveedor que recoge el XGO-Duck. Se usa esta última, la más optimista: hay que
confirmarla al medir. **El par continuo real está sin medir.**

Primer orden con 12 servos de pierna, margen de 1,5 y velocidad en vacío como techo:

| Reparto, 34 cm, 22 GDL | Masa | Par pico con margen | Supera nominal | Marcha humana equivalente | Salto cargado |
|---|---|---|---|---|---|
| **Todo HD-1910** | 1,41 kg | 45 % del bloqueo | Sí | **1,05 m/s** | 1,7 cm |
| Piernas C044, arriba HD-1910 | 1,82 kg | 53 % del bloqueo | No | 0,79 m/s | 1,0 cm |
| Todo C018 | 2,16 kg | 34 % del bloqueo | Sí | 0,41 m/s | 0,3 cm |

Con 26 GDL y todo HD-1910, 1,49 kg y 47 %. De 32 a 36 cm, con 22 GDL: de 1,31 a 1,52 kg y de 40 a 50 %.

**Lectura:**

- **Candidato: todo HD-1910.** Mezclar con STS3215 en las piernas pesa más y va más lento. El C018
  necesita además 12 V, por encima de los 8,4 V del HD-1910.
- **Punto débil: el calor.** Los picos superan los 0,36 N·m nominales. Hay que medir si aguanta el
  apoyo a un pie. Si no, las opciones son bajar a 32 cm, reducir masa o reforzar solo las articulaciones
  que fallen.
- **Saltar y correr:** con la reducción 1:320, el salto estimado es de unos 2 cm, con techo teórico de 5
  cm. La velocidad en vacío no se alcanza bajo carga, y el 60 % usado es una estimación. El aterrizaje
  no está evaluado y puede dañar los engranajes. R7 y R8 pueden requerir actuadores de menor reducción.

Este estudio simplifica mucho: la masa de estructura se escala desde Open Duck, ignora la curva
par-velocidad y no incluye levantarse, que puede ser la carga mayor en los hombros.

## 5. Preguntas abiertas de la etapa 1

- Aguante térmico del HD-1910 a 7,4 V, y curva par-velocidad.
- Si `1910_m6.json` coincide con nuestros servos, y si HD-1910 y STS3215 conviven en un bus.
- Otros actuadores compactos para las piernas, para R7 y R8.
- Modelo de ordenador, batería y su lugar en el torso. Medidas del XGO-Duck: placa de 68,55 × 53,38 mm V.
- Altura exacta y reparto de 22 a 26 GDL. Decidir con cálculo si la pierna lleva 5 o 6 GDL.
- Fuente de movimiento humano y sus licencias, para imitación.
- ACDC4Robot: comprobar con un modelo simple que exporta bien a MJCF desde Fusion.

## 6. Fuentes

[XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware) ·
[Ficha del servo en XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware/blob/main/docs/BOM.md) ·
[xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl) ·
[Licencias de XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware/blob/main/LICENSING.md) ·
[Guía de XGO-Duck](https://xgo-duck.com/) ·
[Compra de servos](https://www.roboduckbuild.com/diary/sixteen-servos-on-pre-order/) ·
[HD-1910 en OpenELAB](https://openelab.io/products/feetech-hd1910-microduck-coreless-servo) ·
[Wiki de Feetech](https://ftservo.github.io/ftservo-wiki/en/products/models/hd-1910-c001/main/) ·
[Microduck](https://pollen-robotics.com/microduck/blog/introducing-microduck/) ·
[Runtime de Microduck](https://github.com/pollen-robotics/microduck) ·
[microduck_rl](https://github.com/pollen-robotics/microduck_rl) ·
[mjlab](https://github.com/mujocolab/mjlab) ·
[BAM](https://github.com/Rhoban/bam) ·
[Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini/tree/v2) ·
[ToddlerBot](https://arxiv.org/html/2502.00893v3) ·
[Zeroth Bot](https://github.com/zeroth-robotics/zeroth-bot) ·
[ACDC4Robot](https://github.com/ACDC4Robot/Fusion360)
