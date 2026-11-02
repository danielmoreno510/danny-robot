# Diseño de cadera y pierna

Etapa 2.2, tercera iteración en CAD, pendiente de impresión y encaje real. Fuente de las medidas:
`cad/fusion/BuildRobot/BuildRobot.py`.

## Criterios de aceptación

Fijados antes de modelar. `analysis/check_robot.py` comprueba los automáticos.

| Criterio | Límite | Estado |
|---|---|---|
| Cada rango del handbook se alcanza solo, con los brazos abiertos 60° | Menos de 1 mm³ en 9 ángulos de cada rango | Cumple, tras cambiar el rango de yaw |
| Pose cero y poses combinadas | Sin interferencias | Cumple |
| MJCF exportado | Carga, con el robot completo | Cumple |
| Masa del MJCF frente al CAD | Menos de un 1 % | Cumple |
| Ejes en pose cero | Pitch +Y, roll +X, yaw +Z, a menos de 1° | Cumple |
| Topes del MJCF | Los del handbook | Cumple |
| Resultados y modelo salen del script actual | Mismo hash | Cumple |
| Encaje real con la maqueta impresa y pieza pesada | Lo hace una persona | **Pendiente: sin esto no se cierra la 2.2** |

**Criterio cambiado:** el yaw hacia dentro pasa de 30° a 15°. A 30° la puntera entra en el otro pie, y por cálculo el
choque empieza cerca de 15,7°. Aprobado por el usuario el 31 de octubre de 2026.

Masa del conjunto: 550 g, con 13 servos que suman 273 g y 277 g impresos, densidad estimada. El total del robot está en
[upper-body-design.md](upper-body-design.md). Cifras en `results/robot_check.json`, `results/robot_cad.json` y `results/robot_interference.json`.

## Decisiones de diseño

| Decisión | Por qué |
|---|---|
| Servo sacado del plano oficial | El STEP de la wiki de Feetech es de un HL-2915 |
| Servo de cintura en la pelvis, entre los de yaw, aprobado por el usuario | Yaw, roll y pitch apilados piden 64 mm donde había 48. Así la cadera sube sin tocar la altura total |
| Eje de cintura en X = +7,5 mm | El servo queda centrado en la pelvis |
| Caderas a ±29 mm y pie de 85 × 48 mm | Dan sitio al alma del eslabón yaw y al servo de tobillo |
| Eje de hip_pitch con el servo hacia atrás | Hacia arriba o adelante choca o sube demasiado |
| Eje de hip_roll a 22 mm sobre hip_pitch | Coste de la compacidad. Los ejes no se cruzan en un punto |
| Servo de `ankle_roll` adelantado 35 mm | Bajo el tobillo no cabe: pitch está a 35 mm y roll a 16 mm |
| Tapa exterior en cada caja de servo con eje en Y | El servo entra de lado, y su bocina pasa por el agujero de la pared opuesta |
| Pelvis partida en dos y tapa superior del eslabón yaw | Si no, el conjunto no se puede montar |
| Medidas en constantes del script | Sustituye a los parámetros de usuario de Fusion: regenerar es reproducible |
| Sin rodamiento añadido en el lado libre | Se apoya en la bocina libre del servo. Sin comprobar |
| Densidad 1,25 g/cm³ | Con paredes tan finas, 0,7 g/cm³ subestimaba unos 109 g (E) |

## Montaje

Revisado sobre los STL con rayos, sin montar nada real. Tornillos con cabeza de botón M2, de 1,2 mm de alto como
máximo: una cabeza cilíndrica no cabe en varias cajeras.

1. Pelvis inferior: cintura y yaw entran desde arriba. Cintura y yaw se atornillan por abajo, con la cabeza de yaw rehundida 1,3 mm: el eslabón pasa a 0,5 mm.
2. Eslabón yaw: sube desde abajo con el alma por la ventana. Atornilla la bocina inferior antes de montar roll.
3. Pelvis superior: baja alrededor del alma. Une las mitades con 4 M1,6 por arriba en (20; ±30) y (-26,4; ±29), y 4 tornillos más a los servos.
4. Tapa del yaw: encaja en la espiga, con los 4 tornillos de bocina, de cabeza rehundida.
5. Caja de `hip_pitch`: el servo entra de lado. Pon la tapa y los tornillos antes de la horquilla.
6. Servo de `hip_roll`: sin la chapa trasera, entra por -X. Después, chapa trasera y sus tornillos.
7. Roll link: sube entre los brazos del eslabón yaw. Los tornillos de bocina se ponen desde ±X.
8. Muslo, tibia, `ankle_link` y pie: conviene armar la pierna fuera de la pelvis. Cada servo entra de lado, con su tapa, y
   después se atornilla la horquilla. El servo del pie va en su cuna antes de unir el pie.

## Cómo reproducirlo

Con Fusion abierto y `DannyBridge` activo:

```bash
.venv/bin/python analysis/run_fusion_script.py BuildRobot --timeout 300
.venv/bin/python analysis/run_fusion_script.py ExportPrintParts
.venv/bin/python analysis/run_fusion_script.py SweepJointRanges --timeout 1500
.venv/bin/python analysis/run_fusion_script.py ExportModel
.venv/bin/python analysis/check_robot.py
.venv/bin/python analysis/check_assembly.py
.venv/bin/python analysis/estimate_fall_energy.py
```

Para ver el modelo:

```bash
.venv/bin/python -m mujoco.viewer --mjcf=cad/export/danny/danny.xml
```

## Limitaciones conocidas

- **Dos piernas a la vez:** los rangos valen por articulación, no combinados. Con ambas piernas, según un barrido por
  vóxeles anterior al estrechado de la pelvis, sin repetir: yaw hacia dentro desde unos ±8° y roll hacia dentro desde ±2° chocan los pies; yaw hacia
  fuera desde ±15° chocan los servos de roll; y `hip_pitch` -60° con yaw ∓30° chocan los muslos. La etapa 3 necesita
  colisiones reales entre eslabones.
- **Fijación incompleta:** el servo de yaw lleva 2 de 4 tornillos, con apoyo lateral, porque su columna cercana está en
  el recorrido del alma. Da unos 55 N (E) a par de bloqueo. La chapa trasera de roll lleva 2 tornillos de pestaña, de 3,8 mm
  como máximo, que roscan solo 1,6 mm.
- **Secciones finas:** quedan puentes bajo 1,2 mm, como las de la tapa del yaw, y
  0,37 mm entre cajera y agujero en la parte exterior de la pared. Sin regla automática de pared mínima.
- **Lado motriz sin definir:** falta saber qué cara del servo es la motriz. Si el par pasa por el alma del eslabón
  yaw o por la barra del `ankle_link`, las tensiones estimadas superan a las del material a par de bloqueo.
- **Resistencia:** solo hay órdenes de magnitud, sin simulación ni ensayo.
- **Cables y conectores:** las ventanas de cada servo suponen la salida por las caras anchas, sin confirmar. Las ranuras
  dejan unos 79 mm² de sección en muslo y tibia (E, sin medir tras el último cambio).
- **Montaje:** las comprobaciones automáticas no ven tornillos, herramientas, paredes mínimas ni holguras por debajo
  de una interferencia real.
- **Pelvis estrecha:** los choques de dos piernas de arriba se deben medir de nuevo.
