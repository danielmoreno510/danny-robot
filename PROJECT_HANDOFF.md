# Relevo del proyecto

Punto de entrada para continuar **Robot Danny** en un chat nuevo o en otra computadora.

**Fecha:** 31 de octubre de 2026. **Etapa actual:** 3, MuJoCo, con 2.2, 2.3 y 2.4 abiertas por lo físico. La etapa 1 está
cerrada salvo la 1.2, que va en paralelo.
**Siguiente paso:** entrenar en la computadora de un amigo del usuario, que tiene la RTX 3080 Ti, con la
[guía de entrenamiento](documentation/training-guide.md). Levantarse es posible en simulación, pero aún frágil. El robot completo ya está en CAD:
[piernas](documentation/lower-body-design.md), [tronco](documentation/upper-body-design.md) y
[paneles](documentation/panels-and-protection.md). El usuario aún no tiene
impresora 3D, así que el encaje y el pesado quedan aplazados, y la 2.2 y la 2.3 siguen abiertas. No hace falta comprar un
servo para seguir: conectores y tornillos salen de proyectos publicados, y solo quedan por medir sus cotas exactas. Claude modela en Fusion por scripts, con `DannyBridge`. No se compra nada todavía.

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

## Lo que no se debe afirmar

- Que el HD-1910 sirva para este robot: nadie lo ha probado en un humanoide, según lo encontrado.
- Que Danny vaya a correr o saltar: el salto estimado es de unos 2 cm, sin evaluar el aterrizaje.
- Que las cifras de `results/actuator_comparison.json` sean de diseño: son de primer orden.

## Pendiente de decidir por el usuario

- Licencia del repositorio, que es público.
- Compra del banco de pruebas, cuando llegue la etapa 5.1. Precios de referencia en la investigación.
- Cuándo comprar impresora.

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
