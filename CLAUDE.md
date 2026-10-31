# Robot Danny — instrucciones para Claude

Humanoide autónomo de 40 cm con aspecto de Tesla Optimus. El usuario escribe en español: responde en
español. **Empieza por [PROJECT_HANDOFF.md](PROJECT_HANDOFF.md)** y sigue con el
[handbook](documentation/HANDBOOK.md). El plan y el estado están en [ROADMAP.md](ROADMAP.md): es la fuente
de verdad y debe estar al día.

**El XGO-Duck es una referencia, no una plantilla.** Se toman su servo y su método, no su forma. Si algo
necesita otra pieza, se cambia: decide con números para cada articulación y componente.

**Objetivo de movimiento:** Danny debe moverse de forma natural, con todo el cuerpo coordinado. No
sacrifiques física, estabilidad ni viabilidad de motores para que se vea humano. No inviertas en
animaciones a mano que se vayan a sustituir.

## Reglas del usuario

1. **Documentación siempre al día.** Cada cambio actualiza lo que corresponda. Sin menciones obsoletas.
2. **Decir cómo verlo.** Tras cada cambio, da el comando exacto para verlo o ejecutarlo.
3. **Quitar lo que no se usa.** Git guarda el historial.
4. **Nombres descriptivos.** Sin versiones como `_v2`. Scripts verbo + objeto. El código compartido va en
   módulos: un script de análisis nunca importa de otro.
5. **Python 3.12**, siempre con el `.venv` del proyecto.
6. **Commit y push solo cuando el usuario lo pida.**
7. **Avisar al terminar cada etapa**, con implementación, validación reproducible y documentación.
8. **Dejar el relevo escrito** en `PROJECT_HANDOFF.md`: debe poder seguirse desde otro chat u otra
   computadora.
9. **Preguntar** cuando falte información que solo tenga el usuario, y avisar de lo que se le pase.

## Cómo se trabaja

- **Fijar los criterios de aceptación antes de ejecutar**, y calcular antes si son físicamente posibles.
  Si se cambian, decirlo y documentarlo.
- **Medir antes de cambiar.** Si un arreglo no explica el fallo, no se deja.
- **Decir lo que no se ha comprobado**, y documentar también lo que falla.
- **No copiar piezas de XGO-Duck:** no tienen licencia final. Su código de entrenamiento es Apache 2.0.
- Unidades SI; ejes REP-103.
- Las cifras de la documentación salen de `results/`, nunca de memoria. Las estimaciones se marcan.
- **No compres nada sin aprobación.**
- Audita de forma independiente al cerrar cada etapa.

## Estado actual

Etapas 2 y 3: CAD y simulación, a falta de pruebas físicas. El entrenamiento, etapa 4, se hace en otra computadora: ver
`documentation/training-guide.md`. Lo anterior está en la etiqueta `archivo-danny-70cm`: es solo consulta.
Decisiones en `documentation/research-and-requirements.md`, y piezas en
`documentation/architecture-and-bom.md`. Altura aprobada: 40 cm.
