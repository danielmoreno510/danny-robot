"""
Danny - Exporta a cad/print/ las piezas impresas del diseño activo, sin los servos.

Un STL por cuerpo de estructura, tapa, panel, protección o suela, en la orientación del modelo: gíralas en el
laminador. Devuelve volumen y masa de cada una, y falla si alguna sale en varios trozos.
"""

import os

import adsk.fusion


PRINT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "print")
PRINTED = ("_structure", "_cover", "_plate", "_sole", "_panel", "_pad")
EXPECTED_PARTS = 62


def build(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    os.makedirs(PRINT_DIR, exist_ok=True)
    parts = {}
    split = []
    for occurrence in design.rootComponent.occurrences:
        for body in occurrence.bRepBodies:
            if not body.name.endswith(PRINTED):
                continue
            if body.lumps.count != 1 or body.name in parts:
                split.append(body.name)
            options = design.exportManager.createSTLExportOptions(body, os.path.join(PRINT_DIR, body.name))
            options.sendToPrintUtility = False
            options.isBinaryFormat = True
            options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            design.exportManager.execute(options)
            properties = body.physicalProperties
            parts[body.name] = {"volume_cm3": round(properties.volume, 2), "mass_g": round(properties.mass * 1000, 1)}
    if len(parts) != EXPECTED_PARTS:
        raise RuntimeError(f"{len(parts)} piezas, no {EXPECTED_PARTS}")
    if split:
        raise RuntimeError(f"piezas en varios trozos: {split}")
    return parts
