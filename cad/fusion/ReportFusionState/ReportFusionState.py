"""
Danny - Informa del estado de Fusion sin cambiar ningún documento.

Sirve para comprobar que el puente responde y que Fusion tiene Z arriba.
"""

import adsk.core
import adsk.fusion


def build(app):
    orientation = app.preferences.generalPreferences.defaultModelingOrientation
    state = {
        "version": app.version,
        "user": bool(app.currentUser),
        "z_up": orientation == adsk.core.DefaultModelingOrientations.ZUpModelingOrientation,
        "active_document": app.activeDocument.name if app.activeDocument else None,
    }
    if app.activeViewport:
        up = app.activeViewport.camera.upVector
        state["camera_up"] = [round(up.x, 3), round(up.y, 3), round(up.z, 3)]
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design:
        box = design.rootComponent.boundingBox
        state["bounds_cm"] = [box.minPoint.asArray(), box.maxPoint.asArray()]
    return state
