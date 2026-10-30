"""
Danny - Esqueleto maestro paramétrico para Autodesk Fusion (etapa 5.1).

Crea un diseño nuevo con los parámetros de usuario, los componentes del robot y dos
bocetos totalmente restringidos, cuyas cotas dependen de esos parámetros. Después mueve
un parámetro para comprobar que el esqueleto lo sigue, lo restaura y escribe
skeleton_report.json junto a este archivo.

No edites skeleton_parameters.json a mano: sale de robot_spec.py con
simulation/scripts/export_cad_parameters.py. Coordenadas del JSON en mm, ejes REP-103.
"""

import json
import os
import sys
import traceback

import adsk.core
import adsk.fusion


HERE = os.path.dirname(os.path.abspath(__file__))
MM = 0.1  # la API trabaja en cm


def point3d(point_mm):
    return adsk.core.Point3D.create(*(value * MM for value in point_mm))


def add_parameters(design, parameters):
    for name, item in parameters.items():
        value = adsk.core.ValueInput.createByString(f"{item['value_mm']} mm")
        if design.userParameters.add(name, value, "mm", item["comment"]) is None:
            raise RuntimeError(f"no se pudo crear el parámetro {name}")


def add_components(parent, tree, found):
    for name, children in tree.items():
        occurrence = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occurrence.component.name = name
        found[name] = occurrence.component
        add_components(occurrence.component, children, found)


def constrain_axis(sketch, line, start, end):
    """Horizontal o vertical en los ejes del boceto, que no son los del modelo."""
    if abs(end.x - start.x) > abs(end.y - start.y):
        sketch.geometricConstraints.addHorizontal(line)
    else:
        sketch.geometricConstraints.addVertical(line)


def add_sketch(component, definition):
    plane = getattr(component, f"{definition['plane']}ConstructionPlane")
    sketch = component.sketches.add(plane)
    sketch.name = definition["name"]
    points = {"origin": sketch.originPoint}
    for segment in definition["segments"]:
        start_point = points[segment["from"]]
        end_model = point3d(definition["points"][segment["to"]])
        end_sketch = sketch.modelToSketchSpace(end_model)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(start_point, end_sketch)
        line.isConstruction = True
        constrain_axis(sketch, line, start_point.geometry, end_sketch)
        text = adsk.core.Point3D.create(
            (start_point.geometry.x + end_sketch.x) / 2 + 1, (start_point.geometry.y + end_sketch.y) / 2 + 1, 0)
        dimension = sketch.sketchDimensions.addDistanceDimension(
            line.startSketchPoint, line.endSketchPoint,
            adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, text)
        dimension.parameter.expression = segment["expression"]
        points[segment["to"]] = line.endSketchPoint
    return sketch, points


def add_named_points(component, sketch_name, points):
    for name, point in points.items():
        if name == "origin":
            continue
        construction = component.constructionPoints.createInput()
        construction.setByPoint(point)
        added = component.constructionPoints.add(construction)
        added.name = f"{sketch_name}_{name}"


def world_mm(point):
    geometry = point.worldGeometry
    return [round(geometry.x / MM, 4), round(geometry.y / MM, 4), round(geometry.z / MM, 4)]


def self_test(design, test, sketch_points):
    """Sube un parámetro, mide cuánto se mueve el punto y lo deja como estaba."""
    point = sketch_points[test["sketch"]][test["point"]]
    parameter = design.userParameters.itemByName(test["parameter"])
    original = parameter.expression
    before = world_mm(point)
    parameter.expression = f"{parameter.value / MM + test['delta_mm']} mm"
    design.computeAll()
    moved = world_mm(point)
    parameter.expression = original
    design.computeAll()
    return {
        "moved_mm": sum(abs(a - b) for a, b in zip(before, moved)),
        "restored": world_mm(point) == before,
    }


def build(app):
    with open(os.path.join(HERE, "skeleton_parameters.json"), encoding="utf-8") as file:
        wanted = json.load(file)

    orientation = app.preferences.generalPreferences.defaultModelingOrientation
    if orientation != adsk.core.DefaultModelingOrientations.ZUpModelingOrientation:
        raise RuntimeError("Fusion debe estar con Z arriba: cámbialo en Preferencias, reinicia y repite")

    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    add_parameters(design, wanted["parameters"])

    components = {}
    add_components(design.rootComponent, wanted["components"], components)
    skeleton = components[wanted["skeleton_component"]]

    sketches, sketch_points = {}, {}
    for definition in wanted["sketches"]:
        sketch, points = add_sketch(skeleton, definition)
        add_named_points(skeleton, definition["name"], points)
        sketches[definition["name"]] = sketch
        sketch_points[definition["name"]] = points

    report = {
        "python": sys.version,
        "fusion": app.version,
        "parameters": {p.name: p.expression for p in design.userParameters},
        "sketches": {
            name: {
                "fully_constrained": sketch.isFullyConstrained,
                "points": {key: world_mm(point) for key, point in sketch_points[name].items()},
            }
            for name, sketch in sketches.items()
        },
        "self_test": self_test(design, wanted["self_test"], sketch_points),
    }
    with open(os.path.join(HERE, "skeleton_report.json"), "w", encoding="utf-8") as file:
        json.dump(report, file, indent=1, ensure_ascii=False)
    return report


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        report = build(app)
        restrained = all(item["fully_constrained"] for item in report["sketches"].values())
        ui.messageBox(f"Esqueleto creado. Bocetos restringidos: {restrained}.\n"
                      f"Movimiento de prueba: {report['self_test']['moved_mm']:.2f} mm.\n"
                      "Informe en skeleton_report.json.")
    except Exception:
        ui.messageBox(f"Fallo al crear el esqueleto:\n{traceback.format_exc()}")
