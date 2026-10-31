"""
Danny - Pieza de prueba de la etapa 2.1: dos eslabones unidos por una bisagra.

Crea un diseño nuevo con base_link, fijo, y arm_link, que gira sobre la cara superior
de la base entre -90° y 90°. Devuelve masas y eje para compararlos con el MJCF exportado.
"""

import math

import adsk.core
import adsk.fusion


MM = 0.1  # la API trabaja en cm
BASE_MM = (40, 40, 40)
ARM_MM = (20, 20, 80)
LIMIT_RAD = math.radians(90)


def add_link(root, name, size_mm, height_mm):
    """Componente con una caja centrada en XY, que empieza a height_mm del origen."""
    occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    component = occurrence.component
    component.name = name

    planes = component.constructionPlanes
    plane_input = planes.createInput()
    plane_input.setByOffset(component.xYConstructionPlane, adsk.core.ValueInput.createByReal(height_mm * MM))
    sketch = component.sketches.add(planes.add(plane_input))

    half_x, half_y = size_mm[0] * MM / 2, size_mm[1] * MM / 2
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(-half_x, -half_y, 0), adsk.core.Point3D.create(half_x, half_y, 0))
    component.features.extrudeFeatures.addSimple(
        sketch.profiles.item(0),
        adsk.core.ValueInput.createByReal(size_mm[2] * MM),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    return occurrence


def face_at_height(occurrence, height_mm):
    """Cara plana de la caja a esa altura, en el contexto del ensamblaje."""
    for face in occurrence.bRepBodies.item(0).faces:
        if abs(face.pointOnFace.z - height_mm * MM) < 1e-6 and abs(face.area - 4) < 1e-6:
            return face
    raise RuntimeError(f"no hay cara a {height_mm} mm")


def add_hinge(root, base, arm):
    geometry = adsk.fusion.JointGeometry.createByPlanarFace(
        face_at_height(arm, BASE_MM[2]), None, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    # ACDC4Robot exporta la primera ocurrencia como hija y la segunda como padre
    joint_input = root.asBuiltJoints.createInput(arm, base, geometry)
    joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
    joint = root.asBuiltJoints.add(joint_input)
    joint.name = "hinge"

    limits = joint.jointMotion.rotationLimits
    limits.isMinimumValueEnabled = True
    limits.minimumValue = -LIMIT_RAD
    limits.isMaximumValueEnabled = True
    limits.maximumValue = LIMIT_RAD
    return joint


def build(app):
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    design.attributes.add("danny", "export_name", "export_test")

    base = add_link(root, "base_link", BASE_MM, 0)
    base.isGrounded = True
    arm = add_link(root, "arm_link", ARM_MM, BASE_MM[2])
    joint = add_hinge(root, base, arm)

    axis = joint.jointMotion.rotationAxisVector
    masses = {
        occurrence.component.name: occurrence.bRepBodies.item(0).physicalProperties.mass
        for occurrence in (base, arm)
    }
    return {
        "masses_kg": masses,
        "total_mass_kg": sum(masses.values()),
        "hinge_axis": [axis.x, axis.y, axis.z],
        "hinge_limits_deg": [-math.degrees(LIMIT_RAD), math.degrees(LIMIT_RAD)],
    }
