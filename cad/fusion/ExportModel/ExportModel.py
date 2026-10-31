"""
Danny - Exporta el diseño activo de Fusion a cad/export/<export_name>, en MJCF.

Usa el código de ACDC4Robot instalado en Fusion, sin sus diálogos: comprobación previa,
MJCF, mallas STL e informe. Si la comprobación falla no escribe nada.
"""

import importlib
import os
import sys

import adsk.fusion


EXPORT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "export")
ADD_INS_MAC = "~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns"
ADD_INS_WINDOWS = "~/AppData/Roaming/Autodesk/Autodesk Fusion 360/API/AddIns"


def load_exporter():
    add_ins = os.path.expanduser(ADD_INS_WINDOWS if sys.platform == "win32" else ADD_INS_MAC)
    if add_ins not in sys.path:
        sys.path.insert(0, add_ins)
    return {
        name: importlib.import_module(f"ACDC4Robot.{path}")
        for name, path in {
            "commands": "commands.ACDC4Robot.acdc4robot",
            "constants": "commands.ACDC4Robot.constants",
            "preflight": "core.preflight",
            "write": "core.write",
        }.items()
    }


def build(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        raise RuntimeError("el documento activo no es un diseño de Fusion")

    attribute = design.attributes.itemByName("danny", "export_name")
    if attribute is None:
        raise RuntimeError("falta el atributo danny/export_name: el componente raíz no se puede renombrar")
    name = attribute.value
    for joint in design.rootComponent.asBuiltJoints:
        if abs(joint.jointMotion.rotationValue) > 1e-6:
            raise RuntimeError(f"{joint.name} no está en pose cero")

    exporter = load_exporter()
    root = design.rootComponent
    exporter["constants"].set_robot_name(name)

    report = exporter["preflight"].inspect_mjcf_design(design)
    if not report["passed"]:
        raise RuntimeError(exporter["preflight"].format_report(report))

    folder = os.path.join(EXPORT_DIR, name)
    os.makedirs(folder, exist_ok=True)
    exporter["preflight"].write_report(folder, report)
    links, _ = exporter["commands"].get_link_joint_list(design)
    exporter["write"].write_mjcf(root, name, folder)
    exporter["commands"].export_stl(design, folder, links)
    source_hash = design.attributes.itemByName("danny", "source_hash")
    if source_hash:
        with open(os.path.join(folder, "source_hash.txt"), "w", encoding="utf-8") as file:
            file.write(source_hash.value)
    return {"model": os.path.join(folder, f"{name}.xml"), "warnings": report["warnings"]}
