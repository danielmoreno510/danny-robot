"""
Danny - Piezas del banco del servo (etapa 5.1): soporte para sujetar un HD-1910 al borde de la mesa y brazo de carga de 100 mm.

Crea un diseño nuevo y escribe los STL en cad/print/bench/. Usa el servo, el hueco, los tornillos y la bocina de BuildRobot, que es la
fuente de las medidas, para que el servo encaje igual que en el robot.

Soporte: caja abierta por arriba con el hueco del servo, eje horizontal (Y), atornillado por sus 4 agujeros de fijación (30 × 16, M2 × 5
como en el robot), y una lengüeta de 90 mm hacia -X con dos agujeros de Ø4,5 para un sargento o tornillos. El eje queda a 18 mm del
borde de la lengüeta para que el brazo pueda bajar por fuera de la mesa. La muesca del cable se pone hacia arriba (abierto); las bocinas entran por una ranura hasta su agujero.

Brazo: cubo de Ø16 y 2,5 mm sobre la bocina motriz con 4 M1,6 × 4 en diagonal, cajera central de Ø7 × 1 mm para el tornillo central, barra de
6 × 12 mm, 1,2 mm por encima de la cara de la bocina, y agujero de Ø4 a LOAD_RADIUS del eje para colgar la carga. Se imprime con la barra apoyada en la cama (boca abajo), para no necesitar soportes.
"""

import importlib.util
import os

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PRINT_DIR = os.path.join(os.path.dirname(os.path.dirname(HERE)), "print", "bench")

_spec = importlib.util.spec_from_file_location("danny_robot_geometry", os.path.join(os.path.dirname(HERE), "BuildRobot", "BuildRobot.py"))
robot = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(robot)

LOAD_RADIUS = 100.0  # mm del eje al agujero de carga
WALL = robot.SHELL_Y - robot.COVER_Y  # pared del soporte en las caras de bocina, como en el robot (2,5 mm)
FLOOR = 4.0  # suelo bajo el servo
TONGUE = (-90.0, -14.0)  # lengüeta en X, sobre la mesa; el eje (x = 0) queda fuera del borde
TONGUE_THICKNESS = 6.0
HUB_RADIUS = robot.HORN_CLEARANCE_RADIUS - 0.3  # dentro del agujero de la bocina del soporte (8,6); con 8, las cajeras de los M1,6 eran tangentes al borde
HUB_THICKNESS = robot.ARM  # 2,5 mm, como las placas de horquilla del robot
BAR_THICKNESS = 6.0
BAR_HALF_WIDTH = 6.0
# La barra va 1,2 mm por encima de la cara de la bocina: la bocina medida asienta a 14,25 mm del eje y la pared del soporte llega a
# 14,4, así que la barra pasa sobre la pared con 1 mm de holgura.
BAR_LIFT = 1.2
TIP_RADIUS = BAR_HALF_WIDTH + 1.0  # punta redonda; si es tangente a la barra, la malla no cierra
HORN_SLOT_HALF = robot.HORN_CLEARANCE_RADIUS - 0.3  # la bocina mide Ø15,7; no tangente al agujero de Ø17,2


def holder():
    servo = robot.Servo((0, 0, 0), (0, 1), (1, 1))  # largo en X, eje en Y, ancho en Z (vertical)
    bottom = -robot.SERVO_WIDTH / 2 - robot.FIT - FLOOR
    x0, x1 = -robot.SERVO_SHORT_END - robot.FIT - WALL, robot.SERVO_LONG_END + robot.FIT + WALL
    body = robot.merged([
        robot.box(x0, x1, -robot.SHELL_Y, robot.SHELL_Y, bottom, robot.SERVO_WIDTH / 2 + 1),  # 1 mm sobre el servo: las cajeras de arriba no son tangentes
        robot.box(TONGUE[0], x1, -robot.SHELL_Y, robot.SHELL_Y, bottom, bottom + TONGUE_THICKNESS),
    ])
    # Ranura para las bocinas, que sobresalen 3 mm por cada cara: el servo entra desde arriba.
    horn_slot = robot.box(-HORN_SLOT_HALF, HORN_SLOT_HALF, -robot.SHELL_Y - 1, robot.SHELL_Y + 1, 0, 30)
    open_top = robot.box(-robot.SERVO_SHORT_END - robot.FIT, robot.SERVO_LONG_END + robot.FIT, -robot.COVER_Y, robot.COVER_Y, 0, 30)
    holes = [robot.cylinder((x, 0, bottom - 1), (x, 0, bottom + TONGUE_THICKNESS + 1), 2.25) for x in (TONGUE[0] + 12, TONGUE[1] - 22)]
    return robot.carved(body, servo.pocket() + servo.head_recesses(robot.SHELL_Y) + [open_top, horn_slot] + holes)


def load_arm():
    diagonal = robot.HORN_BOLT_RADIUS / 2 ** 0.5
    hub = robot.cylinder((0, 0, 0), (0, 0, HUB_THICKNESS), HUB_RADIUS)
    bar = robot.box(HUB_RADIUS - 2, LOAD_RADIUS, -BAR_HALF_WIDTH, BAR_HALF_WIDTH, BAR_LIFT, BAR_LIFT + BAR_THICKNESS)
    tip = robot.cylinder((LOAD_RADIUS, 0, BAR_LIFT), (LOAD_RADIUS, 0, BAR_LIFT + BAR_THICKNESS), TIP_RADIUS)
    cuts = [robot.cylinder((0, 0, -1), (0, 0, robot.CENTER_RELIEF_DEPTH + robot.FORK_GAP), robot.CENTER_RELIEF_RADIUS)]
    for x in (-diagonal, diagonal):
        for y in (-diagonal, diagonal):
            cuts.append(robot.cylinder((x, y, -1), (x, y, HUB_THICKNESS + 1), robot.SCREW_RADIUS))
            cuts.append(robot.cylinder((x, y, HUB_THICKNESS - robot.HEAD_DEPTH), (x, y, HUB_THICKNESS + 1), robot.HEAD_RADIUS))
    cuts.append(robot.cylinder((LOAD_RADIUS, 0, -1), (LOAD_RADIUS, 0, BAR_LIFT + BAR_THICKNESS + 1), 2.0))
    return robot.carved(robot.merged([hub, bar, tip]), cuts)


def build(app):
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    base = root.features.baseFeatures.add()
    base.startEdit()
    bodies = {}
    for name, solid in (("bench_servo_holder", holder()), ("bench_load_arm", load_arm())):
        body = root.bRepBodies.add(solid, base)
        body.name = name
        bodies[name] = body
    base.finishEdit()
    os.makedirs(PRINT_DIR, exist_ok=True)
    report = {}
    for name, body in bodies.items():
        body = root.bRepBodies.itemByName(name)
        options = design.exportManager.createSTLExportOptions(body, os.path.join(PRINT_DIR, name))
        options.sendToPrintUtility = False
        options.isBinaryFormat = True
        options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        design.exportManager.execute(options)
        properties = body.physicalProperties
        report[name] = {"volume_cm3": round(properties.volume, 2), "lumps": body.lumps.count,
                        "bounds_mm": [[round(v * 10, 1) for v in body.boundingBox.minPoint.asArray()],
                                      [round(v * 10, 1) for v in body.boundingBox.maxPoint.asArray()]]}
    return report
