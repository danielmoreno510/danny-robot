"""
Danny - Robot completo con paneles: 24 eslabones, 23 articulaciones y electrónica colocada.

Crea un diseño nuevo. Este script es la fuente de las medidas: se cambia aquí y se
ejecuta otra vez. Coordenadas en mm en pose cero, ejes REP-103 y Z = 0 en el suelo. El
lado derecho es el izquierdo reflejado en Y. Explicación en
documentation/lower-body-design.md, documentation/upper-body-design.md y
documentation/panels-and-protection.md.
"""

import hashlib
import json
import math
import os

import adsk.core
import adsk.fusion


RESULTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
    "results", "robot_cad.json")
MM = 0.1  # la API trabaja en cm

# Servo HD-1910-C001, del plano oficial
SERVO_LONG_END = 24.5  # del eje al extremo largo
SERVO_SHORT_END = 9.5
SERVO_WIDTH = 20.0
SERVO_THICKNESS = 23.0
HORN_RADIUS = 8.0
HORN_LENGTH = 3.0
HORN_BOLT_RADIUS = 6.0
MOUNT_SPACING = (30.0, 16.0)
SERVO_MASS_G = 21.0

# Holguras y paredes
FIT = 0.2
HORN_CLEARANCE_RADIUS = 8.6
ARM = 2.5
SCREW_RADIUS = 1.1
HEAD_RADIUS = 2.0  # cabeza M2
HEAD_DEPTH = 1.2
PILOT_RADIUS = 0.7  # M1.6 roscado en plástico
M2_PILOT_RADIUS = 0.85  # M2 autorroscante en plástico
FORK_GAP = 0.2  # entre bocina y horquilla, para poder meterla
SHELL_Y = 14.4  # cara exterior de las cajas de servo con eje en Y
COVER_Y = SERVO_THICKNESS / 2 + FIT  # plano entre la caja y su tapa
FORK_INNER = 15.0
FORK_OUTER = 17.5

# Alturas
HIP_Y = 29.0
ANKLE_ROLL_Z = 16.0
ANKLE_PITCH_Z = 35.0
KNEE_Z = 117.0
HIP_PITCH_Z = 200.0
HIP_ROLL_Z = 222.0
ROLL_BOSS_RADIUS = 8.5
TENON = 1.5
HIP_YAW_Z = 255.0
PELVIS_BOTTOM = 241.0
PELVIS_TOP = 269.0
PELVIS_Y = 40.5
PELVIS_REAR = -29.5
PELVIS_FRONT = 22.0
WAIST_X = 7.5
HIP_ROLL_X = -7.5
ANKLE_ROLL_X = 35.5
FOOT_Y_OFFSET = 3.0

# Tronco: cintura, hombros y cuello comparten X, y el tronco se centra en ese eje
AXIS_X = WAIST_X
TORSO_X = (-19.7, 34.7)
TORSO_INNER_X = (-17.2, 32.2)
TORSO_Y = 39.0
TORSO_INNER_Y = 36.5
FLOOR_Z = (272.5, 275.0)
SHELF_Z = (313.0, 315.0)
TOP_PLATE_Z = (338.6, 341.0)
WAIST_BOSS_BOTTOM = 269.5  # apoya en la bocina
WAIST_BOSS_RADIUS = 8.9  # deja sitio a las cabezas de la pelvis
WAIST_BEARING = (8.0, 11.0, 4.0)  # 16 × 22 × 4 alrededor de la bocina: radios y ancho
WAIST_SEAT_RADIUS = 11.15  # con holgura: el aro se pega con el torso ya atornillado
WAIST_BEARING_Z = HIP_YAW_Z + 11.7
WAIST_SPIGOT_RADIUS = 7.9
SHOULDER_Z = 325.4
NECK_YAW_Z = 326.9
PITCH_Y = 24.8

# Brazo
ROLL_Y = 56.5
ARM_YAW_Y = 54.5
ARM_YAW_Z = 285.8
ARM_Y = (42.1, 66.9)
ARM_SHELF_Z = (297.5, 300.7)
ARM_CUP_BOTTOM = 271.9
ARM_CUP_X = (-20.9, 20.9)
ARM_SHELF_X = (-20.9, 26.9)
ARM_FOOT_SCREW_Y = (54.0, 59.0)  # entre los tornillos del servo de roll, que pasan por el pie de la tapa
ROLL_WALL_Y = (44.5, 66.7)
ELBOW_Y = 55.4
ELBOW_Z = 231.0
ELBOW_PLATE_Y = (69.9, 73.5)
ELBOW_TOP_Z = (265.1, 271.3)
FOREARM_Y = (47.0, 69.9)
FOREARM_END_Z = 148.0
FOREARM_HALF_X = 13.2
FOREARM_WALL = 1.4

# Cuello y cabeza
NECK_PLATE_Z = (341.4, 344.1)
NECK_PITCH_Z = 360.0
CUP_RADIUS = 15.2
HEAD_X = (-13.0, 33.0)
HEAD_Y = 25.0
HEAD_Z = (358.0, 395.0)
HEAD_WALL = 2.0
CASSETTE_TOP = 392.4
CASSETTE_COLUMN_X = 10.0

# Paneles y protecciones
PANEL_WALL = 1.4
PAD_THICKNESS = 3.0

# Electrónica: (mínimo, máximo) en X, Y, Z y masa en g. Ver documentation/architecture-and-bom.md
ELECTRONICS = {
    "battery": ((-19.6, -0.6), (-32.5, 32.5), (275.2, 312.2), 104.0),
    "computer": ((6.0, 14.0), (-32.5, 32.5), (277.0, 307.0), 15.0),
    "power_board": ((17.0, 29.0), (-30.0, 30.0), (277.0, 307.0), 15.0),
    "regulator": ((17.5, 26.5), (4.0, 22.0), (316.0, 336.0), 3.0),
    "imu": ((27.0, 32.0), (3.0, 29.0), (317.0, 335.0), 3.0),
    "switch": ((19.0, 31.0), (-28.0, 2.0), (316.0, 331.0), 8.0),
}
CAMERA = ((21.8, 30.8), (-12.5, 12.5), (361.0, 385.0), 4.0)
# Sitio reservado, sin masa: (mínimo, máximo) en X, Y, Z
VOICE_RESERVE = ((4.5, 29.5), (-32.0, 32.0), (307.5, 312.2))

# Densidades, en kg/m³
STRUCTURE_DENSITY = 1250.0  # PETG-CF (E): paredes tan finas salen macizas
SOLE_DENSITY = 1200.0  # TPU 95A (E)
BEARING_DENSITY = 7800.0  # acero (E)

temporary = adsk.fusion.TemporaryBRepManager.get()


def point(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def box(x0, x1, y0, y1, z0, z1):
    x0, x1, y0, y1, z0, z1 = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1)
    return temporary.createBox(adsk.core.OrientedBoundingBox3D.create(
        point((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
        adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
        (x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM))


def half_space(a, b, axis, outside, inset=0.0):
    """Sólido al lado de `outside` del plano que pasa por a y b y contiene ese eje, desplazado hacia dentro."""
    along = adsk.core.Vector3D.create(*[b[i] - a[i] for i in range(3)])
    direction = [0, 0, 0]
    direction[axis] = 1
    normal = along.crossProduct(adsk.core.Vector3D.create(*direction))
    normal.normalize()
    if sum(normal.asArray()[i] * (outside[i] - a[i]) for i in range(3)) < 0:
        normal.scaleBy(-1)
    side = normal.crossProduct(adsk.core.Vector3D.create(*direction))
    side.normalize()
    center = [a[i] + normal.asArray()[i] * (200 - inset) for i in range(3)]
    return temporary.createBox(adsk.core.OrientedBoundingBox3D.create(
        point(*center), normal, side, 400 * MM, 400 * MM, 400 * MM))


def cylinder(start, end, radius):
    return temporary.createCylinderOrCone(point(*start), radius * MM, point(*end), radius * MM)


def merged(solids):
    result = solids[0]
    for solid in solids[1:]:
        temporary.booleanOperation(result, solid, adsk.fusion.BooleanTypes.UnionBooleanType)
    return result


def carved(solid, cuts):
    for cut in cuts:
        temporary.booleanOperation(solid, cut, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return solid


def faceted(bounds, facets, inset=0.0):
    x, y, z = bounds
    rear = x[0] - (1 if inset else 0)
    solid = box(rear, x[1] - inset, y[0] + inset, y[1] - inset, z[0] + inset, z[1] - inset)
    return carved(solid, [half_space(*facet, inset) for facet in facets])


def panel_shell(bounds, facets, bosses):
    """Concha abierta por detrás, con un tetón atornillado en cada (y, z). Devuelve panel y pilotos."""
    x0 = bounds[0][0]
    shell = carved(faceted(bounds, facets), [faceted(bounds, facets, PANEL_WALL)])
    front = bounds[0][1] - PANEL_WALL + 0.1
    studs = [cylinder((x0, y, z), (front, y, z), 2.6) for y, z in bosses]
    holes = [cylinder((x0 - 1, y, z), (bounds[0][1] + 1, y, z), SCREW_RADIUS) for y, z in bosses]
    pilots = [cylinder((x0 - 3, y, z), (x0 + 0.1, y, z), M2_PILOT_RADIUS) for y, z in bosses]
    panel = merged([shell] + studs)
    # Los tetones no sobresalen de la forma exterior
    temporary.booleanOperation(panel, faceted(bounds, facets), adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return carved(panel, holes), pilots


class Servo:
    """Servo alineado con los ejes. long_axis y shaft_axis son (eje, signo)."""

    def __init__(self, origin, long_axis, shaft_axis):
        self.origin = origin
        self.long_axis = long_axis
        self.shaft_axis = shaft_axis
        self.width_axis = 3 - long_axis[0] - shaft_axis[0]

    def at(self, along, across, height):
        position = list(self.origin)
        position[self.long_axis[0]] += along * self.long_axis[1]
        position[self.width_axis] += across
        position[self.shaft_axis[0]] += height * self.shaft_axis[1]
        return tuple(position)

    def block(self, margin=0.0):
        corners = (
            self.at(-SERVO_SHORT_END - margin, -SERVO_WIDTH / 2 - margin, -SERVO_THICKNESS / 2 - margin),
            self.at(SERVO_LONG_END + margin, SERVO_WIDTH / 2 + margin, SERVO_THICKNESS / 2 + margin),
        )
        return box(corners[0][0], corners[1][0], corners[0][1], corners[1][1], corners[0][2], corners[1][2])

    def shaft(self, radius, half_length):
        return cylinder(self.at(0, 0, -half_length), self.at(0, 0, half_length), radius)

    def solid(self):
        half = SERVO_THICKNESS / 2 + HORN_LENGTH
        return merged([self.block(), self.shaft(HORN_RADIUS, half)])

    def mount_positions(self, columns=(-1, 1)):
        center = (SERVO_LONG_END - SERVO_SHORT_END) / 2
        return [
            (center + column * MOUNT_SPACING[0] / 2, across)
            for column in columns for across in (-MOUNT_SPACING[1] / 2, MOUNT_SPACING[1] / 2)
        ]

    def pocket(self, columns=(-1, 1)):
        """Hueco del servo, con el paso de las bocinas y los agujeros de fijación."""
        half = SERVO_THICKNESS / 2 + 6
        cuts = [self.block(FIT), self.shaft(HORN_CLEARANCE_RADIUS, half)]
        for along, across in self.mount_positions(columns):
            cuts.append(cylinder(self.at(along, across, -half), self.at(along, across, half), SCREW_RADIUS))
        return cuts

    def head_recesses(self, wall):
        """Cajeras para que las cabezas M2 no sobresalgan de una pared a esa distancia del eje."""
        cuts = []
        for along, across in self.mount_positions():
            for sign in (-1, 1):
                cuts.append(cylinder(
                    self.at(along, across, sign * (wall - HEAD_DEPTH)),
                    self.at(along, across, sign * (wall + 1)), HEAD_RADIUS))
        return cuts

    def connector_slots(self, sides=(-1, 1), depth=3.5):
        """Ventana hacia los conectores: van en una muesca de cada cara ancha, y la clavija entra paralela al eje."""
        cuts = []
        for sign in sides:
            corners = (
                self.at(4, sign * (SERVO_WIDTH / 2), -6),
                self.at(17, sign * (SERVO_WIDTH / 2 + depth), 6),
            )
            cuts.append(box(corners[0][0], corners[1][0], corners[0][1], corners[1][1], corners[0][2], corners[1][2]))
        return cuts

    def connector_exits(self, wall):
        """Salida del cable por una cara de bocina, en una pared a esa distancia del eje."""
        cuts = []
        for sign in (-1, 1):
            corners = (self.at(3, sign * 6, wall - 3), self.at(17, sign * 10.2, wall + 1))
            cuts.append(box(corners[0][0], corners[1][0], corners[0][1], corners[1][1], corners[0][2], corners[1][2]))
        return cuts

    def horn_recesses(self, plane, sign=1):
        """Cajeras para las cabezas de los tornillos de bocina, en una cara a esa distancia del eje."""
        diagonal = HORN_BOLT_RADIUS / math.sqrt(2)
        return [
            cylinder(self.at(along, across, sign * (plane - HEAD_DEPTH)), self.at(along, across, sign * (plane + 1)), HEAD_RADIUS)
            for along in (-diagonal, diagonal) for across in (-diagonal, diagonal)
        ]

    def horn_bolts(self, half=SERVO_THICKNESS / 2 + 9):
        """Tornillos de las bocinas, en diagonal, a través de las placas de la horquilla."""
        diagonal = HORN_BOLT_RADIUS / math.sqrt(2)
        return [
            cylinder(self.at(along, across, -half), self.at(along, across, half), SCREW_RADIUS)
            for along in (-diagonal, diagonal) for across in (-diagonal, diagonal)
        ]


def side_servos(side):
    hip = side * HIP_Y
    return {
        "yaw": Servo((0, hip, HIP_YAW_Z), (0, -1), (2, 1)),
        "roll": Servo((HIP_ROLL_X, hip, HIP_ROLL_Z), (1, -side), (0, 1)),
        "pitch": Servo((0, hip, HIP_PITCH_Z), (0, -1), (1, 1)),
        "knee": Servo((0, hip, KNEE_Z), (2, 1), (1, 1)),
        "ankle_pitch": Servo((0, hip, ANKLE_PITCH_Z), (2, 1), (1, 1)),
        "ankle_roll": Servo((ANKLE_ROLL_X, hip, ANKLE_ROLL_Z), (1, side), (0, 1)),
    }


def split_shell(hip, side, x_half, z0, z1, cuts):
    """Caja de un servo con eje en Y. La tapa exterior se atornilla y deja meter el servo."""
    main = box(-x_half, x_half, hip - side * SHELL_Y, hip + side * COVER_Y, z0, z1)
    cover = box(-x_half, x_half, hip + side * COVER_Y, hip + side * SHELL_Y, z0, z1)
    return carved(main, cuts), carved(cover, cuts)


def pelvis_frame():
    solids = [
        box(PELVIS_REAR, PELVIS_FRONT, -10, 10, PELVIS_BOTTOM, PELVIS_TOP),
        box(18, PELVIS_FRONT, -PELVIS_Y, PELVIS_Y, PELVIS_BOTTOM + 2, PELVIS_TOP - 2),
        box(PELVIS_REAR, -18, -PELVIS_Y, PELVIS_Y, PELVIS_BOTTOM, PELVIS_TOP),
        cylinder((WAIST_X, 0, HIP_YAW_Z), (WAIST_X, 0, PELVIS_TOP), 11.6),  # asiento del rodamiento
    ]
    for side in (-1, 1):
        hip = side * HIP_Y
        # La cara interior queda libre delante de x = -12: por ahí gira el alma del eslabón yaw
        solids.append(box(PELVIS_REAR, -12, hip - side * 12.7, side * PELVIS_Y, PELVIS_BOTTOM, PELVIS_TOP))
        solids.append(box(-12, 12.2, hip - side * 2, side * PELVIS_Y, PELVIS_BOTTOM, PELVIS_TOP))
        solids.append(box(12.2, 18, side * 27, side * PELVIS_Y, PELVIS_BOTTOM, PELVIS_TOP))
    return merged(solids)


def waist_servo():
    return Servo((WAIST_X, 0, HIP_YAW_Z), (0, -1), (2, 1))


def pelvis_panel():
    bounds = ((PELVIS_FRONT, 27.0), (-34.0, 34.0), (243.0, 267.0))
    facets = [
        ((27.0, 22.0, 0), (23.0, 34.0, 0), 2, (40.0, 40.0, 0)),
        ((27.0, -22.0, 0), (23.0, -34.0, 0), 2, (40.0, -40.0, 0)),
        ((27.0, 0, 258.0), (23.0, 0, 267.0), 1, (40.0, 0, 280.0)),
        ((27.0, 0, 250.0), (23.0, 0, 243.0), 1, (40.0, 0, 230.0)),
    ]
    return panel_shell(bounds, facets, [(-24.0, 260.0), (24.0, 260.0), (0.0, 246.0)])


def pelvis_bodies():
    waist = waist_servo()
    left, right = side_servos(1)["yaw"], side_servos(-1)["yaw"]
    cuts = waist.pocket() + left.pocket(columns=(1,)) + right.pocket(columns=(1,))
    cuts += waist.connector_exits(PELVIS_TOP - HIP_YAW_Z)
    # Asiento del rodamiento: toma la carga lateral del torso antes de que llegue al servo
    cuts.append(cylinder((WAIST_X, 0, WAIST_BEARING_Z), (WAIST_X, 0, PELVIS_TOP + 1), WAIST_SEAT_RADIUS))
    cuts += [left.shaft(HORN_CLEARANCE_RADIUS, 20), right.shaft(HORN_CLEARANCE_RADIUS, 20)]
    upper_cuts, lower_cuts = [], []
    panel, pilots = pelvis_panel()
    for side in (-1, 1):
        # Cable de cada pierna: por fuera, porque el alma pellizcaría el paso interior
        cuts.append(box(-17, -4, side * (HIP_Y + 10), side * (PELVIS_Y + 1), 258, 266))
        for x, y in ((20, 30), (-26.4, 29)):
            upper_cuts.append(cylinder((x, side * y, HIP_YAW_Z - 1), (x, side * y, PELVIS_TOP + 1), 0.9))
            lower_cuts.append(cylinder((x, side * y, PELVIS_BOTTOM - 1), (x, side * y, HIP_YAW_Z + 1), PILOT_RADIUS))
        yaw = side_servos(side)["yaw"]
        for along, across in yaw.mount_positions(columns=(1,)):
            # Por debajo el eslabón yaw pasa a 0,5 mm: la cabeza va rehundida
            lower_cuts.append(cylinder(yaw.at(along, across, PELVIS_BOTTOM - HIP_YAW_Z - 1),
                                       yaw.at(along, across, PELVIS_BOTTOM - HIP_YAW_Z + 1.3), HEAD_RADIUS))
    upper_half = box(-60, 60, -60, 60, HIP_YAW_Z, PELVIS_TOP + 1)
    lower_half = box(-60, 60, -60, 60, PELVIS_BOTTOM - 1, HIP_YAW_Z)
    bearing_top = WAIST_BEARING_Z + WAIST_BEARING[2]
    bearing = carved(
        cylinder((WAIST_X, 0, WAIST_BEARING_Z), (WAIST_X, 0, bearing_top), WAIST_BEARING[1]),
        [cylinder((WAIST_X, 0, WAIST_BEARING_Z - 1), (WAIST_X, 0, bearing_top + 1), WAIST_BEARING[0])])
    return {
        "structure": [carved(pelvis_frame(), cuts + lower_cuts + pilots + [upper_half])],
        "cover": [carved(pelvis_frame(), cuts + upper_cuts + pilots + [lower_half])],
        "panel": [panel],
        "bearing": [bearing],
        "servo": [waist.solid(), left.solid(), right.solid()],
    }


def yaw_link_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    roll_x_rear, roll_x_front = -27.5, 12.5
    lower = HIP_YAW_Z - SERVO_THICKNESS / 2 - HORN_LENGTH
    upper = HIP_YAW_Z + SERVO_THICKNESS / 2 + HORN_LENGTH
    web = (hip - side * 17.3, hip - side * 14.3)
    tenon = (-5, 5, web[0], web[1], upper, upper + TENON)
    boss_rear = HIP_ROLL_X - SERVO_THICKNESS / 2 - HORN_LENGTH - FORK_GAP
    boss_front = HIP_ROLL_X + SERVO_THICKNESS / 2 + HORN_LENGTH + FORK_GAP
    body = merged([
        box(roll_x_rear - ARM, roll_x_front + ARM, hip - 8, hip + 8, lower - 3, lower),
        box(-6, 6, web[0], web[1], lower - 3, upper),
        box(*tenon),
        box(-6, 6, web[0], hip, lower - 3, lower),
        box(roll_x_rear - ARM, roll_x_rear, hip - 9, hip + 9, HIP_ROLL_Z, lower),
        box(roll_x_front, roll_x_front + ARM, hip - 9, hip + 9, HIP_ROLL_Z, lower),
        cylinder((roll_x_rear - ARM, hip, HIP_ROLL_Z), (boss_rear, hip, HIP_ROLL_Z), ROLL_BOSS_RADIUS),
        cylinder((boss_front, hip, HIP_ROLL_Z), (roll_x_front + ARM, hip, HIP_ROLL_Z), ROLL_BOSS_RADIUS),
    ])
    cap = merged([
        box(-6, 6, web[0], hip, upper, upper + ARM),
        cylinder((0, hip, upper), (0, hip, upper + ARM), 10),
    ])
    cuts = servos["yaw"].horn_bolts() + servos["roll"].horn_bolts(SERVO_THICKNESS / 2 + 12)
    cap_cuts = servos["yaw"].horn_bolts() + servos["yaw"].horn_recesses(upper + ARM - HIP_YAW_Z)
    cap_cuts.append(box(-5.1, 5.1, web[0], web[1], upper, upper + TENON + 0.1))
    return {"structure": [carved(body, cuts)], "cover": [carved(cap, cap_cuts)]}


def roll_link_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    inner = hip - side * (SERVO_LONG_END + 0.2)
    outer = hip + side * 11
    cuts = servos["pitch"].pocket() + servos["pitch"].head_recesses(SHELL_Y) + servos["roll"].pocket()
    cuts += servos["pitch"].connector_slots(sides=(-1,))
    x0, x1 = -26.5, 11.5
    # Salientes para la chapa trasera: el techo de la caja es fino, y debajo pasa un tornillo del servo
    lug_top = 214.8
    shell = box(x0, x1, hip - side * SHELL_Y, hip + side * COVER_Y, 188, 211.8)
    plate = merged([
        box(x0, x1, hip + side * COVER_Y, hip + side * SHELL_Y, 188, 211.8),
        box(-25.4, -21, hip + side * COVER_Y, hip + side * SHELL_Y, 211.8, lug_top),
    ])
    body = merged([
        shell,
        box(-25.4, -21, hip - side * SHELL_Y, hip - side * 9.5, 211.8, lug_top),
        box(4.2, 6, inner, outer, 211, 232.5),
    ])
    # La chapa trasera se atornilla al final: entre dos chapas fijas el servo no entra
    rear_plate = merged([
        box(-21, -19.2, inner, outer, 211.8, 232.5),
        box(-25.4, -19.2, hip + side * 9.5, hip + side * 14.4, lug_top, lug_top + 2),
        box(-25.4, -19.2, hip - side * 9.5, hip - side * 14.4, lug_top, lug_top + 2),
        box(-21, -19.2, hip + side * 11, hip + side * 14.4, lug_top, lug_top + 2),
    ])
    # Cortos y por fuera de la bocina de roll: debajo está el agujero M2 del servo
    flange_rows = (hip + side * 13, hip - side * 13)
    flange_screws = [cylinder((-23.2, y, lug_top - 1), (-23.2, y, lug_top + 3), SCREW_RADIUS) for y in flange_rows]
    flange_pilots = [cylinder((-23.2, y, 210.4), (-23.2, y, lug_top + 1), PILOT_RADIUS) for y in flange_rows]
    return {
        "structure": [carved(body, cuts + flange_pilots)],
        "cover": [carved(plate, cuts + flange_pilots)],
        "plate": [carved(rear_plate, cuts + flange_screws)],
        "servo": [servos["roll"].solid(), servos["pitch"].solid()],
    }


def fork_arms(hip, axis_z, bottom, half_width, x=0):
    """Las dos placas de una horquilla, con el apoyo sobre cada bocina de un servo en Y."""
    horn = SERVO_THICKNESS / 2 + HORN_LENGTH + FORK_GAP
    arms = []
    for sign in (-1, 1):
        y0, y1 = hip + sign * FORK_INNER, hip + sign * FORK_OUTER
        arms.append(box(x - half_width, x + half_width, y0, y1, bottom, axis_z))
        arms.append(cylinder((x, y0, axis_z), (x, y1, axis_z), half_width))
        arms.append(cylinder((x, hip + sign * horn, axis_z), (x, y0, axis_z), HORN_RADIUS))
    return arms


def thigh_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    cuts = servos["knee"].pocket() + servos["knee"].head_recesses(SHELL_Y) + servos["knee"].connector_slots()
    main, cover = split_shell(hip, side, 12.2, 106, 150, cuts)
    body = merged(fork_arms(hip, HIP_PITCH_Z, 178, 9) + [
        box(-15, 15, hip - FORK_OUTER, hip + FORK_OUTER, 150, 181),
        main,
    ])
    window = box(-20, 20, hip - 15.5, hip + 15.5, 152, 177)
    bounds = ((15.0, 19.0), (hip - 14.0, hip + 14.0), (150.0, 181.0))
    facets = [
        ((19.0, hip + 9, 0), (16.0, hip + 14, 0), 2, (40.0, hip + 30, 0)),
        ((19.0, hip - 9, 0), (16.0, hip - 14, 0), 2, (40.0, hip - 30, 0)),
        ((19.0, 0, 158.0), (16.0, 0, 150.0), 1, (40.0, 0, 130.0)),
    ]
    panel, pilots = panel_shell(bounds, facets, [(hip - 9, 179.0), (hip + 9, 179.0)])
    # Lengüeta dentro de la ventana: sitúa el borde inferior, que no lleva tornillos
    panel = merged([panel, box(13.0, 17.0, hip - 10, hip + 10, 152.4, 155.0)])
    # Por fuera pasa el antebrazo: las cabezas de `hip_pitch` van rehundidas
    fork_cuts = servos["pitch"].horn_bolts() + servos["pitch"].horn_recesses(FORK_OUTER, side)
    return {
        "structure": [carved(body, fork_cuts + [window] + pilots)],
        "cover": [cover],
        "panel": [panel],
        "servo": [servos["knee"].solid()],
    }


def shin_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    cuts = servos["ankle_pitch"].pocket() + servos["ankle_pitch"].head_recesses(SHELL_Y)
    cuts += servos["ankle_pitch"].connector_slots()
    main, cover = split_shell(hip, side, 12.2, 24, 60, cuts)
    body = merged(fork_arms(hip, KNEE_Z, 80, 9) + [
        box(-13, 13, hip - FORK_OUTER, hip + FORK_OUTER, 80, 83),
        box(-13, 13, hip - SHELL_Y, hip + SHELL_Y, 60, 80),
        main,
    ])
    window = box(-20, 20, hip - 11.5, hip + 11.5, 62, 80)
    bounds = ((13.0, 17.0), (hip - 14.4, hip + 14.4), (60.0, 83.0))
    facets = [
        ((17.0, hip + 10, 0), (14.5, hip + 14.4, 0), 2, (40.0, hip + 30, 0)),
        ((17.0, hip - 10, 0), (14.5, hip - 14.4, 0), 2, (40.0, hip - 30, 0)),
    ]
    panel, pilots = panel_shell(bounds, facets, [(hip - 12.9, 70.0), (hip + 12.9, 70.0)])
    knee_pad = box(17.0, 17.0 + PAD_THICKNESS, hip - 8, hip + 8, 64, 78)
    return {
        "structure": [carved(body, servos["knee"].horn_bolts() + [window] + pilots)],
        "cover": [cover],
        "panel": [panel],
        "pad": [knee_pad],
        "servo": [servos["ankle_pitch"].solid()],
    }


def ankle_link_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    roll_half = SERVO_THICKNESS / 2 + HORN_LENGTH + FORK_GAP
    rear = ANKLE_ROLL_X - roll_half
    front = ANKLE_ROLL_X + roll_half
    plate_rear = rear - 4
    post = rear - 1  # deja sitio a las cabezas de la cuna del pie
    plate_front = front + 3
    inner = hip - side * 18
    bar = hip - side * 15.5
    arms = []
    for sign in (-1, 1):
        y0, y1 = hip + sign * FORK_INNER, hip + sign * FORK_OUTER
        arms.append(box(-9, post, y0, y1, 26, 44))
        arms.append(box(plate_rear, post, y0, y1, ANKLE_ROLL_Z, 44))
        arms.append(cylinder((0, y0, ANKLE_PITCH_Z), (0, y1, ANKLE_PITCH_Z), 9))
        arms.append(cylinder(
            (0, hip + sign * (SERVO_THICKNESS / 2 + HORN_LENGTH + FORK_GAP), ANKLE_PITCH_Z), (0, y0, ANKLE_PITCH_Z), HORN_RADIUS))
    body = merged(arms + [
        box(plate_rear, post, inner, hip + side * FORK_OUTER, ANKLE_ROLL_Z, 28),
        cylinder((plate_rear, hip, ANKLE_ROLL_Z), (rear, hip, ANKLE_ROLL_Z), 9),
        box(plate_rear, post, inner, hip, 12, ANKLE_ROLL_Z),
        cylinder((front, hip, ANKLE_ROLL_Z), (plate_front, hip, ANKLE_ROLL_Z), 9),
        box(front, plate_front, inner, hip, 12, 20),
        box(plate_rear, plate_front, inner, bar, 12, 20),
    ])
    cuts = servos["ankle_pitch"].horn_bolts() + servos["ankle_roll"].horn_bolts(SERVO_THICKNESS / 2 + 10)
    return {"structure": [carved(body, cuts)]}


def foot_bodies(side):
    hip = side * HIP_Y
    servos = side_servos(side)
    center = hip + side * FOOT_Y_OFFSET
    y0, y1 = hip + side * 11, hip + side * 27
    body = merged([
        box(-30, 55, center - 24, center + 24, 2, 5),
        box(ANKLE_ROLL_X - 14, ANKLE_ROLL_X + 14, y0, y1, 5, 28.2),
    ])
    return {
        "structure": [carved(body, servos["ankle_roll"].pocket(columns=(1,)))],
        "sole": [box(-30, 55, center - 24, center + 24, 0, 2)],
        "servo": [servos["ankle_roll"].solid()],
    }


def neck_yaw_servo():
    return Servo((AXIS_X, 0, NECK_YAW_Z), (0, -1), (2, 1))


def pitch_servo(side):
    return Servo((AXIS_X, side * PITCH_Y, SHOULDER_Z), (0, -1), (1, 1))


def electronics_solid(extent):
    x, y, z = extent[:3]
    return box(x[0], x[1], y[0], y[1], z[0], z[1])


def torso_bodies():
    neck = neck_yaw_servo()
    servos = {side: pitch_servo(side) for side in (-1, 1)}
    waist = waist_servo()

    shell = box(TORSO_X[0], TORSO_X[1], -TORSO_Y, TORSO_Y, FLOOR_Z[0], TOP_PLATE_Z[0])
    hollow = box(TORSO_INNER_X[0], TORSO_INNER_X[1], -TORSO_INNER_Y, TORSO_INNER_Y, FLOOR_Z[1], TOP_PLATE_Z[0] + 1)
    solids = [
        carved(shell, [hollow]),
        box(TORSO_INNER_X[0], TORSO_INNER_X[1], -TORSO_INNER_Y, TORSO_INNER_Y, SHELF_Z[0], SHELF_Z[1]),
        cylinder((AXIS_X, 0, WAIST_BOSS_BOTTOM), (AXIS_X, 0, WAIST_BEARING_Z + WAIST_BEARING[2] + 0.2), WAIST_SPIGOT_RADIUS),
        cylinder((AXIS_X, 0, WAIST_BEARING_Z + WAIST_BEARING[2] + 0.2), (AXIS_X, 0, FLOOR_Z[0] + 0.1), WAIST_BOSS_RADIUS),
    ]
    for side in (-1, 1):
        solids.append(box(AXIS_X - 24.7, AXIS_X + 9.7, side * 10.2, side * 13.1, SHELF_Z[1], TOP_PLATE_Z[0]))
        solids.append(box(TORSO_INNER_X[0], -13, side * 34, side * TORSO_INNER_Y, FLOOR_Z[1], SHELF_Z[0]))
        solids.append(box(28.2, TORSO_INNER_X[1], side * 32.5, side * TORSO_INNER_Y, 332, TOP_PLATE_Z[0]))
    structure = merged(solids)

    cuts = neck.pocket() + waist.horn_bolts() + neck.head_recesses(TOP_PLATE_Z[1] - NECK_YAW_Z)
    # Ventana del conector del cuello: la clavija baja por delante de la batería
    cuts += [box(AXIS_X - 17, AXIS_X - 4, side * 6, side * 10.2, SHELF_Z[0] - 1, SHELF_Z[1] + 1) for side in (-1, 1)]
    windows = {}
    for side, servo in servos.items():
        cuts += servo.pocket() + servo.head_recesses(14.2)
        windows[side] = box(
            AXIS_X - 24.85, AXIS_X + 9.85, side * TORSO_INNER_Y, side * TORSO_Y,
            SHOULDER_Z - 10.35, SHOULDER_Z + 10.35)
    carved(structure, cuts + list(windows.values()))

    # Trampilla trasera: la batería se cambia sin desmontar nada más
    carved(structure, [box(-20.0, -17.0, -34.0, 34.0, FLOOR_Z[1], SHELF_Z[0])])
    # Paso de las orejas del carro junto al suelo: los carriles dejan 68 mm, y el carro mide 72,8
    carved(structure, [box(-20.0, -12.9, side * 33.9, side * 36.6, FLOOR_Z[1], 277.2) for side in (-1, 1)])
    rear = box(-22.0, TORSO_X[0], -37.0, 37.0, FLOOR_Z[1], SHELF_Z[0])
    carved(rear, [box(-23.0, -19.0, y - 2, y + 2, 284.0, 304.0) for y in (-20.0, -10.0, 0.0, 10.0, 20.0)])
    for y in (-35.3, 35.3):
        for z in (279.0, 310.0):
            carved(structure, [cylinder((-21, y, z), (-14, y, z), PILOT_RADIUS)])
            carved(rear, [cylinder((-23, y, z), (-19, y, z), 0.9)])

    openings = [
        box(18.0, 26.9, 23.0, 33.0, SHELF_Z[0] - 1, SHELF_Z[1] + 1),  # entre los dos pisos
        box(32.0, TORSO_X[1] + 1, -24.0, -6.0, 318.0, 329.0),  # interruptor y carga
    ]
    for side in (-1, 1):
        # Cables: a las piernas por delante de la pelvis, y a los brazos fuera del barrido del hombro
        openings.append(box(29.2, 32.2, side * 4, side * 16, FLOOR_Z[0] - 1, FLOOR_Z[1] + 1))
        openings.append(box(32.0, TORSO_X[1] + 1, side * 30, side * TORSO_INNER_Y, 320.0, 330.0))
    for y in (6.0, 26.0):
        openings.append(cylinder((31.5, y, 326.0), (TORSO_X[1] - 0.8, y, 326.0), PILOT_RADIUS))  # IMU
    for y in (-20.0, -10.0, 0.0, 10.0, 20.0):
        openings.append(box(32.0, TORSO_X[1] + 1, y - 2, y + 2, 284.0, 304.0))
    carved(structure, openings)

    top = box(TORSO_X[0], TORSO_X[1], -TORSO_Y, TORSO_Y, TOP_PLATE_Z[0], TOP_PLATE_Z[1])
    top_cuts = neck.pocket() + neck.head_recesses(TOP_PLATE_Z[1] - NECK_YAW_Z)
    top_cuts.append(box(28.5, 32.2, -8.0, 8.0, TOP_PLATE_Z[0] - 1, TOP_PLATE_Z[1] + 1))  # cable de la cabeza
    for side in (-1, 1):
        top_cuts.append(box(-9.5, 3.5, side * 20, side * 34, TOP_PLATE_Z[0] - 1, TOP_PLATE_Z[1] + 1))  # conector
        top_cuts.append(cylinder((30.2, side * 34.5, TOP_PLATE_Z[0] - 1), (30.2, side * 34.5, TOP_PLATE_Z[1] + 1), 0.9))
        carved(structure, [cylinder((30.2, side * 34.5, 333), (30.2, side * 34.5, TOP_PLATE_Z[0] + 1), PILOT_RADIUS)])
    carved(top, top_cuts)

    tray_slots = [
        box(29.0, 32.2, side * 4, side * 16, FLOOR_Z[0], 283) for side in (-1, 1)
    ]
    tray = merged([
        box(4.5, 30.5, -33.5, 33.5, FLOOR_Z[1], 276.5),
        box(4.5, 5.9, -33, 33, FLOOR_Z[1], 284),
        box(14.1, 16.9, -33, 33, FLOOR_Z[1], 284),
        box(29.1, 30.5, -31, 31, FLOOR_Z[1], 284),
        # Orejas retrasadas: con la batería fuera, sus tornillos se alcanzan desde la trampilla
        box(-1.5, 13, -36.0, -32.7, FLOOR_Z[1], 277),
        box(-1.5, 13, 32.7, 36.0, FLOOR_Z[1], 277),
        # Tope delantero: deja los agujeros de las orejas alineados con el torso
        box(30.4, 32.0, 18, 28, FLOOR_Z[1], 279),
        box(30.4, 32.0, -28, -18, FLOOR_Z[1], 279),
        # Tope de la batería, en el carro: fijo al torso cerraría el paso
        box(-0.5, 4.6, 7, 32, FLOOR_Z[1], 279),
        box(-0.5, 4.6, -32, -7, FLOOR_Z[1], 279),
    ])
    tray_screws = [cylinder((1.0, side * 34.4, FLOOR_Z[1] - 3), (1.0, side * 34.4, 277.5), SCREW_RADIUS) for side in (-1, 1)]
    # Ranuras a todo lo largo: el carro entra recto sobre las cabezas de los tornillos de cintura
    tray_screws += [box(0.0, 31.0, side * 2.0, side * 6.5, FLOOR_Z[1] - 1, 276.6) for side in (-1, 1)]
    carved(tray, tray_slots + tray_screws)
    carved(structure, [cylinder((1.0, side * 34.4, FLOOR_Z[0] + 0.5), (1.0, side * 34.4, FLOOR_Z[1] + 1), PILOT_RADIUS)
                       for side in (-1, 1)])

    covers = {}
    for side, servo in servos.items():
        name = "left_cover" if side > 0 else "right_cover"
        covers[name] = [carved(
            box(AXIS_X - 24.7, AXIS_X + 9.7, side * TORSO_INNER_Y, side * TORSO_Y,
                SHOULDER_Z - 10.2, SHOULDER_Z + 10.2),
            servo.pocket() + servo.head_recesses(14.2))]

    chest_bounds = ((TORSO_X[1], 41.0), (-37.0, 37.0), (277.0, 339.0))
    chest_facets = [
        ((TORSO_X[1], 37.0, 305.0), (TORSO_X[1], 26.0, 277.0), 0, (0, 60.0, 270.0)),
        ((TORSO_X[1], -37.0, 305.0), (TORSO_X[1], -26.0, 277.0), 0, (0, -60.0, 270.0)),
        ((41.0, 0, 324.0), (36.0, 0, 339.0), 1, (60.0, 0, 345.0)),
        ((41.0, 0, 302.0), (36.0, 0, 277.0), 1, (60.0, 0, 270.0)),
        ((41.0, 28.0, 0), (36.0, 37.0, 0), 2, (60.0, 50.0, 0)),
        ((41.0, -28.0, 0), (36.0, -37.0, 0), 2, (60.0, -50.0, 0)),
    ]
    chest, chest_pilots = panel_shell(chest_bounds, chest_facets, [(-15, 286), (15, 286), (-15, 311), (15, 311)])
    chest_openings = [
        box(TORSO_X[1] - 1, 43.0, -24.0, -6.0, 318.0, 329.0),  # interruptor
        box(TORSO_X[1] - 1, 43.0, -24.0, 24.0, 276.0, 281.0),  # aire de las rejillas
    ]
    chest_openings += [box(TORSO_X[1] - 1, 43.0, side * 29, side * 38, 319.0, 331.0) for side in (-1, 1)]  # cables
    carved(chest, chest_openings)
    carved(structure, chest_pilots)
    chest_pad = box(41.0, 41.0 + PAD_THICKNESS, -12.0, 12.0, 304.0, 317.0)
    back_pads = [box(-22.0 - PAD_THICKNESS, -22.0, side * 25, side * 35, 284.0, 304.0) for side in (-1, 1)]

    bodies = {
        "structure": [structure],
        "top_plate": [top],
        "rear_cover": [rear],
        "chest_panel": [chest],
        "chest_pad": [chest_pad],
        "left_back_pad": [back_pads[1]],
        "right_back_pad": [back_pads[0]],
        "tray_plate": [tray],
        **covers,
        "servo": [neck.solid(), servos[1].solid(), servos[-1].solid()],
    }
    for name, extent in ELECTRONICS.items():
        bodies[name] = [electronics_solid(extent)]
    return bodies


def fit_overlaps():
    """Volumen común de piezas del tronco y de la cabeza que deben caber sin tocarse."""
    def common(one, two):
        solid = temporary.copy(one)
        temporary.booleanOperation(solid, temporary.copy(two), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return round(solid.volume * 1000, 3) if solid.faces.count else 0.0

    torso = torso_bodies()
    solids = {
        f"{kind}_{index}": solid
        for kind, group in torso.items() for index, solid in enumerate(group)
    }
    solids["voice_reserve"] = electronics_solid(VOICE_RESERVE)
    names = sorted(solids)
    found = {}
    for position, one in enumerate(names):
        for two in names[position + 1:]:
            volume = common(solids[one], solids[two])
            if volume > 0.01:
                found[f"{one} / {two}"] = volume
    head = head_bodies()
    camera = electronics_solid(CAMERA)
    for kind in ("structure", "cover", "left_cover"):
        volume = common(camera, head[kind][0])
        if volume > 0.01:
            found[f"camera / head_{kind}"] = volume
    return found


def shoulder_link_bodies(side):
    roll = Servo((AXIS_X, side * ROLL_Y, SHOULDER_Z), (2, -1), (0, 1))
    pitch = pitch_servo(side)
    plate = (side * (PITCH_Y + SERVO_THICKNESS / 2 + HORN_LENGTH), side * 42.2)
    solids = [box(AXIS_X - FORK_OUTER, AXIS_X + FORK_OUTER, plate[0], plate[1], SHOULDER_Z - 9, SHOULDER_Z + 9)]
    for sign in (-1, 1):
        x0, x1 = AXIS_X + sign * FORK_INNER, AXIS_X + sign * FORK_OUTER
        axis_y = side * ROLL_Y
        solids.append(box(x0, x1, plate[1], axis_y, SHOULDER_Z - 9, SHOULDER_Z + 9))
        solids.append(cylinder((x0, axis_y, SHOULDER_Z), (x1, axis_y, SHOULDER_Z), 9))
        solids.append(cylinder(
            (AXIS_X + sign * (SERVO_THICKNESS / 2 + HORN_LENGTH + FORK_GAP), axis_y, SHOULDER_Z),
            (x0, axis_y, SHOULDER_Z), HORN_RADIUS))
    cuts = pitch.horn_bolts() + pitch.horn_recesses(42.2 - PITCH_Y, side) + roll.horn_bolts()
    return {"structure": [carved(merged(solids), cuts)]}


def arm_yaw_servo(side):
    return Servo((AXIS_X, side * ARM_YAW_Y, ARM_YAW_Z), (0, -1), (2, 1))


def upper_arm_bodies(side):
    roll = Servo((AXIS_X, side * ROLL_Y, SHOULDER_Z), (2, -1), (0, 1))
    yaw = arm_yaw_servo(side)
    axis_y = side * ROLL_Y
    y0, y1 = side * ROLL_WALL_Y[0], side * ROLL_WALL_Y[1]
    beyond = box(AXIS_X - 20, AXIS_X + 20, y1, y1 + side * 8, ARM_SHELF_Z[0], SHOULDER_Z + 20)

    def wall(x0, x1, bottom):
        # Redondeada alrededor del eje de roll para pasar junto a la placa del hombro
        return carved(merged([
            box(x0, x1, y0, y1, bottom, SHOULDER_Z),
            cylinder((x0, axis_y, SHOULDER_Z), (x1, axis_y, SHOULDER_Z), 14.0),
        ]), [temporary.copy(beyond)])

    structure = merged([
        wall(AXIS_X - SHELL_Y, AXIS_X + COVER_Y, ARM_SHELF_Z[0]),
        box(ARM_SHELF_X[0], ARM_SHELF_X[1], side * ARM_Y[0], side * ARM_Y[1], ARM_SHELF_Z[0], ARM_SHELF_Z[1]),
    ])
    foot_x = (AXIS_X + COVER_Y, ARM_SHELF_X[1])
    foot_z = (ARM_SHELF_Z[1], ARM_SHELF_Z[1] + 2.8)
    cover = merged([
        wall(AXIS_X + COVER_Y, AXIS_X + SHELL_Y, ARM_SHELF_Z[1]),
        box(foot_x[0], foot_x[1], y0, y1, foot_z[0], foot_z[1]),
    ])
    cup = merged([
        box(ARM_CUP_X[0], ARM_CUP_X[1], side * ARM_Y[0], side * ARM_Y[1], ARM_CUP_BOTTOM, ARM_SHELF_Z[0]),
        box(ARM_CUP_X[1], ARM_SHELF_X[1], y0, y1, ARM_SHELF_Z[0] - 6, ARM_SHELF_Z[0]),  # rosca del pie de la tapa
    ])

    roll_cuts = roll.pocket() + roll.head_recesses(SHELL_Y) + roll.connector_slots()
    yaw_cuts = yaw.pocket() + yaw.connector_slots()
    head_cuts = [
        cylinder(yaw.at(along, across, -14.5), yaw.at(along, across, -12.7), HEAD_RADIUS)
        for along, across in yaw.mount_positions()
    ]
    # Sobre el servo de yaw no cabe una cabeza de tornillo: la repisa se une a la copa detrás y por el pie
    corners = [(-19.3, 45.0), (-19.3, 64.0)]
    shelf_holes = [cylinder((x, side * y, ARM_SHELF_Z[0] - 1), (x, side * y, ARM_SHELF_Z[1] + 1), 0.9) for x, y in corners]
    cup_pilots = [cylinder((x, side * y, ARM_SHELF_Z[0] - 8), (x, side * y, ARM_SHELF_Z[0] + 1), PILOT_RADIUS)
                  for x, y in corners]
    foot_holes = [cylinder((24.5, side * y, foot_z[1] + 1), (24.5, side * y, ARM_SHELF_Z[0] - 5), PILOT_RADIUS)
                  for y in ARM_FOOT_SCREW_Y]
    cover_holes = [cylinder((24.5, side * y, foot_z[0] - 1), (24.5, side * y, foot_z[1] + 1), 0.9) for y in ARM_FOOT_SCREW_Y]
    # Paso de la herramienta hasta los dos tornillos del servo de roll que quedan tras el pie
    cover_holes += [box(AXIS_X + SHELL_Y - 0.1, AXIS_X + 21, side * near, side * far, foot_z[0] - 1, roll.at(22.5, 0, 0)[2] + 2.2)
                    for near, far in ((44.0, 50.7), (62.3, 67.0))]
    shaft_hole = yaw.shaft(HORN_CLEARANCE_RADIUS, 16)
    return {
        "structure": [carved(structure, roll_cuts + shelf_holes + foot_holes + [shaft_hole])],
        "cover": [carved(cover, roll_cuts + cover_holes)],
        "plate": [carved(cup, yaw_cuts + head_cuts + cup_pilots + [
            cylinder((24.5, side * y, ARM_SHELF_Z[0] - 5), (24.5, side * y, ARM_SHELF_Z[0] + 1), PILOT_RADIUS)
            for y in ARM_FOOT_SCREW_Y
        ])],
        "servo": [roll.solid(), yaw.solid()],
    }


def elbow_servo(side):
    # Extremo largo adelante: al doblar sube, y hacia atrás bajaría contra muslo y cadera
    return Servo((AXIS_X, side * ELBOW_Y, ELBOW_Z), (0, 1), (1, 1))


def elbow_link_bodies(side):
    plate = (side * ELBOW_PLATE_Y[0], side * ELBOW_PLATE_Y[1])
    axis_y = side * ARM_YAW_Y
    body = merged([
        cylinder((AXIS_X, axis_y, ELBOW_TOP_Z[0]), (AXIS_X, axis_y, ELBOW_TOP_Z[1]), 10),
        box(AXIS_X - 10, AXIS_X + 10, axis_y, plate[1], ELBOW_TOP_Z[0], ELBOW_TOP_Z[1]),
        box(AXIS_X - 10, AXIS_X + 10, plate[0], plate[1], ELBOW_Z, ELBOW_TOP_Z[1]),
        cylinder((AXIS_X, plate[0], ELBOW_Z), (AXIS_X, plate[1], ELBOW_Z), 10),
        box(AXIS_X - 10, AXIS_X + 10, side * 64, plate[0], 261, ELBOW_TOP_Z[0]),  # cartela
    ])
    return {"structure": [carved(body, arm_yaw_servo(side).horn_bolts(23) + elbow_servo(side).horn_bolts(20.5))]}


def forearm_bodies(side):
    servo = elbow_servo(side)
    axis_y = side * ELBOW_Y
    x0, x1 = AXIS_X - 11.9, AXIS_X + 26.9
    z0, z1 = ELBOW_Z - 12.4, ELBOW_Z + 12.4
    cuts = servo.pocket() + servo.head_recesses(SHELL_Y) + servo.connector_slots()
    shell = box(x0, x1, axis_y - side * SHELL_Y, axis_y + side * COVER_Y, z0, z1)
    cover = box(x0, x1, axis_y + side * COVER_Y, axis_y + side * SHELL_Y, z0, z1)
    # El antebrazo queda hueco: caben dos servos en fila, con su eje en X, para muñeca y pinza
    sleeve_x = (AXIS_X - FOREARM_HALF_X, AXIS_X + FOREARM_HALF_X)
    sleeve = carved(
        box(sleeve_x[0], sleeve_x[1], side * FOREARM_Y[0], side * FOREARM_Y[1], FOREARM_END_Z, z0),
        [box(sleeve_x[0] + FOREARM_WALL, sleeve_x[1] - FOREARM_WALL, side * (FOREARM_Y[0] + FOREARM_WALL),
             side * (FOREARM_Y[1] - FOREARM_WALL), FOREARM_END_Z - 1, z0 - FOREARM_WALL)])
    return {
        "structure": [merged([carved(shell, cuts), sleeve])],
        "cover": [carved(cover, cuts)],
        "servo": [servo.solid()],
    }


def neck_pitch_servo():
    return Servo((AXIS_X, 0, NECK_PITCH_Z), (2, 1), (1, 1))


def neck_link_bodies():
    body = merged(fork_arms(0, NECK_PITCH_Z, NECK_PLATE_Z[1], 9, x=AXIS_X) + [
        box(AXIS_X - 9, AXIS_X + 9, -FORK_OUTER, FORK_OUTER, NECK_PLATE_Z[0], NECK_PLATE_Z[1]),
    ])
    neck = neck_yaw_servo()
    cuts = neck.horn_bolts() + neck.horn_recesses(NECK_PLATE_Z[1] - NECK_YAW_Z) + neck_pitch_servo().horn_bolts()
    return {"structure": [carved(body, cuts)]}


def rounded_head(inset):
    """Forma de la cabeza, con esquinas verticales redondeadas y techo curvo."""
    x0, x1 = HEAD_X[0] + inset, HEAD_X[1] - inset
    y = HEAD_Y - inset
    z0, z1 = HEAD_Z[0] - 1, HEAD_Z[1] - inset
    radius = 14 - inset
    solids = [box(x0 + radius, x1 - radius, -y, y, z0, z1), box(x0, x1, -y + radius, y - radius, z0, z1)]
    for corner_x in (x0 + radius, x1 - radius):
        for corner_y in (-y + radius, y - radius):
            solids.append(cylinder((corner_x, corner_y, z0), (corner_x, corner_y, z1), radius))
    shape = merged(solids)
    center_x = sum(HEAD_X) / 2
    intersection = adsk.fusion.BooleanTypes.IntersectionBooleanType
    dome_x = 34 - inset
    temporary.booleanOperation(shape, cylinder(
        (center_x, -40, HEAD_Z[1] - 34), (center_x, 40, HEAD_Z[1] - 34), dome_x), intersection)
    return shape


def head_bodies():
    servo = neck_pitch_servo()
    roof = NECK_PITCH_Z + 27
    bottom = box(AXIS_X - 20, AXIS_X + 20, -20, 20, NECK_PITCH_Z - 20, NECK_PITCH_Z)
    intersection = adsk.fusion.BooleanTypes.IntersectionBooleanType

    def cup(y0, y1):
        # Solo debajo del eje: rodea la esquina del servo al girar sobre el plato del cuello
        solid = cylinder((AXIS_X, y0, NECK_PITCH_Z), (AXIS_X, y1, NECK_PITCH_Z), CUP_RADIUS)
        temporary.booleanOperation(solid, temporary.copy(bottom), intersection)
        return solid

    cuts = servo.pocket() + servo.head_recesses(SHELL_Y) + servo.connector_slots(sides=(-1,))
    pilots = [cylinder((CASSETTE_COLUMN_X, y, roof - 1), (CASSETTE_COLUMN_X, y, CASSETTE_TOP + 1), PILOT_RADIUS)
              for y in (-5, 5)]
    main = merged([
        box(AXIS_X - 12.2, AXIS_X + 12.2, -SHELL_Y, COVER_Y, NECK_PITCH_Z, roof),
        cup(-SHELL_Y, COVER_Y),
    ])
    carved(main, cuts)
    # Soporte de la cámara, columna al techo del cráneo y separador trasero
    rear_x = AXIS_X - 12.2
    skull_rear = HEAD_X[0] + HEAD_WALL
    mount = merged([
        main,
        box(AXIS_X + 12.2, CAMERA[0][0], -11, 11, 361, 385),
        box(CASSETTE_COLUMN_X - 6, CASSETTE_COLUMN_X + 6, -8, 8, roof, CASSETTE_TOP),
        box(skull_rear, rear_x, -5, 5, 372, 382),
    ])
    camera_pilots = [cylinder((AXIS_X + 11, y, 373), (CAMERA[0][0] + 0.1, y, 373), PILOT_RADIUS) for y in (-8, 8)]
    rear_pilot = cylinder((skull_rear - 0.1, 0, 377), (rear_x + 1, 0, 377), PILOT_RADIUS)
    cover = merged([
        box(AXIS_X - 12.2, AXIS_X + 12.2, COVER_Y, SHELL_Y, NECK_PITCH_Z, roof),
        cup(COVER_Y, SHELL_Y),
    ])
    # El cráneo baja por encima del conjunto ya montado y se atornilla por arriba y por detrás
    skull = carved(rounded_head(0), [
        rounded_head(HEAD_WALL),
        box(HEAD_X[0] - 1, HEAD_X[1] + 1, -HEAD_Y - 1, HEAD_Y + 1, HEAD_Z[0] - 2, HEAD_Z[0]),
        cylinder((HEAD_X[1] - 3, 0, 373), (HEAD_X[1] + 1, 0, 373), 7),
        box(26, HEAD_X[1] + 1, -8, 8, HEAD_Z[0] - 1, HEAD_Z[0] + 4),  # paso del cable al mirar abajo
    ])
    holes = [cylinder((CASSETTE_COLUMN_X, y, CASSETTE_TOP - 1), (CASSETTE_COLUMN_X, y, HEAD_Z[1] + 1), 0.9)
             for y in (-5, 5)]
    holes.append(cylinder((HEAD_X[0] - 1, 0, 377), (skull_rear + 0.1, 0, 377), 0.9))
    visor = carved(box(HEAD_X[1], HEAD_X[1] + 1.4, -15.0, 15.0, 362.0, 383.0), [
        cylinder((HEAD_X[1] - 1, 0, 373), (HEAD_X[1] + 2, 0, 373), 7),
    ])
    return {
        "structure": [carved(mount, pilots + camera_pilots + [rear_pilot])],
        "left_cover": [carved(cover, cuts)],
        "cover": [carved(skull, holes)],
        "visor_panel": [visor],
        "servo": [servo.solid()],
        "camera": [electronics_solid(CAMERA)],
    }


def add_link(design, materials, name, bodies):
    occurrence = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    component = occurrence.component
    component.name = name
    feature = component.features.baseFeatures.add()
    feature.startEdit()
    created = []
    for kind, solids in bodies.items():
        for solid in solids:
            body = component.bRepBodies.add(solid, feature)
            body.name = f"{name}_{kind}"
            created.append((body.name, kind))
    feature.finishEdit()
    for body, (body_name, kind) in zip(component.bRepBodies, created):
        body.name = body_name
        body.material = materials.get(kind) or materials[kind.rsplit("_", 1)[-1]]
    return occurrence


def density_material(design, source, name, density):
    material = design.materials.addByCopy(source, name)
    material.materialProperties.itemById("structural_Density").value = density
    return material


def library_material(app):
    for library in app.materialLibraries:
        for material in library.materials:
            if material.materialProperties.itemById("structural_Density"):
                return material
    raise RuntimeError("ninguna biblioteca de Fusion tiene materiales con densidad")


def add_materials(app, design):
    source = library_material(app)
    printed = density_material(design, source, "danny_printed", STRUCTURE_DENSITY)
    envelope_cm3 = (34 * 20 * 23 + 2 * math.pi * HORN_RADIUS ** 2 * HORN_LENGTH) / 1000
    materials = {
        "structure": printed,
        "cover": printed,
        "plate": printed,
        "panel": printed,
        "sole": density_material(design, source, "danny_sole", SOLE_DENSITY),
        "pad": density_material(design, source, "danny_pad", SOLE_DENSITY),
        "bearing": density_material(design, source, "danny_bearing", BEARING_DENSITY),
        "servo": density_material(design, source, "danny_servo", SERVO_MASS_G / envelope_cm3 * 1000),
    }
    for name, (x, y, z, mass_g) in {**ELECTRONICS, "camera": CAMERA}.items():
        volume_cm3 = (x[1] - x[0]) * (y[1] - y[0]) * (z[1] - z[0]) / 1000
        materials[name] = density_material(design, source, f"danny_{name}", mass_g / volume_cm3 * 1000)
    return materials


def add_joint(root, sketch, child, parent, name, origin, direction, limits):
    sketch_point = sketch.sketchPoints.add(point(*origin))
    geometry = adsk.fusion.JointGeometry.createByPoint(sketch_point)
    # ACDC4Robot exporta la primera ocurrencia como hija y la segunda como padre
    joint_input = root.asBuiltJoints.createInput(child, parent, geometry)
    joint_input.setAsRevoluteJointMotion(direction)
    joint = root.asBuiltJoints.add(joint_input)
    joint.name = name
    motion = joint.jointMotion
    motion.rotationLimits.isMinimumValueEnabled = True
    motion.rotationLimits.minimumValue = math.radians(limits[0])
    motion.rotationLimits.isMaximumValueEnabled = True
    motion.rotationLimits.maximumValue = math.radians(limits[1])
    return joint


def add_side(design, materials, sketch, pelvis, side):
    root = design.rootComponent
    prefix = "left" if side > 0 else "right"
    hip = side * HIP_Y
    links = {
        "hip_yaw_link": yaw_link_bodies(side),
        "hip_roll_link": roll_link_bodies(side),
        "thigh": thigh_bodies(side),
        "shin": shin_bodies(side),
        "ankle_link": ankle_link_bodies(side),
        "foot": foot_bodies(side),
    }
    occurrences = {name: add_link(design, materials, f"{prefix}_{name}", bodies) for name, bodies in links.items()}

    directions = adsk.fusion.JointDirections
    hip_roll = (-15, 35) if side > 0 else (-35, 15)
    hip_yaw = (-15, 30) if side > 0 else (-30, 15)
    joints = [
        ("hip_yaw", "hip_yaw_link", pelvis, (0, hip, HIP_YAW_Z), directions.ZAxisJointDirection, hip_yaw),
        ("hip_roll", "hip_roll_link", occurrences["hip_yaw_link"], (HIP_ROLL_X, hip, HIP_ROLL_Z),
         directions.XAxisJointDirection, hip_roll),
        ("hip_pitch", "thigh", occurrences["hip_roll_link"], (0, hip, HIP_PITCH_Z),
         directions.YAxisJointDirection, (-100, 20)),
        ("knee", "shin", occurrences["thigh"], (0, hip, KNEE_Z), directions.YAxisJointDirection, (0, 130)),
        ("ankle_pitch", "ankle_link", occurrences["shin"], (0, hip, ANKLE_PITCH_Z),
         directions.YAxisJointDirection, (-45, 40)),
        ("ankle_roll", "foot", occurrences["ankle_link"], (ANKLE_ROLL_X, hip, ANKLE_ROLL_Z),
         directions.XAxisJointDirection, (-20, 20)),
    ]
    return [
        add_joint(root, sketch, occurrences[child], parent, f"{prefix}_{name}", origin, direction, limits)
        for name, child, parent, origin, direction, limits in joints
    ]


def add_arm(design, materials, sketch, torso, side):
    root = design.rootComponent
    prefix = "left" if side > 0 else "right"
    links = {
        "shoulder_link": shoulder_link_bodies(side),
        "upper_arm": upper_arm_bodies(side),
        "elbow_link": elbow_link_bodies(side),
        "forearm": forearm_bodies(side),
    }
    occurrences = {name: add_link(design, materials, f"{prefix}_{name}", bodies) for name, bodies in links.items()}

    directions = adsk.fusion.JointDirections
    shoulder_roll = (-10, 120) if side > 0 else (-120, 10)
    joints = [
        ("shoulder_pitch", "shoulder_link", torso, (AXIS_X, side * PITCH_Y, SHOULDER_Z),
         directions.YAxisJointDirection, (-170, 45)),
        ("shoulder_roll", "upper_arm", occurrences["shoulder_link"], (AXIS_X, side * ROLL_Y, SHOULDER_Z),
         directions.XAxisJointDirection, shoulder_roll),
        ("shoulder_yaw", "elbow_link", occurrences["upper_arm"], (AXIS_X, side * ARM_YAW_Y, ARM_YAW_Z),
         directions.ZAxisJointDirection, (-80, 80)),
        ("elbow", "forearm", occurrences["elbow_link"], (AXIS_X, side * ELBOW_Y, ELBOW_Z),
         directions.YAxisJointDirection, (-130, 0)),
    ]
    return [
        add_joint(root, sketch, occurrences[child], parent, f"{prefix}_{name}", origin, direction, limits)
        for name, child, parent, origin, direction, limits in joints
    ]


def add_upper_body(design, materials, sketch, pelvis):
    root = design.rootComponent
    directions = adsk.fusion.JointDirections
    torso = add_link(design, materials, "torso", torso_bodies())
    neck = add_link(design, materials, "neck_link", neck_link_bodies())
    head = add_link(design, materials, "head", head_bodies())
    joints = [add_joint(
        root, sketch, torso, pelvis, "waist_yaw", (WAIST_X, 0, HIP_YAW_Z), directions.ZAxisJointDirection, (-60, 60))]
    joints += add_arm(design, materials, sketch, torso, 1) + add_arm(design, materials, sketch, torso, -1)
    joints.append(add_joint(
        root, sketch, neck, torso, "neck_yaw", (AXIS_X, 0, NECK_YAW_Z), directions.ZAxisJointDirection, (-70, 70)))
    joints.append(add_joint(
        root, sketch, head, neck, "neck_pitch", (AXIS_X, 0, NECK_PITCH_Z), directions.YAxisJointDirection, (-30, 40)))
    return joints



def point_camera_up(app):
    viewport = app.activeViewport
    camera = viewport.camera
    camera.upVector = adsk.core.Vector3D.create(0, 0, 1)
    camera.eye = point(900, -700, 450)
    camera.target = point(0, 0, 200)
    camera.isSmoothTransition = False
    viewport.camera = camera


def build(app):
    document = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        return build_design(app)
    except Exception:
        document.close(False)
        raise


def build_design(app):
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.attributes.add("danny", "export_name", "danny")
    with open(os.path.abspath(__file__), "rb") as file:
        source_hash = hashlib.sha256(file.read()).hexdigest()
    design.attributes.add("danny", "source_hash", source_hash)
    root = design.rootComponent

    materials = add_materials(app, design)
    pelvis = add_link(design, materials, "base_link", pelvis_bodies())
    pelvis.isGrounded = True

    sketch = root.sketches.add(root.xYConstructionPlane)
    sketch.name = "joint_origins"
    joints = add_side(design, materials, sketch, pelvis, 1) + add_side(design, materials, sketch, pelvis, -1)
    joints += add_upper_body(design, materials, sketch, pelvis)
    point_camera_up(app)

    masses = {
        occurrence.component.name: sum(body.physicalProperties.mass for body in occurrence.bRepBodies)
        for occurrence in root.occurrences
    }
    panel_mass = sum(
        body.physicalProperties.mass
        for occurrence in root.occurrences for body in occurrence.bRepBodies
        if body.name.endswith(("_panel", "_pad"))
    )
    axes = {
        joint.name: [round(value, 4) for value in joint.jointMotion.rotationAxisVector.asArray()]
        for joint in joints
    }
    report = {
        "source_hash": source_hash,
        "masses_kg": masses,
        "total_mass_kg": sum(masses.values()),
        "axes": axes,
        "panel_mass_kg": panel_mass,
        "fit_overlaps_mm3": fit_overlaps(),
    }
    os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
    with open(RESULTS, "w", encoding="utf-8") as file:
        json.dump(report, file, indent=1)
    return report
