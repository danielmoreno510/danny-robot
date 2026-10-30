"""
Sustituto mínimo de la API de Fusion para probar el script sin Fusion.

Imita los nombres confirmados en la referencia oficial y el cambio de signo del plano
XZ. Resuelve solo las cotas en orden. No demuestra que el script funcione en Fusion.
"""

import re
import types


MM = 0.1


class Point3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    @staticmethod
    def create(x=0.0, y=0.0, z=0.0):
        return Point3D(x, y, z)


class SketchPoint:
    def __init__(self, sketch, model):
        self.sketch, self.model = sketch, model

    @property
    def geometry(self):
        return self.sketch.modelToSketchSpace(self.model)

    @property
    def worldGeometry(self):
        return self.model


class Dimension:
    def __init__(self, design, start, end):
        self.design, self.start, self.end = design, start, end
        self.parameter = types.SimpleNamespace(expression="")

    def solve(self):
        delta = [e - s for s, e in zip(vars(self.start.model).values(), vars(self.end.model).values())]
        length = sum(d * d for d in delta) ** 0.5
        wanted = self.design.evaluate(self.parameter.expression) * MM
        unit = [d / length for d in delta]
        start = vars(self.start.model)
        self.end.model = Point3D(*(s + wanted * u for s, u in zip(start.values(), unit)))


class Line:
    def __init__(self, start, end):
        self.startSketchPoint, self.endSketchPoint = start, end
        self.isConstruction = False
        self.constraint = None


class Sketch:
    def __init__(self, design, plane):
        self.design, self.plane = design, plane
        self.name = ""
        self.lines, self.dimensions = [], []
        self.originPoint = SketchPoint(self, Point3D())
        self.sketchCurves = types.SimpleNamespace(sketchLines=types.SimpleNamespace(addByTwoPoints=self.add_line))
        self.sketchDimensions = types.SimpleNamespace(addDistanceDimension=self.add_dimension)
        self.geometricConstraints = types.SimpleNamespace(addHorizontal=self.horizontal, addVertical=self.vertical)

    def modelToSketchSpace(self, point):
        if self.plane == "xZ":
            return Point3D(point.x, -point.z, 0.0)
        return Point3D(-point.z, point.y, 0.0)

    def sketchToModelSpace(self, point):
        if self.plane == "xZ":
            return Point3D(point.x, 0.0, -point.y)
        return Point3D(0.0, point.y, -point.x)

    def add_line(self, start, end):
        line = Line(start, SketchPoint(self, self.sketchToModelSpace(end)))
        self.lines.append(line)
        return line

    def add_dimension(self, one, two, orientation, text):
        assert isinstance(one, SketchPoint) and isinstance(two, SketchPoint)
        assert isinstance(text, Point3D) and orientation == "aligned"
        dimension = Dimension(self.design, one, two)
        self.dimensions.append(dimension)
        return dimension

    def horizontal(self, line):
        line.constraint = "horizontal"

    def vertical(self, line):
        line.constraint = "vertical"

    @property
    def isFullyConstrained(self):
        return all(line.constraint for line in self.lines) and len(self.dimensions) == len(self.lines) and all(
            d.parameter.expression for d in self.dimensions)


class ConstructionPoints:
    def __init__(self):
        self.items = []

    def createInput(self):
        return types.SimpleNamespace(setByPoint=lambda point: setattr(self, "pending", point))

    def add(self, _):
        assert isinstance(self.pending, SketchPoint)  # en paramétrico, Point3D falla
        item = types.SimpleNamespace(name="", point=self.pending)
        self.items.append(item)
        return item


class Component:
    def __init__(self, design):
        self.design = design
        self.name = ""
        self.children = []
        self.constructionPoints = ConstructionPoints()
        self.xZConstructionPlane, self.yZConstructionPlane = "xZ", "yZ"
        self.sketches = types.SimpleNamespace(add=self.add_sketch, items=[])
        self.occurrences = types.SimpleNamespace(addNewComponent=self.add_occurrence)

    def add_sketch(self, plane):
        sketch = Sketch(self.design, plane)
        self.sketches.items.append(sketch)
        return sketch

    def add_occurrence(self, _):
        component = Component(self.design)
        self.children.append(component)
        return types.SimpleNamespace(component=component)


class UserParameter:
    def __init__(self, name, expression):
        self.name, self.expression = name, expression

    @property
    def value(self):
        return float(self.expression.split()[0]) * MM


class UserParameters(list):
    def add(self, name, value, units, comment):
        assert units == "mm" and comment
        parameter = UserParameter(name, value)
        self.append(parameter)
        return parameter

    def itemByName(self, name):
        return next(p for p in self if p.name == name)


class Design:
    @staticmethod
    def cast(value):
        return value

    def __init__(self):
        self.userParameters = UserParameters()
        self.rootComponent = Component(self)
        self.designType = None

    def evaluate(self, expression):
        values = {p.name: p.value / MM for p in self.userParameters}
        assert re.fullmatch(r"[a-z_ +]+", expression), expression
        return eval(expression, {"__builtins__": {}}, values)

    def computeAll(self):
        for sketch in self.sketches():
            for dimension in sketch.dimensions:
                dimension.solve()
        return True

    def sketches(self):
        pending, found = [self.rootComponent], []
        while pending:
            component = pending.pop()
            found += component.sketches.items
            pending += component.children
        return found


def install(orientation="zup"):
    """Módulos adsk falsos, y la aplicación con un diseño vacío."""
    design = Design()
    messages = []
    app = types.SimpleNamespace(
        activeProduct=design,
        version="fake",
        userInterface=types.SimpleNamespace(messageBox=messages.append),
        documents=types.SimpleNamespace(add=lambda kind: None),
        preferences=types.SimpleNamespace(generalPreferences=types.SimpleNamespace(defaultModelingOrientation=orientation)),
    )
    core = types.ModuleType("adsk.core")
    core.Point3D = Point3D
    core.Matrix3D = types.SimpleNamespace(create=lambda: None)
    core.ValueInput = types.SimpleNamespace(createByString=lambda text: text)
    core.DocumentTypes = types.SimpleNamespace(FusionDesignDocumentType="design")
    core.DefaultModelingOrientations = types.SimpleNamespace(ZUpModelingOrientation="zup")
    core.Application = types.SimpleNamespace(get=lambda: app)
    fusion = types.ModuleType("adsk.fusion")
    fusion.Design = Design
    fusion.DesignTypes = types.SimpleNamespace(ParametricDesignType="parametric")
    fusion.DimensionOrientations = types.SimpleNamespace(AlignedDimensionOrientation="aligned")
    adsk = types.ModuleType("adsk")
    adsk.core, adsk.fusion = core, fusion
    return {"adsk": adsk, "adsk.core": core, "adsk.fusion": fusion}, app, design, messages
