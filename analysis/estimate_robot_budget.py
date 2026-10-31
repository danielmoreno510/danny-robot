"""
Danny - Desglose de piezas, masa, coste, alturas, energía y bus (etapa 1.3).

Reúne las piezas elegidas y comprueba si caben, cuánto pesan y cuánto cuestan antes
de abrir Fusion. Las medidas de cada pieza van en mm, sus masas en g y sus precios en
euros con IVA. Base: V fabricante, S tienda, E estimación.

Criterios en documentation/research-and-requirements.md.
"""

from dataclasses import dataclass
import json
from pathlib import Path

BUDGET = 1500  # €
MASS_LIMIT = 1800  # g, R4, sin manos
MASS_LIMIT_WITH_HANDS = 1900  # g, con manos
RUNTIME_GOAL = 20  # min, R5
PRICE_RESERVE = 0.10

JOINTS_FIRST = {"pierna": 12, "cintura": 1, "brazo": 8, "cuello": 2}
JOINTS_LATER = {"muñeca": 2, "pinza": 2}
SPARE_SERVOS = 3
SERVO_PRICE = 31.98


@dataclass(frozen=True)
class Part:
    name: str
    count: int
    size: str  # mm
    shape: str
    mass: float  # g por unidad, en el robot; 0 si no va montado
    price: float  # € por unidad
    basis: str


first_servos = sum(JOINTS_FIRST.values())
later_servos = sum(JOINTS_LATER.values())

VOICE_PARTS = (
    Part("Micrófono, amplificador y altavoz", 1, "Por elegir", "Placas y altavoz", 18, 18, "E"),
)


def robot_parts(structure_mass, filament_spools):
    return (
    Part("Servo HD-1910-C001", first_servos, "34 × 20 × 23 (29 con ejes)", "Caja, eje doble", 21, SERVO_PRICE, "V/S"),
    Part("Servos de repuesto", SPARE_SERVOS, "34 × 20 × 23", "Caja", 0, SERVO_PRICE, "S"),
    Part("Ordenador Radxa Zero 3W 4 GB", 1, "65 × 30 × 8", "Placa", 15, 59.83, "V/S; masa E"),
    Part("Cámara IMX219", 1, "25 × 24 × 9", "Placa con lente", 4, 20, "E"),
    Part("IMU BNO085", 1, "26 × 18 × 5", "Placa", 3, 25, "E"),
    Part("Regulador 5 V 5 A D24V50F5", 1, "18 × 20 × 9", "Placa", 3, 30, "V/S"),
    Part("Placa propia de energía y bus", 1, "60 × 30 × 12", "Placa a medida", 15, 30, "E"),
    Part("Paquete 2S P30B con BMS", 1, "65 × 37 × 19", "Dos cilindros soldados", 104, 30, "V celda; E paquete"),
    Part("Paquete de repuesto", 1, "65 × 37 × 19", "Dos cilindros", 0, 30, "E"),
    Part("Interruptor, fusible y XT30", 1, "30 × 15 × 12", "Varios", 8, 12, "E"),
    Part("Cables y conectores", 1, "Varios", "Mazo", 40, 20, "E"),
    Part("Rodamientos de los ejes libres", 2, "10 × 15 × 3 y 16 × 22 × 4", "Anillos, 14 por pack", 16, 35.95, "S; masa E"),
    Part("Tornillos e insertos M2", 1, "M2", "Varios", 60, 25, "E"),
    Part("Estructura PETG-CF", filament_spools, "1 kg por bobina", "Impresa", structure_mass / filament_spools, 33, "S; masa E"),
    Part("Suelas TPU 95A, 1 bobina", 1, "1 kg", "Impresa", 10, 35, "S; masa E"),
    Part("FE-URT-2, configuración", 1, "Placa", "Banco", 0, 15.96, "S"),
    Part("Fuente de 3-12 V y 5 A", 1, "Caja", "Banco", 0, 18, "S"),
    Part("Cargador de 8,4 V", 1, "Caja", "Banco", 0, 12, "E"),
    Part("Envíos", 1, "-", "-", 0, 40, "E"),
    )

LATER_PARTS = (
    Part("Servos de muñeca y pinza", later_servos, "34 × 20 × 23", "Caja", 21, SERVO_PRICE, "S"),
    Part("Pinzas impresas", 2, "60 × 30 × 25", "Impresa", 12, 0, "E"),
)


@dataclass(frozen=True)
class Design:
    stack: dict  # altura de cada tramo, de abajo arriba, en mm
    torso_inner: tuple  # ancho, fondo, alto
    structure_mass: float  # g, impreso y paneles
    filament_spools: int


def stack(shin, thigh):
    return {
        "Suelo a tobillo: suela, placa y dos ejes cruzados": 35,
        "Tibia": shin,
        "Muslo": thigh,
        "Cadera: pitch-roll, brida y yaw": 48,
        "Torso: cintura 23, electrónica 37, hombros 24 y paredes 4": 88,
        "Holgura de cuello": 4,
        "Cabeza con cuello y cámara": 55,
    }


# Estructura: 600 g a 36 cm, escalada con exponente 2,5, y 100 g de paneles en el de 40 cm (E)
DESIGNS = {
    "Compacto de 36 cm": Design(stack(65, 65), (74, 46, 84), 600, 2),
    "Proporciones Optimus de 40 cm": Design(stack(82, 83), (66, 46, 84), 880, 3),
}
SHOULDER_SERVOS_WIDTH = 2 * 29
BATTERY_WIDTH = 65
COMPUTER_WIDTH = 65
PI5_WIDTH = 85
ELECTRONICS_DEPTH = 19 + 12 + 12 + 3  # batería, ordenador, placa propia y holgura
ARM_OUTSIDE = 30  # mm por lado: servo de roll y holgura
HEAD_HEIGHT = 55
OPTIMUS_BREADTH_LIMIT = 34  # % de la altura, E
OPTIMUS_LEG_MINIMUM = 49  # % de la altura, E

# Consumo medio caminando, E
COMPUTER_WATTS = 4.0
BATTERY_WH = 2 * 3.6 * 3.0
USABLE_ENERGY = 0.8
BATTERY_VOLTAGE = 7.2
STANCE_AMPS = (6, 0.55)
SWING_AMPS = (6, 0.25)
UPPER_AMPS = (11, 0.10)
PEAK_LEG_AMPS = 12 * 1.2

BUS_BITS_PER_BYTE = 10
BUS_RATE = 1_000_000
BUS_WRITE_BYTES = 8 + 7 * first_servos
BUS_READ_BYTES = 8 + 2 + 17 * first_servos
POLICY_PERIOD_MS = 20


def totals(parts):
    return (sum(p.count * p.mass for p in parts), sum(p.count * p.price for p in parts))


def evaluate(design):
    parts = robot_parts(design.structure_mass, design.filament_spools)
    mass, cost = totals(parts)
    later_mass, later_cost = totals(LATER_PARTS)
    voice_mass, voice_cost = totals(VOICE_PARTS)
    height = sum(design.stack.values())
    leg = sum(list(design.stack.values())[:3])
    breadth = design.torso_inner[0] + 6 + 2 * ARM_OUTSIDE

    servo_amps = sum(n * amps for n, amps in (STANCE_AMPS, SWING_AMPS, UPPER_AMPS))
    watts = servo_amps * BATTERY_VOLTAGE + COMPUTER_WATTS
    runtime = 60 * BATTERY_WH * USABLE_ENERGY / watts
    bus_ms = 1000 * (BUS_WRITE_BYTES + BUS_READ_BYTES) * BUS_BITS_PER_BYTE / BUS_RATE

    return {
        "gdl_inicial": first_servos,
        "gdl_con_manos": first_servos + later_servos,
        "masa_g": round(mass),
        "masa_con_manos_g": round(mass + later_mass),
        "coste_eur": round(cost, 2),
        "coste_con_manos_eur": round(cost + later_cost, 2),
        "voz_masa_g": round(voice_mass),
        "voz_coste_eur": round(voice_cost, 2),
        "coste_con_reserva_eur": round(cost * (1 + PRICE_RESERVE), 2),
        "coste_servos_eur": round((first_servos + SPARE_SERVOS) * SERVO_PRICE, 2),
        "altura_mm": height,
        "pierna_pct": round(100 * leg / height, 1),
        "ancho_hombros_mm": breadth,
        "ancho_hombros_pct": round(100 * breadth / height, 1),
        "cabeza_pct": round(100 * HEAD_HEIGHT / height, 1),
        "potencia_w": round(watts, 1),
        "autonomia_min": round(runtime, 1),
        "pico_piernas_a": round(PEAK_LEG_AMPS, 1),
        "bus_ms_por_ciclo": round(bus_ms, 1),
        "tramos_mm": design.stack,
        "piezas": [
            {"pieza": p.name, "cantidad": p.count, "medidas_mm": p.size, "forma": p.shape,
             "masa_g": round(p.mass, 1), "precio_eur": p.price, "base": p.basis}
            for p in parts
        ],
        "comprobaciones": {
            "Masa inicial en R4": mass <= MASS_LIMIT,
            "Masa con manos en R4": mass + later_mass <= MASS_LIMIT_WITH_HANDS,
            "Coste inicial en el tope": cost <= BUDGET,
            "Coste con manos en el tope": cost + later_cost <= BUDGET,
            "Coste con reserva en el tope": cost * (1 + PRICE_RESERVE) <= BUDGET,
            "Autonomía en R5": runtime >= RUNTIME_GOAL,
            "Hombros de Optimus": 100 * breadth / height <= OPTIMUS_BREADTH_LIMIT,
            "Pierna de Optimus": 100 * leg / height >= OPTIMUS_LEG_MINIMUM,
            "Servos de hombro en el torso": SHOULDER_SERVOS_WIDTH <= design.torso_inner[0],
            "Batería y ordenador a lo ancho": max(BATTERY_WIDTH, COMPUTER_WIDTH) <= design.torso_inner[0],
            "Electrónica en fondo": ELECTRONICS_DEPTH <= design.torso_inner[1],
            "Bus en 20 ms": bus_ms < POLICY_PERIOD_MS,
            "Raspberry Pi 5 a lo ancho": PI5_WIDTH <= design.torso_inner[0],
        },
    }


def main():
    results = {name: evaluate(design) for name, design in DESIGNS.items()}
    for name, row in results.items():
        print(name)
        print(json.dumps({k: v for k, v in row.items() if k not in ("tramos_mm", "piezas")},
                         indent=1, ensure_ascii=False))
    output = Path(__file__).resolve().parents[1] / "results" / "robot_budget.json"
    output.write_text(json.dumps(results, indent=1, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
