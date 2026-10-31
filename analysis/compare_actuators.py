"""
Danny - Comparación de primer orden de servos, alturas y mezclas (etapa 1).

Da órdenes de magnitud para decidir qué medir y calcular después. No sustituye a
la simulación. La estructura se escala desde Open Duck Mini, y los pares y la
velocidad con las relaciones del estudio archivado en `archivo-danny-70cm`.

Fuentes en documentation/research-and-requirements.md.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

GRAVITY = 9.81
OPEN_DUCK_STRUCTURE = 1.19  # kg a 42 cm, estimado
OPEN_DUCK_HEIGHT = 0.42
STRUCTURE_EXPONENT = 2.5
ELECTRONICS = 0.15  # kg, ordenador, cámara, adaptador de bus, IMU y cables (estimado)
BATTERY = 0.096  # kg, dos celdas 18650 (estimado)
HIP_HALF_WIDTH = 0.03  # m (estimado)
ANKLE_HEIGHT = 0.03  # m
THIGH_RATIO = 0.2125
HUMAN_LEG = 0.9  # m
SPEED_PER_LEG_RAD = 0.222  # v = k · L · ω, del estudio archivado
SAFETY_FACTOR = 1.5
LOADED_SPEED_FRACTION = 0.6  # fracción de la velocidad en vacío durante el empuje (estimado)
CROUCH_KNEE = math.radians(90)  # flexión de rodilla al despegar


@dataclass(frozen=True)
class Servo:
    name: str
    stall: float  # N·m
    rated: float  # N·m
    speed: float  # rad/s, en vacío
    mass: float  # kg


HD_1910 = Servo("HD-1910 7,4 V", 15 * 0.0981, 3.7 * 0.0981, (math.pi / 3) / 0.088, 0.021)
STS3215_C018 = Servo("STS3215 C018 12 V", 2.94, 0.98, 4.71, 0.055)
# El par del C044 está en conflicto entre tiendas (1,6 o 2,7 N·m): se usa el peor
STS3215_C044 = Servo("STS3215 C044 7,4 V", 1.6, 0.88, 9.0, 0.055)

LEG_SERVOS = 12
HEIGHTS = (0.32, 0.34, 0.36)
UPPER_SERVOS = {"22 GDL": 10, "26 GDL con muñeca y pinza": 14}
MIXES = {
    "todo HD-1910": (HD_1910, HD_1910),
    "piernas C044, arriba HD-1910": (STS3215_C044, HD_1910),
    "todo C018": (STS3215_C018, STS3215_C018),
}


def evaluate(height: float, legs: Servo, upper: Servo, upper_count: int) -> dict:
    structure = OPEN_DUCK_STRUCTURE * (height / OPEN_DUCK_HEIGHT) ** STRUCTURE_EXPONENT
    mass = LEG_SERVOS * legs.mass + upper_count * upper.mass + structure + ELECTRONICS + BATTERY
    thigh = THIGH_RATIO * height
    leg = ANKLE_HEIGHT + 2 * thigh

    # Relaciones calibradas con 2,69 kg, muslo de 85 mm y pierna de 0,2 m
    hip_roll = mass * GRAVITY * HIP_HALF_WIDTH
    knee = 0.795 * (mass / 2.69) * (thigh / 0.085)
    ankle = 0.957 * (mass / 2.69) * (leg / 0.2)
    peak = max(hip_roll, knee, ankle)

    walk_speed = SPEED_PER_LEG_RAD * leg * legs.speed
    froude = walk_speed**2 / (GRAVITY * leg)

    # Salto: la pierna se estira a la velocidad que dejan rodilla y tobillo
    extension = thigh * math.sin(CROUCH_KNEE / 2) + ANKLE_HEIGHT
    takeoff = extension * legs.speed
    return {
        "masa_kg": round(mass, 3),
        "par_pico_nm": round(peak, 3),
        "pct_bloqueo_con_margen": round(100 * SAFETY_FACTOR * peak / legs.stall, 1),
        "supera_nominal_con_margen": SAFETY_FACTOR * peak > legs.rated,
        "marcha_techo_ms": round(walk_speed, 3),
        "marcha_humana_equivalente_ms": round(math.sqrt(froude * GRAVITY * HUMAN_LEG), 2),
        "salto_techo_cm": round(100 * takeoff**2 / (2 * GRAVITY), 1),
        "salto_cargado_cm": round(100 * (LOADED_SPEED_FRACTION * takeoff) ** 2 / (2 * GRAVITY), 1),
    }


def main():
    results = {}
    for servo in (HD_1910, STS3215_C018, STS3215_C044):
        print(f"{servo.name}: {servo.stall:.2f} N·m, {servo.speed:.1f} rad/s, "
              f"{servo.stall * servo.speed / 4 / servo.mass:.0f} W/kg")
    for mix, (legs, upper) in MIXES.items():
        for dof, upper_count in UPPER_SERVOS.items():
            for height in HEIGHTS:
                row = evaluate(height, legs, upper, upper_count)
                results[f"{mix} | {dof} | {height * 100:.0f} cm"] = row
                print(f"{mix:30} {dof:26} {height * 100:.0f} cm  {row}")

    output = Path(__file__).resolve().parents[1] / "results" / "actuator_comparison.json"
    output.write_text(json.dumps(results, indent=1, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
