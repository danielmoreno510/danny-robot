"""
Danny - Estima la fuerza de una caída desde de pie, con el centro de masas del CAD.

Es un cálculo de primer orden: toda la energía potencial acaba en un solo golpe y la fuerza
crece linealmente con la deformación, así que F = 2 E / recorrido. Escribe
results/fall_estimate.json.

Uso: .venv/bin/python analysis/estimate_fall_energy.py
"""

import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
CHECK = ROOT / "results" / "robot_check.json"
OUTPUT = ROOT / "results" / "fall_estimate.json"

GRAVITY = 9.81
# Recorrido hasta parar, en mm (E)
STOPPING_MM = {
    "panel_rigid": 1.0,
    "pad_and_panel": 3.0,
    "arm_folds_first": 15.0,
    "servos_give_way": 40.0,
}
# Fuerza que aguanta el panel del pecho repartida en sus cuatro tornillos M2 en plástico, en N (E)
PANEL_SCREWS_N = 4 * 150.0


def main():
    check = json.loads(CHECK.read_text(encoding="utf-8"))
    mass = check["estimated_mass_kg"]
    height = check["center_of_mass_mm"][2] / 1000
    energy = mass * GRAVITY * height
    scenarios = {
        name: {
            "stopping_mm": stopping,
            "peak_force_n": 2 * energy / (stopping / 1000),
            "peak_acceleration_g": 2 * energy / (stopping / 1000) / (mass * GRAVITY),
            "panel_screws_hold": 2 * energy / (stopping / 1000) < PANEL_SCREWS_N,
        }
        for name, stopping in STOPPING_MM.items()
    }
    result = {"mass_kg": mass, "center_of_mass_height_m": height, "energy_j": energy, "scenarios": scenarios}
    OUTPUT.write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
