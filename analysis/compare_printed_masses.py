"""
Danny - Compara la masa de las piezas impresas con la del CAD (paso 2.2).

El CAD da masas con las piezas macizas a 1,25 g/cm³ (PETG-CF, `STRUCTURE_DENSITY` en BuildRobot). Las piezas de la primera tanda se
imprimieron en PLA (1,24 g/cm³ de ficha, E) con 4 perímetros y 40 % de relleno, y se pesaron en una báscula de cocina de 1 g de
resolución: cada lectura vale ±0,5 g. Escribe results/printed_masses.json con la masa medida, la del CAD maciza y su cociente.

Criterio: la masa del CAD no puede quedarse corta. En el conjunto de piezas del robot, la medida debe ser como mucho la del CAD
más el error de lectura; si fuera mayor, las masas de la simulación estarían por debajo de las reales.

Uso: .venv/bin/python analysis/compare_printed_masses.py
"""

import json

import trimesh

import danny_sim

CAD_DENSITY_G_CM3 = 1.25
READING_ERROR_G = 0.5

# Pesadas del dueño el 12 de noviembre de 2026 (fotos IMG_5084 a IMG_5093), en gramos
PRINTED = {
    "first_pla_batch": {
        "material": "Elegoo PLA, 0,20 mm, 4 perímetros, 40 % de relleno",
        "parts": {
            "left_hip_roll_link_structure": 8,
            "left_hip_roll_link_cover": 2,
            "left_hip_roll_link_plate": 2,
            "left_shoulder_link_structure": 4,
            "left_upper_arm_plate": 10,
            "left_upper_arm_structure": 8,
            "left_upper_arm_cover": 1,
            "torso_tray_plate": 7,
            "bench/bench_servo_holder": 18,
            "bench/bench_load_arm": 6,
        },
    },
}


def main():
    report = {"cad_density_g_cm3": CAD_DENSITY_G_CM3, "reading_error_g": READING_ERROR_G, "batches": {}}
    for batch, data in PRINTED.items():
        rows = {}
        robot_measured = robot_cad = 0.0
        robot_count = 0
        for name, grams in data["parts"].items():
            volume = trimesh.load(danny_sim.ROOT / "cad" / "print" / f"{name}.stl").volume / 1000
            cad = volume * CAD_DENSITY_G_CM3
            rows[name] = {"measured_g": grams, "volume_cm3": round(float(volume), 2), "cad_solid_g": round(float(cad), 1),
                          "ratio": round(float(grams / cad), 2)}
            if not name.startswith("bench/"):
                robot_measured += grams
                robot_cad += cad
                robot_count += 1
        limit = robot_cad + READING_ERROR_G * robot_count
        report["batches"][batch] = {
            "material": data["material"], "parts": rows,
            "robot_parts": {"measured_g": robot_measured, "cad_solid_g": round(float(robot_cad), 1),
                            "ratio": round(float(robot_measured / robot_cad), 2), "limit_g": round(float(limit), 1),
                            "passes": bool(robot_measured <= limit)},
        }
        print(f"{batch}:")
        for name, row in rows.items():
            print(f"  {name}: {row['measured_g']} g medido, {row['cad_solid_g']} g CAD macizo ({row['ratio']})")
        total = report["batches"][batch]["robot_parts"]
        print(f"  Piezas del robot: {total['measured_g']} g medido, {total['cad_solid_g']} g CAD ({total['ratio']}), "
              f"{'cumple' if total['passes'] else 'NO cumple'}")
    out = danny_sim.ROOT / "results" / "printed_masses.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
