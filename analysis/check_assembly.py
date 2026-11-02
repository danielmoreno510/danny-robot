"""
Danny - Comprueba sobre los STL que el robot se puede montar, etapa 2.5.

Saca las piezas por su camino real, con tornillos y herramienta. SweepJointRanges no ve nada de
esto: mueve sólidos sin tornillería. Escribe results/assembly_check.json y sale con código 1
si algo no se puede montar.

Uso: .venv/bin/python analysis/check_assembly.py
"""

import json
import pathlib
import sys

import numpy as np
import trimesh

ROOT = pathlib.Path(__file__).resolve().parent.parent
PRINT = ROOT / "cad" / "print"
OUTPUT = ROOT / "results" / "assembly_check.json"
MAXIMUM_OVERLAP_MM3 = 0.05
HORN_DIAGONAL = 6.0 / 2 ** 0.5
HIP_Y = 29.0
WAIST_X = 7.5
BATTERY = (-19.6, -0.6, -32.5, 32.5, 275.2, 312.2)
COMPUTER = (6.0, 14.0, -32.5, 32.5, 277.0, 307.0)
POWER_BOARD = (17.0, 29.0, -30.0, 30.0, 277.0, 307.0)
ARM_ROLL_SCREWS_Y = (48.5, 64.5)
ARM_REST_DEG = 8.0
FLANGE_ROWS = (-12.5, 13.4)
HIP_ROLL_IN_REST_DEG = 10.0
FOREARM_MARGIN_MM = 0.4
THIGH_FORK_TOP_Z = 215.0  # por encima, el antebrazo se ensancha y no pasa junto al muslo


def part(name):
    return trimesh.load(PRINT / f"{name}.stl")


def box(x0, x1, y0, y1, z0, z1):
    transform = trimesh.transformations.translation_matrix([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=transform)


def cylinder(start, end, radius):
    start, end = np.array(start, float), np.array(end, float)
    transform = trimesh.geometry.align_vectors([0, 0, 1], (end - start) / np.linalg.norm(end - start))
    transform[:3, 3] = (start + end) / 2
    return trimesh.creation.cylinder(radius=radius, height=np.linalg.norm(end - start), sections=24, transform=transform)


def overlap(one, others):
    """Mayor volumen común, en mm³, entre una malla y las de la lista."""
    return max(float(trimesh.boolean.intersection([one, other], engine="manifold").volume) for other in others)


def path_overlap(moving, fixed, step, steps):
    """Mayor volumen común al desplazar las piezas por el camino, paso a paso."""
    worst = 0.0
    for index in range(steps + 1):
        for mesh in moving:
            worst = max(worst, overlap(mesh.copy().apply_translation(np.array(step) * index), fixed))
    return worst


def waist_heads():
    return [cylinder((WAIST_X + x, y, 275.0), (WAIST_X + x, y, 276.2), 2.0)
            for x in (-HORN_DIAGONAL, HORN_DIAGONAL) for y in (-HORN_DIAGONAL, HORN_DIAGONAL)]


def rotated_about_x(mesh, y, z, degrees):
    matrix = trimesh.transformations.rotation_matrix(np.radians(degrees), [1, 0, 0], [0, y, z])
    return mesh.copy().apply_transform(matrix)


def main():
    torso = part("torso_structure")
    tray = part("torso_tray_plate")
    heads = waist_heads()
    checks = {}

    checks["carro_sale_por_la_trampilla"] = path_overlap(
        [tray, box(*COMPUTER), box(*POWER_BOARD)], [torso] + heads, (-1.0, 0, 0), 60)
    checks["bateria_sale_por_la_trampilla"] = path_overlap([box(*BATTERY)], [torso, tray] + heads, (-1.0, 0, 0), 25)
    checks["clavija_del_cuello_baja_ante_la_bateria"] = overlap(
        box(-0.4, 3.3, 6.2, 10.0, 279.5, 316.0), [torso, tray, box(*BATTERY), box(*COMPUTER)])

    arm_parts = [part(f"left_upper_arm_{name}") for name in ("structure", "cover", "plate")]
    checks["llave_de_los_tornillos_de_roll_del_brazo"] = max(
        overlap(cylinder((22.0, y, 302.9), (55.0, y, 302.9), 1.0), arm_parts) for y in ARM_ROLL_SCREWS_Y)

    flange_heads = [cylinder((-23.2, HIP_Y + row, 216.8), (-23.2, HIP_Y + row, 218.0), 1.5) for row in FLANGE_ROWS]
    yaw_parts = [part("left_hip_yaw_link_structure"), cylinder((-30, HIP_Y, 222), (-22.2, HIP_Y, 222), 8.5)]
    checks["cabezas_de_la_chapa_de_roll_en_todo_el_rango"] = max(
        overlap(rotated_about_x(head, HIP_Y, 222.0, angle), yaw_parts)
        for head in flange_heads for angle in range(-15, 36, 10))
    roll_parts = [part("left_hip_roll_link_plate"), part("left_hip_roll_link_structure"), part("left_hip_roll_link_cover")]
    checks["chapa_de_roll_contra_el_eslabon_yaw"] = max(
        overlap(rotated_about_x(mesh, HIP_Y, 222.0, angle), yaw_parts[:1])
        for mesh in roll_parts[:1] for angle in range(-15, 36, 10))
    screw_hole = cylinder((-22.5, HIP_Y - 15, 208.0), (-22.5, HIP_Y + 15, 208.0), 2.0)
    checks["piloto_de_la_chapa_contra_el_agujero_del_servo"] = overlap(
        screw_hole, [cylinder((-23.2, HIP_Y + row, 211.0), (-23.2, HIP_Y + row, 216.0), 0.7) for row in FLANGE_ROWS])
    forearm_at_rest = rotated_about_x(part("left_forearm_structure"), 56.5, 325.4, ARM_REST_DEG)
    thigh_abducted = rotated_about_x(part("left_thigh_structure"), HIP_Y, 222.0, HIP_ROLL_IN_REST_DEG)
    checks["antebrazo_en_reposo_con_la_cadera_abierta"] = overlap(forearm_at_rest, [thigh_abducted])

    forearm = part("left_forearm_structure").vertices
    forearm_gap = float(forearm[forearm[:, 2] < THIGH_FORK_TOP_Z, 1].min() - (HIP_Y + 17.5))
    failures = [name for name, volume in checks.items() if volume > MAXIMUM_OVERLAP_MM3]
    if forearm_gap < FOREARM_MARGIN_MM:
        failures.append("holgura_entre_muslo_y_antebrazo")
    result = {"overlaps_mm3": checks, "thigh_to_forearm_gap_mm": forearm_gap, "failures": failures}
    OUTPUT.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=1, ensure_ascii=False))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
