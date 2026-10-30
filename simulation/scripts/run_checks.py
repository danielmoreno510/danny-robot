"""
Danny - Ejecuta todas las comprobaciones del proyecto.

Lanza cada script de análisis, guarda su salida completa en results/ y resume
el resultado. Devuelve 1 si alguna falla, así que sirve como test automático.

Uso:
  python simulation/scripts/run_checks.py
"""

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "simulation" / "scripts"
RESULTS = ROOT / "results"

# (archivo de resultados, script, argumentos)
E13_WALK = ["--knee", "56", "--step", "0.08", "--width", "0.05", "--inset", "0.012",
            "--turn", "45", "--speed", "1.27", "--hold", "0.25"]

CHECKS = [
    ("kinematics", "check_kinematics.py", []),
    ("e05_e07_leg_2dof", "analyze_leg_2dof.py", []),
    ("e08_squat", "squat_leg_2dof.py", []),
    ("e09_leg_6dof", "analyze_leg_6dof.py", []),
    ("e10_balance", "balance_biped.py", []),
    ("e11_walk", "walk_biped.py", []),
    ("e12_walk_fast", "walk_biped.py", ["--speed", "1.27", "--hold", "0.25"]),
    ("e13_walk_turn", "walk_biped.py", E13_WALK),
    ("e14_push", "push_biped.py", []),
    ("e15_walk_x2", "walk_biped.py", ["--knee", "56", "--step", "0.08", "--width", "0.05",
                                      "--inset", "0.012", "--speed", "2", "--hold", "0.25"]),
    ("audit_mass_plus10", "walk_biped.py", [*E13_WALK, "--mass-error", "0.1"]),
    ("audit_mass_minus10", "walk_biped.py", [*E13_WALK, "--mass-error", "-0.1"]),
    ("audit_torso_plus10", "walk_biped.py", [*E13_WALK, "--torso-error", "0.1"]),
    ("audit_torso_minus10", "walk_biped.py", [*E13_WALK, "--torso-error", "-0.1"]),
    ("audit_friction_06", "walk_biped.py", [*E13_WALK, "--friction", "0.6"]),
    ("audit_timestep_half", "walk_biped.py", [*E13_WALK, "--timestep", "0.001"]),
]


def main() -> int:
    RESULTS.mkdir(exist_ok=True)
    failed = 0
    for name, script, arguments in CHECKS:
        run = subprocess.run(
            [sys.executable, str(SCRIPTS / script), *arguments],
            capture_output=True,
            text=True,
        )
        (RESULTS / f"{name}.txt").write_text(run.stdout + run.stderr)
        passed = run.returncode == 0
        failed += not passed
        print(f"{'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n{len(CHECKS) - failed} de {len(CHECKS)} comprobaciones pasan. Detalle en results/.")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
