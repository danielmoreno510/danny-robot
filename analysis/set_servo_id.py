"""
Danny - Cambia el ID de un servo HD-1910 por la FE-URT-2 (banco 5.1). Los tres servos vienen con ID 1: conecta **solo uno** al bus.

No activa el par ni mueve nada. Pasos: comprueba que el ID nuevo no responde (no hay otro servo con él), desbloquea la EEPROM
(`lock` = 0), escribe el ID, vuelve a bloquear en el ID nuevo y relee la memoria. Escribe results/bench_servo_<label>_id.json.

Criterio:
1. antes, el ID viejo responde y el nuevo no;
2. después, el ID nuevo responde y el viejo no;
3. la EEPROM (0-39) queda igual salvo el ID;
4. la EEPROM queda bloqueada (`lock` = 1).

Uso: .venv/bin/python analysis/set_servo_id.py --label A --new-id 2
"""

import argparse
import json
import time

import danny_sim
import feetech_bus

EEPROM_BYTES = 40


def responds(bus, servo_id):
    try:
        bus.ping(servo_id)
        return True
    except feetech_bus.BusError:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, help="nombre del servo en el banco (A, B o C), para el informe")
    parser.add_argument("--new-id", type=int, required=True)
    parser.add_argument("--id", type=int, default=1, help="ID actual (1 de fábrica)")
    parser.add_argument("--baud", type=int, default=feetech_bus.DEFAULT_BAUD)
    parser.add_argument("--port", default=None)
    args = parser.parse_args()
    if not 1 <= args.new_id <= 253 or args.new_id == args.id:
        parser.error("el ID nuevo debe ir de 1 a 253 y ser distinto del actual")

    bus = feetech_bus.Bus(args.port, args.baud)
    try:
        old_before, new_before = responds(bus, args.id), responds(bus, args.new_id)
        if not old_before or new_before:
            raise SystemExit(f"Parado sin escribir: el ID {args.id} {'responde' if old_before else 'no responde'} y el {args.new_id} "
                             f"{'ya responde' if new_before else 'no responde'}. Deja un solo servo en el bus.")
        _, before = bus.read(args.id, 0, feetech_bus.MEMORY_BYTES)
        variant, little = feetech_bus.detect_variant(before)
        if variant == "desconocida":
            raise SystemExit("Parado sin escribir: no se reconoce la variante del protocolo")
        print(f"Servo {args.id} ({variant}): cambio a ID {args.new_id}")
        feetech_bus.write_register(bus, args.id, "lock", 0, little)
        feetech_bus.write_register(bus, args.id, "id", args.new_id, little)
        time.sleep(0.05)
        feetech_bus.write_register(bus, args.new_id, "lock", 1, little)
        time.sleep(0.05)
        old_after, new_after = responds(bus, args.id), responds(bus, args.new_id)
        _, after = bus.read(args.new_id, 0, feetech_bus.MEMORY_BYTES) if new_after else (None, before)
    finally:
        bus.close()

    id_address = feetech_bus.REGISTERS["id"][0]
    changed = [a for a in range(EEPROM_BYTES) if a != id_address and before[a] != after[a]]
    criteria = {
        "before_old_responds_new_silent": {"value": [old_before, new_before], "passes": old_before and not new_before},
        "after_new_responds_old_silent": {"value": [new_after, old_after], "passes": new_after and not old_after},
        "eeprom_unchanged_except_id": {"value": changed, "passes": not changed},
        "eeprom_locked": {"value": feetech_bus.value(after, "lock", little), "passes": feetech_bus.value(after, "lock", little) == 1},
    }
    report = {
        "servo": "HD-1910", "label": args.label, "old_id": args.id, "new_id": args.new_id, "date": time.strftime("%Y-%m-%d %H:%M"),
        "memory_before": list(before), "memory_after": list(after), "criteria": criteria,
        "passes": all(c["passes"] for c in criteria.values()),
    }
    out = danny_sim.ROOT / "results" / f"bench_servo_{args.label}_id.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("Criterio:")
    for name, c in criteria.items():
        print(f"  {'cumple' if c['passes'] else 'NO cumple'}  {name}: {c['value']}")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
