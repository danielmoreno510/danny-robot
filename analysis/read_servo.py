"""
Danny - Primera prueba del banco (5.1): lee un servo HD-1910 por la FE-URT-2 sin escribir nada ni moverlo.

Hace ping, lee la memoria entera (0-70), decide la variante del protocolo (SCS o STS) y, con --watch, sigue la posición unos
segundos mientras se gira la bocina a mano con el par desactivado. Escribe results/bench_servo_<label>_read.json con la memoria en
crudo y el criterio de esta prueba:
1. responde al ping (ID 1 a 1 Mbit/s de fábrica);
2. variante identificada por los límites de ángulo;
3. tensión leída a ±0,3 V de la que marca la fuente (--supply-volts);
4. temperatura leída entre 15 y 35 °C;
5. con el par desactivado, la posición cambia al girar la bocina a mano (--watch), más de 100 cuentas.

Uso: .venv/bin/python analysis/read_servo.py --label A --supply-volts 7.4 --watch 10
"""

import argparse
import json
import time

import danny_sim
import feetech_bus

VOLTAGE_TOLERANCE_V = 0.3
ROOM_TEMPERATURE_C = (15, 35)
HAND_TURN_COUNTS = 100


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, help="nombre del servo en el banco (A, B o C), para el informe")
    parser.add_argument("--supply-volts", type=float, required=True, help="tensión que marca la fuente, en V")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--baud", type=int, default=feetech_bus.DEFAULT_BAUD)
    parser.add_argument("--port", default=None, help="puerto serie; si no, se busca el único USB serie")
    parser.add_argument("--watch", type=float, default=0.0, help="segundos siguiendo la posición mientras giras la bocina a mano")
    args = parser.parse_args()

    bus = feetech_bus.Bus(args.port, args.baud)
    print(f"Puerto {bus.port} a {args.baud} bit/s")
    try:
        ping_error = bus.ping(args.id)
        print(f"Ping al ID {args.id}: responde (error {ping_error})")
        error, memory = bus.read(args.id, 0, feetech_bus.MEMORY_BYTES)
        variant, little = feetech_bus.detect_variant(memory)
        values = {name: feetech_bus.value(memory, name, little) for name in feetech_bus.REGISTERS}
        print(f"Variante: {variant}")
        for name, v in values.items():
            print(f"  {name}: {v}")

        positions = []
        if args.watch > 0:
            print(f"Gira la bocina a mano durante {args.watch:g} s...")
            start = time.monotonic()
            while time.monotonic() - start < args.watch:
                _, raw = bus.read(args.id, feetech_bus.REGISTERS["present_position"][0], 2)
                positions.append(feetech_bus.decode("present_position", raw, little))
                time.sleep(0.05)
            print(f"  posición mínima {min(positions)} y máxima {max(positions)}")
    finally:
        bus.close()

    voltage = values["present_voltage_dv"] / 10
    temperature = values["present_temperature_c"]
    span = (max(positions) - min(positions)) if positions else None
    criteria = {
        "responds_to_ping": {"value": True, "passes": True},
        "variant": {"value": variant, "passes": variant != "desconocida"},
        "voltage_v": {"value": voltage, "supply_v": args.supply_volts, "tolerance_v": VOLTAGE_TOLERANCE_V,
                      "passes": abs(voltage - args.supply_volts) <= VOLTAGE_TOLERANCE_V},
        "temperature_c": {"value": temperature, "limits": list(ROOM_TEMPERATURE_C),
                          "passes": ROOM_TEMPERATURE_C[0] <= temperature <= ROOM_TEMPERATURE_C[1]},
        "hand_turn_counts": {"value": span, "limit": HAND_TURN_COUNTS, "torque_enable": values["torque_enable"],
                             "passes": span is not None and span > HAND_TURN_COUNTS and values["torque_enable"] == 0},
    }
    report = {
        "servo": "HD-1910", "label": args.label, "port": bus.port, "baud": args.baud, "id": args.id,
        "date": time.strftime("%Y-%m-%d %H:%M"), "memory_raw": list(memory), "status_error": error,
        "values": values, "criteria": criteria, "passes": all(c["passes"] for c in criteria.values()),
        "note": "Direcciones de la serie STS/SMS de Feetech, sin tabla publicada del HD-1910; solo lectura, nada escrito en el servo.",
    }
    out = danny_sim.ROOT / "results" / f"bench_servo_{args.label}_read.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("\nCriterio:")
    for name, c in criteria.items():
        print(f"  {'cumple' if c['passes'] else 'NO cumple'}  {name}: {c['value']}")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
