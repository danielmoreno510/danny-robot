"""
Danny - Primer movimiento de un servo HD-1910 en el banco (5.1), con el par limitado y sin carga.

El modo 4 que trae el servo no está en la tabla de la serie STS: no se cambia, se observa con un paso pequeño. Antes de activar el
par se copia la posición actual al objetivo (de fábrica el objetivo es 0 y el servo saltaría) y se limitan el par y la velocidad. Con
el par activo, se sigue la posición cada pocos milisegundos y se apaga el par al momento si sale de la ventana, si la temperatura pasa
de TEMPERATURE_STOP_C o si la tensión sale de `servo_bench.VOLTAGE_RANGE_V`. Al terminar, pase lo que pase, se apaga el par y se comprueba.

Secuencia: sujetar 1 s en la posición inicial, ir a +paso, a -paso y volver al inicio. Escribe results/bench_servo_<label>_move.json
con la traza.

Criterio, fijado antes de la primera prueba (13 de noviembre de 2026):
1. tensión a ±0,3 V de la fuente y temperatura por debajo de 40 °C antes de activar el par;
2. al activar el par no salta: en el primer segundo no se aleja más de TOLERANCE_COUNTS de la posición inicial;
3. llega a cada objetivo a ±TOLERANCE_COUNTS en menos de REACH_TIMEOUT_S;
4. nunca sale de la ventana (paso más WINDOW_MARGIN_COUNTS) ni para por seguridad;
5. el par queda apagado al final (`torque_enable` leído = 0).
Con 4096 cuentas por vuelta, 57 cuentas son 5° y TOLERANCE_COUNTS = 10 son 0,9°.

Uso: .venv/bin/python analysis/move_servo.py --label A --id 2 --supply-volts 7.4
"""

import argparse
import json
import time

import danny_sim
import feetech_bus
from servo_bench import stop_on_terminate, check_in_range, COUNTS_PER_TURN, SafetyStop, check_limits, prepare, read_state, torque_off

TOLERANCE_COUNTS = 10
REACH_TIMEOUT_S = 2.0
SETTLE_S = 0.3
WINDOW_MARGIN_COUNTS = 100
TEMPERATURE_STOP_C = 60


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, help="nombre del servo en el banco (A, B o C), para el informe")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--supply-volts", type=float, required=True, help="tensión que marca la fuente, en V")
    parser.add_argument("--step", type=int, default=57, help="paso en cuentas (57 = 5°)")
    parser.add_argument("--torque-limit", type=int, default=300, help="límite de par, de 0 a 1000 (300 = 30 %%)")
    parser.add_argument("--speed", type=int, default=300, help="velocidad objetivo, en cuentas/s (300 = 26°/s)")
    parser.add_argument("--acceleration", type=int, default=20, help="aceleración, en unidades de 100 cuentas/s²")
    parser.add_argument("--baud", type=int, default=feetech_bus.DEFAULT_BAUD)
    parser.add_argument("--port", default=None)
    args = parser.parse_args()
    if not 0 < args.step <= 228 or not 0 < args.torque_limit <= 500:
        parser.error("en la primera prueba, paso de 1 a 228 cuentas (20°) y par como mucho 500")

    stop_on_terminate()
    bus = feetech_bus.Bus(args.port, args.baud)
    trace, stop_reason, phases = [], None, []
    clock = time.monotonic()
    torque_enable_after = None
    try:
        memory, little, values, ready = prepare(bus, args.id, args.supply_volts)
        start = values["present_position"]
        for goal in (start - args.step, start + args.step):
            check_in_range(values, goal, "un objetivo del paso")

        def sample(goal, phase):
            state = read_state(bus, args.id, little)
            state.update(t=round(time.monotonic() - clock, 4), goal=goal, phase=phase)
            trace.append(state)
            offset = state["present_position"] - start
            if abs(offset) > args.step + WINDOW_MARGIN_COUNTS:
                raise SafetyStop(f"fuera de la ventana: {offset} cuentas del inicio")
            check_limits(state, TEMPERATURE_STOP_C)
            return state

        def go(goal, phase):
            feetech_bus.write_register(bus, args.id, "goal_position", goal, little)
            began, inside_since = time.monotonic(), None
            while time.monotonic() - began < REACH_TIMEOUT_S:
                state = sample(goal, phase)
                if abs(state["present_position"] - goal) <= TOLERANCE_COUNTS:
                    inside_since = inside_since or time.monotonic()
                    if time.monotonic() - inside_since >= SETTLE_S:
                        break
                else:
                    inside_since = None
            reached = inside_since is not None
            phases.append({"phase": phase, "goal": goal, "reached": reached,
                           "time_to_reach_s": round(inside_since - began, 3) if reached else None,
                           "final_position": trace[-1]["present_position"]})
            print(f"  {phase}: objetivo {goal}, posición {trace[-1]['present_position']}, {'llega' if reached else 'NO llega'}")

        feetech_bus.write_register(bus, args.id, "goal_position", start, little)
        feetech_bus.write_register(bus, args.id, "torque_limit", args.torque_limit, little)
        feetech_bus.write_register(bus, args.id, "goal_speed", args.speed, little)
        feetech_bus.write_register(bus, args.id, "acceleration", args.acceleration, little)
        feetech_bus.write_register(bus, args.id, "torque_enable", 1, little)
        print(f"Par activado al {args.torque_limit / 10:g} %: sujetando 1 s")
        held = time.monotonic()
        while time.monotonic() - held < 1.0:
            sample(start, "sujetar")
        go(start + args.step, "más")
        go(start - args.step, "menos")
        go(start, "inicio")
    except SafetyStop as reason:
        stop_reason = str(reason)
        print(f"PARO DE SEGURIDAD: {stop_reason}")
    finally:
        try:
            torque_enable_after = torque_off(bus, args.id, little if "little" in locals() else True)
        finally:
            bus.close()
        print(f"Par apagado: torque_enable = {torque_enable_after}")

    hold = [abs(s["present_position"] - start) for s in trace if s["phase"] == "sujetar"]
    criteria = {
        "ready_before_torque": {"value": ready, "passes": all(c["passes"] for c in ready.values())},
        "no_jump_on_torque": {"value": max(hold) if hold else None, "limit": TOLERANCE_COUNTS,
                              "passes": bool(hold) and max(hold) <= TOLERANCE_COUNTS},
        "reaches_goals": {"value": phases, "timeout_s": REACH_TIMEOUT_S,
                          "passes": len(phases) == 3 and all(p["reached"] for p in phases)},
        "no_safety_stop": {"value": stop_reason, "passes": stop_reason is None},
        "torque_off_at_end": {"value": torque_enable_after, "passes": torque_enable_after == 0},
    }
    report = {
        "servo": "HD-1910", "label": args.label, "id": args.id, "date": time.strftime("%Y-%m-%d %H:%M"),
        "settings": {"step_counts": args.step, "step_deg": round(args.step * 360 / COUNTS_PER_TURN, 2), "torque_limit": args.torque_limit,
                     "speed_counts_s": args.speed, "acceleration": args.acceleration, "supply_v": args.supply_volts},
        "memory_before": list(memory), "mode": values["mode"], "start_position": start, "trace": trace, "criteria": criteria,
        "passes": all(c["passes"] for c in criteria.values()),
        "note": "Direcciones de la serie STS/SMS, sin tabla publicada del HD-1910. Sin carga.",
    }
    out = danny_sim.ROOT / "results" / f"bench_servo_{args.label}_move.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("Criterio:")
    for name, c in criteria.items():
        shown = c["value"] if name != "reaches_goals" else [p["reached"] for p in phases]
        print(f"  {'cumple' if c['passes'] else 'NO cumple'}  {name}: {shown}")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
