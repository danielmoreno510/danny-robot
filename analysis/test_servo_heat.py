"""
Danny - Prueba de calor del HD-1910 en el banco (5.1): sujetar 5 minutos el par de apoyo a un pie con el brazo de carga horizontal.

El par que se pide es el pico de `single_support` en results/joint_loads.json (cadera en roll). El servo sostiene el brazo a
`--angle-deg` de la vertical con la carga colgada en el agujero de 100 mm; para una carga quieta basta una cuerda (en esta prueba no
hay movimiento). El programa calcula el par de la gravedad con la masa pesada y para sin activar el par si no llega al pedido.

Antes de activar el par, sin carga, `servo_bench.calibrate` (o, con `--direction`, `calibrate_or_use`, que toma el cero en la posición actual) pide colocar el brazo a mano: vertical (el cero) y girado hacia fuera de
la mesa (el sentido). Se activa el par en el cero, se sube el brazo a 0,5 rad/s hacia el lado libre hasta `--angle-deg` sin la carga, y **entonces el programa pide colgarla** y espera a Intro: así el servo no arranca
con todo el par. Con `--load-attached`, la carga ya va puesta (colgando libre de un tornillo en el agujero) y sube con ella sin preguntar. Se registran temperatura, corriente, tensión y posición cada segundo durante HOLD_S. Al terminar, o si para, se baja
el brazo despacio con la carga puesta y se apaga el par.

Criterio, fijado antes de la primera prueba real (el de 5.1 en el handbook), aprobado por el dueño el 14 de noviembre de 2026:
1. listo antes del par (tensión, temperatura por debajo de 40 °C, par apagado), y el par calculado de la carga llega al pedido;
2. sostiene HOLD_S (5 min) sin pasar de PASS_TEMPERATURE_C (70 °C), sin paro de seguridad;
3. el par sostenido de verdad llega al pedido: el de la gravedad con el ángulo medido (mediana de la sujeción), no con el objetivo;
4. no cede con el tiempo: la caída de los últimos 10 s no supera la de los primeros 10 s en más de SAG_DRIFT_LIMIT_DEG;
5. el par queda apagado al final.
Que el brazo quede algo por debajo del objetivo es normal: el firmware es un control P. Con P = 32 de fábrica y el modelo BAM del
STS3215 (`hd1910_bam_parameters.json`, sin confirmar en el HD-1910), con 0,51 N·m cede unos 6° (estimado); por eso no se fija un límite
de caída sino de deriva, y la carga recomendada es de unos 560 g para que el punto 3 se cumpla con esa caída.
Paros de seguridad: temperatura por encima de PASS_TEMPERATURE_C, tensión fuera de `servo_bench.VOLTAGE_RANGE_V` o el brazo a más de
SAG_STOP_DEG del objetivo.

Si para por temperatura, igualmente baja el brazo con el par antes de apagarlo (unos 3 s), para que la carga no caiga hacia la mesa.

Escribe results/bench_servo_<label>_heat.json con la traza.

Uso: .venv/bin/python analysis/test_servo_heat.py --label A --id 2 --supply-volts 7.4 --mass 0.560
"""

import argparse
import json
import math
import statistics
import time

import danny_sim
import feetech_bus
from servo_bench import (stop_on_terminate, FREE_SIDE_STOP_RAD, TABLE_SIDE_STOP_RAD, check_in_range, FREE_SIDE_LIMIT_RAD, SafetyStop, angle_of, calibrate_or_use, check_limits, counts, gravity_torque_nm, prepare,
                         read_state, torque_off)

HOLD_S = 300
PASS_TEMPERATURE_C = 70
SAG_DRIFT_LIMIT_DEG = 3.0
SAG_STOP_DEG = 20.0
DRIFT_WINDOW_S = 10.0
LIFT_SPEED_RAD_S = 0.5
LOG_PERIOD_S = 1.0
DEFAULT_ARM_KG = 0.006  # brazo impreso pesado el 12 de noviembre (results/printed_masses.json)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--supply-volts", type=float, required=True)
    parser.add_argument("--mass", type=float, required=True, help="masa de la carga con su cuerda, en kg (pesada)")
    parser.add_argument("--length", type=float, default=0.100, help="del eje al punto de cuelgue, en m")
    parser.add_argument("--arm-mass", type=float, default=DEFAULT_ARM_KG)
    parser.add_argument("--angle-deg", type=float, default=80.0, help="ángulo del brazo desde la vertical, hacia fuera de la mesa")
    parser.add_argument("--direction", type=int, choices=(1, -1), default=None,
                        help="+1 si al subir las cuentas el brazo va hacia fuera de la mesa, -1 si va hacia ella; con él, el cero es la "
                        "posición actual (brazo vertical) y no se pregunta nada. Se ve con move_servo.py: su primer paso sube las cuentas")
    parser.add_argument("--load-attached", action="store_true",
                        help="la carga ya cuelga del brazo (por ejemplo, girando sobre un tornillo): sube con ella y no pregunta")
    parser.add_argument("--baud", type=int, default=feetech_bus.DEFAULT_BAUD)
    parser.add_argument("--port", default=None)
    args = parser.parse_args()
    loads = json.loads((danny_sim.ROOT / "results" / "joint_loads.json").read_text())
    target = loads["tasks"]["single_support"]["peak_torque_nm"]
    angle = math.radians(args.angle_deg)
    torque = gravity_torque_nm(args.mass, args.arm_mass, args.length, angle)
    print(f"Par pedido {target:.3f} N·m (apoyo a un pie); con esta carga a {args.angle_deg:g}°: {torque:.3f} N·m")
    if not 0 < angle <= FREE_SIDE_LIMIT_RAD:
        parser.error(f"ángulo de 0 a {math.degrees(FREE_SIDE_LIMIT_RAD):.0f}° (servo_bench.FREE_SIDE_LIMIT_RAD)")

    stop_on_terminate()
    bus = feetech_bus.Bus(args.port, args.baud)
    trace, stop_reason, torque_after, max_sag = [], None, None, None
    ready, enough = None, torque >= target
    try:
        memory, little, values, ready = prepare(bus, args.id, args.supply_volts)
        if not enough:
            raise SystemExit(f"Parado sin activar el par: la carga da {torque:.3f} N·m y hacen falta {target:.3f}")
        zero, direction = calibrate_or_use(bus, args.id, little, args.direction)
        for limit in (TABLE_SIDE_STOP_RAD, FREE_SIDE_STOP_RAD):
            check_in_range(values, counts(zero, direction, limit if limit == FREE_SIDE_STOP_RAD else -limit), "un extremo del recorrido")

        def goal(rad):
            feetech_bus.write_register(bus, args.id, "goal_position", counts(zero, direction, rad), little)

        def sample(target_rad, phase, limit_deg=SAG_STOP_DEG, limits=True):
            state = read_state(bus, args.id, little)
            if limits:
                check_limits(state, PASS_TEMPERATURE_C)
            position = angle_of(zero, direction, state["present_position"])
            sag = math.degrees(abs(position - target_rad))
            if sag > limit_deg:
                raise SafetyStop(f"el brazo está a {sag:.1f}° del objetivo")
            return state, position, sag

        def move(start_rad, end_rad, phase, limit_deg=SAG_STOP_DEG, limits=True):
            steps = max(1, math.ceil(abs(end_rad - start_rad) / (LIFT_SPEED_RAD_S * 0.01)))
            for i in range(1, steps + 1):
                now = start_rad + (end_rad - start_rad) * i / steps
                goal(now)
                sample(now, phase, limit_deg, limits)
                time.sleep(0.01)

        for name, number in (("torque_limit", 1000), ("goal_speed", 0), ("acceleration", 0)):
            feetech_bus.write_register(bus, args.id, name, number, little)
        goal(0.0)
        feetech_bus.write_register(bus, args.id, "torque_enable", 1, little)
        try:
            # Desde que el par está activo, cualquier salida (paro, Ctrl-C en la espera) baja el brazo antes de apagar el par: el
            # 14 de noviembre, un Ctrl-C en la espera apagó el par a 80° y la carga bajó el brazo por su peso.
            move(0.0, angle, "subir")
            if not args.load_attached:
                input(f"Brazo a {args.angle_deg:g}°. Cuelga la carga de {args.mass * 1000:.0f} g en el agujero y pulsa Intro... ")
            clock = time.monotonic()
            while (t := time.monotonic() - clock) < HOLD_S:
                state, position, sag = sample(angle, "sujetar")
                trace.append({"t": round(t, 2), "angle_deg": round(math.degrees(position), 2), "sag_deg": round(sag, 2),
                              "temperature_c": state["present_temperature_c"], "current": state["present_current"],
                              "load": state["present_load"], "voltage_v": state["present_voltage_dv"] / 10})
                if len(trace) % 30 == 1:
                    print(f"  {t:5.0f} s: {state['present_temperature_c']} °C, {sag:.1f}° de caída, corriente {state['present_current']}")
                time.sleep(max(0.0, LOG_PERIOD_S - (time.monotonic() - clock - t)))
        finally:
            print("Bajando el brazo despacio")
            try:
                current = angle_of(zero, direction, read_state(bus, args.id, little)["present_position"])
                # Sin paro por temperatura al bajar (unos 3 s): soltar 0,5 N·m a 80° dejaría caer el brazo hacia la mesa.
                move(current, 0.0, "bajar", limit_deg=90, limits=False)
            except (SafetyStop, feetech_bus.BusError) as reason:
                print(f"No se pudo bajar con el par: {reason}")
    except SafetyStop as reason:
        stop_reason = str(reason)
        print(f"PARO DE SEGURIDAD: {stop_reason}")
    finally:
        try:
            torque_after = torque_off(bus, args.id, little if "little" in locals() else True)
        finally:
            bus.close()
        print(f"Par apagado: torque_enable = {torque_after}")

    held = trace[-1]["t"] if trace else 0
    temperatures = [s["temperature_c"] for s in trace]
    first = [s["sag_deg"] for s in trace if s["t"] <= DRIFT_WINDOW_S]
    last = [s["sag_deg"] for s in trace if s["t"] >= held - DRIFT_WINDOW_S]
    drift = round(statistics.median(last) - statistics.median(first), 2) if first and last else None
    held_angle = statistics.median(s["angle_deg"] for s in trace) if trace else None
    held_torque = (gravity_torque_nm(args.mass, args.arm_mass, args.length, math.radians(held_angle))
                   if held_angle is not None else None)
    criteria = {
        "ready_and_enough_load": {"value": {"ready": ready, "torque_nm": round(torque, 4), "target_nm": round(target, 4)},
                                  "passes": ready is not None and all(c["passes"] for c in ready.values()) and enough},
        "holds_without_overheating": {"value": {"held_s": held, "max_temperature_c": max(temperatures, default=None),
                                                "stop": stop_reason},
                                      "limit": [HOLD_S, PASS_TEMPERATURE_C],
                                      "passes": stop_reason is None and held >= HOLD_S - 2 * LOG_PERIOD_S
                                      and max(temperatures, default=999) <= PASS_TEMPERATURE_C},
        "held_torque_reaches_target": {"value": {"median_angle_deg": held_angle, "torque_nm": held_torque and round(held_torque, 4)},
                                       "limit": round(target, 4), "passes": held_torque is not None and held_torque >= target},
        "does_not_give_way": {"value": {"drift_deg": drift, "max_sag_deg": max((s["sag_deg"] for s in trace), default=None)},
                              "limit": SAG_DRIFT_LIMIT_DEG, "passes": drift is not None and drift <= SAG_DRIFT_LIMIT_DEG},
        "torque_off_at_end": {"value": torque_after, "passes": torque_after == 0},
    }
    report = {
        "servo": "HD-1910", "label": args.label, "id": args.id, "date": time.strftime("%Y-%m-%d %H:%M"),
        "settings": {"mass_kg": args.mass, "arm_mass_kg": args.arm_mass, "length_m": args.length, "angle_deg": args.angle_deg,
                     "supply_v": args.supply_volts, "hold_s": HOLD_S},
        "trace": trace, "criteria": criteria, "passes": all(c["passes"] for c in criteria.values()),
        "note": "Direcciones de la serie STS. Par de la gravedad calculado con la carga puntual y el brazo como barra uniforme.",
    }
    out = danny_sim.ROOT / "results" / f"bench_servo_{args.label}_heat.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("Criterio:")
    for name, c in criteria.items():
        print(f"  {'cumple' if c['passes'] else 'NO cumple'}  {name}: {c['value']}")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
