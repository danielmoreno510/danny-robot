"""
Danny - Graba en el banco (5.1) las trayectorias de identificación de BAM con un HD-1910 y una carga rígida en el brazo.

Método de BAM (Rhoban, Apache 2.0, https://github.com/Rhoban/bam): un péndulo con masa y longitud conocidas sigue cuatro trayectorias de
6 s con varias ganancias P del firmware (en el HD-1910, en modo 4, P no cambia nada: se graba con 32 y se varía la carga), y se ajusta un modelo de rozamiento que reproduce las posiciones medidas. De ahí sale también
la curva par-velocidad (`kt`, `R` y la tensión). Las trayectorias son las de `bam/trajectory.py`, pero dentro de lo que deja el banco:
de 0° (vertical) a 83° hacia el lado libre, ver `servo_bench`. Las de un solo lado (subir y soltar, subir y bajar) van hasta 83°;
las oscilantes (sin(t²) y sin·sin) van de 10° a 83°, porque con poca ganancia se pasan de la vertical. Como en el registro de BAM para
Feetech, la ganancia D se pone a 0 mientras se graba.

Al empezar, con el par apagado, `servo_bench.calibrate` (o, con `--direction`, `calibrate_or_use`, que toma el cero en la posición actual) pide colocar el brazo a mano: vertical (el cero, `q = 0` en BAM) y girado
hacia fuera de la mesa (el sentido). Antes de cada trayectoria se espera a que el brazo esté quieto, se activa el par en la posición
actual (sin salto) y se lleva al punto de partida a 1 rad/s. Al soltar el brazo (par apagado), si baja de CATCH_ANGLE_RAD (3°) camino de la mesa, se
reactiva el par para sujetarlo ahí: sin rozamiento volvería a 83° por el otro lado y daría en lo que hay bajo la mesa. El registro guarda el
par y el objetivo de cada muestra, así que BAM simula también ese tramo. Si la
temperatura llega a COOL_ABOVE_C, se espera con el par apagado hasta COOL_TO_C. P y D se escriben con la EEPROM bloqueada, como
valores de trabajo, se releen y se devuelven a los de fábrica al final; si el servo no los acepta, para.

Paros de seguridad (apagan el par): con el par activo, el brazo sale de los límites del banco más 6°
(de -4° a 89°, `servo_bench.check_window`); la temperatura pasa de TEMPERATURE_STOP_C o la tensión sale de `servo_bench.VOLTAGE_RANGE_V`.

Escribe un JSON por trayectoria y ganancia en results/bench_bam/<label>/raw/ con el formato de BAM (`mass`, `arm_mass`, `length`,
`kp`, `vin`, `motor`, `trajectory` y `entries`), y el resumen con el criterio en results/bench_servo_<label>_record_<gramos>g.json.

Criterio de cada grabación, fijado antes de la primera prueba real y aprobado por el dueño el 14 de noviembre de 2026:
1. listo antes del par: tensión a ±0,3 V de la fuente, temperatura por debajo de 40 °C y par apagado;
2. cero y sentido tomados: brazo quieto al tomar el cero (`servo_bench.ZERO_SPREAD_COUNTS`), giro de 10° o más para el sentido y
   vuelta a 5° o menos del cero (si no, para sin activar el par);
3. el servo acepta las ganancias (P y D releídas iguales a las escritas) y quedan como estaban al final;
4. sin paro de seguridad;
5. muestreo apto para BAM, que lo remuestrea a 5 ms: en cada registro, mediana del periodo de como mucho 3 ms y ningún hueco de más
   de 20 ms;
6. el par queda apagado al final.

Uso (masa de la carga sin el brazo, en kg; la longitud es la del eje al centro de la carga, en m):
.venv/bin/python analysis/record_servo_trajectories.py --label A --id 2 --supply-volts 7.4 --mass 0.100 --length 0.100
"""

import argparse
import json
import math
import statistics
import time

import danny_sim
import feetech_bus
from servo_bench import (stop_on_terminate, FREE_SIDE_STOP_RAD, TABLE_SIDE_STOP_RAD, check_in_range, FREE_SIDE_LIMIT_RAD, TABLE_SIDE_LIMIT_RAD, SafetyStop, angle_of, calibrate_or_use, check_limits, check_window,
                         counts, gravity_torque_nm, prepare, read_state, torque_off)

TEMPERATURE_STOP_C = 65
COOL_ABOVE_C, COOL_TO_C = 50, 45
SAMPLE_PERIOD_S = 0.002
RETURN_SPEED_RAD_S = 1.0
HOLD_S = 1.0
CATCH_ANGLE_RAD = math.radians(3)  # se sujeta al bajar de 3° tras soltar, antes de pasar la vertical
STILL_SPEED_RAD_S, STILL_SECONDS, STILL_TIMEOUT_S = 0.05, 0.5, 10.0
MAX_MEDIAN_PERIOD_S, MAX_GAP_S = 0.003, 0.020
SPEED_HALF_WINDOW = 3  # muestras a cada lado para la velocidad por diferencias (unos ±6 ms)
DEFAULT_ARM_KG = 0.006  # brazo impreso pesado el 12 de noviembre (results/printed_masses.json)


def hermite(keyframes, t):
    """Interpolación cúbica por tramos con (tiempo, valor, derivada) en cada punto, como `cubic_interpolate` de BAM."""
    if t <= keyframes[0][0]:
        return keyframes[0][1]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1]
    for (t0, x0, v0), (t1, x1, v1) in zip(keyframes, keyframes[1:]):
        if t0 <= t <= t1:
            h, s = t1 - t0, (t - t0) / (t1 - t0)
            return ((2 * s**3 - 3 * s**2 + 1) * x0 + (s**3 - 2 * s**2 + s) * h * v0 + (-2 * s**3 + 3 * s**2) * x1
                    + (s**3 - s**2) * h * v1)


SIN_SIN_PEAK = 1.7592  # máximo de |sin(t)·π/2 + 0,5·sin(5t)·sin(2t)| en 6 s
TOP = FREE_SIDE_LIMIT_RAD
# Las oscilantes no bajan de OSCILLATION_FLOOR_RAD: con P = 4, sin(t²) hasta 0° se pasó 4° hacia la mesa y paró (14 de noviembre).
OSCILLATION_FLOOR_RAD = TABLE_SIDE_LIMIT_RAD + math.radians(10)
CENTER = (FREE_SIDE_LIMIT_RAD + OSCILLATION_FLOOR_RAD) / 2  # 46,5°
SWING = (FREE_SIDE_LIMIT_RAD - OSCILLATION_FLOOR_RAD) / 2  # 36,5°
TRAJECTORIES = {
    # nombre: función t -> (ángulo en rad, par activo); 6 s cada una
    "lift_and_drop": lambda t: (hermite([[0, 0, 0], [2, TOP, 0]], t), t < 2.0),
    "sin_time_square": lambda t: (CENTER + SWING * math.sin(t**2), True),
    "up_and_down": lambda t: (hermite([[0, 0, 0], [3, TOP, 0], [6, 0.8 * TOP, 0]], t), True),
    "sin_sin": lambda t: (CENTER + SWING * (math.sin(t) * math.pi / 2 + math.sin(5 * t) * 0.5 * math.sin(2 * t)) / SIN_SIN_PEAK, True),
}
DURATION_S = 6.0


class Bench:
    def __init__(self, bus, servo_id, little, zero, direction):
        self.bus, self.id, self.little, self.zero, self.direction = bus, servo_id, little, zero, direction
        self.torque = False

    def angle(self, state):
        return angle_of(self.zero, self.direction, state["present_position"])

    def goal(self, angle):
        feetech_bus.write_register(self.bus, self.id, "goal_position", counts(self.zero, self.direction, angle), self.little)

    def set_torque(self, on, angle=None):
        if on and angle is not None:
            self.goal(angle)
        feetech_bus.write_register(self.bus, self.id, "torque_enable", int(on), self.little)
        self.torque = on

    def state(self):
        state = read_state(self.bus, self.id, self.little)
        check_limits(state, TEMPERATURE_STOP_C)
        angle = self.angle(state)
        if self.torque:
            check_window(angle)
        return state, angle

    def wait_still(self):
        """Espera con el estado actual del par a que el brazo esté quieto STILL_SECONDS. Devuelve el ángulo."""
        began = time.monotonic()
        last_angle, last_t, still_since = None, None, None
        while time.monotonic() - began < STILL_TIMEOUT_S:
            _, angle = self.state()
            now = time.monotonic()
            if last_angle is not None and now - last_t >= 0.05:
                speed = abs(angle - last_angle) / (now - last_t)
                still_since = (still_since or now) if speed < STILL_SPEED_RAD_S else None
                if still_since and now - still_since >= STILL_SECONDS:
                    return angle
                last_angle, last_t = angle, now
            elif last_angle is None:
                last_angle, last_t = angle, now
            time.sleep(0.01)
        raise SafetyStop(f"el brazo no se queda quieto en {STILL_TIMEOUT_S:g} s")

    def ramp_to(self, target):
        """Con el par activo, lleva el objetivo desde la posición actual hasta `target` a RETURN_SPEED_RAD_S."""
        _, angle = self.state()
        steps = max(1, math.ceil(abs(target - angle) / (RETURN_SPEED_RAD_S * 0.01)))
        for i in range(1, steps + 1):
            self.goal(angle + (target - angle) * i / steps)
            self.state()
            time.sleep(0.01)

    def cool_down(self):
        state, _ = self.state()
        if state["present_temperature_c"] < COOL_ABOVE_C:
            return
        print(f"  {state['present_temperature_c']} °C: espero con el par apagado hasta {COOL_TO_C} °C")
        self.set_torque(False)
        while state["present_temperature_c"] > COOL_TO_C:
            time.sleep(5)
            state, _ = self.state()

    def run(self, trajectory):
        """Una trayectoria de BAM desde el reposo. Devuelve las entradas con el formato de BAM."""
        self.cool_down()
        self.set_torque(False)
        angle = self.wait_still()
        self.set_torque(True, angle)
        self.ramp_to(trajectory(0)[0])
        held = time.monotonic()
        while time.monotonic() - held < HOLD_S:
            self.state()
            time.sleep(0.01)
        entries, clock, caught = [], time.monotonic(), False
        while (t := time.monotonic() - clock) < DURATION_S:
            goal, enable = (CATCH_ANGLE_RAD, True) if caught else trajectory(t)
            if enable != self.torque:
                self.set_torque(enable, goal if enable else None)
            if enable:
                self.goal(goal)
            t0 = time.monotonic() - clock
            state, angle = self.state()
            t1 = time.monotonic() - clock
            caught = caught or (not enable and angle < CATCH_ANGLE_RAD)
            entries.append({
                "timestamp": round((t0 + t1) / 2, 5), "position": round(angle, 5), "goal_position": round(goal, 5),
                "torque_enable": enable, "input_volts": state["present_voltage_dv"] / 10, "temp": state["present_temperature_c"],
                "load": state["present_load"], "current": state["present_current"], "present_speed_raw": state["present_speed"],
            })
            wait = SAMPLE_PERIOD_S - (time.monotonic() - clock - t)
            if wait > 0:
                time.sleep(wait)
        if self.torque:
            self.ramp_to(0.0)
        self.set_torque(False)
        add_speed(entries)
        return entries


def add_speed(entries):
    """Velocidad por diferencias centradas de la posición (la unidad de `present_speed` del HD-1910 no está confirmada)."""
    n = len(entries)
    for i, entry in enumerate(entries):
        a, b = entries[max(0, i - SPEED_HALF_WINDOW)], entries[min(n - 1, i + SPEED_HALF_WINDOW)]
        dt = b["timestamp"] - a["timestamp"]
        entry["speed"] = round((b["position"] - a["position"]) / dt, 4) if dt > 0 else 0.0


def sampling(entries):
    periods = [b["timestamp"] - a["timestamp"] for a, b in zip(entries, entries[1:])]
    return {"samples": len(entries), "median_period_s": round(statistics.median(periods), 5), "max_gap_s": round(max(periods), 5)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, help="nombre del servo en el banco (A, B o C)")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--supply-volts", type=float, required=True, help="tensión que marca la fuente, en V")
    parser.add_argument("--mass", type=float, required=True, help="masa de la carga, sin el brazo, en kg (pesada)")
    parser.add_argument("--length", type=float, default=0.100, help="del eje al centro de masas de la carga, en m (medido)")
    parser.add_argument("--arm-mass", type=float, default=DEFAULT_ARM_KG, help="masa del brazo impreso, en kg")
    parser.add_argument("--kp", default="32", help="ganancias P del firmware, separadas por comas (32 de fábrica; en modo 4 no "
                        "cambian nada, medido el 14 de noviembre)")
    parser.add_argument("--trajectories", default=",".join(TRAJECTORIES), help="trayectorias, separadas por comas")
    parser.add_argument("--direction", type=int, choices=(1, -1), default=None,
                        help="+1 si al subir las cuentas el brazo va hacia fuera de la mesa, -1 si va hacia ella; con él, el cero es la "
                        "posición actual (brazo vertical) y no se pregunta nada. Se ve con move_servo.py: su primer paso sube las cuentas")
    parser.add_argument("--baud", type=int, default=feetech_bus.DEFAULT_BAUD)
    parser.add_argument("--port", default=None)
    args = parser.parse_args()
    kps = [int(k) for k in args.kp.split(",")]
    names = args.trajectories.split(",")
    if not 0 < args.mass <= 0.4 or not 0.02 <= args.length <= 0.20:
        parser.error("carga de 0 a 0,4 kg y longitud de 0,02 a 0,20 m (el agujero del brazo está a 0,100 m; la carga puede colgar más abajo)")
    if any(not 1 <= k <= 64 for k in kps) or any(n not in TRAJECTORIES for n in names):
        parser.error(f"kp de 1 a 64; trayectorias: {', '.join(TRAJECTORIES)}")
    peak = gravity_torque_nm(args.mass, args.arm_mass, args.length, TOP)
    print(f"Par de la gravedad a {math.degrees(TOP):.0f}°: {peak:.3f} N·m")

    grams = round(args.mass * 1000)
    raw_dir = danny_sim.ROOT / "results" / "bench_bam" / args.label / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    stop_on_terminate()
    bus = feetech_bus.Bus(args.port, args.baud)
    stop_reason, logs, gains, calibration, torque_after, restored = None, [], [], None, None, None
    try:
        memory, little, values, ready = prepare(bus, args.id, args.supply_volts)
        factory_gains = {"p_gain": values["p_gain"], "d_gain": values["d_gain"]}
        zero, direction = calibrate_or_use(bus, args.id, little, args.direction)
        for limit in (TABLE_SIDE_STOP_RAD, FREE_SIDE_STOP_RAD):
            check_in_range(values, counts(zero, direction, limit if limit == FREE_SIDE_STOP_RAD else -limit), "un extremo del recorrido")
        calibration = {"zero_counts": zero, "direction": direction}
        vin = values["present_voltage_dv"] / 10
        bench = Bench(bus, args.id, little, zero, direction)
        for name, number in (("torque_limit", 1000), ("goal_speed", 0), ("acceleration", 0)):
            feetech_bus.write_register(bus, args.id, name, number, little)
        try:
            for kp in kps:
                written = {"p_gain": kp, "d_gain": 0}
                for name, number in written.items():
                    feetech_bus.write_register(bus, args.id, name, number, little)
                read = {name: feetech_bus.read_register(bus, args.id, name, little) for name in written}
                gains.append({"written": written, "read": read})
                if read != written:
                    raise SafetyStop(f"el servo no acepta las ganancias: escritas {written}, leídas {read}")
                for name in names:
                    print(f"kp {kp}, {name}")
                    entries = bench.run(TRAJECTORIES[name])
                    log = {"mass": args.mass, "arm_mass": args.arm_mass, "length": args.length, "kp": kp, "vin": vin,
                           "motor": "hd1910", "trajectory": name, "entries": entries}
                    path = raw_dir / f"{grams}g_kp{kp}_{name}.json"
                    path.write_text(json.dumps(log, separators=(",", ":")) + "\n")
                    logs.append({"file": str(path.relative_to(danny_sim.ROOT)), "kp": kp, "trajectory": name, **sampling(entries)})
                    print(f"  {logs[-1]['samples']} muestras, periodo mediano {1000 * logs[-1]['median_period_s']:.2f} ms, "
                          f"hueco máximo {1000 * logs[-1]['max_gap_s']:.1f} ms")
        finally:
            for name, number in factory_gains.items():
                feetech_bus.write_register(bus, args.id, name, number, little)
            restored = {name: feetech_bus.read_register(bus, args.id, name, little) for name in factory_gains}
    except SafetyStop as reason:
        stop_reason = str(reason)
        print(f"PARO DE SEGURIDAD: {stop_reason}")
    finally:
        try:
            torque_after = torque_off(bus, args.id, little if "little" in locals() else True)
        finally:
            bus.close()
        print(f"Par apagado: torque_enable = {torque_after}")

    criteria = {
        "ready_before_torque": {"value": ready, "passes": all(c["passes"] for c in ready.values())},
        "calibrated": {"value": calibration, "passes": calibration is not None},
        "gains_accepted_and_restored": {"value": {"each_kp": gains, "restored": restored, "factory": factory_gains},
                                        "passes": bool(gains) and all(g["read"] == g["written"] for g in gains)
                                        and restored == factory_gains},
        "no_safety_stop": {"value": stop_reason, "passes": stop_reason is None},
        "sampling_fit_for_bam": {"value": logs, "limits_s": [MAX_MEDIAN_PERIOD_S, MAX_GAP_S],
                                 "passes": len(logs) == len(kps) * len(names) and all(
                                     log["median_period_s"] <= MAX_MEDIAN_PERIOD_S and log["max_gap_s"] <= MAX_GAP_S for log in logs)},
        "torque_off_at_end": {"value": torque_after, "passes": torque_after == 0},
    }
    report = {
        "servo": "HD-1910", "label": args.label, "id": args.id, "date": time.strftime("%Y-%m-%d %H:%M"),
        "settings": {"mass_kg": args.mass, "arm_mass_kg": args.arm_mass, "length_m": args.length, "kp": kps, "trajectories": names,
                     "supply_v": args.supply_volts,
                     "range_deg": [round(-math.degrees(TABLE_SIDE_LIMIT_RAD), 1), round(math.degrees(TOP), 1)],
                     "gravity_torque_at_max_angle_nm": round(peak, 4)},
        "criteria": criteria, "passes": all(c["passes"] for c in criteria.values()),
        "note": "Direcciones de la serie STS. Trayectorias de BAM (Apache 2.0) llevadas al rango del banco, de 0° a 83°; D = 0 al grabar.",
    }
    out = danny_sim.ROOT / "results" / f"bench_servo_{args.label}_record_{grams}g.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("Criterio:")
    for name, c in criteria.items():
        shown = c["passes"] if name in ("sampling_fit_for_bam", "gains_accepted_and_restored") else c["value"]
        print(f"  {'cumple' if c['passes'] else 'NO cumple'}  {name}: {shown}")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
