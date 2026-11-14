"""
Danny - Código común de los programas del banco del servo HD-1910 (5.1): comprobar el servo antes de activar el par, leer su estado,
parar por seguridad y apagar el par comprobándolo.

El banco es un péndulo: el servo con el eje horizontal y el brazo de carga (`cad/print/bench/bench_load_arm.stl`) colgando. El ángulo
del brazo se mide desde la vertical hacia abajo, en radianes, como en BAM (`q = 0` colgando), y es positivo hacia fuera de la mesa.
El soporte deja girar el brazo ±90°, pero **la mesa no**: el eje queda 14,2 mm por encima del tablero y, con el soporte en el borde,
a 14 mm por fuera; hacia la mesa, la barra de 12 mm toca el canto a unos 27° (17,7° con 3 mm de margen), y en el primer montaje real
(14 de noviembre) la carga ya tocó algo bajo el tablero a unos 7,5°. Por eso el brazo no pasa al lado de la mesa: va de
TABLE_SIDE_LIMIT_RAD (0°, vertical) a FREE_SIDE_LIMIT_RAD, y con el par activo para si sale de -TABLE_SIDE_STOP_RAD a
FREE_SIDE_STOP_RAD. El reductor no deja que el brazo caiga solo a la vertical: el cero y
el sentido se toman con `calibrate`, colocando el brazo a mano.

Las direcciones de memoria son las de la serie STS (`feetech_bus`), sin tabla publicada del HD-1910.
"""

import math
import signal
import statistics
import time

import feetech_bus

COUNTS_PER_TURN = 4096
RAD_PER_COUNT = 2 * math.pi / COUNTS_PER_TURN
STATE_ADDRESS, STATE_BYTES = 56, 15  # de present_position (56) a present_current (69-70)
STATE_NAMES = ("present_position", "present_speed", "present_load", "present_voltage_dv", "present_temperature_c", "moving",
               "present_current")
START_TEMPERATURE_C = 40
VOLTAGE_TOLERANCE_V = 0.3
VOLTAGE_RANGE_V = (5.0, 8.6)  # con la fuente a 7,4 V, los arranques bruscos bajan a 5,8 V un instante (14 de noviembre); el servo se protege a 4,0
GRAVITY = 9.81
TABLE_SIDE_LIMIT_RAD = 0.0
FREE_SIDE_LIMIT_RAD = math.radians(83)
TABLE_SIDE_STOP_RAD = math.radians(4)
FREE_SIDE_STOP_RAD = math.radians(89)
ZERO_SECONDS, ZERO_SPREAD_COUNTS = 1.0, 3
DIRECTION_MIN_RAD = math.radians(10)
RETURN_TOLERANCE_RAD = math.radians(5)


class SafetyStop(RuntimeError):
    pass


def stop_on_terminate():
    """Convierte SIGTERM y SIGHUP en SystemExit para que los `finally` apaguen el par. Sin esto, al parar el programa desde fuera
    (14 de noviembre, grabación interrumpida) el servo se quedó con el par activo. Contra SIGKILL no hay remedio."""
    def handler(signum, frame):
        raise SystemExit(f"Parado por la señal {signal.Signals(signum).name}")
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, handler)


def read_state(bus, servo_id, little):
    """Posición, velocidad, carga, tensión, temperatura, movimiento y corriente, en una sola lectura (0,5 ms a 1 Mbaud)."""
    _, raw = bus.read(servo_id, STATE_ADDRESS, STATE_BYTES)
    memory = bytes(STATE_ADDRESS) + raw
    return {name: feetech_bus.value(memory, name, little) for name in STATE_NAMES}


def prepare(bus, servo_id, supply_volts, start_temperature_c=START_TEMPERATURE_C):
    """Lee la memoria y comprueba, sin escribir, que se puede activar el par. Para con SystemExit si no."""
    _, memory = bus.read(servo_id, 0, feetech_bus.MEMORY_BYTES)
    variant, little = feetech_bus.detect_variant(memory)
    if variant != "STS":
        raise SystemExit(f"Parado sin escribir: variante {variant}; estos programas usan la tabla STS")
    values = {name: feetech_bus.value(memory, name, little) for name in feetech_bus.REGISTERS}
    voltage, temperature = values["present_voltage_dv"] / 10, values["present_temperature_c"]
    ready = {
        "voltage_v": {"value": voltage, "supply_v": supply_volts, "passes": round(abs(voltage - supply_volts), 3) <= VOLTAGE_TOLERANCE_V},
        "temperature_c": {"value": temperature, "limit": start_temperature_c, "passes": temperature < start_temperature_c},
        "torque_off": {"value": values["torque_enable"], "passes": values["torque_enable"] == 0},
    }
    print(f"Servo {servo_id}: modo {values['mode']}, posición {values['present_position']}, {voltage} V, {temperature} °C")
    check_in_range(values, values["present_position"], "la posición actual")
    if not all(c["passes"] for c in ready.values()):
        raise SystemExit(f"Parado sin activar el par: {ready}")
    return memory, little, values, ready


def check_in_range(values, position, what):
    """Para sin activar el par si `position` sale de los límites de ángulo del servo. En modo 4 la posición pasa de 4095 (varias
    vueltas), pero el objetivo no: el servo B, en 4443, saltó 14° hacia 4095 al activar el par (14 de noviembre)."""
    low, high = values["min_angle"], values["max_angle"]
    if not low <= position <= high:
        raise SystemExit(f"Parado sin activar el par: {what} ({position}) está fuera de {low}-{high}, y el servo iría hacia el límite. "
                         "Gira la bocina (sin el brazo) para que la posición con el brazo vertical quede dentro, con margen")


def check_limits(state, temperature_stop_c):
    """Para si la temperatura pasa del límite o la tensión sale del rango del banco."""
    if state["present_temperature_c"] > temperature_stop_c:
        raise SafetyStop(f"temperatura {state['present_temperature_c']} °C")
    if not VOLTAGE_RANGE_V[0] <= state["present_voltage_dv"] / 10 <= VOLTAGE_RANGE_V[1]:
        raise SafetyStop(f"tensión {state['present_voltage_dv'] / 10} V")


def torque_off(bus, servo_id, little):
    """Apaga el par y devuelve `torque_enable` releído (0 si quedó apagado)."""
    feetech_bus.write_register(bus, servo_id, "torque_enable", 0, little)
    return feetech_bus.read_register(bus, servo_id, "torque_enable", little)


def still_position(bus, servo_id, little):
    """Mediana de la posición durante ZERO_SECONDS; para con SystemExit si el brazo se mueve más de ZERO_SPREAD_COUNTS."""
    positions, began = [], time.monotonic()
    while time.monotonic() - began < ZERO_SECONDS:
        positions.append(read_state(bus, servo_id, little)["present_position"])
        time.sleep(0.01)
    if max(positions) - min(positions) > ZERO_SPREAD_COUNTS:
        raise SystemExit(f"Parado sin activar el par: el brazo se mueve ({max(positions) - min(positions)} cuentas); déjalo quieto")
    return round(statistics.median(positions))


def calibrate(bus, servo_id, little, ask=input):
    """Cero y sentido del brazo, con el par apagado y colocándolo a mano. Devuelve (cero en cuentas, +1 o -1 hacia fuera de la mesa)."""
    ask("Con el par apagado, pon el brazo vertical hacia abajo, suéltalo y pulsa Intro... ")
    zero = still_position(bus, servo_id, little)
    ask("Gíralo a mano unos 30° hacia FUERA de la mesa, sujétalo ahí y pulsa Intro... ")
    turned = (read_state(bus, servo_id, little)["present_position"] - zero) * RAD_PER_COUNT
    if abs(turned) < DIRECTION_MIN_RAD:
        raise SystemExit(f"Parado sin activar el par: el brazo solo giró {math.degrees(turned):.1f}°; hacen falta {math.degrees(DIRECTION_MIN_RAD):g}°")
    direction = 1 if turned > 0 else -1
    ask("Vuelve a ponerlo vertical hacia abajo, suéltalo y pulsa Intro... ")
    back = (still_position(bus, servo_id, little) - zero) * RAD_PER_COUNT
    if abs(back) > RETURN_TOLERANCE_RAD:
        raise SystemExit(f"Parado sin activar el par: quedó a {math.degrees(back):.1f}° de la vertical tomada; repite")
    print(f"Cero en {zero}; hacia fuera de la mesa, las cuentas {'suben' if direction > 0 else 'bajan'}")
    return zero, direction


def calibrate_or_use(bus, servo_id, little, direction):
    """Con `direction` conocido (+1 o -1, ver `--direction`), toma como cero la posición actual, con el brazo ya vertical y quieto;
    si no, `calibrate` lo pregunta todo colocando el brazo a mano."""
    if direction is None:
        return calibrate(bus, servo_id, little)
    zero = still_position(bus, servo_id, little)
    print(f"Cero en {zero} (brazo vertical ahora); hacia fuera de la mesa, las cuentas {'suben' if direction > 0 else 'bajan'}")
    return zero, direction


def counts(zero, direction, angle_rad):
    """Posición del servo, en cuentas, para un ángulo del brazo."""
    return zero + direction * round(angle_rad / RAD_PER_COUNT)


def angle_of(zero, direction, position):
    return direction * (position - zero) * RAD_PER_COUNT


def check_window(angle_rad):
    """Para si el brazo sale de -TABLE_SIDE_STOP_RAD a FREE_SIDE_STOP_RAD (solo tiene sentido con el par activo)."""
    if not -TABLE_SIDE_STOP_RAD <= angle_rad <= FREE_SIDE_STOP_RAD:
        raise SafetyStop(f"brazo a {math.degrees(angle_rad):.0f}°, fuera de -{math.degrees(TABLE_SIDE_STOP_RAD):.0f}° "
                         f"a {math.degrees(FREE_SIDE_STOP_RAD):.0f}°")


def gravity_torque_nm(load_kg, arm_kg, length_m, angle_rad):
    """Par de la gravedad en el eje, con la carga puntual a `length_m` y el brazo como una barra uniforme (el modelo de BAM)."""
    return (load_kg + arm_kg / 2) * GRAVITY * length_m * math.sin(angle_rad)
