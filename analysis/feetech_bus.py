"""
Danny - Bus serie de los servos Feetech (HD-1910) a través de la FE-URT-2, con pyserial.

Paquete: 0xFF 0xFF, ID, longitud, instrucción, parámetros y suma (complemento a uno de la suma desde el ID). Respuesta: 0xFF 0xFF, ID,
longitud, error, datos y suma. Es el mismo en las dos variantes de Feetech: SCS guarda los números de dos bytes en big-endian y STS
en little-endian. El HD-1910 no tiene tabla de memoria publicada (buscado el 11 de noviembre): las direcciones son las de la serie
STS/SMS, y la variante se decide por los límites de ángulo leídos (`detect_variant`).

Las lecturas no cambian nada. Para escribir, `write_register` codifica el valor con la variante; lo usan solo los programas que
configuran o mueven el servo.
"""

import glob
import time

import serial

PING, READ, WRITE = 0x01, 0x02, 0x03
DEFAULT_BAUD = 1_000_000  # de fábrica, según el handbook

# Direcciones de la serie STS/SMS (consulta: tablas públicas de Feetech para STS3215). Sin confirmar en el HD-1910.
REGISTERS = {
    "firmware_major": (0, 1), "firmware_minor": (1, 1), "model": (3, 2), "id": (5, 1), "baud_code": (6, 1),
    "return_delay": (7, 1), "response_level": (8, 1), "min_angle": (9, 2), "max_angle": (11, 2), "max_temperature_c": (13, 1),
    "max_voltage_dv": (14, 1), "min_voltage_dv": (15, 1), "max_torque": (16, 2), "unloading_condition": (19, 1),
    "led_alarm": (20, 1), "p_gain": (21, 1), "d_gain": (22, 1), "i_gain": (23, 1), "protection_current": (28, 2),
    "position_offset": (31, 2), "mode": (33, 1), "protective_torque": (34, 1), "protection_time": (35, 1), "overload_torque": (36, 1),
    "torque_enable": (40, 1), "acceleration": (41, 1), "goal_position": (42, 2), "goal_time": (44, 2), "goal_speed": (46, 2),
    "torque_limit": (48, 2), "lock": (55, 1),
    "present_position": (56, 2), "present_speed": (58, 2), "present_load": (60, 2), "present_voltage_dv": (62, 1),
    "present_temperature_c": (63, 1), "status": (65, 1), "moving": (66, 1), "present_current": (69, 2),
}
MEMORY_BYTES = 71  # de 0 a 70: EEPROM (0-39) y RAM (40-70)
READ_CHUNK = 32


class BusError(RuntimeError):
    pass


def find_port():
    """Puerto USB serie de la FE-URT-2 en macOS (chip CH340/CH343 o similar)."""
    ports = sorted(set(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/cu.usbmodem*")))
    if not ports:
        raise BusError("No hay ningún puerto USB serie: ¿está la FE-URT-2 conectada por USB?")
    if len(ports) > 1:
        raise BusError(f"Hay varios puertos USB serie, elige uno con --port: {ports}")
    return ports[0]


def checksum(body):
    return (~sum(body)) & 0xFF


class Bus:
    def __init__(self, port=None, baud=DEFAULT_BAUD, timeout=0.05):
        self.port = port or find_port()
        self.serial = serial.Serial(self.port, baud, timeout=timeout)

    def close(self):
        self.serial.close()

    def transact(self, servo_id, instruction, params=()):
        body = [servo_id, len(params) + 2, instruction, *params]
        self.serial.reset_input_buffer()
        self.serial.write(bytes([0xFF, 0xFF, *body, checksum(body)]))
        return self.receive(servo_id)

    def receive(self, servo_id):
        deadline = time.monotonic() + 0.1
        buffer = b""
        while time.monotonic() < deadline:
            # Lee lo que haya (al menos 1 byte): pedir un tamaño fijo esperaba al timeout en cada respuesta (55 ms por lectura,
            # medido el 14 de noviembre).
            buffer += self.serial.read(max(1, self.serial.in_waiting))
            start = buffer.find(b"\xff\xff")
            # La placa puede devolver el eco de lo enviado: vale el primer paquete con este ID y la suma correcta.
            while start >= 0 and len(buffer) >= start + 4:
                ident, length = buffer[start + 2], buffer[start + 3]
                end = start + 4 + length
                if len(buffer) < end:
                    break
                packet = buffer[start + 2:end]
                if ident == servo_id and checksum(packet[:-1]) == packet[-1] and length >= 2:
                    error, data = packet[2], bytes(packet[3:-1])
                    return error, data
                start = buffer.find(b"\xff\xff", start + 1)
        raise BusError(f"El servo {servo_id} no responde en {self.port}")

    def ping(self, servo_id):
        error, _ = self.transact(servo_id, PING)
        return error

    def read(self, servo_id, address, length):
        data = b""
        while len(data) < length:
            size = min(READ_CHUNK, length - len(data))
            error, chunk = self.transact(servo_id, READ, (address + len(data), size))
            if len(chunk) != size:
                raise BusError(f"Respuesta corta: {len(chunk)} de {size} bytes en la dirección {address + len(data)}")
            data += chunk
        return error, data

    def write(self, servo_id, address, values):
        error, _ = self.transact(servo_id, WRITE, (address, *values))
        return error


def value(memory, name, little_endian):
    """Valor de un registro en la copia de la memoria, con la variante dada."""
    address, size = REGISTERS[name]
    return decode(name, memory[address:address + size], little_endian)


# En STS, los registros con sentido llevan el signo en un bit (signo y magnitud, no complemento a dos): con 42549 leído como número sin
# signo, la posición era en realidad -9781 (2,4 vueltas hacia atrás al girar la bocina a mano, 11 de noviembre).
SIGN_BITS = {"present_position": 15, "goal_position": 15, "present_speed": 15, "present_load": 10, "position_offset": 11,
             "present_current": 15}


def decode(name, raw, little_endian):
    number = int.from_bytes(raw, "little" if little_endian else "big")
    bit = SIGN_BITS.get(name)
    if bit is not None and number & (1 << bit):
        return -(number & ((1 << bit) - 1))
    return number


def encode(name, number, little_endian):
    """Bytes de un registro para escribirlo; los de sentido van en signo y magnitud, como `decode`."""
    address, size = REGISTERS[name]
    bit = SIGN_BITS.get(name)
    if bit is not None:
        if abs(number) >= 1 << bit:
            raise ValueError(f"{name} = {number} no cabe en {bit} bits más el signo")
        raw = abs(number) | ((1 << bit) if number < 0 else 0)
    else:
        if not 0 <= number < 1 << (8 * size):
            raise ValueError(f"{name} = {number} no cabe en {size} bytes sin signo")
        raw = number
    return list(raw.to_bytes(size, "little" if little_endian else "big"))


def read_register(bus, servo_id, name, little_endian):
    address, size = REGISTERS[name]
    _, raw = bus.read(servo_id, address, size)
    return decode(name, raw, little_endian)


def write_register(bus, servo_id, name, number, little_endian):
    """Escribe un registro y devuelve el byte de error del servo."""
    return bus.write(servo_id, REGISTERS[name][0], encode(name, number, little_endian))


def detect_variant(memory):
    """STS (little-endian) o SCS (big-endian), por los límites de ángulo: de fábrica, el máximo es 4095 en STS y 1023 o 4095 en SCS."""
    little = value(memory, "max_angle", True)
    big = value(memory, "max_angle", False)
    if little in (1023, 4095) and little > value(memory, "min_angle", True):
        return "STS", True
    if big in (1023, 4095) and big > value(memory, "min_angle", False):
        return "SCS", False
    return "desconocida", True
