"""
Danny - Estima la temperatura de los servos con el uso real del robot, a partir de las pruebas de calor del banco (5.1).

Modelo térmico de primer orden, el más sencillo que explica las pruebas: dT/dt = k·τ² - (T - T_amb)/τc. El calor sale de la corriente,
proporcional al par, por eso va con τ² (pérdidas en el bobinado); `k` se ajusta con las pruebas de calor de los servos A y B
(results/bench_servo_{A,B}_heat.json, par sostenido calculado con el ángulo medido). La constante τc no queda bien fijada con 2-3
minutos de calentamiento: el ajuste da TC_FIT y el enfriamiento visto de 50 a 37 °C en 280 s con el brazo colgando (14 de noviembre,
anotado en PROJECT_HANDOFF.md) da unos 380 s. Se dan los dos extremos. Con un par que varía, el calor medio va con el par eficaz (RMS)
al cuadrado, así que cada tarea se resume en el par eficaz de su articulación más cargada (results/joint_loads.json).

Para cada tarea escribe la temperatura a la que se estabilizaría, el tiempo que tardaría en llegar a LIMIT_C haciéndola sin parar
desde AMBIENT_C, y lo que sube en DEMO_S segundos sobre un pie. Escribe results/servo_heating.json.

Lo que no tiene en cuenta (estimación, sin comprobar): que dentro del robot, con las carcasas, el servo se enfría peor que en el
banco al aire; que el sensor de temperatura del servo no mide el bobinado; y que la fuente bajó a 6,4-6,6 V en las pruebas.

Uso: .venv/bin/python analysis/estimate_servo_heating.py
"""

import json
import math

import numpy as np
from scipy.optimize import least_squares

import danny_sim

AMBIENT_C = 24.0  # los servos en reposo marcaban 24-25 °C el 14 de noviembre
LIMIT_C = 70.0
COOLING_TC_S = 380.0  # de 50 a 37 °C en 280 s con 24 °C de ambiente: 280 / ln(26 / 13)
DEMO_S = 10.0  # el dueño: sobre un pie, unos 10 s como mucho, para mostrarlo
LOAD_KG, ARM_KG, LENGTH_M = 0.542, 0.006, 0.100
SERVOS = ("A", "B")


def held_torque(trace):
    return np.array([(LOAD_KG + ARM_KG / 2) * 9.81 * LENGTH_M * math.sin(math.radians(s["angle_deg"])) for s in trace])


def temperature(t, start_c, k, tc, torque):
    return AMBIENT_C + (start_c - AMBIENT_C) * np.exp(-t / tc) + k * torque**2 * tc * (1 - np.exp(-t / tc))


def main():
    runs = []
    for label in SERVOS:
        trace = json.loads((danny_sim.ROOT / "results" / f"bench_servo_{label}_heat.json").read_text())["trace"]
        t = np.array([s["t"] for s in trace])
        temp = np.array([s["temperature_c"] for s in trace], float)
        runs.append((label, t, temp, float(held_torque(trace).mean())))

    def residuals(p):
        return np.concatenate([temperature(t, temp[0], p[0], p[1], torque) - temp for _, t, temp, torque in runs])

    fit = least_squares(residuals, [1.0, 400.0], bounds=([0.0, 30.0], [10.0, 5000.0]))
    k, fit_tc = (float(v) for v in fit.x)
    error = float(np.sqrt(np.mean(fit.fun**2)))
    print(f"k = {k:.3f} °C/s por (N·m)², τc ajustada {fit_tc:.0f} s (error {error:.2f} °C); τc del enfriamiento {COOLING_TC_S:.0f} s")

    loads = json.loads((danny_sim.ROOT / "results" / "joint_loads.json").read_text())["tasks"]
    tasks = {}
    for task, data in loads.items():
        joint, values = max(data["joints"].items(), key=lambda item: item[1]["rms_torque_nm"])
        rms = values["rms_torque_nm"]
        rate = k * rms**2
        row = {"joint": joint, "rms_torque_nm": round(rms, 3), "heating_c_s": round(rate, 4)}
        for name, tc in (("cooling_tc", COOLING_TC_S), ("fit_tc", fit_tc)):
            steady = AMBIENT_C + rate * tc
            reach = None if steady <= LIMIT_C else -tc * math.log(1 - (LIMIT_C - AMBIENT_C) / (rate * tc))
            row[name] = {"steady_c": round(steady, 1), "minutes_to_limit": reach and round(reach / 60, 1)}
        tasks[task] = row
        print(f"  {task:20s} {joint:20s} {rms:.3f} N·m eficaz: estable a {row['cooling_tc']['steady_c']}-{row['fit_tc']['steady_c']} °C, "
              f"a {LIMIT_C:g} °C en {row['cooling_tc']['minutes_to_limit']} / {row['fit_tc']['minutes_to_limit']} min")
    peak = loads["single_support"]["peak_torque_nm"]
    demo_rise = k * peak**2 * DEMO_S
    print(f"  {DEMO_S:g} s sobre un pie a {peak:.3f} N·m: sube {demo_rise:.1f} °C")
    report = {
        "model": "dT/dt = k·τ² - (T - T_amb)/τc, ajustado a las pruebas de calor del banco; estimación, sin comprobar con el uso real",
        "ambient_c": AMBIENT_C, "limit_c": LIMIT_C, "k_c_per_s_nm2": round(k, 4), "fit_tc_s": round(fit_tc), "fit_error_c": round(error, 2),
        "cooling_tc_s": COOLING_TC_S, "runs": {label: {"held_torque_nm": round(torque, 4), "seconds": float(t[-1])}
                                               for label, t, _, torque in runs},
        "tasks": tasks,
        "single_support_demo": {"seconds": DEMO_S, "torque_nm": round(peak, 3), "rise_c": round(demo_rise, 1)},
    }
    out = danny_sim.ROOT / "results" / "servo_heating.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"Escrito {out}")


if __name__ == "__main__":
    main()
