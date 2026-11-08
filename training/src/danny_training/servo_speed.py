"""Velocidad de las articulaciones frente a la velocidad en vacío del servo.

El modelo del actuador (BAM, motor de corriente continua con fuerza contraelectromotriz) deja el par motor en cero a la velocidad en vacío
`vin * max_pwm / kt` y por encima frena con un amortiguamiento de `kt²/R`; no impide que algo externo (la gravedad, un golpe, la inercia del
cuerpo) mueva una articulación algo más deprisa. El límite `max_velocity` del modelo (el objetivo interno del servo) es de 100 rad/s en
`training/params/hd1910_m6.json`, es decir, no limita nada. Aquí se compara la velocidad máxima medida de cada articulación con la velocidad en
vacío del modelo y con la del servo real (hoja del fabricante, 11,9 rad/s a 7,4 V, escalada con la tensión).
"""

from __future__ import annotations

import torch

from danny_training.actuator import FrictionDRBamActuator

SERVO_NO_LOAD_RAD_S_AT_7P4V = 11.9  # PROJECT_HANDOFF / documentation/HANDBOOK.md (hoja del servo HD-1910-C001)
MAX_PWM = 0.97


def joint_speed_report(max_abs_speed: torch.Tensor, joint_names: list[str], vin: float | None, robot) -> dict:
  """Velocidad máxima por articulación (mediana, p95 y máximo entre robots) y fracción de robots que supera la velocidad en vacío.

  `max_abs_speed`: (robots, articulaciones), máximo en valor absoluto de la velocidad de cada articulación en la parte medida.
  """
  act = next(a for a in robot.actuators if isinstance(a, FrictionDRBamActuator))
  bam = act._bam_model
  kt, resistance = float(bam.kt.value), float(bam.R.value)
  vin_v = act.vin_tensor.reshape(-1).to(max_abs_speed.device).expand(max_abs_speed.shape[0])
  model_no_load = vin_v * MAX_PWM / kt
  servo_no_load = SERVO_NO_LOAD_RAD_S_AT_7P4V * vin_v / 7.4
  max_velocity = getattr(getattr(bam, "max_velocity", None), "value", None)
  rows = {}
  for j, name in enumerate(joint_names):
    x = max_abs_speed[:, j]
    row = {
      "max_speed_rad_s": {"median": round(float(x.median()), 2), "p95": round(float(torch.quantile(x, 0.95)), 2), "max": round(float(x.max()), 2)},
      "fraction_of_robots_over_servo_no_load": round(float((x > servo_no_load).float().mean()), 4),
    }
    row["fraction_of_robots_over_model_no_load"] = round(float((x > model_no_load).float().mean()), 4)
    rows[name] = row
  return {
    "servo_no_load_rad_s": {"at_vin_median": round(float(servo_no_load.median()), 2), "datasheet_at_7.4V": SERVO_NO_LOAD_RAD_S_AT_7P4V},
    "model_no_load_rad_s_at_vin_median": round(float(model_no_load.median()), 2),
    "model_kt": round(kt, 4),
    "model_resistance_ohm": round(resistance, 3),
    "model_electrical_damping_n_m_s_per_rad": round(kt * kt / resistance, 4),
    "model_max_velocity_rad_s": None if max_velocity is None else round(float(max_velocity), 2),
    "fastest_joints": sorted(((n, r["max_speed_rad_s"]["max"]) for n, r in rows.items()), key=lambda t: -t[1])[:6],
    "by_joint": rows,
  }
