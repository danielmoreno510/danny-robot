"""Comprueba que el modelo y la aleatorización de entrenamiento son los de la guía 4.2. Sale con código 1 si algo falla.

Modelo (`cad/export/danny/danny_training.xml`): masa de 1,30 kg, 13 `<pair>`, tobillo de -45° a 40° y sitio `imu`.
Contactos en mjlab: las cajas de colisión con condim 3 y prioridad 1, y los 13 pares presentes.
Aleatorización de la fase 2 (etapa 3: armadura ±30 %, holgura de hasta 0,5°, empujones de ±0,2 m/s) y órdenes: el 20 % de entornos
quietos y ninguno forzado a avanzar.
Aleatorización en 256 entornos: la masa total varía ±10 % y el centro de masas del robot entero unos ±10 mm por eje. Pasa de 10 mm
porque el desplazamiento se aplica en el marco de cada cuerpo, y el 24 % de la masa (muslos, tibias, brazos) tiene el marco girado:
el máximo medido es de unos 13 mm por eje, bajo la cota exacta que calcula la comprobación.

Uso (desde training/):
    uv run python scripts/check_training_setup.py
"""

from __future__ import annotations

import math
import re
import sys

import mujoco
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

from danny_training.actuator import stall_torque_nm
from danny_training.env_cfgs import DR_STAGE, DR_STAGES, PUSH_OVERRIDE_M_S
from danny_training.mdp import torque_excess_penalty
from danny_training.evaluation import TASK
from danny_training.robot import DANNY_XML

NUM_ENVS = 256
NOMINAL_MASS_KG = 1.30
MASS_TOLERANCE_KG = 0.01
MASS_RANGE = 0.10
COM_RANGE_M = 0.01


def main() -> None:
  failures: list[str] = []

  def check(ok: bool, text: str) -> None:
    print(("OK    " if ok else "FALLA ") + text)
    if not ok:
      failures.append(text)

  xml = mujoco.MjModel.from_xml_path(str(DANNY_XML))
  mass = float(xml.body_mass.sum())
  check(abs(mass - NOMINAL_MASS_KG) <= MASS_TOLERANCE_KG, f"masa del MJCF {mass:.4f} kg (esperado {NOMINAL_MASS_KG})")
  check(xml.npair == 13, f"{xml.npair} pares de contacto en el MJCF (esperado 13)")
  low, high = (math.degrees(x) for x in xml.jnt_range[xml.joint("left_ankle_pitch").id])
  check(abs(low + 45.0) < 0.1 and abs(high - 40.0) < 0.1, f"tobillo de {low:.1f}° a {high:.1f}° (esperado de -45° a 40°)")
  check(mujoco.mj_name2id(xml, mujoco.mjtObj.mjOBJ_SITE, "imu") >= 0, "sitio imu en el MJCF")

  cfg = load_env_cfg(TASK)
  cfg.scene.num_envs = NUM_ENVS
  env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0" if torch.cuda.is_available() else "cpu")
  env.reset()
  model = env.sim.mj_model
  check(model.npair == 13, f"{model.npair} pares de contacto en el modelo de mjlab (esperado 13)")

  boxes = [i for i in range(model.ngeom) if re.fullmatch(r".*_box_\d+", model.geom(i).name or "")]
  condim = {int(model.geom_condim[i]) for i in boxes}
  priority = {int(model.geom_priority[i]) for i in boxes}
  check(len(boxes) > 0 and condim == {3} and priority == {1}, f"{len(boxes)} cajas de colisión: condim {sorted(condim)}, prioridad {sorted(priority)} (esperado {{3}} y {{1}})")

  m, d = env.sim.model, env.sim.data
  mass_e = m.body_mass
  ratio = mass_e.sum(dim=1) / env.sim.get_default_field("body_mass").sum()
  check(
    float(ratio.min()) >= 1 - MASS_RANGE - 1e-3 and float(ratio.max()) <= 1 + MASS_RANGE + 1e-3
    and float(ratio.min()) <= 1 - 0.8 * MASS_RANGE and float(ratio.max()) >= 1 + 0.8 * MASS_RANGE,
    f"masa total entre {float(ratio.min()):.3f} y {float(ratio.max()):.3f} veces la nominal (esperado casi de 0,90 a 1,10)",
  )

  def com(mass_b: torch.Tensor, ipos: torch.Tensor) -> torch.Tensor:
    world = d.xpos + torch.einsum("ebij,ebj->ebi", d.xmat.reshape(NUM_ENVS, -1, 3, 3), ipos)
    return (mass_b.unsqueeze(-1) * world).sum(dim=1) / mass_b.sum(dim=1, keepdim=True)

  default_mass = env.sim.get_default_field("body_mass").unsqueeze(0).expand_as(mass_e)
  default_ipos = env.sim.get_default_field("body_ipos").unsqueeze(0).expand_as(m.body_ipos)
  shift = com(mass_e, m.body_ipos) - com(default_mass, default_ipos)
  # Se resta el efecto de la masa para ver solo el desplazamiento del centro de masas.
  shift = shift - (com(mass_e, default_ipos) - com(default_mass, default_ipos))
  # Cota exacta por entorno y eje: cada cuerpo desplaza su centro ±10 mm en su marco, y el marco girado (también el del robot entero,
  # que sale con rumbo aleatorio) reparte ese desplazamiento entre los ejes del mundo.
  mass_share = default_mass / default_mass.sum(dim=1, keepdim=True)
  frame_spread = d.xmat.reshape(NUM_ENVS, -1, 3, 3).abs().sum(dim=3)  # (entornos, cuerpos, ejes del mundo)
  bound = COM_RANGE_M * (mass_share.unsqueeze(-1) * frame_spread).sum(dim=1)  # (entornos, ejes)
  for axis, name in enumerate("xyz"):
    top = float(shift[:, axis].abs().max())
    worst_excess = float((shift[:, axis].abs() - bound[:, axis]).max())
    check(
      top >= 0.8 * COM_RANGE_M and worst_excess <= 1e-6,
      f"desplazamiento del centro de masas en {name}: máximo {top * 1000:.2f} mm, siempre bajo la cota de cada entorno (exceso máximo {worst_excess * 1000:.3f} mm)",
    )

  stage = DR_STAGES[DR_STAGE]
  default_armature = env.sim.get_default_field("dof_armature")
  servo = default_armature > 0  # La articulación flotante de la base no tiene armadura.
  armature = m.dof_armature[:, servo] / default_armature[servo].unsqueeze(0)
  low, high = float(armature.min()), float(armature.max())
  check(
    low >= 1 - stage["armature"] - 1e-3 and high <= 1 + stage["armature"] + 1e-3 and low <= 1 - 0.8 * stage["armature"] and high >= 1 + 0.8 * stage["armature"],
    f"armadura de {low:.3f} a {high:.3f} veces la nominal (etapa {DR_STAGE}: ±{stage['armature']:.0%})",
  )
  half_width = env.scene["robot"].actuators[0].backlash_half_width
  top, limit = float(half_width.max()), math.radians(stage["backlash_deg"]) / 2
  check(half_width is not None and 0.6 * limit <= top <= limit + 1e-6, f"semiancho de holgura máximo {math.degrees(top):.3f}° (etapa {DR_STAGE}: hasta {stage['backlash_deg'] / 2:.3f}°)")
  push = env.event_manager.get_term_cfg("push_robot").params["velocity_range"]["x"]
  expected_push = PUSH_OVERRIDE_M_S if PUSH_OVERRIDE_M_S is not None else stage["push"]
  check(abs(push[1] - expected_push) < 1e-9, f"empujones de ±{push[1]} m/s (esperado ±{expected_push})")
  twist = env.command_manager.get_term_cfg("twist")
  check(twist.rel_standing_envs == 0.2 and twist.rel_forward_envs == 0.0, f"órdenes: {twist.rel_standing_envs:.0%} quietos y {twist.rel_forward_envs:.0%} forzados a avanzar")

  # Castigo de par: 0 bajo el 80 % del bloqueo de la tensión de cada entorno, y creciente por encima.
  vin = torch.tensor([[6.0], [7.4], [8.4]], device=env.device)
  stall = stall_torque_nm(vin)
  levels = [0.5, 0.8, 0.9, 1.0, 1.2]
  values = [torque_excess_penalty((level * stall).expand(-1, 2), vin, 0.8) for level in levels]
  check(
    bool((values[0] == 0).all() and (values[1] < 1e-9).all() and all(bool((b > a).all()) for a, b in zip(values[1:], values[2:]))),
    "castigo de par: 0 hasta el 80 % del bloqueo (a 6, 7,4 y 8,4 V) y creciente por encima",
  )
  term = env.reward_manager.get_term_cfg("servo_torque_excess")
  env.reset()
  for _ in range(100):
    env.step(torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device))
  standing = term.func(env, **term.params)
  check(float(standing.max()) == 0.0, f"castigo de par de pie, con acción cero: máximo {float(standing.max()):.6f} (esperado 0)")

  env.close()
  if failures:
    print(f"\n{len(failures)} comprobaciones fallan")
    sys.exit(1)
  print("\nOK: modelo y aleatorización como en la guía 4.2.")


if __name__ == "__main__":
  main()
