"""Comprueba que el modelo y la aleatorización de entrenamiento son los de la guía 4.2. Sale con código 1 si algo falla.

Modelo (`cad/export/danny/danny_training.xml`): masa de 1,30 kg, 13 `<pair>`, tobillo de -45° a 40° y sitio `imu`.
Contactos en mjlab: las cajas de colisión con condim 3 y prioridad 1, y los 13 pares presentes.
Aleatorización en 256 entornos: la masa total varía ±10 % y el centro de masas del robot entero unos ±10 mm por eje. Pasa de 10 mm
porque el desplazamiento se aplica en el marco de cada cuerpo, y el 24 % de la masa (muslos, tibias, brazos) tiene el marco girado:
el máximo medido es de unos 13 mm por eje.

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

from danny_training.evaluation import TASK
from danny_training.robot import DANNY_XML

NUM_ENVS = 256
NOMINAL_MASS_KG = 1.30
MASS_TOLERANCE_KG = 0.01
MASS_RANGE = 0.10
COM_RANGE_M = 0.01
COM_RANGE_ROTATED_FRAMES_M = 0.0135  # Cota por el marco girado de muslos y tibias.


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
  for axis, name in enumerate("xyz"):
    top = float(shift[:, axis].abs().max())
    check(0.8 * COM_RANGE_M <= top <= COM_RANGE_ROTATED_FRAMES_M, f"desplazamiento del centro de masas en {name}: máximo {top * 1000:.2f} mm (esperado de 8 a 13,5 mm)")

  env.close()
  if failures:
    print(f"\n{len(failures)} comprobaciones fallan")
    sys.exit(1)
  print("\nOK: modelo y aleatorización como en la guía 4.2.")


if __name__ == "__main__":
  main()
