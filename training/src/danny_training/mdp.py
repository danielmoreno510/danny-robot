"""Eventos y recompensas propios de Danny para mjlab."""

from __future__ import annotations

import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg

from danny_training.actuator import FrictionDRBamActuator

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


@requires_model_fields("dof_frictionloss", "dof_damping")
def randomize_bam_friction(
  env: ManagerBasedRlEnv,
  env_ids: torch.Tensor | None,
  scale_range: tuple[float, float],
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> None:
  """Aleatoriza por episodio la fricción de los servos, sin acumularla.

  Muestrea un escalar por entorno en `scale_range` y lo aplica al presupuesto de fricción del
  actuador BAM. Antes lo devuelve a 1,0 para no acumular entre episodios.
  """
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
  else:
    env_ids = env_ids.to(env.device, dtype=torch.int)

  asset: Entity = env.scene[asset_cfg.name]
  lo, hi = scale_range
  for actuator in asset.actuators:
    if isinstance(actuator, FrictionDRBamActuator):
      actuator.reset_friction_scale(env_ids)
      samples = torch.rand(len(env_ids), 1, device=env.device) * (hi - lo) + lo
      actuator.set_friction_scale(env_ids, samples)


def track_com_linear_velocity(
  env: ManagerBasedRlEnv,
  std: float,
  command_name: str,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Igual que `track_linear_velocity` de mjlab, pero con la velocidad del centro de masas de `base_link`.

  El origen de `base_link` está en el suelo, a unos 23 cm del centro de masas (sale así del CAD). Medida ahí,
  inclinarse sobre los tobillos casi no cuenta como moverse (guía 4.2). Hipótesis sin confirmar.
  """
  asset: Entity = env.scene[asset_cfg.name]
  command = env.command_manager.get_command(command_name)
  assert command is not None, f"Command '{command_name}' not found."
  actual = asset.data.root_com_lin_vel_b
  xy_error = torch.sum(torch.square(command[:, :2] - actual[:, :2]), dim=1)
  z_error = torch.square(actual[:, 2])
  return torch.exp(-(xy_error + z_error) / std**2)

def gait_contact_phase(env: ManagerBasedRlEnv, sensor_name: str, clock_name: str = "gait") -> torch.Tensor:
  """Premia el contacto de los pies que marca el reloj de marcha (`gait_clock`). Vale de 0 a 1.

  Con orden de movimiento: en la primera mitad del ciclo apoya el izquierdo y el derecho va en el aire, y en la segunda al
  revés. Cada pie que cumple suma 0,5. Con orden cero la fase está congelada y se premia que apoyen los dos pies.
  El primer pie del sensor es el izquierdo.
  """
  clock = env.command_manager.get_term(clock_name)
  found = env.scene[sensor_name].data.found.reshape(env.num_envs, -1)[:, :2] > 0
  left_stance = clock.phase < 0.5
  in_phase = 0.5 * ((found[:, 0] == left_stance).float() + (found[:, 1] == ~left_stance).float())
  both_down = found.float().mean(dim=1)
  return torch.where(clock.moving, in_phase, both_down)
