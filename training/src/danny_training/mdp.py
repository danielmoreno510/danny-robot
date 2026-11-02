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


def action_l2(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Penaliza la magnitud de la acción cruda de la política (suma de cuadrados por entorno).

  Limita las acciones grandes que tumbaban a Danny: sin esto, la acción media subía de 0,05 a 0,63 en 1 000 iteraciones.
  """
  return torch.sum(torch.square(env.action_manager.action), dim=1)


def feet_air_time_at_landing(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  threshold_min: float,
  threshold_max: float,
  command_name: str,
  command_threshold: float,
) -> torch.Tensor:
  """Premia el tiempo en el aire de cada pie, pero lo paga al aterrizar.

  Paga lo mismo que `feet_air_time` de mjlab en un paso completo: un punto por paso de control entre `threshold_min` y
  `threshold_max`. La de mjlab lo paga mientras el pie está en el aire, y con la penalización de aterrizaje de la guía
  Danny aprendió a levantar un pie y no volver a apoyarlo. Aquí un pie que no aterriza no cobra.
  """
  sensor = env.scene[sensor_name]
  first_contact = sensor.compute_first_contact(dt=env.step_dt)
  paid = (sensor.data.last_air_time.clamp(max=threshold_max) - threshold_min).clamp(min=0.0) / env.step_dt
  reward = torch.sum(paid * first_contact.float(), dim=1)
  command = env.command_manager.get_command(command_name)
  total_command = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
  return reward * (total_command > command_threshold).float()

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
