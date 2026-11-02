"""Eventos propios de Danny para mjlab."""

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
