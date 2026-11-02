"""Eventos y recompensas propios de Danny para mjlab."""

from __future__ import annotations

import torch
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.reward_manager import RewardTermCfg
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

def feet_lateral_clearance(
  env: ManagerBasedRlEnv,
  min_clearance: float,
  sole_width: float,
  sole_length: float,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Penaliza que las suelas se acerquen de lado por debajo de `min_clearance`, antes de que choquen.

  Mide en el marco del cuerpo, con los sitios de la suela izquierda y la derecha (en ese orden): separación lateral entre
  centros menos `sole_width`, con signo, así que cruzar los pies cuenta como separación negativa. Solo cuenta si las suelas
  se solapan a lo largo del paso (a menos de `sole_length` en el eje de avance). Vale (hundimiento / min_clearance)², con el
  hundimiento limitado a 3 veces `min_clearance`. De pie hay 16 mm de separación.
  """
  asset: Entity = env.scene[asset_cfg.name]
  delta = asset.data.site_pos_w[:, asset_cfg.site_ids[0], :2] - asset.data.site_pos_w[:, asset_cfg.site_ids[1], :2]
  heading = asset.data.heading_w
  cos, sin = torch.cos(heading), torch.sin(heading)
  forward = cos * delta[:, 0] + sin * delta[:, 1]
  lateral = -sin * delta[:, 0] + cos * delta[:, 1]
  depth = ((min_clearance - (lateral - sole_width)) / min_clearance).clamp(min=0.0, max=3.0)
  return depth.square() * (forward.abs() < sole_length).float()

class link_pair_contacts:
  """Cuenta, por entorno, los contactos activos (penetración) de los `<pair>` del MJCF entre eslabones.

  Son los choques pie con pie, tibia con tibia, cadera con cadera y brazo con pierna que la política antigua atravesaba. El
  sensor de contactos de mjlab no los detecta con sub-árboles, así que se leen de los contactos de la simulación. Registra
  `Metrics/link_contact_fraction`, la fracción de entornos con algún choque.
  """

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    model = env.sim.mj_model
    geom1 = torch.as_tensor(model.pair_geom1, dtype=torch.long)
    geom2 = torch.as_tensor(model.pair_geom2, dtype=torch.long)
    self.num_geoms = model.ngeom
    self.pair_keys = (torch.minimum(geom1, geom2) * self.num_geoms + torch.maximum(geom1, geom2)).to(env.device)

  def __call__(self, env: ManagerBasedRlEnv) -> torch.Tensor:
    data = env.sim.data
    geom = data.contact.geom.long()
    keys = torch.minimum(geom[:, 0], geom[:, 1]) * self.num_geoms + torch.maximum(geom[:, 0], geom[:, 1])
    active = torch.arange(keys.shape[0], device=keys.device) < data.nacon[0]
    hit = active & (data.contact.dist < 0.0) & torch.isin(keys, self.pair_keys)
    count = torch.zeros(env.num_envs, device=keys.device).index_add_(0, data.contact.worldid[hit].long(), torch.ones_like(keys[hit], dtype=torch.float32))
    env.extras["log"]["Metrics/link_contact_fraction"] = (count > 0).float().mean()
    return count

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    pass
