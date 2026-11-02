"""Actuador BAM con aleatorización de fricción por entorno.

Adaptado de `src/mjlab_microduck/actuator/friction_dr_bam.py` de xgoduck_rl (Apache 2.0,
LuwuDynamics). El BAM canónico (`bam.mjlab.BamActuator`) no tiene gancho de fricción, así que
esta subclase añade `friction_scale`, que multiplica el presupuesto de fricción independiente de
la velocidad (Coulomb, Stribeck y dependiente de la carga).
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from bam.mjlab import BamActuator, BamActuatorCfg


class FrictionDRBamActuator(BamActuator):
  """BamActuator con `friction_scale` por entorno."""

  def initialize(self, mj_model, model, data, device) -> None:
    super().initialize(mj_model, model, data, device)
    # kp_scale tiene forma (num_envs, 1); se replica para el multiplicador de fricción.
    self.friction_scale = torch.ones_like(self.kp_scale)
    self.default_friction_scale = self.friction_scale.clone()
    # El servo limita la velocidad del objetivo interno; mjlab no crea ese buffer, así que se crea aquí.
    act = self._bam_model.actuator
    if hasattr(act, "default_max_velocity") and not hasattr(act, "q_target_smooth"):
      act.q_target_smooth = 0.0

  def _compute_friction_budget(
    self,
    motor_torque: torch.Tensor,
    external_torque: torch.Tensor,
    stribeck_coeff: torch.Tensor,
  ) -> torch.Tensor:
    base = super()._compute_friction_budget(motor_torque, external_torque, stribeck_coeff)
    fs = getattr(self, "friction_scale", None)
    return base if fs is None else base * fs  # (N, J) * (N, 1)

  def set_friction_scale(self, env_ids, friction_scale: torch.Tensor) -> None:
    self.friction_scale[env_ids] = friction_scale

  def reset_friction_scale(self, env_ids) -> None:
    self.friction_scale[env_ids] = self.default_friction_scale[env_ids]


@dataclass(kw_only=True)
class FrictionDRBamActuatorCfg(BamActuatorCfg):
  """Igual que BamActuatorCfg, pero construye un actuador con `friction_scale`."""

  def build(self, entity, target_ids, target_names) -> FrictionDRBamActuator:
    return FrictionDRBamActuator(self, entity, target_ids, target_names)
