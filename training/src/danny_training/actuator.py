"""Actuador BAM con aleatorización de fricción y holgura por entorno.

Adaptado de `src/mjlab_microduck/actuator/friction_dr_bam.py` de xgoduck_rl (Apache 2.0,
LuwuDynamics). El BAM canónico (`bam.mjlab.BamActuator`) no tiene gancho de fricción, así que
esta subclase añade `friction_scale`, que multiplica el presupuesto de fricción independiente de
la velocidad (Coulomb, Stribeck y dependiente de la carga). También añade una holgura mecánica (`backlash_max_rad`) como un juego
sobre la posición objetivo (ver `apply_backlash`).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import torch
from bam.mjlab import BamActuator, BamActuatorCfg


# Par de bloqueo del servo con el límite de PWM de 0,97, medido por tensión (N·m): 0,739 a 6 V y 0,912 a 7,4 V. Crece con la tensión.
STALL_TORQUE_6V_NM = 0.739
STALL_TORQUE_7P4V_NM = 0.912


def stall_torque_nm(vin):
  """Par de bloqueo a `vin` voltios (número o tensor), por la recta de los dos puntos medidos (a 8,4 V es una extrapolación)."""
  return STALL_TORQUE_6V_NM + (vin - 6.0) * (STALL_TORQUE_7P4V_NM - STALL_TORQUE_6V_NM) / (7.4 - 6.0)


def apply_backlash(target: torch.Tensor, output: torch.Tensor, half_width: torch.Tensor) -> torch.Tensor:
  """Juego mecánico sobre la posición objetivo: la salida sigue al objetivo con un retraso de hasta `half_width`.

  Es el operador de juego (play): la salida no se mueve mientras el objetivo se mantenga a menos de `half_width` de ella, y
  al cambiar de sentido el objetivo tiene que recorrer el juego completo (2 × `half_width`) antes de que la salida lo siga.
  """
  return torch.minimum(torch.maximum(output, target - half_width), target + half_width)


class FrictionDRBamActuator(BamActuator):
  """BamActuator con `friction_scale` y holgura por entorno."""

  def initialize(self, mj_model, model, data, device) -> None:
    super().initialize(mj_model, model, data, device)
    # kp_scale tiene forma (num_envs, 1); se replica para el multiplicador de fricción.
    self.friction_scale = torch.ones_like(self.kp_scale)
    self.default_friction_scale = self.friction_scale.clone()
    # Holgura: semiancho de juego por entorno, sorteado una vez entre 0 y la mitad de `backlash_max_rad`.
    num_envs = self.kp_scale.shape[0]
    max_rad = self.cfg.backlash_max_rad
    self.backlash_half_width = torch.rand(num_envs, 1, device=device) * (max_rad / 2.0) if max_rad > 0.0 else None
    self._backlash_output = torch.zeros(num_envs, len(self._target_names), device=device)
    self._backlash_fresh = torch.ones(num_envs, 1, dtype=torch.bool, device=device)
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

  def apply_delay(self, cmd):
    cmd = super().apply_delay(cmd)
    if self.backlash_half_width is None:
      return cmd
    target = cmd.position_target
    output = torch.where(self._backlash_fresh, target, self._backlash_output)
    self._backlash_output = apply_backlash(target, output, self.backlash_half_width)
    self._backlash_fresh[:] = False
    return dataclasses.replace(cmd, position_target=self._backlash_output)

  def reset(self, env_ids=None) -> None:
    super().reset(env_ids)
    self._backlash_fresh[slice(None) if env_ids is None else env_ids] = True

  def set_friction_scale(self, env_ids, friction_scale: torch.Tensor) -> None:
    self.friction_scale[env_ids] = friction_scale

  def reset_friction_scale(self, env_ids) -> None:
    self.friction_scale[env_ids] = self.default_friction_scale[env_ids]


@dataclass(kw_only=True)
class FrictionDRBamActuatorCfg(BamActuatorCfg):
  """Igual que BamActuatorCfg, pero construye un actuador con `friction_scale` y holgura.

  `backlash_max_rad`: holgura total máxima (rad); cada entorno sortea la suya entre 0 y ese valor. 0 la desactiva.
  """

  backlash_max_rad: float = 0.0

  def build(self, entity, target_ids, target_names) -> FrictionDRBamActuator:
    return FrictionDRBamActuator(self, entity, target_ids, target_names)
