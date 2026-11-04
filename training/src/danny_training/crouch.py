"""Agacharse (fase 3): orden de altura de la pelvis y la pose de referencia que le corresponde.

La orden es la bajada de la pelvis (`pelvis_bottom`) respecto a la de pie, de 0 a `-0,05` m. Para cada bajada, la pose de referencia
(cadera, rodilla y tobillo de cada pierna) y la inclinación del tronco salen del cálculo estático de `scripts/check_crouch_pose.py`
(`results/training_crouch_static.json`): pies planos, centro de masas sobre las suelas y el tronco inclinado 20° a -0,05 m.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg

MAX_DROP_M = 0.05
# Altura de la pelvis (`pelvis_bottom`) de pie, medida en el modelo con la política de la fase 2 (origen de base_link a -0,0127 m).
STAND_PELVIS_Z_M = 0.228
LEAN_RAD_PER_M = 0.35 / MAX_DROP_M

# Bajadas de la tabla estática (m) y, para cada una, cadera, rodilla y tobillo de cada pierna (rad).
_DROPS = (0.0, 0.025, 0.05)
_HIP = (-0.364, -0.925, -1.419)
_KNEE = (0.827, 1.389, 1.754)
_ANKLE = (-0.463, -0.639, -0.686)
JOINT_TABLE = {"hip_pitch": _HIP, "knee": _KNEE, "ankle_pitch": _ANKLE}


def interpolate(drop: torch.Tensor, values: tuple[float, ...]) -> torch.Tensor:
  """Interpolación lineal de `values` en las bajadas de la tabla (sin extrapolar más allá de los extremos)."""
  xs = torch.tensor(_DROPS, device=drop.device)
  ys = torch.tensor(values, device=drop.device)
  d = drop.clamp(0.0, MAX_DROP_M)
  upper = torch.bucketize(d, xs).clamp(1, len(xs) - 1)
  x0, x1, y0, y1 = xs[upper - 1], xs[upper], ys[upper - 1], ys[upper]
  return y0 + (y1 - y0) * (d - x0) / (x1 - x0)


class HeightCommand(CommandTerm):
  """Bajada de la pelvis en metros (valor ≤ 0), una por entorno. Observable tal cual, con forma (entornos, 1)."""

  cfg: HeightCommandCfg

  def __init__(self, cfg: HeightCommandCfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    self.height = torch.zeros(self.num_envs, 1, device=self.device)

  @property
  def command(self) -> torch.Tensor:
    return self.height

  def _update_metrics(self) -> None:
    pass

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    lo, hi = self.cfg.range_m
    sample = torch.rand(len(env_ids), 1, device=self.device) * (hi - lo) + lo
    zero = torch.rand(len(env_ids), 1, device=self.device) < self.cfg.zero_fraction
    self.height[env_ids] = torch.where(zero, torch.zeros_like(sample), sample)

  def _update_command(self) -> None:
    pass


@dataclass(kw_only=True)
class HeightCommandCfg(CommandTermCfg):
  range_m: tuple[float, float] = (-MAX_DROP_M, 0.0)
  zero_fraction: float = 0.2
  resampling_time_range: tuple[float, float] = (4.0, 8.0)

  def build(self, env: ManagerBasedRlEnv) -> HeightCommand:
    return HeightCommand(self, env)
