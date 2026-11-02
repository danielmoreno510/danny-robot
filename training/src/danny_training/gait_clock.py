"""Reloj de marcha de Danny: una fase que avanza con un periodo fijo mientras hay orden de movimiento.

No es una trayectoria: solo marca el ritmo. La política ve seno y coseno de la fase, y `mdp.gait_contact_phase` premia
que en la primera mitad del ciclo apoye el pie izquierdo y el derecho esté en el aire, y al revés en la segunda. Con orden
cero la fase se congela. Cada episodio empieza en fase 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg


class GaitClockCommand(CommandTerm):
  """Fase de marcha en [0, 1). El valor observable es (seno, coseno) de 2π·fase."""

  cfg: GaitClockCommandCfg

  def __init__(self, cfg: GaitClockCommandCfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    self.phase = torch.zeros(self.num_envs, device=self.device)
    self.moving = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

  @property
  def command(self) -> torch.Tensor:
    angle = 2.0 * math.pi * self.phase
    return torch.stack((torch.sin(angle), torch.cos(angle)), dim=1)

  def _update_metrics(self) -> None:
    pass

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    # Solo se llama al reiniciar el episodio: el intervalo de nuevo muestreo es enorme.
    self.phase[env_ids] = 0.0

  def _update_command(self) -> None:
    twist = self._env.command_manager.get_command(self.cfg.velocity_command_name)
    assert twist is not None
    self.moving = (torch.norm(twist[:, :2], dim=1) + torch.abs(twist[:, 2])) > self.cfg.command_threshold
    step = self._env.step_dt / self.cfg.period_s
    self.phase = torch.where(self.moving, torch.remainder(self.phase + step, 1.0), self.phase)


@dataclass(kw_only=True)
class GaitClockCommandCfg(CommandTermCfg):
  period_s: float = 0.8
  velocity_command_name: str = "twist"
  command_threshold: float = 0.01
  resampling_time_range: tuple[float, float] = (1.0e9, 1.0e9)

  def build(self, env: ManagerBasedRlEnv) -> GaitClockCommand:
    return GaitClockCommand(self, env)
