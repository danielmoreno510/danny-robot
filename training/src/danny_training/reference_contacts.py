"""Apoyos de la suela en una referencia de movimiento (solo cinemática, sin simulador).

Criterio de pies de la fase 6 en los movimientos de piernas (decisión del dueño, 19 de noviembre): en cada apoyo,
    exceso = recorrido horizontal de la suela del robot - recorrido horizontal de la suela de la referencia en ese mismo apoyo,
y pasa si el 95 % de los robots tienen un exceso de 1 cm o menos en todos sus apoyos.

Un apoyo se detecta en la referencia, no en el robot. La primera versión (pie a menos de 5 mm de su altura mínima) confundía apoyo y
vuelo: al empezar y acabar un paso la suela está a menos de 5 mm pero ya se mueve a 7-20 cm/s. Un fotograma es de apoyo solo si a la vez:
  - la suela está a menos de `HEIGHT_BAND_M` sobre su altura mínima en todo el movimiento,
  - su velocidad horizontal es menor de `MAX_SPEED_M_S`,
  - su velocidad vertical es menor de `MAX_SPEED_M_S`.
Los apoyos de menos de `MIN_STANCE_FRAMES` fotogramas se descartan, y al medir se recortan `ERODE_FRAMES` fotogramas por cada extremo
(el robot aterriza y despega con algo de retraso respecto a la referencia, y eso no es deslizamiento).
Constantes fijadas antes de mirar los resultados de los cuatro movimientos de piernas.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


HEIGHT_BAND_M = 0.005
MAX_SPEED_M_S = 0.02
MIN_STANCE_FRAMES = 5
ERODE_FRAMES = 2
EXCESS_MAX_M = 0.01
EXCESS_ROBOT_FRACTION = 0.95
SIDES = ("left", "right")
DANNY_XML = Path(__file__).resolve().parents[3] / "cad" / "export" / "danny" / "danny_training.xml"


@dataclass
class Stance:
  start: int  # primer paso del apoyo (el paso t va del fotograma t al t+1)
  end: int  # paso siguiente al último (intervalo [start, end))
  reference_path_m: float


def _rotate(quat_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
  t = 2.0 * np.cross(quat_wxyz[:, 1:], v)
  return v + quat_wxyz[:, :1] * t + np.cross(quat_wxyz[:, 1:], t)


def sole_offsets() -> dict[str, np.ndarray]:
  """Posición de cada sitio `*_sole` en el marco de su pie (constante), leída del MJCF."""
  model = mujoco.MjModel.from_xml_path(str(DANNY_XML))
  return {s: model.site(f"{s}_sole").pos.copy() for s in SIDES}


def body_names() -> list[str]:
  model = mujoco.MjModel.from_xml_path(str(DANNY_XML))
  return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, model.nbody)]


def reference_soles(motion: Path | dict) -> dict[str, np.ndarray]:
  """Posición mundo (frames, 3) de la suela de cada pie en la referencia."""
  data = np.load(motion) if isinstance(motion, Path) else motion
  names, offsets = body_names(), sole_offsets()
  out = {}
  for s in SIDES:
    b = names.index(f"{s}_foot_1")
    out[s] = data["body_pos_w"][:, b] + _rotate(data["body_quat_w"][:, b], offsets[s])
  return out


def stance_mask(sole: np.ndarray, fps: float) -> np.ndarray:
  """Fotogramas de apoyo de una suela (bool, frames)."""
  v = np.gradient(sole, 1.0 / fps, axis=0)
  speed_xy = np.linalg.norm(v[:, :2], axis=1)
  low = sole[:, 2] - sole[:, 2].min() < HEIGHT_BAND_M
  mask = low & (speed_xy < MAX_SPEED_M_S) & (np.abs(v[:, 2]) < MAX_SPEED_M_S)
  # Descarta rachas cortas.
  edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
  for a, b in zip(edges[::2], edges[1::2]):
    if b - a < MIN_STANCE_FRAMES:
      mask[a:b] = False
  return mask


def stances(sole: np.ndarray, fps: float) -> list[Stance]:
  """Apoyos medibles de una suela: intervalos de pasos recortados, con el recorrido horizontal de la referencia en cada uno."""
  mask = stance_mask(sole, fps)
  step_xy = np.linalg.norm(np.diff(sole[:, :2], axis=0), axis=1)  # paso t: fotograma t -> t+1
  edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
  out = []
  for a, b in zip(edges[::2], edges[1::2]):
    start, end = a + ERODE_FRAMES, b - 1 - ERODE_FRAMES  # fotogramas [a, b) -> pasos [a, b-1), recortados
    if end > start:
      out.append(Stance(int(start), int(end), float(step_xy[start:end].sum())))
  return out
