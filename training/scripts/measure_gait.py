"""Mide si Danny camina con paso alterno a una orden fija, y guarda el resultado en results/.

Orden fija: por defecto 0,25 m/s hacia delante; con `--turn` y `--lateral` se añaden giro y paso lateral. Las medidas están en
`danny_training/gait_measure.py`.

Uso (desde training/):
    uv run python scripts/measure_gait.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name marcha_3000
    uv run python scripts/measure_gait.py --checkpoint ... --forward 0.0 --turn 0.5 --name giro_en_sitio
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from danny_training.evaluation import RESULTS
from danny_training.gait_measure import measure_gait

FORWARD_CMD = 0.25  # m/s, la velocidad de la guía 4.2.


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--forward", type=float, default=FORWARD_CMD, help="Orden de avance en m/s")
  parser.add_argument("--lateral", type=float, default=0.0, help="Orden lateral en m/s, positiva a la izquierda")
  parser.add_argument("--turn", type=float, default=0.0, help="Orden de giro en rad/s, positivo a la izquierda")
  parser.add_argument("--height", type=float, default=0.0, help="Bajada de la pelvis en m (≤ 0, hasta -0,05); \"random\" no: sin esta opción vale 0")
  parser.add_argument("--random-height", action="store_true", help="Alturas del entrenamiento (de 0 a -0,05 m, el 20 % en 0) en vez de una fija")
  parser.add_argument("--vin", type=float, default=None, help="Tensión fija de los servos en V; sin ella, la del entrenamiento (6 a 8,4)")
  parser.add_argument("--pushes", action="store_true", help="Mantiene los empujones del entrenamiento (±0,20 m/s)")
  parser.add_argument("--push", type=float, default=None, metavar="M_S", help="Empujones de ±M_S m/s en XY cada 3 a 6 s (la guía usa 0,12)")
  parser.add_argument("--nominal-mass", action="store_true", help="Sin aleatorización de masa ni de centro de masas (diagnóstico)")
  parser.add_argument("--drop-link-contacts", default=None, metavar="REGEX", help="Quita los <pair> de contacto entre eslabones cuyo nombre de geometría encaje (\".\" los quita todos; diagnóstico)")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  result = measure_gait(
    args.checkpoint,
    forward=args.forward,
    turn=args.turn,
    lateral=args.lateral,
    height=None if args.random_height else args.height,
    vin=args.vin,
    nominal_mass=args.nominal_mass,
    drop_link_contacts=args.drop_link_contacts,
    pushes=args.pushes,
    push_speed=args.push,
    num_envs=args.num_envs,
    seconds=args.seconds,
  )
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_gait_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
