"""Mide el cambio de la política de marcha de la fase 2 a la de agacharse quieto (fase 3), y guarda el resultado en results/.

Danny parte de pie con la política de la fase 2 y orden de velocidad cero; a los 3 s se cambia a la política de agacharse con la
misma orden cero y una orden de altura fija. Se mide durante los 5 s siguientes: fracción que no cae, velocidad media entre los 2 y
los 5 s tras el cambio (debe estar quieto) y error de altura de la pelvis en esa misma ventana. La política de la fase 2 es el punto de
partida de la fase 3 con la entrada de altura a cero (`extend_checkpoint_inputs.py`), que camina igual (error de velocidad 0,0375 m/s).

Uso (desde training/):
    uv run python scripts/measure_policy_switch.py --stand-checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name fase3_quieto
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, fix_command, load_policy, load_policy_on
from danny_training.crouch import STAND_PELVIS_Z_M

PHASE2_START = Path("logs/rsl_rl/danny_walk/2026-11-05_00-00-00_fase3_inicio/model_8995.pt")
BEFORE_S, AFTER_S, SETTLE_S = 3.0, 5.0, 2.0


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--stand-checkpoint", required=True, type=Path)
  parser.add_argument("--phase2-checkpoint", type=Path, default=PHASE2_START)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--vin", type=float, default=6.6)
  parser.add_argument("--num-envs", type=int, default=256)
  args = parser.parse_args()

  heights = {}
  for height in (0.0, -0.025, -0.05):
    cfg = eval_env_cfg(args.num_envs, BEFORE_S + AFTER_S + 2.0, args.vin, False)
    fix_command(cfg, 0.0, height=height)
    env, walk_policy = load_policy(cfg, args.phase2_checkpoint)
    stand_policy = load_policy_on(env, args.stand_checkpoint)
    u = env.unwrapped
    robot = u.scene["robot"]
    pelvis = robot.find_sites(["pelvis_bottom"])[0][0]
    dt = u.step_dt
    switch, settle, total = int(BEFORE_S / dt), int(SETTLE_S / dt), int((BEFORE_S + AFTER_S) / dt)
    alive = torch.ones(args.num_envs, dtype=torch.bool, device=u.device)
    alive_at_switch = None
    speed = torch.zeros(args.num_envs, device=u.device)
    z = torch.zeros(args.num_envs, device=u.device)
    counted = 0
    obs = env.get_observations()
    with torch.inference_mode():
      for i in range(total):
        policy = walk_policy if i < switch else stand_policy
        obs, _, _, _ = env.step(policy(obs))
        alive &= ~u.reset_terminated.bool()
        if i == switch - 1:
          alive_at_switch = alive.clone()
        if i >= switch + settle:
          counted += 1
          speed += robot.data.root_com_lin_vel_b[:, :2].norm(dim=1)
          z += robot.data.site_pos_w[:, pelvis, 2]
    ok = alive
    err = (z / counted - (STAND_PELVIS_Z_M + height)).abs()
    heights[str(height)] = {
      "fraction_not_fallen_before_switch": round(float(alive_at_switch.float().mean()), 4),
      "fraction_not_fallen": round(float(ok.float().mean()), 4),
      "mean_xy_speed_m_s": round(float((speed / counted)[ok].mean()), 4) if ok.any() else None,
      "mean_pelvis_height_error_m": round(float(err[ok].mean()), 4) if ok.any() else None,
    }
    print(height, heights[str(height)], flush=True)
    env.close()

  out = {
    "stand_checkpoint": str(args.stand_checkpoint),
    "phase2_checkpoint": str(args.phase2_checkpoint),
    "vin_v": args.vin,
    "seconds_before_switch": BEFORE_S,
    "seconds_after_switch": AFTER_S,
    "window_after_switch_s": [SETTLE_S, AFTER_S],
    "note": "Sin empujones. Orden de velocidad cero. Velocidad y altura se miden entre los 2 y 5 s tras el cambio, solo en entornos que no caen.",
    "by_height_command_m": heights,
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  path = RESULTS / f"training_switch_{args.name}.json"
  path.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {path}")


if __name__ == "__main__":
  main()
