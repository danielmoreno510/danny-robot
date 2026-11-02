"""Mide el par que aplican los actuadores HD-1910 (modelo BAM) durante la marcha y guarda el resultado en results/.

Corre un checkpoint en simulación sin empujones y registra, por articulación, el par aplicado
(`data.actuator_force`) frente al par de parada a 6 V (0,762 N·m, de la guía y params/hd1910_m6.json).
Sirve para saber si los servos se quedan sin par cuando la política pide movimiento.

Uso (desde training/):
    uv run python scripts/measure_actuator_torque.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --action-scale 0.25 --name rewardw_300
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

TASK = "Mjlab-Velocity-Flat-Danny"
STALL_6V_NM = 0.762
RESULTS = Path(__file__).resolve().parents[2] / "results"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--action-scale", required=True, type=float, help="Escala de acción con la que se entrenó")
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--num-envs", type=int, default=64)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env_cfg = load_env_cfg(TASK)
  env_cfg.scene.num_envs = args.num_envs
  env_cfg.episode_length_s = args.seconds
  env_cfg.events.pop("push_robot", None)  # Sin empujones: se mide la marcha.
  env_cfg.actions["joint_pos"].scale = args.action_scale

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  agent_cfg = load_rl_cfg(TASK)
  env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
  env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
  runner = load_runner_cls(TASK)(env, asdict(agent_cfg), device=device)
  runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
  policy = runner.get_inference_policy(device=device)

  u = env.unwrapped
  robot = u.scene["robot"]
  joint_names = list(robot.joint_names)
  steps = int(args.seconds / u.step_dt)
  settle = int(1.0 / u.step_dt)
  obs = env.get_observations()

  tau_max = None
  tau_sum = torch.zeros(len(joint_names), device=device)
  over_sum = torch.zeros(len(joint_names), device=device)
  count = 0
  with torch.inference_mode():
    for step in range(steps):
      obs, _, _, _ = env.step(policy(obs))
      if step < settle:
        continue
      tau = robot.data.actuator_force.abs()
      tau_max = tau if tau_max is None else torch.maximum(tau_max, tau)
      tau_sum += tau.mean(dim=0)
      over_sum += (tau > 0.9 * STALL_6V_NM).float().mean(dim=0)
      count += 1

  mx = tau_max.amax(dim=0)
  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "action_scale": args.action_scale,
    "num_envs": args.num_envs,
    "seconds_simulated": args.seconds,
    "stall_torque_6v_nm": STALL_6V_NM,
    "joints": {
      name: {
        "max_abs_nm": round(mx[i].item(), 4),
        "mean_abs_nm": round((tau_sum[i] / count).item(), 4),
        "frac_time_above_90pct_stall": round((over_sum[i] / count).item(), 4),
      }
      for i, name in enumerate(joint_names)
    },
    "note": "Sin empujones. Perturbaciones de entrenamiento activas. Actuadores HD-1910 sin confirmar.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_torque_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(f"Escrito: {out}")
  env.close()


if __name__ == "__main__":
  main()
