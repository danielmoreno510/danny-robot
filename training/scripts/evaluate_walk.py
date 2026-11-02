"""Evalúa un checkpoint de marcha de Danny sin interfaz y guarda el resultado en results/.

Mide cuánto aguanta cada entorno sin caerse (en segundos simulados) y el error de seguimiento de la
velocidad. Corre con las mismas perturbaciones del entrenamiento.

Uso (desde training/):
    uv run python scripts/evaluate_walk.py --checkpoint logs/rsl_rl/danny_walk/<run>/model_XXXX.pt --name smoke_30
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

TASK = "Mjlab-Velocity-Flat-Danny"
RESULTS = Path(__file__).resolve().parents[2] / "results"


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--checkpoint", required=True, type=Path)
  parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
  parser.add_argument("--num-envs", type=int, default=256)
  parser.add_argument("--seconds", type=float, default=20.0)
  args = parser.parse_args()

  env_cfg = load_env_cfg(TASK)
  env_cfg.scene.num_envs = args.num_envs
  env_cfg.episode_length_s = args.seconds
  env_cfg.events.pop("push_robot", None)  # Sin empujones: se mide la marcha, no la recuperación.

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  agent_cfg = load_rl_cfg(TASK)
  env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
  env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

  runner_cls = load_runner_cls(TASK) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=device)
  runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
  policy = runner.get_inference_policy(device=device)

  steps = int(args.seconds / env.unwrapped.step_dt)
  alive = torch.ones(args.num_envs, dtype=torch.bool, device=device)
  survival_steps = torch.zeros(args.num_envs, device=device)
  track_err = []

  obs = env.get_observations()
  with torch.inference_mode():
    for _ in range(steps):
      actions = policy(obs)
      obs, _, dones, _ = env.step(actions)
      twist = env.unwrapped.command_manager.get_command("twist")
      robot = env.unwrapped.scene["robot"]
      vel_err = (robot.data.root_link_lin_vel_b[:, :2] - twist[:, :2]).norm(dim=1)
      track_err.append(vel_err[alive].mean().item() if alive.any() else float("nan"))
      survival_steps += alive.float()
      # Solo cuenta como caída la terminación, no el fin de episodio por tiempo.
      alive &= ~env.unwrapped.reset_terminated.bool()

  survival_s = survival_steps * env.unwrapped.step_dt
  result = {
    "checkpoint": str(args.checkpoint),
    "task": TASK,
    "num_envs": args.num_envs,
    "seconds_simulated": args.seconds,
    "fraction_standing_full_episode": float(alive.float().mean()),
    "survival_s": {
      "mean": float(survival_s.mean()),
      "min": float(survival_s.min()),
      "median": float(survival_s.median()),
    },
    "mean_xy_velocity_error_m_s": float(torch.tensor([e for e in track_err if e == e]).mean()),
    "note": "Sin empujones. Perturbaciones de entrenamiento activas. Actuadores HD-1910 sin confirmar.",
  }
  RESULTS.mkdir(parents=True, exist_ok=True)
  out = RESULTS / f"training_walk_{args.name}.json"
  out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  env.close()


if __name__ == "__main__":
  main()
