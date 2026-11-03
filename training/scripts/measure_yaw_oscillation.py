"""Separa el error de giro de Danny en oscilación del paso y giro medio corto, y guarda el resultado en results/.

Para cada punto de control y cada orden (adelante, atrás, giro a la izquierda, quieto) mide 64 robots a 7,4 V durante 12 s, sin
empujones, descartando los 2 primeros. Por orden da: el error por paso (el del criterio de la fase 2), el error tras promediar un
ciclo de marcha de 0,8 s, el error de la media por robot, la amplitud de la oscilación de la velocidad de giro (RMS) y sus tres
frecuencias dominantes. Si el error por paso es mucho mayor que el promediado, el error viene del ritmo del paso; si coinciden, el
giro medio queda corto.

Uso (desde training/):
    uv run python scripts/measure_yaw_oscillation.py --name fase2 logs/rsl_rl/danny_walk/<run>/model_XXXX.pt [más puntos de control]
"""

import argparse
import json
from pathlib import Path

import torch

from danny_training.evaluation import RESULTS, eval_env_cfg, fix_command, load_policy

CASES = {"adelante": (0.25, 0.0, 0.0), "atras": (-0.25, 0.0, 0.0), "giro_izq": (0.0, 0.0, 0.5), "quieto": (0.0, 0.0, 0.0)}
N, SECONDS, WARM = 64, 12.0, 2.0
parser = argparse.ArgumentParser()
parser.add_argument("--name", required=True, help="Nombre del resultado en results/")
parser.add_argument("checkpoints", nargs="+")
args = parser.parse_args()
out = {}
for ck in args.checkpoints:
  out[ck] = {}
  for name, (fw, lat, turn) in CASES.items():
    cfg = eval_env_cfg(N, SECONDS, 7.4, False)
    fix_command(cfg, fw, turn, lat)
    env, policy = load_policy(cfg, Path(ck))
    u = env.unwrapped
    robot = u.scene["robot"]
    dt = u.step_dt
    alive = torch.ones(N, dtype=torch.bool, device=u.device)
    ys, ws, wz_pelvis = [], [], []
    obs = env.get_observations()
    with torch.inference_mode():
      for i in range(int(SECONDS / dt)):
        obs, _, _, _ = env.step(policy(obs))
        alive &= ~u.reset_terminated.bool()
        if i >= int(WARM / dt):
          ys.append(robot.data.root_link_ang_vel_b[:, 2].clone())
          cmd = u.command_manager.get_command("twist")
          ws.append(cmd[:, 2].clone())
    y = torch.stack(ys, 1)[alive]  # (envs, T)
    c = torch.stack(ws, 1)[alive]
    T = y.shape[1]
    win = int(0.8 / dt)
    kern = torch.ones(1, 1, win, device=y.device) / win
    smooth = torch.nn.functional.conv1d(y[:, None], kern)[:, 0]
    csm = c[:, win - 1 :]
    raw_err = (y - c).abs().mean().item()
    smooth_err = (smooth - csm).abs().mean().item()
    mean_err = (y.mean(1) - c.mean(1)).abs().mean().item()
    osc_rms = (y - y.mean(1, keepdim=True)).pow(2).mean().sqrt().item()
    spec = torch.fft.rfft(y - y.mean(1, keepdim=True), dim=1).abs().mean(0)
    freqs = torch.fft.rfftfreq(T, dt)
    top = torch.topk(spec[1:], 3).indices + 1
    out[ck][name] = {
      "envs_alive": int(alive.sum()),
      "mean_yaw": round(y.mean().item(), 4),
      "raw_abs_error": round(raw_err, 4),
      "error_after_0.8s_average": round(smooth_err, 4),
      "error_of_per_robot_mean": round(mean_err, 4),
      "oscillation_rms": round(osc_rms, 4),
      "top_freq_hz": [round(freqs[i].item(), 2) for i in top],
      "dt": dt,
    }
    env.close()
    print(Path(ck).parent.name[-8:], name, out[ck][name], flush=True)
RESULTS.mkdir(parents=True, exist_ok=True)
Path(RESULTS / f"training_yaw_oscillation_{args.name}.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
