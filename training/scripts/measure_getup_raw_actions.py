"""Mide la acción cruda media de la política de levantarse (fase 4) en pie, antes del recorte a los límites, sin entrenar.

Corre 128 robots 20 s a 6,6 V y promedia la acción de los últimos 10 s. Con escala 1 y objetivo = defecto + acción, compara el objetivo sin recorte con el
límite de cada articulación: si está muy por fuera, el recorte deja sin gradiente a esa articulación (la corrida 6 lo mostró).

Uso (desde training/):
    uv run python scripts/measure_getup_raw_actions.py logs/rsl_rl/danny_walk/<run>/model_XXXX.pt
"""
import json, math, sys
import torch
from danny_training.evaluation import eval_env_cfg, load_policy
ck = sys.argv[1]
cfg = eval_env_cfg(128, 21.0, 6.6, task="Mjlab-GetUp-Danny")
env, policy = load_policy(cfg, ck, task="Mjlab-GetUp-Danny")
u = env.unwrapped; robot = u.scene["robot"]
names = list(robot.joint_names)
lim = robot.data.soft_joint_pos_limits[0]
default = robot.data.default_joint_pos[0]
obs = env.get_observations(); acc = torch.zeros(128, len(names), device=u.device); n = 0
with torch.inference_mode():
  for step in range(int(20 / u.step_dt)):
    a = policy(obs)
    if step >= int(10 / u.step_dt):
      acc += a; n += 1
    obs, _, _, _ = env.step(a)
raw = (acc / n).mean(0)
out = {}
for j, nm in enumerate(names):
  tgt = default[j] + raw[j]  # escala 1
  out[nm] = {"accion_cruda_rad_mean": round(float(raw[j]), 2), "objetivo_sin_recorte_deg": round(math.degrees(float(tgt)), 0), "limite_deg": [round(math.degrees(float(lim[j, 0])), 0), round(math.degrees(float(lim[j, 1])), 0)]}
print(json.dumps(out, ensure_ascii=False))
