"""Registro de las tareas de Danny en mjlab (punto de entrada `danny_training.tasks`)."""

from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from danny_training.env_cfgs import danny_flat_env_cfg
from danny_training.getup import danny_getup_env_cfg
from danny_training.rl_cfg import danny_flat_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Danny",
  env_cfg=danny_flat_env_cfg(),
  play_env_cfg=danny_flat_env_cfg(play=True),
  rl_cfg=danny_flat_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-GetUp-Danny",
  env_cfg=danny_getup_env_cfg(),
  play_env_cfg=danny_getup_env_cfg(play=True),
  rl_cfg=danny_flat_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
