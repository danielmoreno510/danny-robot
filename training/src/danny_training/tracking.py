"""Seguimiento de movimiento (fase 6): Danny imita una referencia (CSV a 50 Hz de `results/retargeted/`) con la tarea de BeyondMimic de mjlab.

Parte de `make_tracking_env_cfg` (pensada para un G1 de 1,3 m) y la adapta a Danny, de 40 cm:
  - Robot, aleatorización (suelo, masa, centro de masas, servo, holgura, empujones de ±0,20 m/s en XY cada 3 a 6 s) y tensión 6 a 8,4 V, los de la marcha.
  - El actor no ve la posición ni la velocidad lineal de la base: el robot real no las mide (como `No-State-Estimation` del G1).
  - Umbrales y desviaciones de las recompensas y de las terminaciones reducidos a la escala de Danny (primer intento, marcado).
La referencia se pasa con `--env.commands.motion.motion-file <npz>` (ver `scripts/convert_motion_csv.py`).
"""

from __future__ import annotations

from copy import deepcopy

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.observation_manager import ObservationGroupCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.tasks.tracking.tracking_env_cfg import make_tracking_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from danny_training import env_cfgs, mdp as danny_mdp
from danny_training.robot import DANNY_ROBOT_CFG

ANCHOR_BODY = "torso_1"  # la IMU del robot real va en el torso.
# Los codos entran porque el criterio de aceptación los mide y sin ellos en la recompensa fallaban (saludar_brazo_alto, encoger_hombros, peek_a_boo).
# El primero es la raíz (mjlab inicia el robot desde el cuerpo 0 de la lista).
MOTION_BODIES = (
  "base_link",
  "torso_1",
  "head_1",
  "left_thigh_1",
  "left_shin_1",
  "left_foot_1",
  "right_thigh_1",
  "right_shin_1",
  "right_foot_1",
  "left_upper_arm_1",
  "left_forearm_1",
  "left_elbow_link_1",
  "right_upper_arm_1",
  "right_forearm_1",
  "right_elbow_link_1",
)
END_BODIES = ("left_foot_1", "right_foot_1", "left_forearm_1", "right_forearm_1")
# Escala de Danny frente al G1 de mjlab: 0,4 m frente a 1,3 m.
FALL_Z_M = 0.08  # el G1 usa 0,25 m: la altura del ancla o de un extremo se desvía más que esto -> caída.
POS_STD_M = 0.10  # el G1 usa 0,3 m.
LIN_VEL_STD_M_S = 0.5  # el G1 usa 1,0 m/s.


def danny_tracking_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  cfg = make_tracking_env_cfg()
  walk = env_cfgs.danny_flat_env_cfg()

  # Simulación, robot y contactos, como en la marcha.
  cfg.sim.njmax = walk.sim.njmax
  cfg.sim.nconmax = walk.sim.nconmax
  cfg.sim.contact_sensor_maxmatch = walk.sim.contact_sensor_maxmatch
  cfg.sim.mujoco.ccd_iterations = walk.sim.mujoco.ccd_iterations
  cfg.scene.entities = {"robot": deepcopy(DANNY_ROBOT_CFG)}
  cfg.scene.sensors = tuple(s for s in walk.scene.sensors if s.name == "feet_ground_contact")
  # Holgura mecánica máxima del actuador, la de la marcha.
  cfg.scene.entities["robot"].articulation.actuators[0].backlash_max_rad = walk.scene.entities["robot"].articulation.actuators[0].backlash_max_rad

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = env_cfgs.ACTION_SCALE
  joint_pos_action.actuator_names = (".*",)

  motion = cfg.commands["motion"]
  assert isinstance(motion, MotionCommandCfg)
  motion.anchor_body_name = ANCHOR_BODY
  motion.body_names = MOTION_BODIES
  # Inicio con ruido pequeño, a la escala de Danny (el G1 usa ±5 cm y ±0,5 m/s).
  motion.pose_range = {"x": (-0.02, 0.02), "y": (-0.02, 0.02), "z": (-0.005, 0.005), "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-0.2, 0.2)}
  motion.velocity_range = {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.05, 0.05), "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.4, 0.4)}
  motion.joint_position_range = (-0.05, 0.05)

  # Eventos de la marcha (suelo, masa, centro de masas, servo, holgura, empujones).
  cfg.events = deepcopy(walk.events)

  # El actor solo ve lo que el robot real mide.
  actor_terms = {k: v for k, v in cfg.observations["actor"].terms.items() if k not in ("motion_anchor_pos_b", "base_lin_vel")}
  actor_terms["base_ang_vel"].noise = Unoise(n_min=-0.06, n_max=0.06)
  actor_terms["joint_vel"].noise = Unoise(n_min=-0.25, n_max=0.25)
  cfg.observations["actor"] = ObservationGroupCfg(terms=actor_terms, concatenate_terms=True, enable_corruption=True)

  # Recompensas a la escala de Danny; los choques entre piezas como en la marcha (los 13 pares del MJCF).
  for name in ("motion_global_root_pos", "motion_body_pos"):
    cfg.rewards[name].params["std"] = POS_STD_M
  cfg.rewards["motion_body_lin_vel"].params["std"] = LIN_VEL_STD_M_S
  del cfg.rewards["self_collisions"]
  cfg.rewards["link_pair_contacts"] = RewardTermCfg(func=danny_mdp.link_pair_contacts, weight=-1.0)
  # Deslizamiento de la suela apoyada: peso 0 (apagado); se activa en los movimientos de piernas con `--env.rewards.feet_slip.weight -0.5`.
  cfg.rewards["feet_slip"] = RewardTermCfg(
    func=danny_mdp.feet_slip_in_contact,
    weight=0.0,
    params={"sensor_name": "feet_ground_contact", "asset_cfg": SceneEntityCfg("robot", site_names=env_cfgs.FOOT_SITES)},
  )

  cfg.terminations["anchor_pos"].params["threshold"] = FALL_Z_M
  cfg.terminations["ee_body_pos"].params["threshold"] = FALL_Z_M
  cfg.terminations["ee_body_pos"].params["body_names"] = END_BODIES

  cfg.viewer.body_name = ANCHOR_BODY
  cfg.viewer.distance = 0.9
  cfg.viewer.lookat = (0.0, 0.0, 0.2)
  cfg.viewer.elevation = -10.0
  cfg.viewer.azimuth = 135.0

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    motion.pose_range = {}
    motion.velocity_range = {}
    motion.sampling_mode = "start"
  return cfg
