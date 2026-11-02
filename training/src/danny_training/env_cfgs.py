"""Configuraciones del entorno de marcha de Danny sobre terreno plano (etapa 4.2).

Parte de la tarea de velocidad de mjlab (`make_velocity_env_cfg`), con los nombres de cuerpos,
sitios y geometrías de Danny. Los valores que no vienen de la guía de entrenamiento están marcados
como primer intento.
"""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import (
    ContactMatch,
    ContactSensorCfg,
    ObjRef,
    RingPatternCfg,
    TerrainHeightSensorCfg,
)
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from danny_training import mdp as danny_mdp
from danny_training.robot import DANNY_ROBOT_CFG

FEET_BODIES = r"^(left_foot_1|right_foot_1)$"
FOOT_SITES = ("left_sole", "right_sole")
FOOT_GEOMS = ("left_sole", "right_sole")
TRUNK_BODY = "base_link"

# Escala de la acción sobre la pose por defecto (rad por unidad de acción). Primer intento: la
# referencia usa 0,5, pero Danny tiene un par máximo de 0,94 N·m y no se ha calculado a partir de él.
ACTION_SCALE = 0.5


def danny_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Marcha de Danny sobre suelo plano, con aleatorización de la guía de entrenamiento."""
    cfg = make_velocity_env_cfg()

    cfg.sim.njmax = 300
    cfg.sim.mujoco.ccd_iterations = 50
    cfg.sim.contact_sensor_maxmatch = 64
    cfg.sim.nconmax = None

    cfg.scene.entities = {"robot": deepcopy(DANNY_ROBOT_CFG)}

    # Suelo plano: sin escaneo de terreno ni curriculum de terreno.
    assert cfg.scene.terrain is not None
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None
    cfg.scene.sensors = tuple(s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan")

    # La altura bajo cada pie se mide desde las suelas, con un anillo de rayos.
    for sensor in cfg.scene.sensors or ():
        if sensor.name == "foot_height_scan":
            assert isinstance(sensor, TerrainHeightSensorCfg)
            sensor.frame = tuple(ObjRef(type="site", name=s, entity="robot") for s in FOOT_SITES)
            sensor.pattern = RingPatternCfg.single_ring(radius=0.03, num_samples=6)
    del cfg.observations["actor"].terms["height_scan"]
    del cfg.observations["critic"].terms["height_scan"]
    cfg.terminations.pop("out_of_terrain_bounds", None)
    cfg.curriculum.pop("terrain_levels", None)

    # Contacto de los pies con el suelo: lo usan las recompensas de balanceo y deslizamiento.
    feet_ground_cfg = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(mode="subtree", pattern=FEET_BODIES, entity="robot"),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="netforce",
        num_slots=1,
        track_air_time=True,
    )
    cfg.scene.sensors = (cfg.scene.sensors or ()) + (feet_ground_cfg,)

    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg)
    joint_pos_action.scale = ACTION_SCALE
    joint_pos_action.actuator_names = (".*",)

    cfg.viewer.body_name = TRUNK_BODY

    # El currículo de velocidad de mjlab fija los rangos en ±1 m/s desde el paso 0 y luego los amplía a 3 m/s,
    # lo que pisaba los rangos de abajo. Para Danny se quita: las órdenes se quedan en los de la guía.
    cfg.curriculum.pop("command_vel", None)

    # Órdenes de velocidad (guía 4.2: caminar a 0,25 m/s). Sin desplazamiento lateral, con giro suave.
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    # Se reemplaza el objeto entero: el Ranges por defecto se comparte entre tareas y otra tarea lo pisaba.
    twist_cmd.ranges = replace(
        twist_cmd.ranges,
        lin_vel_x=(-0.25, 0.25),
        lin_vel_y=(-0.1, 0.1),
        ang_vel_z=(-0.5, 0.5),
    )
    twist_cmd.viz.z_offset = 0.5

    # Aleatorización de la guía: suelo 0,4 a 1,2; masa ±10 %; centro de masas ±10 mm; servo ±30 %.
    cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
    cfg.events["foot_friction"].params["ranges"] = (0.4, 1.2)
    cfg.events["base_com"].params["asset_cfg"].body_names = (TRUNK_BODY,)
    cfg.events["base_com"].params["ranges"] = (-0.01, 0.01)  # ±10 mm en x, y y z.
    cfg.events["base_com"].params["axes"] = [0, 1, 2]
    cfg.events["encoder_bias"].params["bias_range"] = (-math.radians(0.5), math.radians(0.5))
    cfg.events["randomize_body_mass"] = EventTermCfg(
        func=dr.body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(r".*",)),
            "ranges": (0.9, 1.1),
            "operation": "scale",
        },
    )
    cfg.events["randomize_servo_friction"] = EventTermCfg(
        func=danny_mdp.randomize_bam_friction,
        mode="reset",
        params={"scale_range": (0.7, 1.3)},
    )

    # Recompensas: la postura de referencia y los pies se ajustan a Danny (primer intento, a revisar en 4.2).
    cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
    cfg.rewards["pose"].params["std_walking"] = {
        r".*hip_pitch.*": 0.3,
        r".*hip_roll.*": 0.15,
        r".*hip_yaw.*": 0.15,
        r".*knee.*": 0.35,
        r".*ankle_pitch.*": 0.25,
        r".*ankle_roll.*": 0.1,
        r".*waist_yaw.*": 0.2,
        r".*shoulder_pitch.*": 0.15,
        r".*shoulder_roll.*": 0.15,
        r".*shoulder_yaw.*": 0.1,
        r".*elbow.*": 0.15,
        r".*neck.*": 0.1,
    }
    cfg.rewards["pose"].params["std_running"] = dict(cfg.rewards["pose"].params["std_walking"])

    cfg.rewards["upright"].params["asset_cfg"].body_names = (TRUNK_BODY,)
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = (TRUNK_BODY,)
    for reward_name in ("foot_clearance", "foot_slip"):
        cfg.rewards[reward_name].params["asset_cfg"].site_names = FOOT_SITES

    # Más peso al seguimiento y menos a la postura: con los pesos de la referencia, el robot aprendía a quedarse quieto.
    cfg.rewards["track_linear_velocity"].weight = 4.0
    cfg.rewards["pose"].weight = 0.5
    cfg.rewards["body_ang_vel"].weight = -0.05
    cfg.rewards["angular_momentum"].weight = -0.02
    cfg.rewards["air_time"].weight = 0.0

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}

    return cfg
