"""Configuraciones del entorno de marcha de Danny sobre terreno plano (etapa 4.2).

Parte de la tarea de velocidad de mjlab (`make_velocity_env_cfg`), con los nombres de cuerpos,
sitios y geometrías de Danny. Los valores que no vienen de la guía de entrenamiento están marcados
como primer intento.
"""

from __future__ import annotations

import math
import os
from copy import deepcopy
from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr, generated_commands
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
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
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from danny_training import crouch, mdp as danny_mdp
from danny_training.gait_clock import GaitClockCommandCfg
from danny_training.robot import DANNY_ROBOT_CFG

FEET_BODIES = r"^(left_foot_1|right_foot_1)$"
FOOT_SITES = ("left_sole", "right_sole")
FOOT_GEOMS = ("left_sole", "right_sole")
TRUNK_BODY = "base_link"

# Escala de la acción sobre la pose por defecto (rad por unidad de acción). Igual que xgoduck_rl (guía 4.2).
# Con 0,25 y 0,5 no hubo marcha.
ACTION_SCALE = 1.0

# Aleatorización que se añade poco a poco en la fase 2: armadura de los servos (±), holgura mecánica máxima y empujones (± m/s en XY).
# La etapa se elige con DANNY_DR_STAGE al entrenar; sin ella, y al evaluar, vale la 3, la completa de la guía (armadura ±30 %, holgura
# de hasta 0,5°). Los empujones llegan a ±0,2 m/s, como en xgoduck_rl.
DR_STAGES = {
    1: {"armature": 0.10, "backlash_deg": 0.15, "push": 0.12},
    2: {"armature": 0.20, "backlash_deg": 0.30, "push": 0.16},
    3: {"armature": 0.30, "backlash_deg": 0.50, "push": 0.20},
}
DR_STAGE = int(os.environ.get("DANNY_DR_STAGE", "3"))
assert DR_STAGE in DR_STAGES, f"DANNY_DR_STAGE debe ser 1, 2 o 3, no {DR_STAGE}"
# DANNY_PUSH_M_S fija la fuerza de los empujones al entrenar, sea cual sea la etapa (la guía usa 0,12).
PUSH_OVERRIDE_M_S = float(os.environ["DANNY_PUSH_M_S"]) if "DANNY_PUSH_M_S" in os.environ else None


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

    # El actor no ve la velocidad lineal de la base: el robot real no la mide. Ruido de xgoduck_rl.
    actor_terms = cfg.observations["actor"].terms
    del actor_terms["base_lin_vel"]
    actor_terms["base_ang_vel"].noise = Unoise(n_min=-0.06, n_max=0.06)
    actor_terms["projected_gravity"].noise = Unoise(n_min=-0.03, n_max=0.03)
    actor_terms["joint_vel"].noise = Unoise(n_min=-0.25, n_max=0.25)
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
    # Cámara para un robot de 40 cm (la de mjlab es para uno de 1,3 m): cerca, a media altura del cuerpo y de tres cuartos.
    cfg.viewer.distance = 0.9
    cfg.viewer.lookat = (0.0, 0.0, 0.2)
    cfg.viewer.elevation = -10.0
    cfg.viewer.azimuth = 135.0

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
    # Fase 2: avanzar, retroceder, paso lateral, girar y parar. El 20 % de los entornos recibe orden cero; mjlab fuerza por defecto otro
    # 20 % a avanzar a 0,3 m/s sin girar, y aquí se quita para no sesgar las órdenes.
    twist_cmd.rel_standing_envs = 0.2
    twist_cmd.rel_forward_envs = 0.0

    # Reloj de marcha: fase de 0,8 s que solo avanza con orden de movimiento. La política ve su seno y su coseno.
    cfg.commands["gait"] = GaitClockCommandCfg(period_s=0.8, velocity_command_name="twist")
    for group in ("actor", "critic"):
        cfg.observations[group].terms["gait_clock"] = ObservationTermCfg(
            func=generated_commands, params={"command_name": "gait"}
        )

    # Fase 3, agacharse: orden de bajada de la pelvis, de 0 a -0,05 m, que la política ve como una observación más (la última).
    cfg.commands["height"] = crouch.HeightCommandCfg()
    for group in ("actor", "critic"):
        cfg.observations[group].terms["height"] = ObservationTermCfg(
            func=generated_commands, params={"command_name": "height"}
        )

    # Aleatorización de la guía: suelo 0,4 a 1,2; masa ±10 %; centro de masas ±10 mm; servo ±30 %.
    # Masa y centro de masas con `shared_random`: todos los cuerpos comparten la muestra, así que la masa total varía ±10 % y
    # el centro de masas del robot entero ±10 mm. Con una muestra por cuerpo los errores se compensarían entre sí.
    cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
    cfg.events["foot_friction"].params["ranges"] = (0.4, 1.2)
    cfg.events["base_com"].params["asset_cfg"].body_names = (r".*",)
    cfg.events["base_com"].params["ranges"] = (-0.01, 0.01)  # ±10 mm en x, y y z.
    cfg.events["base_com"].params["axes"] = [0, 1, 2]
    cfg.events["base_com"].params["shared_random"] = True
    cfg.events["encoder_bias"].params["bias_range"] = (-math.radians(0.5), math.radians(0.5))
    cfg.events["randomize_body_mass"] = EventTermCfg(
        func=dr.body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=(r".*",)),
            "ranges": (0.9, 1.1),
            "operation": "scale",
            "shared_random": True,
        },
    )
    # Empujones de la guía: solo en XY, ±0,12 m/s, cada 3 a 6 s.
    cfg.events["push_robot"].interval_range_s = (3.0, 6.0)
    stage = DR_STAGES[DR_STAGE]
    push = PUSH_OVERRIDE_M_S if PUSH_OVERRIDE_M_S is not None else stage["push"]
    cfg.events["push_robot"].params["velocity_range"] = {"x": (-push, push), "y": (-push, push)}
    cfg.events["randomize_servo_armature"] = EventTermCfg(
        func=dr.joint_armature,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)),
            "ranges": (1.0 - stage["armature"], 1.0 + stage["armature"]),
            "operation": "scale",
        },
    )
    cfg.scene.entities["robot"].articulation.actuators[0].backlash_max_rad = math.radians(stage["backlash_deg"])
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
    # Agacharse: el tronco se inclina hacia delante en proporción a la bajada pedida, y la pose de referencia baja con ella.
    cfg.rewards["upright"].func = danny_mdp.crouch_upright
    cfg.rewards["upright"].params = {"std": cfg.rewards["upright"].params["std"], "command_name": "height", "asset_cfg": SceneEntityCfg("robot", body_names=(TRUNK_BODY,))}
    cfg.rewards["pose"].func = danny_mdp.crouch_posture
    cfg.rewards["pose"].params["crouch_command_name"] = "height"
    cfg.rewards["track_pelvis_height"] = RewardTermCfg(
        func=danny_mdp.track_pelvis_height,
        weight=3.0,
        params={"std": 0.006, "command_name": "height", "twist_name": "twist", "asset_cfg": SceneEntityCfg("robot", site_names=("pelvis_bottom",))},
    )
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = (TRUNK_BODY,)
    for reward_name in ("foot_clearance", "foot_slip"):
        cfg.rewards[reward_name].params["asset_cfg"].site_names = FOOT_SITES

    # Valores de la guía (training-guide.md, 4.2). Con los de mjlab por defecto, quedarse quieto daba el 91 % de la
    # recompensa de velocidad (results/walk_incentive.json).
    # Velocidad medida en el centro de masas de base_link, no en su origen, que está en el suelo (ver mdp).
    cfg.rewards["track_linear_velocity"].func = danny_mdp.track_com_linear_velocity
    cfg.rewards["track_linear_velocity"].weight = 3.0
    cfg.rewards["track_linear_velocity"].params["std"] = 0.125
    # Recompensa de giro doble: la de mjlab (peso 2, std 0,71 rad/s) guía con errores grandes y un segundo término igual con std 0,25 y
    # peso 1 premia la precisión cerca de la orden. Cambiar la primera a std 0,25 y peso 3 solo empeoró atrás y el lateral derecho
    # (results/training_movements_fase2_d_7496.json). Ambos incluyen también la velocidad angular en x e y, como la función de mjlab.
    cfg.rewards["track_angular_velocity_fine"] = RewardTermCfg(
        func=cfg.rewards["track_angular_velocity"].func,
        weight=1.0,
        params={"command_name": "twist", "std": 0.25},
    )
    cfg.rewards["pose"].weight = 1.0
    cfg.rewards["body_ang_vel"].weight = -0.05
    cfg.rewards["angular_momentum"].weight = -0.02
    # El reloj de marcha sustituye a air_time: con air_time Danny se quedaba sobre un pie y subía y bajaba el otro.
    del cfg.rewards["air_time"]
    for reward_name in ("foot_clearance", "foot_swing_height"):
        cfg.rewards[reward_name].params["target_height"] = 0.025
    cfg.rewards["foot_swing_height"].weight = -5.0
    # Contactos entre eslabones y separación entre suelas. Con el modelo final, la política que caminaba sin ellos cae casi siempre:
    # los pies chocan entre sí (16 mm de holgura de pie, ver PROJECT_HANDOFF.md).
    cfg.rewards["link_pair_contacts"] = RewardTermCfg(func=danny_mdp.link_pair_contacts, weight=-1.0)
    cfg.rewards["feet_lateral_clearance"] = RewardTermCfg(
        func=danny_mdp.feet_lateral_clearance,
        weight=-2.0,
        params={
            "min_clearance": 0.010,
            "sole_width": 0.048,
            "sole_length": 0.085,
            "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
        },
    )
    # Par de rodillas y tobillos (cabeceo) por encima del 80 % del bloqueo de la tensión de cada entorno: a 6,6 V la rodilla pasaba del
    # 90 % más del 5 % del tiempo (guía 4.2, results/training_torque_fase2_d_doble_push012_6.6V.json). Solo castiga el exceso.
    # Peso medido con fase2_d_doble model_7496 (etapa 2, empujones de 0,12): el castigo vale el 7,5 % de la recompensa por paso.
    cfg.rewards["servo_torque_excess"] = RewardTermCfg(
        func=danny_mdp.servo_torque_excess,
        weight=-317.0,
        params={"threshold": 0.8, "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*knee.*", r".*ankle_pitch.*"))},
    )
    # Contacto en contrafase según el reloj de marcha, solo con orden de movimiento (ver gait_clock y mdp).
    cfg.rewards["gait_contact_phase"] = RewardTermCfg(
        func=danny_mdp.gait_contact_phase, weight=1.5, params={"sensor_name": "feet_ground_contact"}
    )

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}

    return cfg
