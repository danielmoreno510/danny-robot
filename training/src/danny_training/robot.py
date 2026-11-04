"""Robot Danny para mjlab: MJCF de entrenamiento, sensores y actuadores HD-1910.

El MJCF lo genera `analysis/export_training_model.py` y no se edita a mano. Aquí solo se añaden
los sensores de la IMU, que el robot real llevará en el torso (BNO085).
"""

from __future__ import annotations

import re
from pathlib import Path

import mujoco
import numpy as np
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from danny_training.actuator import FrictionDRBamActuatorCfg

REPO_ROOT: Path = Path(__file__).resolve().parents[3]
DANNY_XML: Path = REPO_ROOT / "cad" / "export" / "danny" / "danny_training.xml"
HD1910_JSON: Path = Path(__file__).resolve().parents[2] / "params" / "hd1910_m6.json"

assert DANNY_XML.exists(), f"No existe el MJCF de entrenamiento: {DANNY_XML}"
assert HD1910_JSON.exists(), f"No existen los parámetros HD-1910: {HD1910_JSON}"

# Sitio de la IMU del MJCF, en torso_1, donde irá el BNO085 (auditoría del 2 de noviembre).
IMU_SITE = "imu"
TRUNK_BODY = "base_link"

# Colores de las mallas visuales (grupo 2) para el visor; no cambian la física. Del primero que encaje: cabeza y tronco plateados,
# extremidades claras, pies negros y el resto (articulaciones, pelvis, cuello) en grafito.
VISUAL_GEOM_GROUP = 2
VISUAL_COLORS = (
    (r"^(head|torso)_1_geom$", (0.86, 0.87, 0.90, 1.0)),
    (r"(thigh|shin|upper_arm|forearm)_1_geom$", (0.70, 0.72, 0.76, 1.0)),
    (r"foot_1_geom$", (0.10, 0.10, 0.11, 1.0)),
    (r".*", (0.22, 0.23, 0.26, 1.0)),
)

# Altura de los pies sobre el suelo al empezar, en metros.
SOLE_CLEARANCE = 0.001


def get_danny_spec() -> mujoco.MjSpec:
    """MJCF de Danny con sensores de IMU (velocidad lineal, velocidad angular, aceleración, orientación) y momento angular."""
    spec = mujoco.MjSpec.from_file(str(DANNY_XML))
    for geom in spec.geoms:
        if geom.group == VISUAL_GEOM_GROUP:
            geom.rgba[:] = next(color for pattern, color in VISUAL_COLORS if re.search(pattern, geom.name))
    spec.add_sensor(
        name="imu_lin_vel",
        type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=IMU_SITE,
    )
    spec.add_sensor(
        name="imu_ang_vel",
        type=mujoco.mjtSensor.mjSENS_GYRO,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=IMU_SITE,
    )
    spec.add_sensor(
        name="imu_accel",
        type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=IMU_SITE,
    )
    spec.add_sensor(
        name="root_angmom",
        type=mujoco.mjtSensor.mjSENS_SUBTREEANGMOM,
        objtype=mujoco.mjtObj.mjOBJ_BODY,
        objname=TRUNK_BODY,
    )
    spec.add_sensor(
        name="orientation",
        type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=IMU_SITE,
    )
    return spec


def _stand_pose() -> tuple[float, dict[str, float]]:
    """Altura de la base y ángulos de articulación de la pose `stand` del MJCF.

    La base se sube hasta que la suela más baja quede a SOLE_CLEARANCE del suelo.
    """
    model = get_danny_spec().compile()
    data = mujoco.MjData(model)
    key = model.key("stand")
    data.qpos[:] = key.qpos
    mujoco.mj_forward(model, data)

    # Las suelas son cajas: su punto más bajo está a la altura del centro menos la proyección de sus semiejes.
    sole_ids = [model.geom(name).id for name in ("left_sole", "right_sole")]
    lowest = min(
        data.geom_xpos[i][2] - (np.abs(data.geom_xmat[i].reshape(3, 3)) @ model.geom_size[i])[2]
        for i in sole_ids
    )
    base_z = float(key.qpos[2] + SOLE_CLEARANCE - lowest)

    joints = {
        model.joint(j).name: float(key.qpos[model.jnt_qposadr[j]])
        for j in range(1, model.njnt)
    }
    return base_z, joints


_STAND_Z, _STAND_JOINTS = _stand_pose()

DANNY_STAND_FRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, _STAND_Z),
    joint_pos={rf"^{name}$": angle for name, angle in _STAND_JOINTS.items()},
    joint_vel={".*": 0.0},
)

# Contactos: la suela y las cajas de colisión de cada eslabón tocan el suelo. Los eslabones no chocan entre sí por
# contype/conaffinity (conaffinity=0), pero los 13 `<pair>` del MJCF sí (pie con pie, brazo con pierna). Las cajas usan
# condim=3 y priority=1 como la suela: con prioridad 0 el suelo les pondría su fricción de 1,0 en lugar de la 0,35 del MJCF
# (guía 4.2).
DANNY_COLLISION = CollisionCfg(
    geom_names_expr=(r".*_box_\d+", r"^(left|right)_sole$"),
    contype=1,
    conaffinity=0,
    condim=3,
    priority=1,
    friction={r"^(left|right)_sole$": (1.0,)},
)

# Ganancia del firmware del servo. 5 es el valor de xgoduck_rl; sin política, con 5 Danny cae a un pie y con 10 aguanta a 7,4 V
# (results/standing_check.json). El valor real se mide en el banco (5.1).
KP_FW = 5.0

# Actuadores HD-1910 con el modelo BAM de xgoduck_rl (sin confirmar en los servos reales).
# Rangos de la guía de entrenamiento: tensión 6 a 8,4 V, retardo de 10 a 30 ms. Con paso de física
# de 5 ms, eso son 2 a 6 pasos.
DANNY_ACTUATORS = FrictionDRBamActuatorCfg(
    json_path=str(HD1910_JSON),
    target_names_expr=(r".*",),
    kp_fw=KP_FW,
    vin_range=(6.0, 8.4),
    vin_drop_gain_range=(0.0, 0.2),
    vin_min=6.0,
    delay_min_lag=2,
    delay_max_lag=6,
)

DANNY_ROBOT_CFG = EntityCfg(
    spec_fn=get_danny_spec,
    init_state=DANNY_STAND_FRAME,
    collisions=(DANNY_COLLISION,),
    articulation=EntityArticulationInfoCfg(
        actuators=(DANNY_ACTUATORS,),
        soft_joint_pos_limit_factor=0.9,
    ),
)
