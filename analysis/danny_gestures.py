"""
Danny - Gestos de pie adaptados de una persona (CMU, ASF/AMC), fase 6 (etapa 4.4).

Módulo compartido. Criterio aprobado por el dueño el 10 de noviembre (PROJECT_HANDOFF.md): piernas y pelvis fijas en la pose
`stand`; de la persona se copian brazos, cintura y cuello.

- Cintura (`waist_yaw`): la guiñada del pecho humano respecto a su pelvis, dentro del rango de Danny.
- Cuello (`neck_yaw`, `neck_pitch`): la guiñada y el cabeceo de la cabeza humana respecto a su pecho, dentro del rango.
- Brazos: el brazo y el antebrazo de Danny apuntan, respecto a su tronco, como los humanos respecto a su pecho, con las longitudes de
  Danny. Las cuatro articulaciones de cada brazo se resuelven con mínimos cuadrados amortiguados hacia el codo y la mano.

Ejes REP-103: X adelante, Y a la izquierda, Z arriba. CMU usa Y arriba, Z adelante y X a la izquierda.
"""

import pathlib

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import danny_sim

TRAINING_MODEL = danny_sim.MODEL.parent / "danny_training.xml"
CMU_TO_ROBOT = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
INCH = 0.0254
CMU_FRAME_TIME = 1.0 / 120.0
RATE_HZ = 50.0

# Velocidad en vacío del modelo de servo a 6,6 V (vin · 0,97 / kt, kt = 0,6238) y el 80 % que pide el criterio.
NO_LOAD_SPEED_RAD_S = 6.6 * 0.97 / 0.6238
SPEED_LIMIT_RAD_S = 0.8 * NO_LOAD_SPEED_RAD_S
AT_LIMIT_RAD = np.radians(2.0)

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")
IK_ITERATIONS = 30
# Si al terminar el brazo sigue tocando el cuerpo, se siguen haciendo iteraciones: el empuje y los objetivos de codo y mano tardan en
# ponerse de acuerdo (en 18_08 hacen falta unas 40). Los gestos de 141_16 aprobados nunca tocan, así que no les afecta.
IK_EXTRA_ITERATIONS = 90
IK_DAMPING = 0.02  # m: evita saltos cuando el brazo está estirado
POSTURE_WEIGHT = 1e-4  # empuja suave hacia la pose de pie; decide la guiñada del hombro con el brazo recto
# Ajustes por gesto (`--continuity` y `--body-margin`). Por defecto valen lo que se usó en los gestos de 141_16 aprobados por el dueño,
# que así se reproducen igual.
# Continuidad: empuja hacia la postura del fotograma anterior. Con el brazo levantado hay varias posturas que ponen codo y mano en el
# mismo sitio, y sin esto el cálculo puede saltar de una a otra (hasta 49,7 rad/s en el hombro de 143_25). 1e-3 pesa 0,03 m de error por
# radián de cambio. En 141_16 cambia hasta 10° un instante y sube la velocidad a 8,42 rad/s, por eso no se usa allí.
CONTINUITY_WEIGHT = 0.0
# El brazo se aparta del cuerpo: si una pieza del brazo queda a menos de este margen de una del cuerpo, se empuja hacia fuera.
BODY_MARGIN_M = 0.0005
BODY_PUSH_WEIGHT = 30.0

# Piezas que el brazo no puede atravesar: tronco, cuello, cabeza, pelvis y piernas. Se miden con las mallas del CAD (su envolvente
# convexa: si las envolventes no se tocan, las piezas tampoco). Las cajas de colisión simplificadas del entrenamiento sí se tocan en el
# hombro (2,6 mm en 141_16) sin que las piezas reales lo hagan.
BODY_PARTS = ("torso_1", "neck_link_1", "head_1", "base_link", "hip_yaw_link_1", "hip_roll_link_1", "thigh_1")
ARM_PARTS = ("upper_arm_1", "elbow_link_1", "forearm_1")


class CmuMotion:
    """Esqueleto ASF y movimiento AMC de CMU. Por hueso, la orientación global es la del padre por C · M · C⁻¹ (C, el eje del
    ASF; M, los grados del AMC, en ejes fijos x, y, z), y su final está en su inicio más esa orientación por dirección por largo.
    Guarda, en REP-103 y metros, la posición del final de cada hueso y la orientación global de cada uno."""

    def __init__(self, amc_path):
        amc_path = pathlib.Path(amc_path)
        asf = (amc_path.parent / (amc_path.stem.split("_")[0] + ".asf")).read_text().split("\n")
        bones, children, scale, section, current = {}, {}, 1.0, None, None
        for line in asf:
            words = line.split()
            if not words or words[0].startswith("#"):
                continue
            if words[0].startswith(":"):
                section = words[0]
                continue
            if section == ":units" and words[0] == "length":
                scale = INCH / float(words[1])
            elif section == ":bonedata":
                if words[0] == "begin":
                    current = {"dof": []}
                elif words[0] == "end":
                    bones[current["name"]] = current
                elif words[0] == "name":
                    current["name"] = words[1]
                elif words[0] == "direction":
                    current["direction"] = np.array([float(v) for v in words[1:4]])
                elif words[0] == "length":
                    current["length"] = float(words[1]) * scale
                elif words[0] == "axis":
                    current["axis"] = Rotation.from_euler(words[4].lower(), [float(v) for v in words[1:4]], degrees=True).as_matrix()
                elif words[0] == "dof":
                    current["dof"] = words[1:]
            elif section == ":hierarchy" and words[0] not in ("begin", "end"):
                children[words[0]] = words[1:]
        frames = []
        for line in amc_path.read_text().split("\n"):
            words = line.split()
            if not words or words[0].startswith(("#", ":")):
                continue
            if words[0].isdigit():
                frames.append({})
            else:
                frames[-1][words[0]] = np.array([float(v) for v in words[1:]])
        count = len(frames)
        root = np.array([f["root"] for f in frames])
        rotation = {"root": Rotation.from_euler("xyz", root[:, 3:6], degrees=True).as_matrix()}
        end = {"root": root[:, :3] * scale}
        order = ["root"]
        for name in order:
            for child in children.get(name, []):
                order.append(child)
                bone = bones[child]
                angles = np.zeros((count, 3))
                for k, dof in enumerate(bone["dof"]):
                    angles[:, "xyz".index(dof[1])] = [f[child][k] if child in f else 0.0 for f in frames]
                local = Rotation.from_euler("xyz", angles, degrees=True).as_matrix()
                rotation[child] = rotation[name] @ bone["axis"] @ local @ bone["axis"].T
                end[child] = end[name] + rotation[child] @ (bone["direction"] * bone["length"])
        self.frame_time = CMU_FRAME_TIME
        self.end = {b: end[b] @ CMU_TO_ROBOT.T for b in order}
        self.rotation = {b: CMU_TO_ROBOT @ rotation[b] @ CMU_TO_ROBOT.T for b in order}
        self.count = count


def gesture_model(xml_path=TRAINING_MODEL):
    """Modelo de Danny con un sitio en el hombro (`shoulder_roll`) y en el codo de cada brazo, en el ancla de su articulación."""
    base = mujoco.MjModel.from_xml_path(str(xml_path))
    spec = mujoco.MjSpec.from_file(str(xml_path))
    for side in ("left", "right"):
        for site, joint in (("shoulder_site", "shoulder_roll"), ("elbow_site", "elbow")):
            j = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{joint}")
            body = mujoco.mj_id2name(base, mujoco.mjtObj.mjOBJ_BODY, base.jnt_bodyid[j])
            spec.body(body).add_site(name=f"{side}_{site}", pos=base.jnt_pos[j].tolist())
    return spec.compile()


def name_id(model, kind, name):
    return mujoco.mj_name2id(model, kind, name)


def yaw_of(rotation):
    return np.arctan2(rotation[1, 0], rotation[0, 0])


def pelvis_yaw(motion):
    """Guiñada de la pelvis humana por la línea entre las dos caderas."""
    across = motion.end["lhipjoint"] - motion.end["rhipjoint"]
    return np.arctan2(-across[:, 0], across[:, 1])


class GestureRetarget:
    """Adapta los brazos, la cintura y el cuello, fotograma a fotograma, con las piernas y la pelvis en la pose de pie.

    Con `both_hands`, los dos brazos de Danny copian el brazo derecho de la persona, el izquierdo en espejo: así un gesto de una mano
    se hace con las dos."""

    def __init__(self, model, both_hands=False, continuity=CONTINUITY_WEIGHT, body_margin=BODY_MARGIN_M, speed_cap=None):
        """`speed_cap` (rad/s): si se da, ninguna articulación del brazo cambia más deprisa que eso entre fotogramas; donde la persona va
        más rápido, Danny se queda atrás y el error de codo y mano lo muestra. Necesita `frame_dt`, el tiempo entre fotogramas de salida."""
        self.model = model
        self.both_hands = both_hands
        self.continuity, self.body_margin, self.speed_cap = continuity, body_margin, speed_cap
        self.frame_dt = CMU_FRAME_TIME
        self.data = mujoco.MjData(model)
        key = name_id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        self.stand = model.key_qpos[key].copy()
        self.data.qpos[:] = self.stand
        mujoco.mj_forward(model, self.data)
        self.torso_stand = self.data.xmat[name_id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_1")].reshape(3, 3).copy()
        self.qadr, self.dofadr, self.limits, self.length = {}, {}, {}, {}
        for side in ("left", "right"):
            ids = [name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{j}") for j in ARM_JOINTS]
            self.qadr[side] = np.array([model.jnt_qposadr[j] for j in ids])
            self.dofadr[side] = np.array([model.jnt_dofadr[j] for j in ids])
            self.limits[side] = model.jnt_range[ids].copy()
            shoulder, elbow, hand = (self.site(f"{side}_{n}") for n in ("shoulder_site", "elbow_site", "hand"))
            self.length[side] = (np.linalg.norm(elbow - shoulder), np.linalg.norm(hand - elbow))
        self.arm_geoms = {side: [g for g in part_geoms(model, ARM_PARTS) if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]).startswith(side)]
                          for side in ("left", "right")}
        self.body_geoms = part_geoms(model, BODY_PARTS)
        self.waist = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, "waist_yaw")
        self.neck = [name_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ("neck_yaw", "neck_pitch")]

    def site(self, name):
        return self.data.site_xpos[name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def set_joint(self, joint, value):
        low, high = self.model.jnt_range[joint]
        self.data.qpos[self.model.jnt_qposadr[joint]] = np.clip(value, low, high)

    def frame(self, motion, frame, previous):
        """qpos del fotograma y objetivos de codo y mano por lado."""
        m, d = self.model, self.data
        d.qpos[:] = previous
        d.qpos[:7] = self.stand[:7]
        leg_and_base = slice(7, m.jnt_qposadr[self.waist])
        d.qpos[leg_and_base] = self.stand[leg_and_base]

        chest = motion.rotation["thorax"][frame]
        waist = yaw_of(chest) - pelvis_yaw(motion)[frame]
        self.set_joint(self.waist, (waist + np.pi) % (2 * np.pi) - np.pi)
        head = chest.T @ motion.rotation["head"][frame]
        yaw, pitch, _ = Rotation.from_matrix(head).as_euler("ZYX")
        self.set_joint(self.neck[0], yaw)
        self.set_joint(self.neck[1], pitch)
        mujoco.mj_forward(m, d)

        torso = d.xmat[name_id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_1")].reshape(3, 3) @ self.torso_stand.T
        targets = {}
        mirror = np.diag([1.0, -1.0, 1.0])
        for side, p in (("left", "l"), ("right", "r")):
            source = "r" if self.both_hands else p
            upper = motion.end[f"{source}humerus"][frame] - motion.end[f"{source}clavicle"][frame]
            fore = motion.end[f"{source}radius"][frame] - motion.end[f"{source}humerus"][frame]
            # Dirección respecto al pecho humano, llevada al tronco de Danny (en espejo si el brazo copia al del otro lado).
            flip = mirror if source != p else np.eye(3)
            upper = torso @ flip @ chest.T @ (upper / np.linalg.norm(upper))
            fore = torso @ flip @ chest.T @ (fore / np.linalg.norm(fore))
            elbow = self.site(f"{side}_shoulder_site") + upper * self.length[side][0]
            targets[side] = (elbow, elbow + fore * self.length[side][1])
            self.solve_arm(side, *targets[side])
        return d.qpos.copy(), targets

    def solve_arm(self, side, elbow_target, hand_target):
        m, d = self.model, self.data
        jac = np.zeros((3, m.nv))
        qadr, dofadr, (low, high) = self.qadr[side], self.dofadr[side], self.limits[side].T
        stand, previous = self.stand[qadr], d.qpos[qadr].copy()
        n = len(qadr)
        for iteration in range(IK_ITERATIONS + IK_EXTRA_ITERATIONS):
            if iteration >= IK_ITERATIONS and self.arm_clear(side):
                break
            rows, errors = [], []
            for name, target in ((f"{side}_elbow_site", elbow_target), (f"{side}_hand", hand_target)):
                mujoco.mj_jacSite(m, d, jac, None, name_id(m, mujoco.mjtObj.mjOBJ_SITE, name))
                rows.append(jac[:, dofadr].copy())
                errors.append(target - self.site(name))
            for row, error in self.body_push(side, dofadr):
                rows.append(row)
                errors.append(error)
            rows += [np.sqrt(POSTURE_WEIGHT) * np.eye(n), np.sqrt(self.continuity) * np.eye(n)]
            errors += [np.sqrt(POSTURE_WEIGHT) * (stand - d.qpos[qadr]), np.sqrt(self.continuity) * (previous - d.qpos[qadr])]
            J = np.vstack(rows)
            e = np.concatenate(errors)
            step = np.linalg.solve(J.T @ J + IK_DAMPING ** 2 * np.eye(len(qadr)), J.T @ e)
            d.qpos[qadr] = np.clip(d.qpos[qadr] + step, low, high)
            if self.speed_cap is not None:
                limit = self.speed_cap * self.frame_dt
                d.qpos[qadr] = previous + np.clip(d.qpos[qadr] - previous, -limit, limit)
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)  # mj_jacSite necesita los ejes de cada grado de libertad al día

    def arm_clear(self, side):
        """El brazo de ese lado no toca el cuerpo (distancia de 0 o más con las mallas)."""
        m, d = self.model, self.data
        fromto = np.zeros(6)
        return all(mujoco.mj_geomDistance(m, d, a, b, 0.01, fromto) >= 0 for a in self.arm_geoms[side] for b in self.body_geoms)

    def body_push(self, side, dofadr):
        """Filas de la cinemática inversa que apartan del cuerpo cada pieza del brazo que esté a menos del margen: el punto más
        cercano del brazo se mueve, por la normal entre las dos piezas, hasta quedar a ese margen."""
        m, d = self.model, self.data
        fromto, jac = np.zeros(6), np.zeros((3, m.nv))
        out = []
        for a in self.arm_geoms[side]:
            for b in self.body_geoms:
                distance = mujoco.mj_geomDistance(m, d, a, b, max(self.body_margin * 5, 0.005), fromto)
                if distance >= self.body_margin:
                    continue
                on_arm, on_body = fromto[:3].copy(), fromto[3:].copy()
                normal = on_arm - on_body
                if np.linalg.norm(normal) < 1e-9:
                    continue
                normal /= np.linalg.norm(normal)
                if distance < 0:
                    normal = -normal  # al atravesar, los dos puntos se cruzan
                mujoco.mj_jac(m, d, jac, None, on_arm, m.geom_bodyid[a])
                out.append((BODY_PUSH_WEIGHT * (normal @ jac[:, dofadr])[None, :], np.array([BODY_PUSH_WEIGHT * (self.body_margin - distance)])))
        return out

    def run(self, motion):
        """qpos a la frecuencia de CMU y objetivos de codo y mano por fotograma."""
        qpos, targets = [], []
        previous = self.stand.copy()
        for frame in range(motion.count):
            previous, frame_targets = self.frame(motion, frame, previous)
            qpos.append(previous)
            targets.append(frame_targets)
        return np.array(qpos), targets


def clip_name(amc_path, slowdown=1.0, both_hands=False, amplitude=1.0):
    """Nombre de los archivos de un clip: cmu_<clip>, con _dos_manos, _amplitud<factor> y _lento<factor> si aplican."""
    return (f"cmu_{pathlib.Path(amc_path).stem}" + ("_dos_manos" if both_hands else "") + ("" if amplitude == 1.0 else f"_amplitud{amplitude:g}")
            + ("" if slowdown == 1.0 else f"_lento{slowdown:g}"))


SHAKE_SMOOTHING_S = 0.1  # sigma del suavizado que separa la agitación rápida del movimiento lento (subir y bajar las manos)


def gaussian_smooth(values, sigma_frames):
    """Suaviza cada columna con una gaussiana de `sigma_frames` fotogramas, repitiendo los extremos."""
    offsets = np.arange(-int(4 * sigma_frames), int(4 * sigma_frames) + 1)
    kernel = np.exp(-0.5 * (offsets / sigma_frames) ** 2)
    kernel /= kernel.sum()
    padded = np.pad(values, ((len(offsets) // 2, len(offsets) // 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(padded[:, c], kernel, mode="valid") for c in range(values.shape[1])], axis=1)


def smooth_arms(qpos, model, sigma_s, frame_time):
    """Quita la vibración de brazos, cintura y cuello: el tope de velocidad hace que el giro del hombro choque con él en un sentido y en el
    otro en fotogramas seguidos (hasta 8° en 18_08). Las piernas y la pelvis no se tocan."""
    names = [f"{side}_{j}" for side in ("left", "right") for j in ARM_JOINTS] + ["waist_yaw", "neck_yaw", "neck_pitch"]
    columns = [model.jnt_qposadr[name_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
    out = qpos.copy()
    out[:, columns] = gaussian_smooth(qpos[:, columns], sigma_s / frame_time)
    return out


def shrink_arms(qpos, model, amplitude, frame_time):
    """Reduce la agitación rápida de los brazos (lo que se aparta de su versión suavizada) y deja igual el movimiento lento: las manos
    suben igual, pero se agitan menos, al mismo ritmo, y la velocidad de esa agitación baja en la misma proporción."""
    sigma = SHAKE_SMOOTHING_S / frame_time
    offsets = np.arange(-int(4 * sigma), int(4 * sigma) + 1)
    kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
    kernel /= kernel.sum()
    out = qpos.copy()
    for side in ("left", "right"):
        for joint in ARM_JOINTS:
            j = model.jnt_qposadr[name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{joint}")]
            padded = np.pad(qpos[:, j], len(offsets) // 2, mode="edge")
            slow = np.convolve(padded, kernel, mode="valid")
            low, high = model.jnt_range[name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{joint}")]
            out[:, j] = np.clip(slow + amplitude * (qpos[:, j] - slow), low, high)
    return out


def resample(qpos, source_dt, target_dt):
    """Remuestrea qpos (base libre más articulaciones): lineal en posiciones, esférica en la orientación de la base."""
    times = np.arange(len(qpos)) * source_dt
    new_times = np.arange(0.0, times[-1] + 1e-9, target_dt)
    out = np.empty((len(new_times), qpos.shape[1]))
    for column in list(range(3)) + list(range(7, qpos.shape[1])):
        out[:, column] = np.interp(new_times, times, qpos[:, column])
    out[:, 3:7] = Slerp(times, Rotation.from_quat(qpos[:, 3:7], scalar_first=True))(new_times).as_quat(scalar_first=True)
    return out


def part_geoms(model, parts):
    """Mallas del CAD de las piezas cuyo cuerpo termina en alguno de esos nombres."""
    out = []
    for g in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) or ""
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and any(body.endswith(p) for p in parts):
            out.append(g)
    return out


def arm_clearance(model, data):
    """Distancia mínima (m) entre las mallas del brazo y las del cuerpo; negativa si se atraviesan."""
    arms, bodies = part_geoms(model, ARM_PARTS), part_geoms(model, BODY_PARTS)
    fromto = np.zeros(6)
    return min(mujoco.mj_geomDistance(model, data, a, b, 0.05, fromto) for a in arms for b in bodies)
