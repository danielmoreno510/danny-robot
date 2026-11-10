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

    def __init__(self, amc_path, end_s=None):
        """`end_s`: corta el clip en ese segundo (por ejemplo, para quitar los pasos con que termina un baile)."""
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
        if end_s is not None:
            frames = frames[:int(round(end_s / CMU_FRAME_TIME)) + 1]
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

    def __init__(self, model, both_hands=False, continuity=CONTINUITY_WEIGHT, body_margin=BODY_MARGIN_M, speed_cap=None, limit_margin=0.0):
        """`speed_cap` (rad/s): si se da, ninguna articulación del brazo cambia más deprisa que eso entre fotogramas; donde la persona va
        más rápido, Danny se queda atrás y el error de codo y mano lo muestra. Necesita `frame_dt`, el tiempo entre fotogramas de salida.
        `limit_margin` (rad): el brazo se queda a esa distancia de sus topes, para no apoyar el servo en el tope mecánico; si la persona pide
        más, el error de codo y mano lo muestra."""
        self.model = model
        self.both_hands = both_hands
        self.continuity, self.body_margin, self.speed_cap = continuity, body_margin, speed_cap
        self.legs = None  # LegRetarget: si se da, mueve piernas y pelvis; si no, quedan en la pose `stand`
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
            self.limits[side] = model.jnt_range[ids].copy() + np.array([limit_margin, -limit_margin])
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
        if self.legs is not None:
            self.legs.frame_dt = self.frame_dt
            pose = d.qpos.copy()
            self.legs.pose(frame, pose)
            d.qpos[:] = pose

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


def clip_name(amc_path, slowdown=1.0, both_hands=False, amplitude=1.0, legs=False, end_s=None):
    """Nombre de los archivos de un clip: cmu_<clip>, con _hasta<s>, _piernas, _dos_manos, _amplitud<factor> y _lento<factor> si aplican."""
    return (f"cmu_{pathlib.Path(amc_path).stem}" + ("" if end_s is None else f"_hasta{end_s:g}") + ("_piernas" if legs else "") + ("_dos_manos" if both_hands else "")
            + ("" if amplitude == 1.0 else f"_amplitud{amplitude:g}") + ("" if slowdown == 1.0 else f"_lento{slowdown:g}"))


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
    new_times = np.minimum(np.arange(0.0, times[-1] + 1e-9, target_dt), times[-1])  # el redondeo no puede pasarse del final
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


# --- Piernas (criterio del 10 de noviembre): se copia lo que la persona cambia respecto a su postura de pie, desde la pose `stand`. ---

LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
REFERENCE_S = 0.3  # la postura de pie de la persona: la mediana de sus primeros 0,3 s
LEG_LIMIT_MARGIN_RAD = np.radians(3.0)
# Pie humano apoyado: dedos y tobillo cerca del suelo y casi quietos (estimación).
CONTACT_TOE_HEIGHT_M = 0.05
CONTACT_ANKLE_HEIGHT_M = 0.13
CONTACT_SPEED_M_S = 0.35
FOOT_CONTACT_M = 0.001  # en Danny, un pie toca el suelo si su suela queda a menos de esto


def human_leg_angles(motion):
    """Ángulos de la pierna humana en el orden de las articulaciones de Danny: guiñada, balanceo y cabeceo de cadera (fémur respecto a la
    pelvis, ejes z, x, y como la cadena de Danny), rodilla (tibia respecto al fémur, eje y) y cabeceo y balanceo del tobillo (pie respecto
    a la tibia, ejes y, x). Las orientaciones de CMU son la identidad de pie con las piernas rectas, así que los ángulos son de ese reposo."""
    def relative(parent, child):
        return np.swapaxes(motion.rotation[parent], 1, 2) @ motion.rotation[child]

    out = {}
    for side, p in (("left", "l"), ("right", "r")):
        hip = Rotation.from_matrix(relative("root", f"{p}femur")).as_euler("ZXY")
        knee = Rotation.from_matrix(relative(f"{p}femur", f"{p}tibia")).as_euler("YXZ")[:, 0]
        ankle = Rotation.from_matrix(relative(f"{p}tibia", f"{p}foot")).as_euler("YXZ")[:, :2]
        out[side] = np.column_stack([hip[:, 0], hip[:, 1], hip[:, 2], knee, ankle[:, 0], ankle[:, 1]])
    return out


def human_foot_contacts(motion):
    """Pie humano apoyado en cada fotograma, por lado."""
    floor = min(np.percentile(motion.end[f"{p}foot"][:, 2], 1) for p in "lr")
    out = {}
    for side, p in (("left", "l"), ("right", "r")):
        toe, ankle = motion.end[f"{p}foot"], motion.end[f"{p}tibia"]
        speed = np.linalg.norm(np.gradient(toe, motion.frame_time, axis=0), axis=1)
        ankle_floor = np.percentile(ankle[:, 2], 1)
        raw = (toe[:, 2] - floor < CONTACT_TOE_HEIGHT_M) & (ankle[:, 2] - ankle_floor < CONTACT_ANKLE_HEIGHT_M - 0.08) & (speed < CONTACT_SPEED_M_S)
        out[side] = debounce(raw, int(round(MIN_CONTACT_CHANGE_S / motion.frame_time)))
    return out


def walking_foot_contacts(motion):
    """Pie humano apoyado al andar: el talón (tobillo) o los dedos casi quietos. No se mira la altura.

    En 07_01 el suelo de la captura está inclinado: el pie derecho apoyado está a 1,0 cm de altura a los 0,2 s, a 4,1 cm a 1,2 s y a 6,7 cm a
    2,2 s (unos 2,7 cm por metro andado), así que con la altura solo salía apoyado el 20-31 % del tiempo. Al andar, el pie solo se para en el
    suelo (va a menos de 0,1 m/s apoyado y a 1-4 m/s en el aire). Se usa solo al andar, para no cambiar los bailes aprobados."""
    out = {}
    for side, p in (("left", "l"), ("right", "r")):
        still = np.zeros(motion.count, dtype=bool)
        for point in (motion.end[f"{p}foot"], motion.end[f"{p}tibia"]):
            still |= np.linalg.norm(np.gradient(point, motion.frame_time, axis=0), axis=1) < WALK_CONTACT_SPEED_M_S
        out[side] = debounce(still, int(round(MIN_CONTACT_CHANGE_S / motion.frame_time)))
    return out


WALK_CONTACT_SPEED_M_S = 0.4  # estimación
MIN_CONTACT_CHANGE_S = 0.1  # un apoyo o una levantada más corta es ruido de la captura (en 143_34, el pie derecho «se levantaba» 0,03 s)


def debounce(flags, frames):
    """Quita los tramos más cortos que `frames`, primero los de en el aire y luego los de apoyo."""
    out = flags.copy()
    for value in (False, True):
        start = None
        for i in range(len(out) + 1):
            inside = i < len(out) and out[i] == value
            if inside and start is None:
                start = i
            elif not inside and start is not None:
                if i - start < frames and start > 0 and i < len(out):
                    out[start:i] = not value
                start = None
    return out


def sole_geom(model, side):
    """La caja más fina del pie: la suela de 85 × 48 mm."""
    body = name_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot_1")
    boxes = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body and model.geom_contype[g]]
    return min(boxes, key=lambda g: model.geom_size[g][2])


def sole_corners(model, data, geom):
    """Las cuatro esquinas inferiores de la suela, en el mundo."""
    size, rot, pos = model.geom_size[geom], data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]
    return np.array([pos + rot @ np.array([sx * size[0], sy * size[1], -size[2]]) for sx in (-1, 1) for sy in (-1, 1)])


class LegRetarget:
    """Piernas y pelvis por fotograma: la pose `stand` más el cambio de la persona; la pelvis gira lo que gira la persona, se desplaza a
    escala, sube o baja para que el pie apoyado toque el suelo, y se corre lo justo para que un pie apoyado no resbale."""

    def __init__(self, model, motion, rest_reference=False):
        """`rest_reference`: la postura de pie de la persona es la de reposo del esqueleto de CMU (piernas rectas, ángulos a cero), no la de
        sus primeros 0,3 s. Hace falta al andar: el clip empieza a mitad de paso, con una rodilla doblada y la otra recta, y restar esa
        postura dejaba las rodillas de Danny 26° distintas (07_01)."""
        self.model, self.motion = model, motion
        self.data = mujoco.MjData(model)
        self.stand = model.key_qpos[name_id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")].copy()
        angles = human_leg_angles(motion)
        count = max(1, int(REFERENCE_S / motion.frame_time))
        self.delta = {}
        for side in ("left", "right"):
            if rest_reference:
                # Cabeceo de cadera y rodilla: el reposo de CMU (piernas rectas). Giro y balanceo de cadera y tobillo: la mediana con ese
                # pie apoyado, porque en el reposo de CMU el pie no está plano y el tobillo de Danny salía muy inclinado al andar.
                stance = walking_foot_contacts(motion)[side]
                reference = np.median(angles[side][stance], axis=0)
                reference[[LEG_JOINTS.index("hip_pitch"), LEG_JOINTS.index("knee")]] = 0.0
            else:
                reference = np.median(angles[side][:count], axis=0)
            self.delta[side] = (angles[side] - reference + np.pi) % (2 * np.pi) - np.pi
        self.root_reference = motion.rotation["root"][:count].mean(axis=0)
        u, _, vt = np.linalg.svd(self.root_reference)
        self.root_reference = u @ vt
        self.root_start = np.median(motion.end["root"][:count], axis=0)
        human_leg = (np.linalg.norm(motion.end["lfemur"][0] - motion.end["lhipjoint"][0])
                     + np.linalg.norm(motion.end["ltibia"][0] - motion.end["lfemur"][0]) + motion.end["ltibia"][0][2]
                     - min(motion.end["lfoot"][:, 2].min(), motion.end["rfoot"][:, 2].min()))
        self.scale = 0.200 / human_leg  # pierna de Danny estirada, cadera a suela (estimación de la etapa de andar)
        self.contacts = walking_foot_contacts(motion) if rest_reference else human_foot_contacts(motion)
        self.joints = {side: [name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{j}") for j in LEG_JOINTS] for side in ("left", "right")}
        self.qadr = {side: model.jnt_qposadr[self.joints[side]] for side in ("left", "right")}
        self.soles = {side: sole_geom(model, side) for side in ("left", "right")}
        self.anchor = {"left": None, "right": None}
        self.previous = None
        self.com_task = True  # al andar se quita: el centro de masas no va sobre el pie apoyado (se mide el ZMP)
        self.plan = self.plan_footsteps() if rest_reference else None
        self.com_plan = None  # al andar, el centro de masas planificado (`plan_com`), cuando se sepa el ritmo
        # Peso de la postura humana por articulación (en el orden de LEG_JOINTS, izquierda y derecha). Al andar no se copia el giro de
        # cadera: oscila a cada paso y en Danny llegaba a su tope (-15°/30°), y los pies van hacia delante (ver `solve`).
        self.posture_scale = np.tile([0.0 if (rest_reference and j == "hip_yaw") else 1.0 for j in LEG_JOINTS], 2)
        # Al andar, 0: la pelvis avanza lo que dan los pasos de Danny. Tirar de ella hacia la persona a escala, con los pies apoyados fijos,
        # estiraba la pierna más de lo que da (en 07_01 el pie se arrastraba hasta 13 cm y se hundía 2,7 cm).
        self.base_xy_weight = 0.0 if rest_reference else BASE_XY_WEIGHT
        # Tope de velocidad de las piernas, como el de los brazos; `frame_dt` lo pone GestureRetarget con su ritmo.
        self.speed_cap, self.frame_dt = 0.97 * SPEED_LIMIT_RAD_S, CMU_FRAME_TIME

    def plan_footsteps(self):
        """Pisadas planificadas al andar: dónde apoya cada pie y por dónde va en el aire, por fotograma de la persona.

        Cada apoyo de la persona (`walking_foot_contacts`) da una pisada: la mediana del punto medio entre tobillo y dedos, en la dirección
        de la marcha, a escala de Danny (pierna de Danny entre muslo y tibia de la persona más 8 cm de tobillo, estimación). La pisada se
        mide desde el primer apoyo, que se pone donde está el pie de Danny de pie. De lado, la separación de Danny de pie más lo que la
        persona se aparta de su media, a escala. Entre dos pisadas, el pie va en línea con transición suave y sube SWING_HEIGHT_M en
        medio, plano. Así el pie no se ancla donde esté al tocar el suelo (primer intento de andar: se arrastraba 7-13 cm)."""
        motion = self.motion
        root = motion.end["root"]
        heading = root[-1, :2] - root[0, :2]
        forward = heading / np.linalg.norm(heading)
        lateral = np.array([-forward[1], forward[0]])
        human_leg = (np.linalg.norm(motion.end["lfemur"][0] - motion.end["lhipjoint"][0])
                     + np.linalg.norm(motion.end["ltibia"][0] - motion.end["lfemur"][0]) + HUMAN_ANKLE_HEIGHT_M)
        scale = DANNY_LEG_M / human_leg
        self.scale = scale
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.stand
        mujoco.mj_kinematics(self.model, data)
        plan = {}
        for side, p in (("left", "l"), ("right", "r")):
            stand = data.site_xpos[name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_sole")].copy()
            point = (motion.end[f"{p}foot"] + motion.end[f"{p}tibia"])[:, :2] / 2 - root[0, :2]
            stance = self.contacts[side]
            segments, i = [], 0
            while i < len(stance):
                if stance[i]:
                    j = i
                    while j < len(stance) and stance[j]:
                        j += 1
                    middle = np.median(point[i:j], axis=0)
                    segments.append((i, j, middle @ forward, middle @ lateral))
                    i = j
                else:
                    i += 1
            plan[side] = {"segments": segments, "stand": stand, "mean_y": np.mean([seg[3] for seg in segments]) if segments else 0.0}
        start_x = min(seg[2] for side in plan for seg in plan[side]["segments"])
        targets = {}
        for side in ("left", "right"):
            info = plan[side]
            holds = [np.array([info["stand"][0] + scale * (x - start_x), info["stand"][1] + scale * (y - info["mean_y"]), 0.0])
                     for _, _, x, y in info["segments"]]
            positions, planted = np.zeros((motion.count, 3)), np.zeros(motion.count, dtype=bool)
            # Antes del primer apoyo, el pie ya está en su primera pisada: el clip empieza a mitad de paso.
            previous_end, previous_hold = 0, (holds[0] if holds else np.array([info["stand"][0], info["stand"][1], 0.0]))
            for (begin, end, _, _), hold in zip(info["segments"], holds):
                swing = max(begin - previous_end, 1)
                for f in range(previous_end, begin):
                    phase = (f - previous_end) / swing
                    smooth = phase * phase * (3 - 2 * phase)
                    positions[f] = previous_hold + smooth * (hold - previous_hold)
                    positions[f, 2] = SWING_HEIGHT_M * np.sin(np.pi * phase)
                positions[begin:end] = hold
                planted[begin:end] = True
                previous_end, previous_hold = end, hold
            positions[previous_end:] = previous_hold
            planted[previous_end:] = True
            targets[side] = (positions, planted)
        return targets

    def plan_com(self):
        """Centro de masas al andar: control predictivo del ZMP sobre las pisadas planificadas. La referencia del ZMP es el pie apoyado, o el
        punto medio con los dos, suavizada ZMP_TRANSITION_S. Sin esto, Danny dejaba el cuerpo entre los dos pies y con un pie apoyado el ZMP
        caía fuera de la huella (17-35 % dentro en 07_01 y 07_02, igual al ralentizar)."""
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.stand
        mujoco.mj_forward(self.model, data)
        com_height = data.subtree_com[name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")][2]
        reference = np.zeros((self.motion.count, 2))
        for f in range(self.motion.count):
            feet = [self.plan[side][0][f][:2] for side in ("left", "right") if self.plan[side][1][f]]
            reference[f] = np.mean(feet or [self.plan[side][0][f][:2] for side in ("left", "right")], axis=0)
        reference = gaussian_smooth(reference, ZMP_TRANSITION_S / self.frame_dt)
        # El centro de la suela está en el sitio; el ZMP se pide ahí.
        return preview_com(reference, com_height, self.frame_dt)

    def pose(self, frame, qpos):
        """Escribe en qpos la pelvis y las piernas del fotograma; devuelve qué pies de la persona están apoyados.

        Postura: la pose `stand` más el cambio de la persona, y la pelvis girada lo que gira la persona. Con otras longitudes que las
        humanas, los mismos ángulos moverían los pies (en 143_34 se separaban 2,5 cm al agacharse y se inclinaban); por eso una cinemática
        inversa de piernas y altura de pelvis deja cada pie apoyado fijo y plano donde se apoyó, y se aparta de la postura lo justo."""
        m, d = self.model, self.data
        posture = qpos.copy()
        for side in ("left", "right"):
            low, high = m.jnt_range[self.joints[side]].T
            posture[self.qadr[side]] = np.clip(self.stand[self.qadr[side]] + self.delta[side][frame], low + LEG_LIMIT_MARGIN_RAD, high - LEG_LIMIT_MARGIN_RAD)
        turn = self.motion.rotation["root"][frame] @ self.root_reference.T
        if self.plan is not None:
            # Al andar no se copia el giro de la pelvis (la persona la gira unos grados a cada paso): con los pies fijos, la guiñada de cadera
            # de Danny (de -15° a 30°) no da para compensarlo y era la articulación que más veces llegaba a su tope en 07_01.
            yaw = np.arctan2(turn[1, 0], turn[0, 0])
            turn = Rotation.from_euler("z", -yaw).as_matrix() @ turn
        base = Rotation.from_quat(self.stand[3:7], scalar_first=True).as_matrix()
        posture[3:7] = Rotation.from_matrix(turn @ base).as_quat(scalar_first=True)
        target_xy = self.stand[:2] + self.scale * (self.motion.end["root"][frame][:2] - self.root_start[:2])
        first = self.previous is None
        if first:
            self.previous = posture.copy()
            self.previous[:3] = self.stand[:3]
        qpos[:] = posture
        qpos[:3] = self.previous[:3]
        for side in ("left", "right"):
            qpos[self.qadr[side]] = self.previous[self.qadr[side]]
        d.qpos[:] = qpos
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        swing_targets = {}
        if self.plan is not None and self.com_plan is None:
            self.com_plan = self.plan_com()
        if self.plan is not None:
            planted = [side for side in ("left", "right") if self.plan[side][1][frame]]
            for side in ("left", "right"):
                self.anchor[side] = self.plan[side][0][frame] if side in planted else None
                if side not in planted:
                    swing_targets[side] = self.plan[side][0][frame]
        else:
            planted = [side for side in ("left", "right") if self.contacts[side][frame]]
            for side in ("left", "right"):
                if side not in planted:
                    self.anchor[side] = None
                elif self.anchor[side] is None:
                    self.anchor[side] = np.array([*self.site(f"{side}_sole")[:2], 0.0])
        # El primer fotograma empieza en la postura de la persona, sin tope de velocidad: un clip de andar empieza a mitad de paso, con un pie
        # 15 cm por delante del otro a escala, y desde la pose de pie no se puede llegar en un fotograma.
        self.frame_index = frame
        self.solve(planted, posture, target_xy, swing_targets, unlimited=first)
        qpos[:] = d.qpos
        self.previous = qpos.copy()
        return planted or ["left", "right"]

    def site(self, name):
        return self.data.site_xpos[name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()

    def foot_forward_rows(self, site, jacr, dofs, rows, errors):
        """Al andar, el pie apunta hacia delante (eje x del mundo): fila de la guiñada de la suela."""
        forward = self.data.site_xmat[site].reshape(3, 3)[:, 0]
        rows.append(FLAT_WEIGHT * jacr[2:3, dofs])
        errors.append(FLAT_WEIGHT * np.array([-np.arctan2(forward[1], forward[0])]))

    def solve(self, planted, posture, target_xy, swing_targets=None, unlimited=False):
        swing_targets = swing_targets or {}
        m, d = self.model, self.data
        dofs = np.concatenate([[0, 1, 2]] + [m.jnt_dofadr[self.joints[side]] for side in ("left", "right")])
        qaddr = np.concatenate([[0, 1, 2]] + [self.qadr[side] for side in ("left", "right")])
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        low = np.concatenate([[-np.inf] * 3] + [m.jnt_range[self.joints[side], 0] + LEG_LIMIT_MARGIN_RAD for side in ("left", "right")])
        high = np.concatenate([[np.inf] * 3] + [m.jnt_range[self.joints[side], 1] - LEG_LIMIT_MARGIN_RAD for side in ("left", "right")])
        start = d.qpos[qaddr].copy()
        cap = np.concatenate([[BASE_SPEED_M_S] * 3, [self.speed_cap] * (len(dofs) - 3)]) * self.frame_dt
        if unlimited:
            cap = np.full(len(dofs), np.inf)
        for _ in range(LEG_IK_ITERATIONS * (5 if unlimited else 1)):
            rows, errors = [], []
            for side in ("left", "right"):
                site = name_id(m, mujoco.mjtObj.mjOBJ_SITE, f"{side}_sole")
                mujoco.mj_jacSite(m, d, jacp, jacr, site)
                if side in swing_targets:
                    # Pie en el aire planificado: posición y plano.
                    rows.append(FOOT_WEIGHT * jacp[:, dofs])
                    errors.append(FOOT_WEIGHT * (swing_targets[side] - d.site_xpos[site]))
                    up = d.site_xmat[site].reshape(3, 3)[:, 2]
                    rows.append(FLAT_WEIGHT * jacr[:2, dofs])
                    errors.append(FLAT_WEIGHT * np.cross(up, [0.0, 0.0, 1.0])[:2])
                    self.foot_forward_rows(site, jacr, dofs, rows, errors)
                elif side in planted:
                    rows.append(FOOT_WEIGHT * jacp[:, dofs])
                    errors.append(FOOT_WEIGHT * (self.anchor[side] - d.site_xpos[site]))
                    up = d.site_xmat[site].reshape(3, 3)[:, 2]
                    rows.append(FLAT_WEIGHT * jacr[:2, dofs])
                    errors.append(FLAT_WEIGHT * np.cross(up, [0.0, 0.0, 1.0])[:2])
                    if self.plan is not None:
                        self.foot_forward_rows(site, jacr, dofs, rows, errors)
                else:
                    # Cada esquina de la suela del pie en el aire que baje de SWING_CLEARANCE_M se empuja hacia arriba: con el pie
                    # inclinado, la punta o el talón rozaban aunque el centro estuviera alto.
                    body = m.geom_bodyid[self.soles[side]]
                    for corner in sole_corners(m, d, self.soles[side]):
                        if corner[2] < SWING_CLEARANCE_M:
                            mujoco.mj_jac(m, d, jacp, None, corner, body)
                            rows.append(SWING_WEIGHT * jacp[2:3, dofs])
                            errors.append(SWING_WEIGHT * np.array([SWING_CLEARANCE_M - corner[2]]))
            # Equilibrio: el centro de masas dentro de la huella de los pies apoyados, con COM_MARGIN_M hacia dentro. Una persona pasa el
            # peso al pie apoyado sin pensarlo; Danny copiaba la postura sin hacerlo (93 % en 143_34, el criterio pide 95 %).
            support = np.vstack([sole_corners(m, d, self.soles[side])[:, :2] for side in (planted or ("left", "right"))])
            com = d.subtree_com[name_id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")][:2]
            goal = inside_support(support, com, COM_MARGIN_M) if self.com_task else None
            if self.com_plan is not None:
                goal = self.com_plan[self.frame_index]
            if goal is not None:
                jac_com = np.zeros((3, m.nv))
                mujoco.mj_jacSubtreeCom(m, d, jac_com, name_id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link"))
                weight = COM_WALK_WEIGHT if self.com_plan is not None else COM_WEIGHT
                rows.append(weight * jac_com[:2, dofs])
                errors.append(weight * (goal - com))
            rows.append(self.base_xy_weight * np.eye(len(dofs))[:2])
            errors.append(self.base_xy_weight * (target_xy - d.qpos[:2]))
            weights = LEG_POSTURE_WEIGHT * self.posture_scale
            rows.append(weights[:, None] * np.eye(len(dofs))[3:])
            errors.append(weights * (posture[qaddr[3:]] - d.qpos[qaddr[3:]]))
            J, e = np.vstack(rows), np.concatenate(errors)
            # Las articulaciones en su tope que el paso empujaría más allá se quitan y se vuelve a resolver: si no, el cálculo les seguía
            # pidiendo más y la pelvis no subía, y en el fondo de la sentadilla de 143_34 el pie apoyado se hundía (rodilla a 127° y tobillo
            # a -42°, el rango de Danny menos el margen).
            # Solo sale la que pasa su rango: la que llega al tope de velocidad se mueve lo máximo permitido (si también saliera, la pelvis,
            # que lo pasaba en la primera iteración, se quedaba congelada).
            free = np.ones(len(dofs), dtype=bool)
            for _ in range(3):
                Jf = J * free
                step = np.linalg.solve(Jf.T @ Jf + LEG_IK_DAMPING ** 2 * np.eye(len(dofs)), Jf.T @ e) * free
                stuck = free & (((d.qpos[qaddr] + step) < low) | ((d.qpos[qaddr] + step) > high))
                if not stuck.any():
                    break
                free &= ~stuck
            d.qpos[qaddr] = np.clip(np.clip(d.qpos[qaddr] + step, low, high), start - cap, start + cap)
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)


def inside_support(corners, point, margin):
    """Si `point` queda fuera de la huella (envolvente convexa de `corners`) encogida `margin`, el punto más cercano dentro; si no, None."""
    from scipy.spatial import ConvexHull

    hull = ConvexHull(corners)
    centre = corners[hull.vertices].mean(axis=0)
    polygon = centre + (corners[hull.vertices] - centre) * np.clip(1 - margin / np.maximum(np.linalg.norm(corners[hull.vertices] - centre, axis=1), 1e-9), 0, 1)[:, None]
    def turn(a, b):  # componente z del producto vectorial en el plano (NumPy 2 no lo admite con vectores de 2)
        return a[0] * b[1] - a[1] * b[0]

    inside = all(turn(polygon[(i + 1) % len(polygon)] - polygon[i], point - polygon[i]) >= 0 for i in range(len(polygon)))
    if inside:
        return None
    best, best_distance = None, np.inf
    for i in range(len(polygon)):
        a, b = polygon[i], polygon[(i + 1) % len(polygon)]
        t = np.clip(np.dot(point - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0, 1)
        candidate = a + t * (b - a)
        if np.linalg.norm(candidate - point) < best_distance:
            best, best_distance = candidate, np.linalg.norm(candidate - point)
    return best


def preview_com(zmp_reference, com_height, dt, preview_s=1.6, zmp_weight=1.0, jerk_weight=1e-6):
    """Centro de masas en el plano que lleva el ZMP del modelo de carro y mesa a `zmp_reference` (Kajita y otros, 2003: control
    predictivo con un estado posición-velocidad-aceleración y el tirón como entrada). Devuelve el centro de masas por fotograma."""
    from scipy.linalg import solve_discrete_are

    a = np.array([[1.0, dt, dt * dt / 2], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
    b = np.array([[dt ** 3 / 6], [dt * dt / 2], [dt]])
    c = np.array([[1.0, 0.0, -com_height / GRAVITY]])
    # Sistema aumentado con el error de ZMP integrado (la forma estándar del control predictivo).
    aa = np.block([[np.eye(1), c @ a], [np.zeros((3, 1)), a]])
    bb = np.vstack([c @ b, b])
    q = np.diag([zmp_weight, 0.0, 0.0, 0.0])
    r = np.array([[jerk_weight]])
    p = solve_discrete_are(aa, bb, q, r)
    k = np.linalg.solve(r + bb.T @ p @ bb, bb.T @ p @ aa)
    closed = aa - bb @ k
    steps = int(preview_s / dt)
    ki, kx = k[0, 0], k[0, 1:]
    # Ganancias de la referencia futura: Gd(1) = -Gi, Gd(j) = (R + B'PB)^-1 B' X(j-1), con X(1) = -Ac' P I y X(j) = Ac' X(j-1).
    gains, x = [-ki], -closed.T @ p @ np.vstack([np.eye(1), np.zeros((3, 1))])
    for _ in range(steps - 1):
        gains.append(np.linalg.solve(r + bb.T @ p @ bb, bb.T @ x).item())
        x = closed.T @ x
    count = len(zmp_reference)
    padded = np.vstack([zmp_reference, np.repeat(zmp_reference[-1:], steps, axis=0)])
    out = np.zeros((count, 2))
    for axis in range(2):
        state, error = np.array([zmp_reference[0, axis], 0.0, 0.0]), 0.0
        for i in range(count):
            out[i, axis] = state[0]
            error += (c @ state).item() - zmp_reference[i, axis]
            u = -ki * error - kx @ state - sum(g * padded[i + 1 + j, axis] for j, g in enumerate(gains))
            state = a @ state + b[:, 0] * u
    return out


HUMAN_ANKLE_HEIGHT_M = 0.08  # estimación
DANNY_LEG_M = 0.200  # cadera a suela con la pierna estirada (estimación de la etapa de andar)
ZMP_TRANSITION_S = 0.08  # suavizado de la referencia del ZMP al pasar de un pie a otro (estimación)
SWING_HEIGHT_M = 0.015  # altura del pie en el aire al andar (estimación; la marcha de la fase 2 sube 2 cm)
COM_MARGIN_M = 0.008  # el centro de masas, a 8 mm del borde de la huella (estimación)
COM_WEIGHT = 10.0
COM_WALK_WEIGHT = 1.0  # al andar, el pie apoyado manda: con 10, como el pie, el pie cedía y resbalaba 6-10 cm (07_01 y 07_02)
LEG_IK_ITERATIONS = 40
LEG_IK_DAMPING = 0.01
FOOT_WEIGHT = 10.0  # pie apoyado fijo donde se apoyó
FLAT_WEIGHT = 0.5  # y plano
SWING_WEIGHT = 30.0  # el pie en el aire no roza el suelo
SWING_CLEARANCE_M = 0.005  # el pie en el aire no baja de aquí (por encima de FOOT_CONTACT_M, para no contarlo como apoyado)
BASE_SPEED_M_S = 1.0  # solo evita saltos de la pelvis; la frenan las piernas, que tienen el tope del servo (con 0,3 m/s los pies se despegaban al levantarse en 143_34)
BASE_XY_WEIGHT = 0.3  # la pelvis sigue suave el desplazamiento de la persona, a escala
LEG_POSTURE_WEIGHT = 0.05  # la postura de la persona pesa 5 cm por radián frente a los pies


FOOT_SLIDE_M = 0.005  # punto 7 (estimación aprobada por el dueño)
FLOOR_TOLERANCE_M = 0.0005  # punto 8: el pie puede quedar como mucho medio milímetro bajo el suelo, por redondeo
BALANCED_FRACTION = 0.95  # punto 9


ZMP_SMOOTHING_S = 0.03  # suavizado del centro de masas antes de derivar dos veces (la derivada a 50 Hz amplifica el ruido)
GRAVITY = 9.81


def leg_criteria(model, qpos, frame_dt, walk=False):
    """Puntos 7 a 9 sobre la trayectoria entregada. Un pie de Danny está apoyado si una esquina de su suela queda a menos de FOOT_CONTACT_M.

    Con `walk`, el punto 9 usa el ZMP del modelo de carro y mesa, x_zmp = x_cdm - z_cdm / g · a_cdm, en lugar del centro de masas: al andar,
    una persona va cayendo hacia el paso siguiente."""
    from scipy.spatial import Delaunay

    data = mujoco.MjData(model)
    soles = {side: sole_geom(model, side) for side in ("left", "right")}
    others = [g for g in range(model.ngeom) if model.geom_contype[g] and g not in soles.values()
              and any((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) or "").endswith(p)
                      for p in ("shin_1", "ankle_link_1", "foot_1", "thigh_1"))]
    start = {"left": None, "right": None}
    slide, foot_low, leg_low, balanced, coms, supports = 0.0, np.inf, np.inf, [], [], []
    for q in qpos:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        support = []
        for side in ("left", "right"):
            corners = sole_corners(model, data, soles[side])
            foot_low = min(foot_low, corners[:, 2].min())
            if corners[:, 2].min() < FOOT_CONTACT_M:
                centre = corners[:, :2].mean(axis=0)
                start[side] = centre if start[side] is None else start[side]
                slide = max(slide, float(np.linalg.norm(centre - start[side])))
                support.extend(corners[:, :2])
            else:
                start[side] = None
        for g in others:
            size, rot, pos = model.geom_size[g], data.geom_xmat[g].reshape(3, 3), data.geom_xpos[g]
            leg_low = min(leg_low, min((pos + rot @ (np.array([sx, sy, sz]) * size))[2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)))
        com = data.subtree_com[name_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")].copy()
        coms.append(com)
        supports.append(np.array(support))
        balanced.append(len(support) >= 3 and Delaunay(np.array(support)).find_simplex(com[:2]) >= 0)
    if walk:
        coms = gaussian_smooth(np.array(coms), ZMP_SMOOTHING_S / frame_dt)
        acceleration = np.gradient(np.gradient(coms, frame_dt, axis=0), frame_dt, axis=0)
        zmp = coms[:, :2] - coms[:, 2:3] / GRAVITY * acceleration[:, :2]
        balanced = [len(s) >= 3 and Delaunay(s).find_simplex(z) >= 0 for s, z in zip(supports, zmp)]
    return {"slide": round(slide, 4), "foot_low": round(float(foot_low), 4), "leg_low": round(float(leg_low), 4),
            "balanced": round(float(np.mean(balanced)), 4)}
