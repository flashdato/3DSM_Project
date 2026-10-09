"""Visualize GY-87 sensor box orientation.

Live 3D render of the sensor box (45 x 35 x 65 mm) with the USB-C face
(smallest face) highlighted bright orange. Flipping / rotating the physical
sensor should rotate the on-screen box. The title calls out which face is
currently resting against the ground.

Boot sequence:
    1. HOLD THE BOX WITH USB-C FACE UP while the ESP32 does its 5-sec N-pose
       (visible as a '# CALIBRATING ...' line in the terminal).
    2. After that, flip or rotate the box onto any face. The model follows.

Usage:
    python tools/visualize_box_gy87.py --port /dev/ttyUSB0

Keyboard:
    1-6 : "face N is DOWN right now" — stores current accel as that face's
          reference AND logs detect-vs-truth. The classifier uses stored
          accels to decide the live face. Repeat to re-learn a face.
    s   : save current face calibration to disk (auto-saved too when 6/6)
    l   : reload face calibration from disk
    r   : reset all learned faces
    q   : quit

Calibrations are persisted per sensor node_id under ../calibrations/.
Loaded automatically at startup for the node that comes online first.

Why accel-only: the firmware quaternion depends on the magnetometer, which on
this GY-87 clone is noisy — holding the box still, the quaternion drifts around
even though gravity is stable. Accel alone gives us 2 DOF (pitch + roll), which
is all we need to tell which face is on the ground.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_DIR = os.path.abspath(os.path.join(HERE, "..", "calibrations"))
NODE_LINE_RE = re.compile(r"#\s*node\s+(\d+)", re.IGNORECASE)


# ---------- Box geometry (mm, world-aligned at rest with USB-C face up) ----------
# USB-C face is the smallest face => the 45 x 35 face. Choose it as +Z face so
# "USB-C up at calibration" means box's +Z aligns with world +Z (up).
# Long axis (65 mm) is Z.
HX = 45.0 / 2.0   # 22.5
HY = 35.0 / 2.0   # 17.5
HZ = 65.0 / 2.0   # 32.5

# (name, 4-vertex face in rest frame, color, is_usbc)
FACES = [
    ("USB-C",   np.array([[-HX, -HY, +HZ], [+HX, -HY, +HZ], [+HX, +HY, +HZ], [-HX, +HY, +HZ]]), "#ff5500", True),
    ("BOTTOM",  np.array([[-HX, -HY, -HZ], [-HX, +HY, -HZ], [+HX, +HY, -HZ], [+HX, -HY, -HZ]]), "#444444", False),
    ("+X",      np.array([[+HX, -HY, -HZ], [+HX, +HY, -HZ], [+HX, +HY, +HZ], [+HX, -HY, +HZ]]), "#88c8ff", False),
    ("-X",      np.array([[-HX, -HY, -HZ], [-HX, -HY, +HZ], [-HX, +HY, +HZ], [-HX, +HY, -HZ]]), "#4477aa", False),
    ("+Y",      np.array([[-HX, +HY, -HZ], [-HX, +HY, +HZ], [+HX, +HY, +HZ], [+HX, +HY, -HZ]]), "#88ff88", False),
    ("-Y",      np.array([[-HX, -HY, -HZ], [+HX, -HY, -HZ], [+HX, -HY, +HZ], [-HX, -HY, +HZ]]), "#44aa44", False),
]


# ---------- Serial / pose ----------

@dataclass
class Pose:
    q:   np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyr: np.ndarray = field(default_factory=lambda: np.zeros(3))  # deg/s
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set(self, q, a, g):
        with self.lock:
            self.q, self.acc, self.gyr = q, a, g

    def snapshot(self):
        with self.lock:
            return self.q.copy(), self.acc.copy(), self.gyr.copy()


def reader_thread(ser, pose, stop, data_ready=None, node_state=None):
    n_ok = 0
    while not stop.is_set():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception as e:
            print(f"[reader] serial error: {e}")
            time.sleep(0.05)
            continue
        if not line:
            continue
        if line.startswith("#"):
            print(f"[esp32] {line}")
            m = NODE_LINE_RE.search(line)
            if m and node_state is not None:
                node_state["id"] = int(m.group(1))
            continue
        parts = line.split(",")
        # Receiver v2 tags lines as "D,<node_id>,qw,qx,qy,qz,ax,...". Older
        # firmware emitted the quat straight away. Detect either shape.
        if parts[0] == "D" and len(parts) >= 12:
            try:
                nid = int(parts[1])
            except ValueError:
                continue
            if node_state is not None:
                node_state["id"] = nid
            fields = parts[2:]
        else:
            fields = parts
        if len(fields) < 10:
            continue
        try:
            q = np.array([float(fields[0]), float(fields[1]),
                          float(fields[2]), float(fields[3])])
            a = np.array([float(fields[4]), float(fields[5]), float(fields[6])])
            g = np.array([float(fields[7]), float(fields[8]), float(fields[9])])
        except ValueError:
            continue
        n = np.linalg.norm(q)
        if n < 1e-6:
            continue
        pose.set(q / n, a, g)
        n_ok += 1
        if n_ok == 1:
            print("[reader] first data line ok")
            if data_ready is not None:
                data_ready.set()


# ---------- Math ----------

def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


# When face i is physically down, world +Z expressed in BOX frame equals this
# vector (= -face_normal_in_box). The accel measurement at that pose equals
# R_sb^T @ (world +Z in box), so collecting 6 accels lets us solve for R_sb.
#   face 0 USB-C down -> +Z_box face down -> world +Z in box = (0, 0, -1)
#   face 1 BOTTOM down -> -Z_box face down -> world +Z in box = (0, 0, +1)
#   face 2 +X down     -> world +Z in box = (-1, 0, 0)
#   face 3 -X down     -> world +Z in box = (+1, 0, 0)
#   face 4 +Y down     -> world +Z in box = (0, -1, 0)
#   face 5 -Y down     -> world +Z in box = (0, +1, 0)
WORLD_UP_IN_BOX_PER_FACE = np.array([
    [ 0,  0, -1],
    [ 0,  0, +1],
    [-1,  0,  0],
    [+1,  0,  0],
    [ 0, -1,  0],
    [ 0, +1,  0],
], dtype=float)


def build_R_sb(face_accels):
    """Build sensor-to-box rotation from the learned accel-per-face dict.

    Returns a 3x3 proper rotation, or None if fewer than 2 non-opposite faces
    have been calibrated (not enough constraints).
    """
    pts_sensor, pts_box = [], []
    for i, a in enumerate(face_accels):
        if a is None: continue
        a = a / np.linalg.norm(a)
        pts_sensor.append(a)
        pts_box.append(WORLD_UP_IN_BOX_PER_FACE[i])
    if len(pts_sensor) < 2:
        return None
    S = np.array(pts_sensor)   # (N, 3), accels in sensor frame
    B = np.array(pts_box)      # (N, 3), expected world-up in box frame
    # Need R_sb such that R_sb^T @ (box vec) = sensor vec, equivalently
    # R_sb @ (sensor vec) = box vec. Minimize ||R_sb S^T - B^T||.
    # Kabsch: H = S^T B, SVD -> R_sb = V diag(1,1,sign) U^T.
    H = S.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R_sb = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R_sb


class OrientationFilter:
    """Mahony-style complementary filter without mag.

    Integrates gyro (box frame, rad/s) for smooth continuous rotation, and
    nudges toward the accel-measured gravity direction each step so pitch
    and roll stay anchored. Yaw drifts slowly over time but there is no
    singularity at the poles — the box can roll through "upside down"
    without flickering.

    State q is box->world (q0, q1, q2, q3) = (w, x, y, z).
    """
    def __init__(self, kp=2.0, bias_alpha=0.01):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.Kp = kp
        self.bias_alpha = bias_alpha           # per-sample bias LPF rate when still
        self.gyro_bias = np.zeros(3)           # rad/s, box frame
        self.last_t = None
        self.initialized = False

    def reset(self):
        # Keep gyro_bias — it's a sensor property, not an orientation one.
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_t = None
        self.initialized = False

    def _init_from_accel(self, a_box):
        n = np.linalg.norm(a_box)
        if n < 1e-6:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
            self.initialized = True
            return
        a = a_box / n
        # Shortest rotation taking a (world +Z in box) to (0, 0, 1) in world.
        cos_a = float(np.clip(a[2], -1.0, 1.0))
        if cos_a > 1 - 1e-9:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
        elif cos_a < -1 + 1e-9:
            self.q = np.array([0.0, 1.0, 0.0, 0.0])  # 180° around X
        else:
            axis = np.array([a[1], -a[0], 0.0])
            axis /= np.linalg.norm(axis)
            half = np.arccos(cos_a) / 2.0
            s = np.sin(half)
            self.q = np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s])
        self.initialized = True

    def update(self, a_box, g_box_rad, t_now):
        """a_box: measured gravity-opposing accel in BOX frame (any scale).
        g_box_rad: angular velocity in BOX frame, rad/s."""
        if not self.initialized:
            self._init_from_accel(a_box)
            self.last_t = t_now
            return
        if self.last_t is None:
            self.last_t = t_now
            return
        dt = t_now - self.last_t
        self.last_t = t_now
        if dt <= 0 or dt > 0.2:
            return

        # ---- Online bias refinement ----
        # If accel ≈ 1 g and raw gyro is small, we're stationary — slide
        # gyro_bias toward the raw reading. The accel ⨯ v correction below
        # fixes pitch/roll bias on its own, but yaw bias has no accel-visible
        # effect, so this is the only path that stops yaw drift.
        a_mag = np.linalg.norm(a_box)
        g_mag = np.linalg.norm(g_box_rad)
        if 0.9 < a_mag < 1.1 and g_mag < np.deg2rad(10.0):
            self.gyro_bias = (1 - self.bias_alpha) * self.gyro_bias \
                             + self.bias_alpha * g_box_rad

        gyro = g_box_rad - self.gyro_bias

        an = a_mag
        if an > 1e-6:
            a = a_box / an
            q0, q1, q2, q3 = self.q
            # Predicted "world +Z in box" from current q (= R_bw^T @ (0,0,1)).
            v = np.array([
                2.0 * (q1 * q3 - q0 * q2),
                2.0 * (q0 * q1 + q2 * q3),
                q0*q0 - q1*q1 - q2*q2 + q3*q3,
            ])
            err = np.cross(a, v)              # correction axis
            gyro = gyro + self.Kp * err

        q0, q1, q2, q3 = self.q
        gx, gy, gz = gyro
        qdot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx,
        ])
        self.q = self.q + qdot * dt
        self.q /= np.linalg.norm(self.q)

    def matrix(self):
        return quat_to_matrix(self.q)


def box_rotation_from_accel(a, R_sb):
    """Shortest-rotation 3x3 that tilts the box so its down-direction aligns
    with display -Z. Uses only accel + known mounting R_sb (no yaw)."""
    if R_sb is None or np.linalg.norm(a) < 1e-6:
        return np.eye(3)
    a_dir = a / np.linalg.norm(a)
    w_in_box = R_sb @ a_dir                 # world +Z expressed in box
    n = np.linalg.norm(w_in_box)
    if n < 1e-6: return np.eye(3)
    w_in_box /= n
    # Rotate box so that w_in_box (box's "up at rest") maps to display +Z.
    cos_a = float(np.clip(w_in_box[2], -1.0, 1.0))
    if cos_a > 1 - 1e-9: return np.eye(3)
    if cos_a < -1 + 1e-9: return np.diag([1.0, -1.0, -1.0])
    axis = np.array([w_in_box[1], -w_in_box[0], 0.0])
    axis /= np.linalg.norm(axis)
    angle = np.arccos(cos_a)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


# ---------- Persistent calibration (per sensor node_id) ----------

def _calib_path(node_id):
    nid = node_id if node_id is not None else 0
    return os.path.join(CALIB_DIR, f"box_calib_node{nid}.json")


def save_calibration(node_id, face_cal_accel):
    os.makedirs(CALIB_DIR, exist_ok=True)
    path = _calib_path(node_id)
    data = {
        "node_id":  node_id,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "faces":    {FACES[i][0]: a.tolist()
                     for i, a in enumerate(face_cal_accel) if a is not None},
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[saved] {path}  ({len(data['faces'])}/6 faces)")


def load_calibration(node_id, face_cal_accel):
    """Fill in face_cal_accel from disk if a file exists. Returns # loaded."""
    path = _calib_path(node_id)
    if not os.path.exists(path):
        print(f"[load] no saved calibration at {path}")
        return 0
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception as e:
        print(f"[load] failed to read {path}: {e}")
        return 0
    name_to_idx = {name: i for i, (name, _, _, _) in enumerate(FACES)}
    for name, vec in data.get("faces", {}).items():
        if name in name_to_idx:
            face_cal_accel[name_to_idx[name]] = np.array(vec, dtype=float)
    count = sum(1 for x in face_cal_accel if x is not None)
    print(f"[loaded] {path}  ({count}/6 faces, saved {data.get('saved_at')})")
    return count


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000,
                    help="Default matches esp32_receiver_now (1 Mbaud). "
                         "Use 115200 for the old wired esp32_visualize_gy87.")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.5)
    ser.reset_input_buffer()

    pose = Pose()
    stop = threading.Event()
    data_ready = threading.Event()
    node_state = {"id": None}
    t = threading.Thread(target=reader_thread,
                         args=(ser, pose, stop, data_ready, node_state),
                         daemon=True)
    t.start()

    print("\n*** HOLD THE BOX WITH USB-C FACE POINTING UP ***")
    print("ESP32 is doing a 5-sec N-pose calibration (plus 2s warmup).")
    print("Keep it still with USB-C up until you see 'first data line ok'.\n")

    if not data_ready.wait(timeout=25.0):
        print("ERROR: no data from ESP32 within 25s. Check port/wiring.")
        stop.set(); ser.close(); return
    time.sleep(0.3)
    print("\nCalibration done. Flip/rotate the box; the model follows.\n")

    # ---- Figure ----
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    LIM = 55
    ax.set_xlim(-LIM, LIM); ax.set_ylim(-LIM, LIM); ax.set_zlim(-LIM, LIM)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z (up)")
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=18, azim=-60)
    title = ax.set_title("box orientation — flip me", fontsize=11)
    fig.text(0.5, 0.02,
             "USB-C held UP at calibration. Press 1-6 when placing a face down to log detect vs. truth.",
             ha="center", fontsize=9, color="#555")
    fig.text(0.5, 0.055,
             "Note: with no magnetometer, rotation of the physical box around world +Z "
             "cannot be seen by the sensor. The rendered box's yaw is a fixed display "
             "convention — only the DOWN face tracks reality. R_sb is unaffected.",
             ha="center", fontsize=8, color="#888", style="italic")

    # Static legend top-left — colored labels, don't rotate with box.
    legend_entries = [(f"[{i+1}] {name}", color)
                      for i, (name, _, color, _) in enumerate(FACES)]
    for i, (txt, color) in enumerate(legend_entries):
        fig.text(0.02, 0.95 - i * 0.035, "■ " + txt,
                 fontsize=10, color=color, weight="bold")

    # Ground wireframe at Z = -LIM
    gx, gy = np.meshgrid(np.linspace(-LIM, LIM, 5), np.linspace(-LIM, LIM, 5))
    ax.plot_wireframe(gx, gy, np.full_like(gx, -LIM),
                      color="#dddddd", linewidth=0.4)

    # Create face polygons + centers (in rest frame)
    face_polys = []
    face_centers_rest = []
    for name, verts, color, is_usbc in FACES:
        poly = Poly3DCollection([verts], facecolors=color,
                                edgecolor="black",
                                linewidth=(2.5 if is_usbc else 0.6),
                                alpha=0.9)
        ax.add_collection3d(poly)
        face_polys.append(poly)
        face_centers_rest.append(verts.mean(axis=0))
    face_centers_rest = np.array(face_centers_rest)

    KEY_TO_FACE = {str(i + 1): i for i in range(len(FACES))}
    log = {"total": 0, "match": 0}
    face_cal_accel = [None] * len(FACES)   # measured accel per face when down
    R_sb_state = {"R": None}                # current sensor-to-box rotation
    orient = OrientationFilter(kp=2.0)      # gyro+accel filter, mag-free

    # Try to load saved calibration for the sensor that came online first.
    # (node_state["id"] was populated by the "# node X online" line.)
    if load_calibration(node_state["id"], face_cal_accel) > 0:
        R_sb_state["R"] = build_R_sb(face_cal_accel)

    # Status panel on the right showing cal state + live detection.
    status_text = fig.text(0.98, 0.95, "", ha="right", va="top",
                           fontsize=9, family="monospace")

    def detect_face(a):
        """Classify current accel against the learned face accels.
        Returns (idx, cosine_similarity) or (None, 0) if nothing learned yet."""
        if np.linalg.norm(a) < 1e-6: return None, 0.0
        a_dir = a / np.linalg.norm(a)
        best, best_sim = None, -2.0
        for i, cal in enumerate(face_cal_accel):
            if cal is None: continue
            cal_dir = cal / np.linalg.norm(cal)
            sim = float(np.dot(a_dir, cal_dir))
            if sim > best_sim:
                best, best_sim = i, sim
        return best, best_sim

    def on_key(event):
        if event.key == "q":
            stop.set(); plt.close(fig); return
        if event.key == "r":
            for i in range(len(face_cal_accel)): face_cal_accel[i] = None
            R_sb_state["R"] = None
            orient.reset()
            print("[reset] cleared all learned faces + orientation filter")
            return
        if event.key == "s":
            save_calibration(node_state["id"], face_cal_accel)
            return
        if event.key == "l":
            for i in range(len(face_cal_accel)): face_cal_accel[i] = None
            load_calibration(node_state["id"], face_cal_accel)
            R_sb_state["R"] = build_R_sb(face_cal_accel)
            orient.reset()
            return
        if event.key in KEY_TO_FACE:
            truth_idx = KEY_TO_FACE[event.key]
            truth_name = FACES[truth_idx][0]
            q, a, _ = pose.snapshot()

            # Classify BEFORE storing this press, so match reflects what the
            # model knew up until now (not what it will know including this).
            detected_idx, sim = detect_face(a)
            detected_name = (FACES[detected_idx][0] if detected_idx is not None
                             else "(no learn)")
            match = (detected_idx is not None and detected_idx == truth_idx)
            log["total"] += 1
            if match: log["match"] += 1

            # Store the measurement, rebuild R_sb, reset the orientation
            # filter so it re-initializes against the new mounting.
            face_cal_accel[truth_idx] = a.copy()
            R_sb_state["R"] = build_R_sb(face_cal_accel)
            orient.reset()
            calibrated = sum(1 for x in face_cal_accel if x is not None)
            if calibrated == 6:
                save_calibration(node_state["id"], face_cal_accel)

            print(f"\n=== key [{event.key}] "
                  f"({log['match']}/{log['total']} correct so far,"
                  f" {calibrated}/6 faces learned) ===")
            print(f"  Truth    : {truth_name}  is DOWN")
            print(f"  Detected : {detected_name}  (cos sim {sim:+.3f})   "
                  f"-->  {'MATCH' if match else 'MISMATCH'}")
            print(f"  accel    = [{a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f}]  "
                  f"|a|={np.linalg.norm(a):.3f}")
            print(f"  q (fw)   = [{q[0]:+.3f}, {q[1]:+.3f}, "
                  f"{q[2]:+.3f}, {q[3]:+.3f}]  (ignored — mag noisy)")
            if R_sb_state["R"] is not None:
                with np.printoptions(precision=3, suppress=True):
                    print(f"  R_sb now =\n{R_sb_state['R']}")

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _: stop.set())

    plt.ion()
    plt.show()

    try:
        while not stop.is_set() and plt.fignum_exists(fig.number):
            _, a, g = pose.snapshot()
            now = time.time()

            # Accel-only rendering. A gyro-fed Mahony filter would let the box
            # rotate when you spin it around world +Z — but it also picks an
            # arbitrary yaw at init, so after a face-key press the box always
            # jumps to the filter's chosen yaw (not yours). We don't have a
            # magnetometer, so there's no absolute heading reference either.
            # Using accel alone gives a deterministic 2-DOF tilt from the
            # measured gravity direction, with a fixed canonical yaw. Same
            # physical face-down → same display, every time. R_sb's correctness
            # is unaffected (it only cares about per-face accel directions).
            W = box_rotation_from_accel(a, R_sb_state["R"])

            rotated_centers = (W @ face_centers_rest.T).T
            for i, (_, verts, _, _) in enumerate(FACES):
                rot_v = (W @ verts.T).T
                face_polys[i].set_verts([rot_v])

            detected_idx, sim = detect_face(a)
            if detected_idx is not None:
                bottom_name = FACES[detected_idx][0]
                bottom_tag = f"{bottom_name} (sim {sim:+.2f})"
            else:
                # Fall back to geometry when nothing has been learned yet.
                g_idx = int(np.argmin(rotated_centers[:, 2]))
                bottom_tag = f"{FACES[g_idx][0]} (geom)"

            a_norm = float(np.linalg.norm(a))
            title.set_text(f"BOTTOM: {bottom_tag}   |a|={a_norm:.3f} g")

            cal_lines = ["learned faces:"]
            for i, (name, _, _, _) in enumerate(FACES):
                mark = "x" if face_cal_accel[i] is not None else "."
                cal_lines.append(f"  [{i+1}] {mark} {name}")
            cal_lines.append("")
            cal_lines.append(f"R_sb {'ready' if R_sb_state['R'] is not None else 'needs >=2 faces'}")
            status_text.set_text("\n".join(cal_lines))

            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try: ser.close()
        except Exception: pass


if __name__ == "__main__":
    main()
