"""Visualize the right arm driven by a GY-87 sensor over ESP-NOW.

Loads the per-node mounting calibration that visualize_box_gy87.py wrote
(calibrations/box_calib_node<N>.json), runs the same accel+gyro complementary
filter, and drives the HumanModel's right shoulder from the box's rotation
delta since you anchored the rest pose.

Workflow:
    1. First time per sensor: run visualize_box_gy87.py and learn all 6 faces.
       That writes the mounting file this tool needs.
    2. Strap/tape the sensor box to your upper arm.
    3. Launch this tool. It auto-loads the mounting for the sensor whose
       '# node X online' line appears first.
    4. Hold your arm in the HumanModel's rest pose (hanging straight down,
       palm toward thigh). Press 'c' once still — that captures the current
       orientation as the shoulder's "zero".
    5. Move your arm; the model follows.

Keys:
    c   : capture current pose as rest reference (shoulder zero)
    t   : capture current pose as "forward 90°" — tool solves for the yaw
          offset that makes your physical forward align with HumanModel +Y
    [ ] : nudge yaw correction by -/+ 5°  (manual fine-tune)
    { } : nudge yaw correction by -/+ 45°
    0   : zero yaw correction
    r   : clear rest + yaw AND delete the saved file for this node
    q   : quit

The rest reference and yaw correction are auto-saved to
calibrations/arm_rest_node<N>.json after every change, so you only need to
press c + t once per sensor+strap setup. Re-launches load it automatically.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import serial
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Stop matplotlib hijacking keys we repurpose for calibration.
mpl.rcParams["keymap.fullscreen"] = []   # was 'f'
mpl.rcParams["keymap.save"]       = []   # was 's'
mpl.rcParams["keymap.home"]       = ["h", "home"]      # drop 'r'
mpl.rcParams["keymap.back"]       = ["left", "backspace"]  # drop 'c'

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from human_model import HumanModel, SEGMENTS  # noqa: E402

CALIB_DIR    = os.path.abspath(os.path.join(HERE, "..", "calibrations"))
NODE_LINE_RE = re.compile(r"#\s*node\s+(\d+)", re.IGNORECASE)

# Must match the ordering in visualize_box_gy87.py's FACES list — these are
# the box-frame face normals, and the saved file names each face.
FACE_NAMES              = ["USB-C", "BOTTOM", "+X", "-X", "+Y", "-Y"]
WORLD_UP_IN_BOX_PER_FACE = np.array([
    [ 0,  0, -1], [ 0,  0, +1],
    [-1,  0,  0], [+1,  0,  0],
    [ 0, -1,  0], [ 0, +1,  0],
], dtype=float)


# ---------- Serial / pose ----------

@dataclass
class Pose:
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyr: np.ndarray = field(default_factory=lambda: np.zeros(3))  # deg/s
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set(self, a, g):
        with self.lock:
            self.acc, self.gyr = a, g

    def snapshot(self):
        with self.lock:
            return self.acc.copy(), self.gyr.copy()


def reader_thread(ser, pose, stop, data_ready=None, node_state=None):
    n_ok = 0
    while not stop.is_set():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception as e:
            print(f"[reader] serial error: {e}")
            time.sleep(0.05); continue
        if not line:
            continue
        if line.startswith("#"):
            print(f"[esp32] {line}")
            m = NODE_LINE_RE.search(line)
            if m and node_state is not None:
                node_state["id"] = int(m.group(1))
            continue
        parts = line.split(",")
        if len(parts) < 10:
            continue
        try:
            a = np.array([float(parts[4]), float(parts[5]), float(parts[6])])
            g = np.array([float(parts[7]), float(parts[8]), float(parts[9])])
        except ValueError:
            continue
        pose.set(a, g)
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


def _calib_path(node_id):
    nid = node_id if node_id is not None else 0
    return os.path.join(CALIB_DIR, f"box_calib_node{nid}.json")


def load_calibration(node_id):
    """Return {face_idx: accel_vec} loaded from disk, or empty dict."""
    path = _calib_path(node_id)
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception as e:
        print(f"[load] failed to read {path}: {e}")
        return {}
    name_to_idx = {name: i for i, name in enumerate(FACE_NAMES)}
    out = {}
    for name, vec in data.get("faces", {}).items():
        if name in name_to_idx:
            out[name_to_idx[name]] = np.array(vec, dtype=float)
    print(f"[loaded] {path}  ({len(out)}/6 faces, saved {data.get('saved_at')})")
    return out


def _arm_calib_path(node_id):
    nid = node_id if node_id is not None else 0
    return os.path.join(CALIB_DIR, f"arm_rest_node{nid}.json")


def save_arm_calibration(node_id, rest_W_T, yaw_rad):
    """Persist rest reference + yaw correction so future launches skip c/t."""
    if rest_W_T is None:
        return
    os.makedirs(CALIB_DIR, exist_ok=True)
    path = _arm_calib_path(node_id)
    data = {
        "node_id":  node_id,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "rest_W":   rest_W_T.T.tolist(),   # store W, not its transpose
        "yaw_rad":  float(yaw_rad),
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[arm saved] {path}  yaw={np.rad2deg(yaw_rad):+.1f}°")


def load_arm_calibration(node_id):
    """Return (rest_W_T, yaw_rad) or (None, 0.0) if no file."""
    path = _arm_calib_path(node_id)
    if not os.path.exists(path):
        return None, 0.0
    try:
        with open(path) as f:
            data = json.load(f)
    except Exception as e:
        print(f"[arm load] failed {path}: {e}")
        return None, 0.0
    rest_W = np.array(data["rest_W"], dtype=float)
    yaw_rad = float(data.get("yaw_rad", 0.0))
    print(f"[arm loaded] {path}  yaw={np.rad2deg(yaw_rad):+.1f}°  "
          f"(saved {data.get('saved_at')})")
    return rest_W.T, yaw_rad


def build_R_sb(face_accels):
    """Kabsch: find R such that R @ accel_sensor ≈ world_up_in_box per face."""
    pts_sensor, pts_box = [], []
    for i, a in face_accels.items():
        a = a / np.linalg.norm(a)
        pts_sensor.append(a)
        pts_box.append(WORLD_UP_IN_BOX_PER_FACE[i])
    if len(pts_sensor) < 2:
        return None
    S = np.array(pts_sensor); B = np.array(pts_box)
    H = S.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


class OrientationFilter:
    """Mahony-style accel+gyro filter (no mag), with online bias refinement.
    Identical semantics to the one in visualize_box_gy87.py."""
    def __init__(self, kp=2.0, bias_alpha=0.01):
        self.q          = np.array([1.0, 0.0, 0.0, 0.0])
        self.Kp         = kp
        self.bias_alpha = bias_alpha
        self.gyro_bias  = np.zeros(3)
        self.last_t     = None
        self.initialized = False

    def reset(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_t = None
        self.initialized = False

    def _init_from_accel(self, a_box):
        n = np.linalg.norm(a_box)
        if n < 1e-6:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
            self.initialized = True; return
        a = a_box / n
        cos_a = float(np.clip(a[2], -1.0, 1.0))
        if cos_a > 1 - 1e-9:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
        elif cos_a < -1 + 1e-9:
            self.q = np.array([0.0, 1.0, 0.0, 0.0])
        else:
            axis = np.array([a[1], -a[0], 0.0])
            axis /= np.linalg.norm(axis)
            half = np.arccos(cos_a) / 2.0
            s = np.sin(half)
            self.q = np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s])
        self.initialized = True

    def update(self, a_box, g_box_rad, t_now):
        if not self.initialized:
            self._init_from_accel(a_box); self.last_t = t_now; return
        if self.last_t is None:
            self.last_t = t_now; return
        dt = t_now - self.last_t
        self.last_t = t_now
        if dt <= 0 or dt > 0.2:
            return

        a_mag = np.linalg.norm(a_box)
        g_mag = np.linalg.norm(g_box_rad)
        if 0.9 < a_mag < 1.1 and g_mag < np.deg2rad(10.0):
            self.gyro_bias = (1 - self.bias_alpha) * self.gyro_bias \
                             + self.bias_alpha * g_box_rad
        gyro = g_box_rad - self.gyro_bias

        if a_mag > 1e-6:
            a = a_box / a_mag
            q0, q1, q2, q3 = self.q
            v = np.array([
                2.0 * (q1 * q3 - q0 * q2),
                2.0 * (q0 * q1 + q2 * q3),
                q0*q0 - q1*q1 - q2*q2 + q3*q3,
            ])
            err = np.cross(a, v)
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


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--node", type=int, default=None,
                    help="Override the node id used to pick the calibration file. "
                         "Default: whatever '# node X online' line arrives first.")
    ap.add_argument("--driven-joint", default="r_shoulder")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.5)
    ser.reset_input_buffer()

    pose = Pose()
    stop = threading.Event()
    data_ready = threading.Event()
    node_state = {"id": args.node}
    t = threading.Thread(target=reader_thread,
                         args=(ser, pose, stop, data_ready, node_state),
                         daemon=True)
    t.start()

    print("\nWaiting for sensor data...")
    if not data_ready.wait(timeout=20.0):
        print("ERROR: no data from receiver within 20s. Check port/wiring.")
        stop.set(); ser.close(); return
    time.sleep(0.3)

    face_cal = load_calibration(node_state["id"])
    R_sb = build_R_sb(face_cal) if face_cal else None
    if R_sb is None:
        print(f"ERROR: no 6-face mounting calibration for node {node_state['id']}.\n"
              f"Run first:  python tools/visualize_box_gy87.py --port {args.port}\n"
              f"and learn all six faces, then come back here.")
        stop.set(); ser.close(); return

    orient = OrientationFilter(kp=2.0)
    # Rest reference + yaw correction. The filter's world X/Y are arbitrary
    # (no magnetometer), so we rotate the delta by Rz(yaw) around world +Z
    # to align the filter's horizontal frame with HumanModel's +X=right/+Y=forward.
    rest_W_T = {"R": None, "yaw_rad": 0.0}

    # Autoload any previously saved rest + yaw for this node.
    loaded_R, loaded_yaw = load_arm_calibration(node_state["id"])
    if loaded_R is not None:
        rest_W_T["R"] = loaded_R
        rest_W_T["yaw_rad"] = loaded_yaw

    def _save():
        save_arm_calibration(node_state["id"],
                             rest_W_T["R"], rest_W_T["yaw_rad"])

    # Human model
    model = HumanModel()

    # ---- Figure ----
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0.0, 1.9)
    ax.set_xlabel("X (right)"); ax.set_ylabel("Y (forward)"); ax.set_zlabel("Z (up)")
    ax.set_box_aspect((1, 1, 1.4))
    ax.view_init(elev=10, azim=-70)
    title = ax.set_title("hold arm at rest, press 'c' to capture")
    fig.text(0.5, 0.02,
             "c=rest   t=fwd-90°   [ ]=yaw±5   { }=yaw±45   0=yaw0   r=clear   q=quit",
             ha="center", fontsize=9, color="#555")

    # Ground
    gx, gy = np.meshgrid(np.linspace(-0.7, 0.7, 5), np.linspace(-0.7, 0.7, 5))
    ax.plot_wireframe(gx, gy, np.zeros_like(gx), color="#dddddd", linewidth=0.4)

    seg_names = list(SEGMENTS.keys())
    seg_polys = {}
    for name, (faces, color) in zip(seg_names, model.get_segment_polygons()):
        pc = Poly3DCollection(faces, facecolors=color,
                              edgecolor="black", linewidth=0.3, alpha=0.9)
        ax.add_collection3d(pc)
        seg_polys[name] = pc

    # Head face indicators (static front/back cue in body frame).
    eyes_local = np.array([[-0.035, 0.096, 0.155], [0.035, 0.096, 0.155]])
    nose_local = np.array([[0.0, 0.096, 0.125], [0.0, 0.135, 0.115]])
    eyes_scatter = ax.scatter([0], [0], [0], c="black", s=40, zorder=10)
    (nose_line,) = ax.plot([0, 0], [0, 0], [0, 0], color="black", linewidth=2.5)

    def on_key(event):
        if event.key == "c":
            # Capture current filter state as the rest reference. From this
            # moment on, r_shoulder.joint_rot = Rz(yaw) · W_now · W_rest^T · Rz(-yaw)
            # — the strap orientation cancels, and Rz(yaw) aligns filter's
            # arbitrary horizontal frame with HM's +X=right / +Y=forward.
            W_now = orient.matrix()
            rest_W_T["R"] = W_now.T
            print("[rest captured]")
            _save()
        elif event.key == "t":
            if rest_W_T["R"] is None:
                print("[fwd] press 'c' at rest first"); return
            # Delta of a forward raise, in filter frame.
            delta = orient.matrix() @ rest_W_T["R"]
            angle = float(np.arccos(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)))
            if angle < np.deg2rad(30):
                print(f"[fwd] arm hasn't moved enough ({np.rad2deg(angle):.0f}°) — "
                      f"raise ~90° forward then press t"); return
            # Pull rotation axis out of the matrix.
            axis = np.array([delta[2, 1] - delta[1, 2],
                             delta[0, 2] - delta[2, 0],
                             delta[1, 0] - delta[0, 1]]) / (2.0 * np.sin(angle))
            ax_h, ay_h = axis[0], axis[1]
            horiz = float(np.hypot(ax_h, ay_h))
            if horiz < 0.5:
                print(f"[fwd] rotation axis isn't horizontal ({horiz:.2f}) — "
                      f"are you rotating around vertical by accident?"); return
            # In HM, forward-raise axis is +X. In filter, that same axis is
            # Rz(-yaw) · (1,0,0) = (cos yaw, -sin yaw, 0), so yaw = atan2(-ay, ax).
            yaw = float(np.arctan2(-ay_h / horiz, ax_h / horiz))
            rest_W_T["yaw_rad"] = yaw
            print(f"[fwd captured]  yaw correction = {np.rad2deg(yaw):+.1f}°  "
                  f"(delta {np.rad2deg(angle):.0f}°)")
            _save()
        elif event.key == "[":
            rest_W_T["yaw_rad"] -= np.deg2rad(5)
            print(f"[yaw] {np.rad2deg(rest_W_T['yaw_rad']):+.1f}°")
            _save()
        elif event.key == "]":
            rest_W_T["yaw_rad"] += np.deg2rad(5)
            print(f"[yaw] {np.rad2deg(rest_W_T['yaw_rad']):+.1f}°")
            _save()
        elif event.key == "{":
            rest_W_T["yaw_rad"] -= np.deg2rad(45)
            print(f"[yaw] {np.rad2deg(rest_W_T['yaw_rad']):+.1f}°")
            _save()
        elif event.key == "}":
            rest_W_T["yaw_rad"] += np.deg2rad(45)
            print(f"[yaw] {np.rad2deg(rest_W_T['yaw_rad']):+.1f}°")
            _save()
        elif event.key == "0":
            rest_W_T["yaw_rad"] = 0.0
            print("[yaw] 0°")
            _save()
        elif event.key == "r":
            rest_W_T["R"] = None
            rest_W_T["yaw_rad"] = 0.0
            path = _arm_calib_path(node_state["id"])
            if os.path.exists(path):
                os.remove(path)
                print(f"[deleted] {path}")
            print("[rest + yaw cleared]")
        elif event.key == "q":
            stop.set(); plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _: stop.set())

    plt.ion()
    plt.show()

    try:
        while not stop.is_set() and plt.fignum_exists(fig.number):
            a, g = pose.snapshot()
            if np.linalg.norm(a) < 1e-6:
                plt.pause(0.03); continue

            a_box     = R_sb @ a
            g_box_rad = (R_sb @ g) * (np.pi / 180.0)
            orient.update(a_box, g_box_rad, time.time())
            W_box = orient.matrix()

            if rest_W_T["R"] is not None:
                yaw = rest_W_T["yaw_rad"]
                cy, sy = np.cos(yaw), np.sin(yaw)
                Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
                delta = Rz @ W_box @ rest_W_T["R"] @ Rz.T
            else:
                delta = np.eye(3)
            model.joint_rot[args.driven_joint] = delta

            for name, (faces, _) in zip(seg_names, model.get_segment_polygons()):
                seg_polys[name].set_verts(faces)

            world = model.forward_kinematics()
            h_pos, h_rot = world["head_base"]
            eyes_world = (h_rot @ eyes_local.T).T + h_pos
            nose_world = (h_rot @ nose_local.T).T + h_pos
            eyes_scatter._offsets3d = (eyes_world[:, 0], eyes_world[:, 1], eyes_world[:, 2])
            nose_line.set_data_3d(nose_world[:, 0], nose_world[:, 1], nose_world[:, 2])

            status = "LIVE" if rest_W_T["R"] is not None else "WAITING (press c)"
            title.set_text(f"[{status}]  node={node_state['id']}  "
                           f"joint={args.driven_joint}  "
                           f"yaw={np.rad2deg(rest_W_T['yaw_rad']):+.0f}°")

            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try: ser.close()
        except Exception: pass


if __name__ == "__main__":
    main()
