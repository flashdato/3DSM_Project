"""Visualize the right arm driven by TWO GY-87 sensors over ESP-NOW.

Node mapping (fixed):
    node 1 = right forearm  → drives r_elbow
    node 2 = right upper arm → drives r_shoulder

Each node has its own 6-face mounting calibration (R_sb) saved by
visualize_box_gy87.py and its own rest + yaw saved by this tool.

Elbow handling: the forearm sensor measures a world-frame rotation
delta_forearm. In the HumanModel kinematic tree the forearm's world
rotation is r_shoulder.joint_rot @ r_elbow.joint_rot, so we set

    r_elbow.joint_rot = r_shoulder.joint_rot.T @ delta_forearm

so the forearm sensor's delta ends up expressed in the upper-arm frame.

Calibration flow (3 guided poses):

    1. Launch; both nodes must stream (receiver prints "# node 1/2 online").
       The viewer shows a GREEN translucent ghost of the next target pose.
    2. Match the ghost pose with your real right arm (elbow straight for
       every pose), press the numbered key, HOLD STILL for ~2 seconds.
         1 → REST     (arms hanging straight down)
         2 → FORWARD  (right arm straight out in front)
         3 → SIDE     (right arm straight out to the right)
       The first 0.5s of each capture is discarded so the Mahony filter has
       time to settle; the remaining ~1.5s is averaged per node.
    3. After all 3 poses are captured for both nodes, yaw is solved for
       each node from the mean of two independent axis alignments (fwd →
       +X, side → -Y). Rest + yaw are auto-saved to arm_rest_node<N>.json.
    4. Mode switches to LIVE; shoulder + elbow track the real arm.

Press a pose key again at any time to re-capture that pose.

Keys:
    1 / 2 / 3 : capture pose 1 / 2 / 3  (2-second hold with ghost preview)
    r         : clear all captures + delete saved files, restart calibration
    q         : quit
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import serial
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Stop matplotlib hijacking keys we repurpose.
mpl.rcParams["keymap.fullscreen"] = []
mpl.rcParams["keymap.save"]       = []
mpl.rcParams["keymap.home"]       = ["h", "home"]
mpl.rcParams["keymap.back"]       = ["left", "backspace"]

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from human_model import HumanModel, SEGMENTS  # noqa: E402

CALIB_DIR = os.path.abspath(os.path.join(HERE, "..", "calibrations"))

# Must match the ordering in visualize_box_gy87.py's FACES list.
FACE_NAMES              = ["USB-C", "BOTTOM", "+X", "-X", "+Y", "-Y"]
WORLD_UP_IN_BOX_PER_FACE = np.array([
    [ 0,  0, -1], [ 0,  0, +1],
    [-1,  0,  0], [+1,  0,  0],
    [ 0, -1,  0], [ 0, +1,  0],
], dtype=float)

NODE_IDS       = [1, 2]
JOINT_FOR_NODE = {1: "r_elbow",    2: "r_shoulder"}
LABEL_FOR_NODE = {1: "forearm",    2: "upper arm"}

CAPTURE_SECONDS   = 2.0
CAPTURE_WARMUP    = 0.5   # ignore first N seconds of each capture window

# Auto re-anchor at rest. Without a magnetometer, each sensor's filter yaw
# drifts independently and the difference manifests as the elbow curling.
# When the user returns the arm to rest, we detect it from accel direction
# alone (independent of filter drift) and reset W_0 + yaw so the drift
# accumulated since the last anchor is zeroed out.
REST_ACCEL_SIM        = 0.98   # cos(~11°)
REST_ACCEL_MAG_RANGE  = (0.95, 1.05)   # g
REST_GYRO_MAX_DPS     = 10.0           # per-sample ‖gyro‖, deg/s
REST_HOLD_SECONDS     = 0.5            # both sensors must stay at rest this long
REANCHOR_COOLDOWN     = 2.0            # min seconds between re-anchors


# ---------- Math ----------

def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def Rx(deg):
    r = np.deg2rad(deg); c, s = np.cos(r), np.sin(r)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def Ry(deg):
    r = np.deg2rad(deg); c, s = np.cos(r), np.sin(r)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Three calibration poses. All three use elbow extended so upper-arm and
# forearm share the same world rotation — simplifies the ghost rendering
# (we only need r_shoulder; r_elbow stays identity).
TARGET_POSES = [
    {"name": "REST",    "hint": "arms hanging straight down, palms toward thighs",
     "shoulder": np.eye(3)},
    {"name": "FORWARD", "hint": "right arm straight out IN FRONT, elbow extended",
     "shoulder": Rx(90)},
    {"name": "SIDE",    "hint": "right arm straight out to the RIGHT, elbow extended",
     "shoulder": Ry(-90)},
]

# Expected rotation axis of delta_i = W_i @ W_0.T, in HM world coords.
# For FORWARD (Rx(90°)) axis is +X; for SIDE (Ry(-90°)) axis is -Y.
TARGET_DELTA_AXIS = {
    1: np.array([ 1.0,  0.0, 0.0]),
    2: np.array([ 0.0, -1.0, 0.0]),
}


# ---------- Per-node calibration I/O ----------

def _box_path(nid): return os.path.join(CALIB_DIR, f"box_calib_node{nid}.json")
def _arm_path(nid): return os.path.join(CALIB_DIR, f"arm_rest_node{nid}.json")


def load_box_calibration(nid):
    path = _box_path(nid)
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


def build_R_sb(face_accels):
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


def save_arm_calibration(nid, rest_W_T, yaw_rad):
    if rest_W_T is None:
        return
    os.makedirs(CALIB_DIR, exist_ok=True)
    path = _arm_path(nid)
    data = {
        "node_id":  nid,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "rest_W":   rest_W_T.T.tolist(),
        "yaw_rad":  float(yaw_rad),
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[arm saved] node {nid}  yaw={np.rad2deg(yaw_rad):+.1f}°")


def load_arm_calibration(nid):
    path = _arm_path(nid)
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
    print(f"[arm loaded] node {nid}  yaw={np.rad2deg(yaw_rad):+.1f}°  "
          f"(saved {data.get('saved_at')})")
    return rest_W.T, yaw_rad


# ---------- Orientation filter ----------

class OrientationFilter:
    """Mahony-style accel+gyro filter (no mag), with online bias refinement."""
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
        stationary = 0.9 < a_mag < 1.1 and g_mag < np.deg2rad(10.0)
        if stationary:
            self.gyro_bias = (1 - self.bias_alpha) * self.gyro_bias \
                             + self.bias_alpha * g_box_rad
        gyro = g_box_rad - self.gyro_bias

        # Accel-based pitch/roll correction is only trustworthy when the
        # sensor is near-stationary: during motion the measured accel is
        # (gravity + linear accel), and feeding that to the filter as if
        # it were pure gravity perturbs the quaternion. Looser magnitude
        # gate than the bias update because small arm motions are OK.
        accel_valid = 0.85 < a_mag < 1.15
        if accel_valid and a_mag > 1e-6:
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


# ---------- Multi-pose yaw solver ----------

def _rotation_axis(delta):
    """Unit axis of rotation plus angle (rad). (None, angle) if axis is undefined."""
    cos_th = (np.trace(delta) - 1.0) / 2.0
    angle = float(np.arccos(np.clip(cos_th, -1.0, 1.0)))
    if angle < 1e-3:
        return None, angle
    axis = np.array([delta[2, 1] - delta[1, 2],
                     delta[0, 2] - delta[2, 0],
                     delta[1, 0] - delta[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return None, angle
    return axis / n, angle


def _average_rotations(Ws):
    """Mean of several SO(3) matrices, projected back to SO(3) via SVD."""
    S = np.mean(np.stack(Ws, axis=0), axis=0)
    U, _, Vt = np.linalg.svd(S)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def solve_yaw(W0, poses):
    """Given rest W0 and a dict {pose_idx: W_avg}, return yaw (rad) that
    rotates each delta's rotation axis onto its expected HM-frame axis.
    Averages two or more estimates via circular mean so neither pose
    dominates and small misalignments cancel out."""
    thetas = []
    for pi, Wi in poses.items():
        target = TARGET_DELTA_AXIS[pi]
        delta  = Wi @ W0.T
        axis, angle = _rotation_axis(delta)
        if axis is None:
            print(f"[solve] pose {pi+1}: rotation too small (angle={np.rad2deg(angle):.1f}°), skip")
            continue
        horiz = np.hypot(axis[0], axis[1])
        if horiz < 0.3:
            print(f"[solve] pose {pi+1}: axis nearly vertical (horiz={horiz:.2f}), skip")
            continue
        # Rotate axis in the XY plane onto target.
        theta = float(np.arctan2(target[1], target[0]) -
                      np.arctan2(axis[1],  axis[0]))
        thetas.append(theta)
        print(f"[solve] pose {pi+1}: axis=({axis[0]:+.2f},{axis[1]:+.2f},{axis[2]:+.2f})"
              f" angle={np.rad2deg(angle):5.1f}°  → yaw candidate {np.rad2deg(theta):+6.1f}°")
    if not thetas:
        return 0.0
    xs = np.mean([np.cos(t) for t in thetas])
    ys = np.mean([np.sin(t) for t in thetas])
    return float(np.arctan2(ys, xs))


# ---------- Per-node pose + reader ----------

@dataclass
class Pose:
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyr: np.ndarray = field(default_factory=lambda: np.zeros(3))
    has_data: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set(self, a, g):
        with self.lock:
            self.acc, self.gyr, self.has_data = a, g, True

    def snapshot(self):
        with self.lock:
            return self.acc.copy(), self.gyr.copy(), self.has_data


class NodeState:
    def __init__(self, nid):
        self.nid        = nid
        self.pose       = Pose()
        self.R_sb       = None
        self.orient     = OrientationFilter(kp=2.0)
        self.rest_W_T   = None
        self.yaw_rad    = 0.0
        # Auto-re-anchor bookkeeping.
        self.rest_match_since = None
        self.last_reanchor_t  = 0.0


def reader_thread(ser, nodes, stop, any_data):
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
            continue
        parts = line.split(",")
        if parts[0] != "D" or len(parts) < 12:
            continue
        try:
            nid = int(parts[1])
            a = np.array([float(parts[6]), float(parts[7]), float(parts[8])])
            g = np.array([float(parts[9]), float(parts[10]), float(parts[11])])
        except (ValueError, IndexError):
            continue
        node = nodes.get(nid)
        if node is None:
            continue
        node.pose.set(a, g)
        any_data.set()


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Per-node state + box calibration.
    nodes = {nid: NodeState(nid) for nid in NODE_IDS}
    for nid in NODE_IDS:
        face_cal = load_box_calibration(nid)
        R_sb = build_R_sb(face_cal) if face_cal else None
        if R_sb is None:
            print(f"ERROR: no 6-face mounting calibration for node {nid}.\n"
                  f"Run first:  python tools/visualize_box_gy87.py --port {args.port}\n"
                  f"with ONLY node {nid} powered, and learn all six faces.")
            ser.close(); return
        nodes[nid].R_sb = R_sb

        rest, yaw = load_arm_calibration(nid)
        nodes[nid].rest_W_T = rest
        nodes[nid].yaw_rad  = yaw

    stop     = threading.Event()
    any_data = threading.Event()
    t = threading.Thread(target=reader_thread,
                         args=(ser, nodes, stop, any_data), daemon=True)
    t.start()

    print("\nWaiting for sensor data from either node...")
    if not any_data.wait(timeout=20.0):
        print("ERROR: no data from receiver within 20s. Check port/wiring.")
        stop.set(); ser.close(); return
    time.sleep(0.3)

    model       = HumanModel()
    ghost_model = HumanModel()

    # Calibration state machine.
    cal = {
        "mode":           "live" if all(nodes[n].rest_W_T is not None
                                        for n in NODE_IDS) else "calib",
        "next_pose":      0,
        "capturing":      False,
        "capture_pose":   None,
        "capture_start":  0.0,
        "captures":       {},   # pose_idx -> {node_id: W_avg}
        "capture_bufs":   {},   # pose_idx -> {node_id: [W, W, ...]}
    }
    if cal["mode"] == "live":
        print("[LIVE] both nodes have saved calibrations; press 'r' to redo.")

    # ---- Figure ----
    fig = plt.figure(figsize=(9, 8))
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0.0, 1.9)
    ax.set_xlabel("X (right)"); ax.set_ylabel("Y (forward)"); ax.set_zlabel("Z (up)")
    ax.set_box_aspect((1, 1, 1.4))
    ax.view_init(elev=10, azim=-70)
    title = ax.set_title("…")
    fig.text(0.5, 0.02,
             "1 = REST   2 = FORWARD   3 = SIDE   (hold 2s)      r = redo   q = quit",
             ha="center", fontsize=9, color="#555")

    gx, gy = np.meshgrid(np.linspace(-0.7, 0.7, 5), np.linspace(-0.7, 0.7, 5))
    ax.plot_wireframe(gx, gy, np.zeros_like(gx), color="#dddddd", linewidth=0.4)

    seg_names = list(SEGMENTS.keys())
    seg_polys = {}
    for name, (faces, color) in zip(seg_names, model.get_segment_polygons()):
        pc = Poly3DCollection(faces, facecolors=color,
                              edgecolor="black", linewidth=0.3, alpha=0.9)
        ax.add_collection3d(pc)
        seg_polys[name] = pc

    # Ghost (target-pose) overlay — the two arm segments only, bright green.
    ghost_polys = {}
    for seg in ("r_upper_arm", "r_forearm"):
        pc = Poly3DCollection([], facecolors=(0.2, 0.95, 0.3, 0.35),
                              edgecolor="#009800", linewidth=2.5)
        ax.add_collection3d(pc)
        ghost_polys[seg] = pc

    eyes_local = np.array([[-0.035, 0.096, 0.155], [0.035, 0.096, 0.155]])
    nose_local = np.array([[0.0, 0.096, 0.125], [0.0, 0.135, 0.115]])
    eyes_scatter = ax.scatter([0], [0], [0], c="black", s=40, zorder=10)
    (nose_line,) = ax.plot([0, 0], [0, 0], [0, 0], color="black", linewidth=2.5)

    def set_ghost(pose_idx):
        """Draw the two arm segments at target pose, or hide if pose_idx is None."""
        if pose_idx is None:
            for pc in ghost_polys.values():
                pc.set_verts([])
            return
        ghost_model.reset_pose()
        ghost_model.joint_rot["r_shoulder"] = TARGET_POSES[pose_idx]["shoulder"]
        polys = dict(zip(seg_names, ghost_model.get_segment_polygons()))
        for seg in ghost_polys:
            faces, _ = polys[seg]
            ghost_polys[seg].set_verts(faces)

    def start_capture(pose_idx):
        if cal["mode"] == "live":
            # Entering a re-capture from live mode: fall back to calib.
            cal["mode"] = "calib"
        cal["capturing"]     = True
        cal["capture_pose"]  = pose_idx
        cal["capture_start"] = time.time()
        cal["capture_bufs"][pose_idx] = {nid: [] for nid in NODE_IDS}
        pose = TARGET_POSES[pose_idx]
        print(f"[capture] pose {pose_idx+1} ({pose['name']}): {pose['hint']} "
              f"— hold {CAPTURE_SECONDS:.1f}s")

    def finish_capture():
        pi = cal["capture_pose"]
        bufs = cal["capture_bufs"][pi]
        ok = True
        for nid in NODE_IDS:
            if len(bufs[nid]) < 10:
                print(f"[WARN] node {nid} pose {pi+1}: only {len(bufs[nid])} samples "
                      f"after warmup — try again")
                ok = False
        if ok:
            cal["captures"][pi] = {nid: _average_rotations(bufs[nid]) for nid in NODE_IDS}
            print(f"[capture] pose {pi+1} done ({len(bufs[NODE_IDS[0]])} samples per node)")
            # Advance next_pose to the lowest still-missing one.
            cal["next_pose"] = next((p for p in range(3) if p not in cal["captures"]), 3)
        cal["capturing"] = False
        cal["capture_pose"] = None
        # All three captured → solve.
        if cal["next_pose"] >= 3:
            for nid in NODE_IDS:
                W0 = cal["captures"][0][nid]
                others = {p: cal["captures"][p][nid] for p in (1, 2)}
                yaw = solve_yaw(W0, others)
                nodes[nid].rest_W_T = W0.T
                nodes[nid].yaw_rad  = yaw
                save_arm_calibration(nid, nodes[nid].rest_W_T, yaw)
                print(f"[SOLVED] node {nid} ({LABEL_FOR_NODE[nid]}): "
                      f"yaw = {np.rad2deg(yaw):+.1f}°")
            cal["mode"] = "live"
            print("[LIVE] both arms are now tracking.")

    def reset_calibration():
        cal["mode"]           = "calib"
        cal["next_pose"]      = 0
        cal["capturing"]      = False
        cal["capture_pose"]   = None
        cal["captures"].clear()
        cal["capture_bufs"].clear()
        for n in nodes.values():
            n.rest_W_T = None
            n.yaw_rad  = 0.0
            path = _arm_path(n.nid)
            if os.path.exists(path):
                os.remove(path)
                print(f"[deleted] {path}")
        print("[RESET] start over from pose 1.")

    def on_key(event):
        k = event.key
        if k in ("1", "2", "3"):
            if cal["capturing"]:
                print("[busy] still capturing a pose — wait for the timer")
                return
            start_capture(int(k) - 1)
        elif k == "r":
            reset_calibration()
        elif k == "q":
            stop.set(); plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _: stop.set())

    plt.ion()
    plt.show()

    def joint_delta(n):
        if n.rest_W_T is None or not n.pose.has_data:
            return np.eye(3)
        W = n.orient.matrix()
        cy, sy = np.cos(n.yaw_rad), np.sin(n.yaw_rad)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        return Rz @ W @ n.rest_W_T @ Rz.T

    def is_node_at_rest(n):
        """True iff this node is physically stationary AND its accel direction
        in box frame matches the stored rest reference. Independent of any
        accumulated filter yaw drift because it uses raw accel, not the
        filter quaternion."""
        if n.rest_W_T is None:
            return False
        a, g, has_data = n.pose.snapshot()
        if not has_data:
            return False
        a_mag = float(np.linalg.norm(a))
        if not (REST_ACCEL_MAG_RANGE[0] < a_mag < REST_ACCEL_MAG_RANGE[1]):
            return False
        if float(np.linalg.norm(g)) > REST_GYRO_MAX_DPS:
            return False
        a_box = n.R_sb @ a
        a_box /= np.linalg.norm(a_box)
        # Stored rest world-up direction expressed in box frame = W_0.T @ e_z
        # (which equals n.rest_W_T @ e_z, since rest_W_T stores W_0.T).
        rest_up_box = n.rest_W_T @ np.array([0.0, 0.0, 1.0])
        rest_up_box /= np.linalg.norm(rest_up_box)
        return float(np.dot(a_box, rest_up_box)) > REST_ACCEL_SIM

    def try_auto_reanchor(now):
        """If both nodes have been at rest for REST_HOLD_SECONDS and the
        cooldown has elapsed, re-anchor both. Returns True on re-anchor.

        For each node, extracting δθ (filter yaw drift since last anchor) from
        M = W_now @ W_0.T lets us update W_0 and yaw simultaneously so the
        joint delta formula keeps returning R_WL after the anchor."""
        both_at_rest = True
        for n in nodes.values():
            if is_node_at_rest(n):
                if n.rest_match_since is None:
                    n.rest_match_since = now
            else:
                n.rest_match_since = None
                both_at_rest = False
        if not both_at_rest:
            return False
        if min(now - n.rest_match_since for n in nodes.values()) < REST_HOLD_SECONDS:
            return False
        if max(n.last_reanchor_t for n in nodes.values()) + REANCHOR_COOLDOWN > now:
            return False
        for n in nodes.values():
            W_new = n.orient.matrix()
            W_old = n.rest_W_T.T          # stored as W_0.T → W_0 = (W_0.T).T
            M = W_new @ W_old.T           # ≈ Rz(-δθ) if drift is yaw-only
            dtheta = -float(np.arctan2(M[1, 0], M[0, 0]))
            n.rest_W_T = W_new.T
            n.yaw_rad += dtheta
            n.last_reanchor_t = now
            n.rest_match_since = None
            save_arm_calibration(n.nid, n.rest_W_T, n.yaw_rad)
        print(f"[auto-reanchor] drift zeroed   "
              f"n1 yaw={np.rad2deg(nodes[1].yaw_rad):+.1f}°   "
              f"n2 yaw={np.rad2deg(nodes[2].yaw_rad):+.1f}°")
        return True

    flash = {"until": 0.0}

    try:
        while not stop.is_set() and plt.fignum_exists(fig.number):
            now = time.time()

            # 1) Advance every node's filter from its latest sample.
            for n in nodes.values():
                a, g, has_data = n.pose.snapshot()
                if not has_data or np.linalg.norm(a) < 1e-6:
                    continue
                a_box     = n.R_sb @ a
                g_box_rad = (n.R_sb @ g) * (np.pi / 180.0)
                n.orient.update(a_box, g_box_rad, now)

            # 2) If capturing, accumulate W per node (after the warmup window).
            if cal["capturing"]:
                elapsed = now - cal["capture_start"]
                pi = cal["capture_pose"]
                if elapsed >= CAPTURE_WARMUP:
                    for n in nodes.values():
                        if n.orient.initialized and n.pose.has_data:
                            cal["capture_bufs"][pi][n.nid].append(n.orient.matrix().copy())
                if elapsed >= CAPTURE_SECONDS:
                    finish_capture()

            # 3) Pose the live model.
            if cal["mode"] == "live":
                if try_auto_reanchor(now):
                    flash["until"] = now + 1.5
                d_shoulder = joint_delta(nodes[2])
                d_forearm  = joint_delta(nodes[1])
                model.joint_rot["r_shoulder"] = d_shoulder
                model.joint_rot["r_elbow"]    = d_shoulder.T @ d_forearm
            else:
                model.reset_pose()

            # 4) Ghost: shows the pose currently being captured, or next-to-do.
            if cal["mode"] == "calib":
                set_ghost(cal["capture_pose"] if cal["capturing"] else cal["next_pose"])
            else:
                set_ghost(None)

            # 5) Update polygons + head markers.
            for name, (faces, _) in zip(seg_names, model.get_segment_polygons()):
                seg_polys[name].set_verts(faces)
            world = model.forward_kinematics()
            h_pos, h_rot = world["head_base"]
            eyes_world = (h_rot @ eyes_local.T).T + h_pos
            nose_world = (h_rot @ nose_local.T).T + h_pos
            eyes_scatter._offsets3d = (eyes_world[:, 0], eyes_world[:, 1], eyes_world[:, 2])
            nose_line.set_data_3d(nose_world[:, 0], nose_world[:, 1], nose_world[:, 2])

            # 6) Title / status.
            if cal["capturing"]:
                remaining = max(0.0, CAPTURE_SECONDS - (now - cal["capture_start"]))
                pose = TARGET_POSES[cal["capture_pose"]]
                title.set_text(f"CAPTURING pose {cal['capture_pose']+1} — {pose['name']}   "
                               f"{remaining:4.1f}s  —  HOLD STILL")
            elif cal["mode"] == "calib":
                done = sorted(cal["captures"].keys())
                if cal["next_pose"] < 3:
                    pose = TARGET_POSES[cal["next_pose"]]
                    done_str = ",".join(str(p+1) for p in done) if done else "none"
                    title.set_text(f"CALIB — press {cal['next_pose']+1} for {pose['name']}  "
                                   f"({pose['hint']})     done: {done_str}")
                else:
                    title.set_text("solving…")
            else:
                parts = []
                for nid in NODE_IDS:
                    n = nodes[nid]
                    live = "ok" if n.pose.has_data else "no-data"
                    atrest = "·rest" if n.rest_match_since is not None else ""
                    parts.append(f"n{nid}:{live}{atrest} yaw={np.rad2deg(n.yaw_rad):+.0f}°")
                prefix = "LIVE  [RE-ANCHORED]  " if now < flash["until"] else "LIVE   "
                title.set_text(prefix + "   ".join(parts))

            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try: ser.close()
        except Exception: pass


if __name__ == "__main__":
    main()
