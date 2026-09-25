"""Virtual IMU: produce the exact numbers our sensor nodes send, from model motion.

The body model is animated, each IMU node is rigidly attached to its segment,
and the ideal accelerometer and gyroscope signals are computed from the node's
motion, then passed through an MPU-6050 error model. The output is the same CSV
the real receiver produces (tools/log_serial.py format), so everything
downstream can't tell simulated from real data.

    python src/virtual_imu.py --move curl --seconds 12
    python src/virtual_imu.py --move all --seconds 24 --plot
    python src/virtual_imu.py --move wave --ideal          # no noise, no bias

Output folder (default recordings/sim_<move>_<time>/):
    imu_raw.csv   raw counts, same columns as a real recording
    truth.csv     ideal signals in g and deg/s + true sensor orientation + joint angles
    meta.json     node config, scales, error model, sensor mounting
    signals.png   (--plot) accel and gyro per node

MPU-6050 conventions (datasheet; Adafruit_MPU6050 and i2cdevlib read the same
14 registers): big-endian int16, sensitivity per full-scale range below, and
temp_C = raw / 340 + 36.53. At rest the accelerometer reads +1 g on the axis
pointing UP (it measures the reaction to gravity).
"""
import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from human_model import HumanModel, SEGMENTS
from animations import elbow_curl, arm_raise, side_raise, wave_arm, joint_angle_deg

G = 9.80665                                             # m/s^2
ACC_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYR_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

MOVES = {"curl": elbow_curl, "raise": arm_raise, "side": side_raise, "wave": wave_arm}

# Sensor mounting: GY-87 flat on the outside of the limb, components facing
# out, the board's X arrow pointing toward the hand. Columns = sensor x, y, z
# axes expressed in the segment frame (segment: +X outward, +Y forward, +Z up the limb).
R_MOUNT = np.column_stack([[0.0, 0.0, -1.0],   # x_S: along the limb toward the hand
                           [0.0, 1.0, 0.0],    # y_S: forward
                           [1.0, 0.0, 0.0]])   # z_S: out of the skin

NODES = [
    {"node_id": 1, "segment_id": 2, "segment": "r_upper_arm", "along": 0.5},
    {"node_id": 2, "segment_id": 3, "segment": "r_forearm", "along": 0.8},
]


# ---------------------------------------------------------------- kinematics
def sensor_poses(model, pose):
    """World position and rotation (R_WS) of every node for one body pose."""
    model.set_pose(pose)
    world = model.forward_kinematics()
    out = []
    for n in NODES:
        spec = SEGMENTS[n["segment"]]
        j_pos, j_rot = world[spec["joint"]]
        offset = np.array([spec["W"] / 2 + 0.013, 0.0, -(spec["inset"] + spec["L"] * n["along"])])
        out.append((j_pos + j_rot @ offset, j_rot @ R_MOUNT))
    return out


def rot_log(R):
    """Rotation matrix -> rotation vector (axis * angle, rad)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-7:
        return 0.5 * v
    return angle / (2.0 * np.sin(angle)) * v


def rot_to_quat(R):
    """Rotation matrix -> unit quaternion [w, x, y, z] (scalar first, Hamilton)."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def ideal_signals(model, fn, t, h=1e-3):
    """Ideal specific force (g) and angular rate (deg/s) in each sensor frame at time t.

    Central differences on the analytic animation: omega from R(t-h)^T R(t+h),
    acceleration from positions at t-h, t, t+h. Specific force f_S = R_SW (a_W - g_W).
    """
    before = sensor_poses(model, fn(t - h))
    now = sensor_poses(model, fn(t))
    after = sensor_poses(model, fn(t + h))
    g_w = np.array([0.0, 0.0, -G])
    out = []
    for (p0, R0), (p1, R1), (p2, R2) in zip(before, now, after):
        omega = rot_log(R0.T @ R2) / (2 * h)                  # rad/s, sensor frame
        a_w = (p2 - 2 * p1 + p0) / (h * h)                    # m/s^2, world frame
        f_s = R1.T @ (a_w - g_w)                              # m/s^2, sensor frame
        out.append((f_s / G, np.degrees(omega), R1))
    return out


# ---------------------------------------------------------------- error model
def make_errors(rng, ideal):
    """Per-node MPU-6050 error parameters.

    Placeholders until the v0.3 15-minute static recording gives our boards' real
    numbers. Noise from the datasheet noise densities (gyro 0.005 deg/s/sqrtHz,
    accel 400 ug/sqrtHz) at the 42 Hz DLPF, rounded up.
    """
    if ideal:
        z = np.zeros(3)
        return [{"gyro_bias_dps": z, "acc_bias_g": z, "gyro_noise_dps": 0.0,
                 "acc_noise_g": 0.0, "temp_c": 25.0} for _ in NODES]
    return [{
        "gyro_bias_dps": rng.normal(0.0, 1.0, 3),     # after power-up, uncalibrated
        "acc_bias_g": rng.normal(0.0, 0.02, 3),
        "gyro_noise_dps": 0.05,                       # RMS per sample
        "acc_noise_g": 0.004,
        "temp_c": 25.0 + rng.uniform(-2, 2),
    } for _ in NODES]


def to_counts(value, lsb):
    return np.clip(np.rint(value * lsb), -32768, 32767).astype(int)


# ---------------------------------------------------------------- main
def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--move", default="curl", choices=list(MOVES) + ["all"],
                    help="movement to simulate; 'all' plays curl, raise, side, wave in turn")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--hz", type=int, default=100)
    ap.add_argument("--batch", type=int, default=2, help="samples per ESP-NOW packet")
    ap.add_argument("--acc-fs", type=int, default=8, choices=sorted(ACC_LSB_PER_G))
    ap.add_argument("--gyr-fs", type=int, default=1000, choices=sorted(GYR_LSB_PER_DPS))
    ap.add_argument("--loss", type=float, default=0.0, help="probability a packet is lost (0..1)")
    ap.add_argument("--ideal", action="store_true", help="no bias, no noise (quantization only)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=None, help="output folder (default recordings/sim_<move>_<time>)")
    ap.add_argument("--plot", action="store_true", help="also save signals.png")
    return ap.parse_args()


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    model = HumanModel()
    acc_lsb, gyr_lsb = ACC_LSB_PER_G[args.acc_fs], GYR_LSB_PER_DPS[args.gyr_fs]
    errors = make_errors(rng, args.ideal)

    if args.move == "all":
        seq_moves = ["curl", "raise", "side", "wave"]
        per = args.seconds / len(seq_moves)
        def movement_at(t):
            i = min(int(t // per), len(seq_moves) - 1)
            return MOVES[seq_moves[i]], t - i * per
    else:
        def movement_at(t):
            return MOVES[args.move], t

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.out or f"recordings/sim_{args.move}_{stamp}")
    out.mkdir(parents=True, exist_ok=True)

    n_samples = int(round(args.seconds * args.hz))
    t0_us = 5_000_000                                   # receiver clock at start
    host0 = time.time()
    temp_raw = [int(round((e["temp_c"] - 36.53) * 340)) for e in errors]

    raw_rows, truth_rows = [], []
    dropped = 0
    for k0 in range(0, n_samples, args.batch):
        batch = range(k0, min(k0 + args.batch, n_samples))
        batch_rows = []
        for k in batch:
            t = k / args.hz
            fn, local_t = movement_at(t)
            sig = ideal_signals(model, fn, local_t)
            model.set_pose(fn(local_t))
            elbow = joint_angle_deg(model.joint_rot["r_elbow"])
            shoulder = joint_angle_deg(model.joint_rot["r_shoulder"])
            t_us = t0_us + int(round(t * 1e6)) + int(rng.normal(0, 20)) * (not args.ideal)
            for n, e, tr, (acc_g, gyr_dps, R) in zip(NODES, errors, temp_raw, sig):
                acc_meas = acc_g + e["acc_bias_g"] + rng.normal(0, 1, 3) * e["acc_noise_g"]
                gyr_meas = gyr_dps + e["gyro_bias_dps"] + rng.normal(0, 1, 3) * e["gyro_noise_dps"]
                a, g = to_counts(acc_meas, acc_lsb), to_counts(gyr_meas, gyr_lsb)
                temp = tr + (0 if args.ideal else int(rng.integers(-3, 4)))
                batch_rows.append([n["node_id"], n["segment_id"], k % 65536, t_us,
                                   *a, *g, 0, 0, 0, temp, 1, 3900, -45])
                q = rot_to_quat(R)
                truth_rows.append([n["node_id"], k, t_us, *np.round(acc_g, 6), *np.round(gyr_dps, 5),
                                   *np.round(q, 7), round(elbow, 3), round(shoulder, 3)])
        # one packet per node per batch, arriving ~2 ms after its last sample
        last_t = batch_rows[-1][3]
        for n in NODES:
            if rng.random() < args.loss:
                dropped += 1
                continue
            rx_us = last_t + 2000 + (0 if args.ideal else int(rng.uniform(0, 800)))
            for row in batch_rows:
                if row[0] == n["node_id"]:
                    raw_rows.append(row + [rx_us, f"{host0 + (rx_us - t0_us) / 1e6:.6f}"])

    with open(out / "imu_raw.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node", "segment", "seq", "t_us", "ax", "ay", "az", "gx", "gy", "gz",
                    "mx", "my", "mz", "temp", "flags", "batt_mv", "rssi", "rx_us", "host_time"])
        w.writerows(raw_rows)
    with open(out / "truth.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node", "sample", "t_us", "acc_x_g", "acc_y_g", "acc_z_g",
                    "gyr_x_dps", "gyr_y_dps", "gyr_z_dps", "qw", "qx", "qy", "qz",
                    "elbow_deg", "shoulder_deg"])
        w.writerows(truth_rows)

    meta = {
        "session": out.name, "simulated": True, "move": args.move, "seconds": args.seconds,
        "seed": args.seed, "ideal": args.ideal, "packet_loss": args.loss, "packets_dropped": dropped,
        "units": {"acc": f"counts; g = counts / {acc_lsb:g}",
                  "gyr": f"counts; deg/s = counts / {gyr_lsb:g}",
                  "temp": "counts; degC = counts / 340 + 36.53",
                  "t_us": "microseconds, receiver clock (flags & 1 = SYNCED)"},
        "mounting": "GY-87 on the outside of the limb, components out, board X arrow toward the hand",
        "R_segment_sensor": R_MOUNT.tolist(),
        "nodes": {str(n["node_id"]): {
            "segment_id": n["segment_id"], "segment": n["segment"], "sample_hz": args.hz,
            "acc_fs_g": args.acc_fs, "gyr_fs_dps": args.gyr_fs,
            "error_model": {k: (v.round(4).tolist() if isinstance(v, np.ndarray) else round(v, 4))
                            for k, v in e.items()}} for n, e in zip(NODES, errors)},
        "samples_written": len(raw_rows),
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))

    print(f"Simulated {args.seconds:g} s of '{args.move}' at {args.hz} Hz -> {out}")
    print(f"{len(raw_rows)} samples written, {dropped} packets dropped\n")
    print("First lines as the receiver would print them:")
    for row in raw_rows[:6]:
        print("D," + ",".join(str(x) for x in row[:-1]))

    if args.plot:
        plot_signals(out, raw_rows, acc_lsb, gyr_lsb, args.move)


def plot_signals(out, rows, acc_lsb, gyr_lsb, move):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"x": "#2a78d6", "y": "#eb6834", "z": "#1baf7a"}   # fixed categorical order
    surface, ink, muted, grid = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    data = np.array([[float(v) for v in r[:17]] for r in rows])
    fig, axes = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True, facecolor=surface)
    for col, node in enumerate(NODES):
        d = data[data[:, 0] == node["node_id"]]
        t = (d[:, 3] - d[0, 3]) / 1e6
        for row, (label, cols, lsb, unit) in enumerate([("Accelerometer", (4, 5, 6), acc_lsb, "g"),
                                                        ("Gyroscope", (7, 8, 9), gyr_lsb, "°/s")]):
            ax = axes[row, col]
            ax.set_facecolor(surface)
            for axis_name, c in zip("xyz", cols):
                ax.plot(t, d[:, c] / lsb, color=colors[axis_name], lw=1.4, label=axis_name)
            ax.set_title(f"Node {node['node_id']} · {node['segment'].replace('_', ' ')} · {label}",
                         color=ink, fontsize=10.5, loc="left")
            ax.set_ylabel(unit, color=muted)
            ax.grid(color=grid, lw=0.6)
            ax.tick_params(colors=muted, labelsize=9)
            for s in ax.spines.values():
                s.set_color(grid)
            if row == 1:
                ax.set_xlabel("time (s)", color=muted)
    axes[0, 1].legend(loc="upper right", frameon=False, ncol=3, fontsize=9, labelcolor=ink)
    fig.suptitle(f"Simulated MPU-6050 output, move: {move} (converted from raw counts)",
                 color=ink, fontsize=12, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(out / "signals.png", dpi=110, facecolor=surface)
    print(f"\nPlot: {out / 'signals.png'}")


if __name__ == "__main__":
    main()
