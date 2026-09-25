"""Record the receiver ESP32's serial stream to disk.

Reads the line protocol printed by firmware/receiver (see HARDWARE.md):
    N,...  node announcement   -> meta.json
    D,...  one IMU sample      -> imu_raw.csv
    S,...  per-node stats      -> printed live (and stats.csv)
    #...   comments            -> meta.json["comments"]

Usage:
    python tools/log_serial.py --port /dev/ttyUSB0
    python tools/log_serial.py --port COM5 --out recordings --duration 900

Stop with Ctrl+C. Raw counts are stored unchanged, and conversion to physical
units happens later in the pipeline using the scales saved in meta.json.
"""
import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

SEGMENT_NAMES = {
    0: "pelvis", 1: "chest", 2: "r_upper_arm", 3: "r_forearm", 4: "l_upper_arm",
    5: "l_forearm", 6: "r_thigh", 7: "r_shin", 8: "l_thigh", 9: "l_shin", 10: "head",
}

D_COLUMNS = ["node", "segment", "seq", "t_us", "ax", "ay", "az", "gx", "gy", "gz",
             "mx", "my", "mz", "temp", "flags", "batt_mv", "rssi", "rx_us"]
S_COLUMNS = ["node", "pkts_per_s", "samples_per_s", "lost_total", "rssi",
             "latency_ms", "synced", "batt_mv"]


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the receiver ESP32")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default="recordings", help="parent folder for session folders")
    ap.add_argument("--label", default="", help="optional label added to the session folder name")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C)")
    return ap.parse_args()


def format_stats(row):
    node, pkts, samples, lost, rssi, lat, synced, batt = row
    parts = [f"node {node}: {float(pkts):5.1f} pkt/s", f"{float(samples):6.1f} samples/s",
             f"lost {lost}", f"rssi {rssi} dBm",
             f"latency {float(lat):.1f} ms" if float(lat) >= 0 else "latency n/a",
             "synced" if synced == "1" else "NOT SYNCED"]
    if batt != "0":
        parts.append(f"batt {batt} mV")
    return ", ".join(parts)


def main():
    args = parse_args()
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is missing: pip install -r requirements.txt")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{stamp}_{args.label}" if args.label else stamp
    session = Path(args.out) / name
    session.mkdir(parents=True, exist_ok=True)

    meta = {
        "session": name,
        "started_local": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "baud": args.baud,
        "units": {
            "acc": "counts; g = counts / (16384, 8192, 4096, 2048 for 2/4/8/16 g)",
            "gyr": "counts; deg/s = counts / (131, 65.5, 32.8, 16.4 for 250/500/1000/2000 dps)",
            "temp": "counts; degC = counts / 340 + 36.53",
            "t_us": "microseconds, receiver clock when flags & 1 (SYNCED), else node clock",
            "rx_us": "microseconds, receiver clock at packet arrival",
            "host_time": "seconds since epoch on this computer when the line was read",
        },
        "nodes": {},
        "comments": [],
    }

    raw_f = open(session / "imu_raw.csv", "w", newline="")
    stats_f = open(session / "stats.csv", "w", newline="")
    raw_w = csv.writer(raw_f)
    stats_w = csv.writer(stats_f)
    raw_w.writerow(D_COLUMNS + ["host_time"])
    stats_w.writerow(S_COLUMNS + ["host_time"])

    def save_meta():
        (session / "meta.json").write_text(json.dumps(meta, indent=2))

    save_meta()
    print(f"Recording to {session}  (Ctrl+C to stop)")

    n_samples = 0
    n_bad = 0
    t0 = time.time()
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    try:
        while True:
            if args.duration and time.time() - t0 >= args.duration:
                break
            line = ser.readline()
            if not line:
                continue
            now = time.time()
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError:
                n_bad += 1
                continue
            if not text:
                continue
            kind, _, rest = text.partition(",")
            if kind == "D":
                fields = rest.split(",")
                if len(fields) != len(D_COLUMNS):
                    n_bad += 1
                    continue
                raw_w.writerow(fields + [f"{now:.6f}"])
                n_samples += 1
            elif kind == "S":
                fields = rest.split(",")
                if len(fields) == len(S_COLUMNS):
                    stats_w.writerow(fields + [f"{now:.6f}"])
                    print(format_stats(fields))
            elif kind == "N":
                f = rest.split(",")
                if len(f) == 6:
                    node, seg, hz, acc_g, gyr_dps, mac = f
                    meta["nodes"][node] = {
                        "segment_id": int(seg),
                        "segment": SEGMENT_NAMES.get(int(seg), f"segment_{seg}"),
                        "sample_hz": int(hz), "acc_fs_g": int(acc_g),
                        "gyr_fs_dps": int(gyr_dps), "mac": mac,
                    }
                    save_meta()
                    print(f"node {node} joined: {meta['nodes'][node]['segment']}, {hz} Hz, "
                          f"±{acc_g} g, ±{gyr_dps} °/s, {mac}")
            elif text.startswith("#"):
                meta["comments"].append(text)
                save_meta()
                print(text)
            else:
                n_bad += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        raw_f.close()
        stats_f.close()
        meta["stopped_local"] = datetime.now().isoformat(timespec="seconds")
        meta["samples_written"] = n_samples
        meta["unparsed_lines"] = n_bad
        save_meta()
        dur = time.time() - t0
        print(f"\nSaved {n_samples} samples in {dur:.1f} s to {session} ({n_bad} unparsed lines)")


if __name__ == "__main__":
    main()
