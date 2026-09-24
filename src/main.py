"""Interactive 3D visualization of the human model cycling through predefined
movements. Two synchronized viewports: dead-on front and 3/4 angled.

Default: right-arm demo (upper arm + forearm with the two IMU nodes of the
Phase 3 hardware bring-up). Use --full for the original full-body demo.

    python src/main.py                     # right arm
    python src/main.py --full              # full body (wave + squat)
    python src/main.py --snapshot arm.png --t 1.5   # save one frame, no window
"""
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from human_model import HumanModel, SEGMENTS
from animations import (wave_arm, squat, elbow_curl, arm_raise, side_raise,
                        joint_angle_deg, ground_lock)


FPS = 30
CYCLE_SECONDS = 6.0
FULL_BODY_MOVEMENTS = [("Waving right arm", wave_arm), ("Squatting", squat)]
ARM_MOVEMENTS = [
    ("Elbow curl", elbow_curl),
    ("Arm raise (forward)", arm_raise),
    ("Side raise (abduction)", side_raise),
    ("Wave", wave_arm),
]
ARM_SEGMENTS = {"r_upper_arm", "r_forearm"}

# Wearable IMU nodes (see HARDWARE.md). along = fraction of segment length from the joint.
SENSOR_NODES = [
    ("N1", "r_upper_arm", 0.5),
    ("N2", "r_forearm", 0.8),
]

BG_COLOR = "#0d1117"
EDGE_COLOR = "#0d1117"
TEXT_COLOR = "#e6edf3"
NODE_COLOR = "#3fb950"

VIEWS = [
    ("Front", dict(elev=5, azim=90)),
    ("Angled", dict(elev=12, azim=75)),
]


def _setup_axes(ax, arm_only):
    if arm_only:
        # 1.2 m cube covering the right arm's reach (shoulder at x = 0.19, z = 1.40),
        # zoomed in so the arm fills the view.
        ax.set_xlim(-0.25, 0.95)
        ax.set_ylim(-0.45, 0.75)
        ax.set_zlim(0.80, 2.00)
        ax.set_box_aspect((1, 1, 1), zoom=1.6)
    else:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(0.0, 2.0)
        ax.set_box_aspect((1, 1, 1))
    ax.set_axis_off()
    ax.set_facecolor(BG_COLOR)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor((0, 0, 0, 0))


def _node_positions(model):
    """World position of each IMU node, on the outer (+x) face of its segment."""
    world = model.forward_kinematics()
    out = []
    for label, seg, along in SENSOR_NODES:
        spec = SEGMENTS[seg]
        j_pos, j_rot = world[spec["joint"]]
        local = np.array([spec["W"] / 2 + 0.013, 0.0, -(spec["inset"] + spec["L"] * along)])
        out.append((label, j_pos + j_rot @ local))
    return out


def parse_args():
    ap = argparse.ArgumentParser(description="3D human model demo")
    ap.add_argument("--full", action="store_true", help="full-body demo instead of the right arm")
    ap.add_argument("--snapshot", metavar="PNG", help="save a single frame to this file and exit")
    ap.add_argument("--t", type=float, default=1.5, help="time (s) of the snapshot frame")
    return ap.parse_args()


def main():
    args = parse_args()
    arm_only = not args.full
    movements = ARM_MOVEMENTS if arm_only else FULL_BODY_MOVEMENTS
    only = ARM_SEGMENTS if arm_only else None

    if args.snapshot:
        plt.switch_backend("Agg")

    model = HumanModel()
    fig = plt.figure(figsize=(14, 8), facecolor=BG_COLOR)
    axes = []
    for i, (name, view) in enumerate(VIEWS, start=1):
        ax = fig.add_subplot(1, len(VIEWS), i, projection="3d")
        _setup_axes(ax, arm_only)
        ax.view_init(**view)
        ax.text2D(0.5, 0.97, name, transform=ax.transAxes,
                  color=TEXT_COLOR, fontsize=12, ha="center", va="top")
        axes.append(ax)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=0)
    suptitle = fig.text(0.5, 0.03, "", color=TEXT_COLOR, fontsize=13, ha="center")

    artists_per_ax = [[] for _ in axes]

    def render(frame_idx):
        t = frame_idx / FPS
        movement_idx = int(t // CYCLE_SECONDS) % len(movements)
        local_t = t - movement_idx * CYCLE_SECONDS
        name, fn = movements[movement_idx]

        model.set_pose(fn(local_t))
        ground_lock(model)
        polys = model.get_segment_polygons(only=only)
        nodes = _node_positions(model) if arm_only else []

        for ax, artists in zip(axes, artists_per_ax):
            for a in artists:
                a.remove()
            artists.clear()
            for faces, color in polys:
                pc = Poly3DCollection(
                    faces,
                    facecolor=color,
                    edgecolor=EDGE_COLOR,
                    linewidth=0.4,
                    alpha=1.0,
                )
                ax.add_collection3d(pc)
                artists.append(pc)
            for label, p in nodes:
                artists.append(ax.scatter(*p, s=60, c=NODE_COLOR, depthshade=False,
                                          edgecolors=BG_COLOR, zorder=10))
                artists.append(ax.text(p[0] + 0.04, p[1], p[2], label, color=NODE_COLOR,
                                       fontsize=10, zorder=11))

        text = f"{name}    t = {t:5.2f} s"
        if arm_only:
            elbow = joint_angle_deg(model.joint_rot["r_elbow"])
            shoulder = joint_angle_deg(model.joint_rot["r_shoulder"])
            text += f"    elbow {elbow:5.1f}°    shoulder {shoulder:5.1f}°"
        suptitle.set_text(text)
        return [a for artists in artists_per_ax for a in artists] + [suptitle]

    if args.snapshot:
        render(int(round(args.t * FPS)))
        fig.savefig(args.snapshot, facecolor=BG_COLOR, dpi=80)
        print(f"saved {args.snapshot}")
        return

    _ = FuncAnimation(
        fig,
        render,
        interval=1000 / FPS,
        blit=False,
        cache_frame_data=False,
    )
    plt.show()


if __name__ == "__main__":
    main()
