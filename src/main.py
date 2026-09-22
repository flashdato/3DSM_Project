"""Interactive 3D visualization of the 10-segment human model cycling through
predefined movements. Two synchronized viewports: dead-on front and 3/4 angled."""
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from human_model import HumanModel
from animations import wave_arm, squat, ground_lock


FPS = 30
CYCLE_SECONDS = 6.0
MOVEMENTS = [("Waving right arm", wave_arm), ("Squatting", squat)]

BG_COLOR = "#0d1117"
EDGE_COLOR = "#0d1117"
TEXT_COLOR = "#e6edf3"

VIEWS = [
    ("Front", dict(elev=5, azim=90)),
    ("Angled", dict(elev=12, azim=75)),
]


def _setup_axes(ax, fig):
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(0.0, 2.0)
    ax.set_box_aspect((1, 1, 1))
    ax.set_axis_off()
    ax.set_facecolor(BG_COLOR)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor((0, 0, 0, 0))


def main():
    model = HumanModel()
    fig = plt.figure(figsize=(14, 8), facecolor=BG_COLOR)
    axes = []
    view_titles = []
    for i, (name, view) in enumerate(VIEWS, start=1):
        ax = fig.add_subplot(1, len(VIEWS), i, projection="3d")
        _setup_axes(ax, fig)
        ax.view_init(**view)
        ax.text2D(0.5, 0.97, name, transform=ax.transAxes,
                  color=TEXT_COLOR, fontsize=12, ha="center", va="top")
        axes.append(ax)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=0)
    suptitle = fig.text(0.5, 0.03, "", color=TEXT_COLOR, fontsize=13, ha="center")

    collections_per_ax = [[] for _ in axes]

    def render(frame_idx):
        t = frame_idx / FPS
        movement_idx = int(t // CYCLE_SECONDS) % len(MOVEMENTS)
        local_t = t - movement_idx * CYCLE_SECONDS
        name, fn = MOVEMENTS[movement_idx]

        model.set_pose(fn(local_t))
        ground_lock(model)
        polys = model.get_segment_polygons()

        for ax, coll in zip(axes, collections_per_ax):
            for c in coll:
                c.remove()
            coll.clear()
            for faces, color in polys:
                pc = Poly3DCollection(
                    faces,
                    facecolor=color,
                    edgecolor=EDGE_COLOR,
                    linewidth=0.4,
                    alpha=1.0,
                )
                ax.add_collection3d(pc)
                coll.append(pc)

        suptitle.set_text(f"{name}    t = {t:5.2f} s")
        return [c for coll in collections_per_ax for c in coll] + [suptitle]

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
