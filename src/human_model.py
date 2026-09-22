"""Articulated human body model with box-shaped segments.

Coordinate convention: right-handed world frame, +X right, +Y forward, +Z up.
Rest pose: standing upright, arms hanging along -Z at the sides.

Each joint stores a local 3x3 rotation. Forward kinematics propagates rotations
and translations down the tree. Each segment is drawn as a rectangular box
axis-aligned in its joint's local frame, occupying local Z in [inset, inset+L]
for "up" segments or [-inset, -(inset+L)] for "down" segments. The `inset` gap
recesses the box from the joint so limbs don't visually clip into each other
during rotation and the joints stay visible as small hinges.
"""
import numpy as np


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# child_joint: (parent_joint_or_None, offset_in_parent_local_frame_at_rest)
JOINT_TREE = {
    "pelvis":     (None,         np.array([ 0.00, 0.0,  0.00])),
    "neck":       ("pelvis",     np.array([ 0.00, 0.0,  0.55])),
    "head_base":  ("neck",       np.array([ 0.00, 0.0,  0.06])),
    "l_shoulder": ("neck",       np.array([-0.19, 0.0, -0.01])),
    "r_shoulder": ("neck",       np.array([ 0.19, 0.0, -0.01])),
    "l_elbow":    ("l_shoulder", np.array([ 0.00, 0.0, -0.30])),
    "r_elbow":    ("r_shoulder", np.array([ 0.00, 0.0, -0.30])),
    "l_hip":      ("pelvis",     np.array([-0.09, 0.0, -0.02])),
    "r_hip":      ("pelvis",     np.array([ 0.09, 0.0, -0.02])),
    "l_knee":     ("l_hip",      np.array([ 0.00, 0.0, -0.42])),
    "r_knee":     ("r_hip",      np.array([ 0.00, 0.0, -0.42])),
}

# Parent-before-child traversal order.
JOINT_ORDER = [
    "pelvis", "neck", "head_base",
    "l_shoulder", "r_shoulder", "l_elbow", "r_elbow",
    "l_hip", "r_hip", "l_knee", "r_knee",
]

# dir: "up" (+Z from joint) or "down" (-Z from joint)
# inset: gap between joint and near end of segment (leaves the joint visible)
# L: segment length along the axis after the inset gap
# W, D: cross-section widths perpendicular to the segment axis
SEGMENTS = {
    "head":        {"joint": "head_base",  "dir": "up",   "inset": 0.00, "L": 0.22, "W": 0.16, "D": 0.19, "color": "#f2c8a8"},
    "neck_seg":    {"joint": "neck",       "dir": "up",   "inset": 0.005,"L": 0.05, "W": 0.09, "D": 0.10, "color": "#e8bfa0"},
    "torso":       {"joint": "pelvis",     "dir": "up",   "inset": 0.03, "L": 0.52, "W": 0.28, "D": 0.18, "color": "#4a7fc7"},
    "l_upper_arm": {"joint": "l_shoulder", "dir": "down", "inset": 0.015,"L": 0.275,"W": 0.085,"D": 0.085,"color": "#e8834a"},
    "r_upper_arm": {"joint": "r_shoulder", "dir": "down", "inset": 0.015,"L": 0.275,"W": 0.085,"D": 0.085,"color": "#e8834a"},
    "l_forearm":   {"joint": "l_elbow",    "dir": "down", "inset": 0.02, "L": 0.26, "W": 0.07, "D": 0.07, "color": "#c9612a"},
    "r_forearm":   {"joint": "r_elbow",    "dir": "down", "inset": 0.02, "L": 0.26, "W": 0.07, "D": 0.07, "color": "#c9612a"},
    "l_thigh":     {"joint": "l_hip",      "dir": "down", "inset": 0.015,"L": 0.395,"W": 0.13, "D": 0.13, "color": "#5cb85c"},
    "r_thigh":     {"joint": "r_hip",      "dir": "down", "inset": 0.015,"L": 0.395,"W": 0.13, "D": 0.13, "color": "#5cb85c"},
    "l_shin":      {"joint": "l_knee",     "dir": "down", "inset": 0.02, "L": 0.40, "W": 0.10, "D": 0.10, "color": "#3a8a3a"},
    "r_shin":      {"joint": "r_knee",     "dir": "down", "inset": 0.02, "L": 0.40, "W": 0.10, "D": 0.10, "color": "#3a8a3a"},
}

BOX_FACES = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [0, 3, 7, 4],
]


def _segment_z_range(spec):
    sign = 1.0 if spec["dir"] == "up" else -1.0
    return sign * spec["inset"], sign * (spec["inset"] + spec["L"])


def _local_box_corners(spec):
    z_near, z_far = _segment_z_range(spec)
    hw, hd = spec["W"] / 2.0, spec["D"] / 2.0
    corners = []
    for zi in (z_near, z_far):
        for xi, yi in [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]:
            corners.append(np.array([xi, yi, zi]))
    return np.array(corners)


class HumanModel:
    def __init__(self, rest_pelvis_height=0.86):
        self.rest_pelvis_height = rest_pelvis_height
        self.root_pos = np.array([0.0, 0.0, rest_pelvis_height])
        self.joint_rot = {j: np.eye(3) for j in JOINT_TREE}
        self._local_corners = {name: _local_box_corners(spec) for name, spec in SEGMENTS.items()}

    def reset_pose(self):
        self.root_pos = np.array([0.0, 0.0, self.rest_pelvis_height])
        for j in self.joint_rot:
            self.joint_rot[j] = np.eye(3)

    def set_pose(self, pose_dict):
        self.reset_pose()
        for joint, R in pose_dict.items():
            self.joint_rot[joint] = R

    def forward_kinematics(self):
        world = {}
        for j in JOINT_ORDER:
            parent, offset = JOINT_TREE[j]
            if parent is None:
                world[j] = (self.root_pos.copy(), self.joint_rot[j].copy())
            else:
                p_pos, p_rot = world[parent]
                pos = p_pos + p_rot @ offset
                rot = p_rot @ self.joint_rot[j]
                world[j] = (pos, rot)
        return world

    def get_segment_polygons(self):
        world = self.forward_kinematics()
        out = []
        for name, spec in SEGMENTS.items():
            j_pos, j_rot = world[spec["joint"]]
            local = self._local_corners[name]
            world_corners = (j_rot @ local.T).T + j_pos
            faces = [world_corners[idx] for idx in BOX_FACES]
            out.append((faces, spec["color"]))
        return out
