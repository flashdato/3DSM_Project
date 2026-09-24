"""Predefined joint-rotation trajectories driving HumanModel."""
import numpy as np

from human_model import Rx, Ry, Rz, SEGMENTS, _segment_z_range


def wave_arm(t):
    """Right-arm wave. Upper arm out to the side, forearm bent up 90 deg,
    oscillating about the upper-arm axis."""
    pose = {}
    pose["r_shoulder"] = Ry(np.deg2rad(-90))
    osc = np.deg2rad(35) * np.sin(2 * np.pi * 1.5 * t)
    pose["r_elbow"] = Rz(osc) @ Ry(np.deg2rad(-90))
    return pose


def squat(t):
    """Symmetric squat, one full cycle every 3 seconds. Uses knee = -hip so
    the shin stays vertical (natural squat rather than a chair-sit)."""
    period = 3.0
    depth = (1.0 - np.cos(2 * np.pi * t / period)) / 2.0
    pose = {}
    hip_angle = np.deg2rad(80) * depth
    pose["l_hip"] = Rx(hip_angle)
    pose["r_hip"] = Rx(hip_angle)
    pose["l_knee"] = Rx(-hip_angle)
    pose["r_knee"] = Rx(-hip_angle)
    return pose


def _cycle01(t, period):
    """Smooth 0 -> 1 -> 0 profile, one cycle per `period` seconds."""
    return (1.0 - np.cos(2 * np.pi * t / period)) / 2.0


def elbow_curl(t):
    """Right elbow flexion 0 -> 135 deg: the first thing the two arm IMU nodes measure."""
    return {"r_elbow": Rx(np.deg2rad(135) * _cycle01(t, 3.0))}


def arm_raise(t):
    """Right shoulder flexion: straight arm raised forward to 150 deg."""
    d = _cycle01(t, 3.0)
    return {"r_shoulder": Rx(np.deg2rad(150) * d), "r_elbow": Rx(np.deg2rad(10) * d)}


def side_raise(t):
    """Right shoulder abduction: arm raised sideways to 90 deg."""
    return {"r_shoulder": Ry(np.deg2rad(-90) * _cycle01(t, 3.0))}


def joint_angle_deg(R):
    """Rotation angle of a local joint rotation, i.e. how far the child segment is
    rotated relative to its parent. For the elbow this is what two IMU nodes give
    via q_UF = q_WU^-1 * q_WF."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def ground_lock(model):
    """Shift pelvis vertically so the lowest shin corner sits at z=0."""
    world = model.forward_kinematics()
    lowest = float("inf")
    for shin_name in ("l_shin", "r_shin"):
        spec = SEGMENTS[shin_name]
        j_pos, j_rot = world[spec["joint"]]
        _, z_far = _segment_z_range(spec)
        hw, hd = spec["W"] / 2.0, spec["D"] / 2.0
        for xi, yi in [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]:
            corner = j_pos + j_rot @ np.array([xi, yi, z_far])
            if corner[2] < lowest:
                lowest = corner[2]
    model.root_pos[2] -= lowest
