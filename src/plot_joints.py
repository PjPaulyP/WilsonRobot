"""
plot_joints.py

Helper to plot a robot leg given a 4-row DH table in the format
(a, alpha, d, theta).

This keeps your 4-row table (one of the rows can be a fixed 90° transform)
and treats the theta value in each row as an offset. The function expects
an array of joint angle increments (one per DH row). If a row represents
a fixed transform, pass 0 for its joint angle in the angles list.

Usage:
    from plot_joints import plot_from_dh
    dh_table = [ (a, alpha, d, theta_offset), ... ]  # 4 rows
    angles = [theta1, theta2, theta3, theta4]       # radians
    plot_from_dh(dh_table, angles)

"""

import matplotlib.pyplot as plt
from robot import forward_kinematics, dh_matrix


def plot_from_dh(dh_table, joint_angles, ax=None, show=True, frame_size=0.2):
    """Plot the robot defined by dh_table and joint_angles.

    - dh_table: list of (a, alpha, d, theta_offset)
    - joint_angles: list of angles in radians (same length as dh_table)
    - ax: optional matplotlib 3d axis
    - show: if True, call plt.show()
    - frame_size: length of frame axes arrows

    Returns (ax, positions)
    """
    if ax is None:
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection="3d")

    positions, transforms = forward_kinematics(dh_table, joint_angles)

    # Plot joints
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c="r", s=50)

    # Plot links
    for i in range(len(positions) - 1):
        xs = [positions[i, 0], positions[i + 1, 0]]
        ys = [positions[i, 1], positions[i + 1, 1]]
        zs = [positions[i, 2], positions[i + 1, 2]]
        ax.plot(xs, ys, zs, "b-", linewidth=2)

    # Plot coordinate frames at each joint
    colors = ["r", "g", "b"]  # x, y, z
    for T in transforms:
        origin = T[:3, 3]
        R = T[:3, :3]
        for j in range(3):
            vec = R[:, j] * frame_size
            ax.quiver(
                origin[0],
                origin[1],
                origin[2],
                vec[0],
                vec[1],
                vec[2],
                color=colors[j],
                length=1.0,
                normalize=False,
            )

    # Labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Equal aspect (approximate)
    xs = positions[:, 0]
    ys = positions[:, 1]
    zs = positions[:, 2]
    max_range = max(xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min())
    if max_range == 0:
        max_range = 1.0
    mid_x = (xs.max() + xs.min()) / 2.0
    mid_y = (ys.max() + ys.min()) / 2.0
    mid_z = (zs.max() + zs.min()) / 2.0
    r = max_range / 2.0
    ax.set_xlim(mid_x - r, mid_x + r)
    ax.set_ylim(mid_y - r, mid_y + r)
    ax.set_zlim(mid_z - r, mid_z + r)

    if show:
        plt.show()

    return ax, positions
