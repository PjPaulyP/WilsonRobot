import numpy as np
from scipy.optimize import fsolve


def dh_matrix(a, alpha, d, theta):
    """Return the standard DH homogeneous transform A_i using parameters
    in the order (a, alpha, d, theta). All angles in radians.

    A = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward_kinematics(dh_table, joint_angles):
    """Compute successive transforms and joint positions.

    dh_table: list of tuples (a, alpha, d, theta_offset)
    joint_angles: list of angles (radians) to add to each row's theta_offset

    Returns: positions: numpy array shape (N+1, 3) including base at origin
            transforms: list of 4x4 transforms for each joint frame
    """
    if len(joint_angles) < len(dh_table):
        # pad with zeros if fewer angles given
        joint_angles = list(joint_angles) + [0.0] * (len(dh_table) - len(joint_angles))

    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    transforms = [T.copy()]

    for (a, alpha, d, theta_offset), theta in zip(dh_table, joint_angles):
        th = theta_offset + theta
        A = dh_matrix(a, alpha, d, th)
        T = T @ A
        positions.append(T[:3, 3].copy())
        transforms.append(T.copy())

    return np.array(positions), transforms


def inverse_kinematics(link_lengths: list, target_pos: list, initial_guess=None):
    # def inverse_kinematics(dh_table, target_pos, initial_guess=None):
    """Compute joint angles to reach target position using numerical solver.

    link_lengths: list of link lengths [L1, L2, L3]
    target_pos: numpy array [x, y, z] desired end-effector position
    initial_guess: optional list of initial joint angles (radians)

    Returns: list of joint angles (radians)
    """
    # ============================================================ #
    # Uncomment below to use fsolve for general IK solving
    # if initial_guess is None:
    #     initial_guess = [0.0] * len(dh_table)

    # def equations(joint_angles):
    #     pos, _ = forward_kinematics(dh_table, joint_angles)
    #     end_effector_pos = pos[-1]
    #     return end_effector_pos - target_pos

    # solution = fsolve(equations, initial_guess)
    # return solution.tolist()
    # ============================================================ #

    L1, L2, L3 = link_lengths[0], link_lengths[1], link_lengths[2]
    X_val, Y_val, Z_val = target_pos[0], target_pos[1], target_pos[2]

    def equations(thetas):
        theta1, theta2, theta3 = thetas
        X_eq = (
            -L1 * np.sin(theta1)
            - L2 * np.cos(theta1) * np.cos(theta2)
            - L3 * np.cos(theta1) * np.cos(theta2 + theta3)
            - X_val
        )
        Y_eq = (
            L1 * np.cos(theta1)
            - L2 * np.sin(theta1) * np.cos(theta2)
            - L3 * np.sin(theta1) * np.cos(theta2 + theta3)
            - Y_val
        )
        Z_eq = L2 * np.sin(theta2) + L3 * np.sin(theta2 + theta3) - Z_val
        return [X_eq, Y_eq, Z_eq]

    if initial_guess is None:
        initial_guess = [0, -45, 90]  # approximately close to neutral

    solutions = fsolve(equations, np.radians(initial_guess))

    # Print solutions in clean decimal format
    print("θ1, θ2, θ3 (radians) =")
    for i, angle in enumerate(solutions, 1):
        print(f"θ{i}: {angle:.6f}")

    print("\nθ1, θ2, θ3 (degrees) =")
    degrees = np.degrees(solutions)
    for i, angle in enumerate(degrees, 1):
        print(f"θ{i}: {angle:.6f}")

    return solutions.tolist()
