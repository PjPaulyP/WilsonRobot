import numpy as np
from scipy.optimize import least_squares

def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transform (theta, alpha in radians)."""
    ct = np.cos(theta); st = np.sin(theta)
    ca = np.cos(alpha); sa = np.sin(alpha)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d ],
        [  0,      0,      0,    1 ]
    ], dtype=float)

def forward_kinematics(dh_table, thetas_full):
    """Compute full FK transform from base to end-effector.
    dh_table: list of dicts with keys 'a','alpha','d','theta' (theta may be ignored here)
    thetas_full: list/array of numeric theta values, same length as dh_table
    """
    T = np.eye(4)
    for i, p in enumerate(dh_table):
        a = p['a']; alpha = p['alpha']; d = p['d']; theta = thetas_full[i]
        A = dh_transform(a, alpha, d, theta)
        T = T @ A
    return T

def inverse_kinematics(dh_table, target_pos, initial_guess, var_indices, fixed_values=None, weights=(1.0,1.0,1.0)):
    """
    Numerical IK that solves for angles at var_indices to reach target_pos (x,y,z).
    dh_table: list of DH params (a,alpha,d,...). The dh_table length = n_joints.
    target_pos: [x,y,z] target in base frame.
    initial_guess: initial vector for the variable angles (len = len(var_indices)).
    var_indices: list of integers indicating which DH theta entries are variable.
    fixed_values: dict mapping index -> fixed numeric theta (radians) for joints not in var_indices.
    weights: tuple to weight residuals (x,y,z)
    Returns: result.x as solution for variable theta vector
    """
    n = len(dh_table)
    fixed_values = {} if fixed_values is None else fixed_values

    def residuals(vars_vec):
        # Build full theta list
        thetas_full = [0.0]*n
        # fill fixed values (if any)
        for idx, p in enumerate(dh_table):
            if idx in fixed_values:
                thetas_full[idx] = fixed_values[idx]
        # fill variable values
        for k, vidx in enumerate(var_indices):
            thetas_full[vidx] = vars_vec[k]
        # If any remaining joint entries had numeric 'theta' values stored in dh_table, use them:
        for idx, p in enumerate(dh_table):
            if isinstance(p.get('theta', None), (int,float)) and idx not in fixed_values and idx not in var_indices:
                thetas_full[idx] = float(p['theta'])
        T = forward_kinematics(dh_table, thetas_full)
        pos = T[:3,3]
        res = (pos - target_pos) * np.array(weights)
        return res

    sol = least_squares(residuals, x0=np.asarray(initial_guess), method='lm')  # 'lm' or 'trf'
    return sol.x # return solution list of radians [theta1, theta2, ...]

