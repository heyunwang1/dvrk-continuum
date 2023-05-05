"""
ndi_registration.py

This module provides functions for estimating the center of rotation using NDI sensors.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""

import numpy as np
from scipy.optimize import least_squares

def residual_error(C, positions, rotation_axes):
    """
    Compute the squared perpendicular distance e_i between each position
    vector p_i and its projection q_i onto the line defined by the center
    C and the rotation axis r_i.

    Args:
    C: The center of rotation (a 3-element list or numpy array)
    positions: A list of position vectors (numpy arrays)
    rotation_axes: A list of rotation axes (PyKDL Vectors)

    Returns:
    A list of squared perpendicular distances for all position vectors
    and rotation axes.
    """
    C = np.array(C)
    errors = []

    for p_i, r_i in zip(positions, rotation_axes):
        d_i = p_i - C
        r_i_np = np.array([r_i.x(), r_i.y(), r_i.z()])
        
        # Project p_i onto the line defined by the center C and the rotation axis r_i
        q_i = C + (np.dot(d_i, r_i_np) / np.linalg.norm(r_i_np)**2) * r_i_np
        
        # Compute the squared perpendicular distance between p_i and its projection q_i
        e_i = np.linalg.norm(p_i - q_i)**2
        errors.append(e_i)

    return errors