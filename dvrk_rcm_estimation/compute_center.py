"""
compute_center.py

This script computes the center of rotation using the residual_error function and the recorded base positions. 
It then saves the estimated center to a CSV file, appending the result to the end of the file for experimental purposes.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""

import pickle
import numpy as np
import csv
from dvrk_rcm_estimation import residual_error as residual
from scipy.optimize import least_squares

def compute_center():
    # Load base_cp_list from the file
    with open('base_cp_list.pkl', 'rb') as f:
        loaded_base_cp_list = pickle.load(f)

    rotation_axes = []

    # Calculate rotation axes between consecutive frames
    for i in range(len(loaded_base_cp_list) - 1):
        Bi = loaded_base_cp_list[i]
        Bj = loaded_base_cp_list[i + 1]
        F_BiBj = Bi.Inverse() * Bj
        angle, axis = F_BiBj.M.GetRotAngle()
        axis.Normalize()
        rotation_axes.append(axis)

    # Convert PyKDL frame positions to NumPy arrays
    positions = [np.array([frame.p.x(), frame.p.y(), frame.p.z()]) for frame in loaded_base_cp_list]

    initial_center = [0, 0, 0]  # An initial guess for the center C

    # Perform least-squares optimization to estimate the center of rotation
    result = least_squares(residual, initial_center, args=(positions, rotation_axes))
    center = result.x  # location of rcm in EM tracking frame

    print("Estimated center of rotation:", center)

    # Save the estimated center to a CSV file
    with open('estimated_centers.csv', 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(center)

    return center

if __name__ == '__main__':
    compute_center()
