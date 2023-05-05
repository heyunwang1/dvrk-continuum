"""
main.py

This script connects all the functions from the dvrk_rcm_estimation package to estimate the center of rotation.
It also provides information about the saved files and Excel sheets.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""

import sys
import os
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

from dvrk_rcm_estimation import record_base_manual, record_base_auto, compute_center

def main():
    print("Select recording method for base positions:")
    print("1. Manual recording")
    print("2. Automatic recording")
    choice = int(input("Enter your choice (1 or 2): "))

    if choice == 1:
        print("\nStarting manual base position recording...")
        record_base_manual()
        print("\nManual base positions have been saved in 'base_cp_list.pkl'")
    elif choice == 2:
        print("\nStarting automatic base position recording...")
        record_base_auto()
        print("\nAutomatic base positions have been saved in 'base_cp_list.pkl'")
    else:
        print("Invalid choice. Exiting.")
        return

    print("\nComputing center of rotation...")
    center = compute_center()
    print("Estimated center of rotation: ", center)
    print("The estimated center has been appended to the 'estimated_centers.csv' file.")

if __name__ == '__main__':
    main()
