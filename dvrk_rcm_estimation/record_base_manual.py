"""
record_base_manual.py

This script records a sequence of base positions using the NDIsensor class and saves them to a file.
The user manually triggers each frame collection, up to 10 frames.

Usage:
- Run this script to record the base positions using an EM tracker with a specified serial number.
- Press Enter to collect each frame or 'q' to quit the frame collection process.
- The recorded base positions will be saved to a file named 'base_cp_list.pkl'.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""

import rospy
import PyKDL
import pickle
from dvrk_rcm_estimation import NDIsensor

def record_base_manual():
    # Initialize the ROS node
    rospy.init_node('aurora_client', anonymous=True, log_level=rospy.WARN)

    # Instantiate the NDIsensor with the EM tracker's serial number
    t1 = NDIsensor('/NDI/01_3ACC9000_610066___T6D0_S01296')

    base_cp = PyKDL.Frame()
    tip_cp = PyKDL.Frame()

    base_cp_list = []  # The list to store the recorded base positions
    counter = 0        # A counter to keep track of the collected frames

    # Collect up to 10 frames, with user manually triggering each frame collection
    while counter < 10:
        base_cp.p = t1.measured_cp().p
        base_cp.M = t1.measured_cp().M
        print(f' Base position recorded[m]: \n{type(base_cp.p)}, \n Base rotation: \n{base_cp.M} \n ')
        print('----------------------------')
        print("Press Enter to continue or 'q' to quit...")
        user_input = input()
        if user_input.lower() == 'q':
            break
        base_cp_list.append(base_cp)
        counter += 1

    # Print the collected base positions
    print("\nCollected base_cp values:")
    for idx, base_cp in enumerate(base_cp_list):
        print(f"{idx + 1}: {base_cp}")

    # Save the base_cp_list to a file
    with open('base_cp_list.pkl', 'wb') as f:
        pickle.dump(base_cp_list, f)

    # Inform the user that the base_cp_list has been saved
    print("Saved base_cp_list to 'base_cp_list.pkl'")

if __name__ == '__main__':
    record_base_manual()
