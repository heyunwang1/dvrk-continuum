"""
record_base_auto.py

This script records a sequence of base positions using the NDIsensor class and saves them to a file. 
It automatically collects 100 frames with a 0.1s time interval between each frame.

Usage:
- Run this script to record the base positions using an EM tracker with a specified serial number.
- Press Enter to start collecting frames.
- The recorded base positions will be saved to a file named 'base_cp_list.pkl'.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""

import time
import rospy
import PyKDL
import pickle
from dvrk_rcm_estimation import NDIsensor

def record_base_auto():
    # Initialize the ROS node
    rospy.init_node('aurora_client', anonymous=True, log_level=rospy.WARN)

    # Instantiate the NDIsensor with the EM tracker's serial number
    t1 = NDIsensor('/NDI/01_3ACC9000_610066___T6D0_S01296')

    num_frames = 100          # The number of frames to record
    time_interval = 0.1       # The time interval between consecutive frames in seconds
    base_cp_list = []         # The list to store the recorded base positions

    # Prompt the user to start collecting frames
    print("Press Enter to start collecting 100 frames with 0.1s time interval...")
    input()

    # Collect the specified number of frames with the given time interval
    for i in range(num_frames):
        base_cp = PyKDL.Frame()
        base_cp.p = t1.measured_cp().p
        base_cp.M = t1.measured_cp().M
        base_cp_list.append(base_cp)
        time.sleep(time_interval)

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
    record_base_auto()
