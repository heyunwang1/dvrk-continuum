#!/usr/bin/env python

# Author: Heyun Wang
# Date: 2023-02-22

# (C) Copyright 2015-2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

from json import load
import dvrk
import math
import sys
import time
import rospy
import numpy
import PyKDL
import argparse
import numpy as np
import formula as f
from Inverse import Inverse
import csv
from ndi_sensor import NDIsensor

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_arm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.arm(arm_name = robot_name,
                            expected_interval = expected_interval)

    # homing example
    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print_id('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.18
            goal[0] = math.radians(-5)
            goal[1] = 0
            goal[3] = 0
            goal[4] = 0
            goal[5] = 0
        # move and wait
        print_id('moving to starting position')
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print_id('testing move to current position')
        move_handle = self.arm.move_jp(goal)
        time.sleep(1.0) # add some artificial latency on this side
        move_handle.wait()
        print_id('home complete')

    # get methods
    def run_get(self):
        [p, v, e, t] = self.arm.measured_js()
        d = self.arm.measured_jp()
        [d, t] = self.arm.measured_jp(extra = True)
        d = self.arm.measured_jv()
        [d, t] = self.arm.measured_jv(extra = True)
        d = self.arm.measured_jf()
        [d, t] = self.arm.measured_jf(extra = True)
        d = self.arm.measured_cp()
        [d, t] = self.arm.measured_cp(extra = True)
        d = self.arm.local.measured_cp()
        [d, t] = self.arm.local.measured_cp(extra = True)
        d = self.arm.measured_cv()
        [d, t] = self.arm.measured_cv(extra = True)
        d = self.arm.body.measured_cf()
        [d, t] = self.arm.body.measured_cf(extra = True)
        d = self.arm.spatial.measured_cf()
        [d, t] = self.arm.spatial.measured_cf(extra = True)

        [p, v, e, t] = self.arm.setpoint_js()
        d = self.arm.setpoint_jp()
        [d, t] = self.arm.setpoint_jp(extra = True)
        d = self.arm.setpoint_jv()
        [d, t] = self.arm.setpoint_jv(extra = True)
        d = self.arm.setpoint_jf()
        [d, t] = self.arm.setpoint_jf(extra = True)
        d = self.arm.setpoint_cp()
        [d, t] = self.arm.setpoint_cp(extra = True)
        d = self.arm.local.setpoint_cp()
        [d, t] = self.arm.local.setpoint_cp(extra = True)     

    # # goal joint control example
    # def run_move_jp(self):
    #     print_id('starting move_jp')
    #     inverse = Inverse(f.forward, f.jacob)
    #     initial_angle = inverse.deg2rad([0, 0, 0, 0.0001, 0.0001, 0.0001])
    #     initial_angle[2] = 0.18
    #     g0 = f.forward(initial_angle)
    #     g = np.copy(g0)
    #     g[1,3] = 0.15
    #     g[0,3] = 0.15                      
    #     result = inverse.run_inverse(initial_angle, g)
    #     self.arm.move_jp(result).wait()
    #     initial_angle = numpy.copy(self.arm.setpoint_jp())
    #     g[1,3] = 0.15
    #     g[0,3] = -0.15
    #     inter = np.linspace(np.array([0.15,0.15]),np.array([-0.15,0.15]),21)
    #     for i, point in enumerate(inter):
    #         g[0,3] = point[0]
    #         g[1,3] = point[1]
    #         new_result = inverse.run_inverse(initial_angle, g)
    #         self.arm.move_jp(new_result).wait()
    #         initial_angle = numpy.copy(self.arm.setpoint_jp()) 
    #     result = inverse.run_inverse(initial_angle, g)  
    #     self.arm.move_jp(result).wait() 
    #     initial_angle = numpy.copy(self.arm.setpoint_jp())
    #     g[1,3] = -0.15
    #     g[0,3] = -0.15
    #     inter = np.linspace(np.array([-0.15,0.15]),np.array([-0.15,-0.15]),21)
    #     for i, point in enumerate(inter):
    #         g[0,3] = point[0]
    #         g[1,3] = point[1]
    #         new_result = inverse.run_inverse(initial_angle, g)
    #         self.arm.move_jp(new_result).wait()
    #         initial_angle = numpy.copy(self.arm.setpoint_jp()) 
    #     result = inverse.run_inverse(initial_angle, g)    
    #     self.arm.move_jp(result).wait() 
    #     initial_angle = numpy.copy(self.arm.setpoint_jp())
    #     g[1,3] = -0.15
    #     g[0,3] = 0.15
    #     result = inverse.run_inverse(initial_angle, g)

    #     self.arm.move_jp(result).wait() 

    #     print_id('move_jp complete')
    # goal joint control example
    def video(self):
        # Instantiate the NDIsensor with the EM tracker's serial number
        t1 = NDIsensor('/NDI/01_3ACC9000_610066___T6D0_S01296')

        tip_cp_list = []  # The list to store the recorded base positions
        results = np.loadtxt('results.csv',delimiter=',')
        for res in results:
            tip_cp = PyKDL.Frame()
            self.arm.move_jp(res).wait()
            tip_cp.p = t1.measured_cp().p
            tip_cp.M = t1.measured_cp().M
            tip_cp_list.append(tip_cp)
        # Initialize two lists to store position and rotation data
        position_data = []
        rotation_data = []
        for frame in tip_cp_list:
            # Position
            p = [frame.p[i] for i in range(3)]
            position_data.append(p)
            print(p)

            # Rotation matrix
            R = [frame.M[i, j] for i in range(3) for j in range(3)]
            rotation_data.append(R)

        # Save the position data into a CSV file
        with open('position_data_file.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(position_data)
        # Save the rotation data into another CSV file
        with open('rotation_data_file.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(rotation_data)

        print('finished')        
    def run_move_jp(self):
        print_id('starting move_jp')
        inverse = Inverse(f.forward, f.jacob)
        initial_angle = inverse.deg2rad([0, 0, 0, 0.0001, 0.0001, 0.0001])
        initial_angle[2] = 0.18
        g0 = f.forward(initial_angle)
        g = np.copy(g0)
        g[1,3] = 0.15
        g[0,3] = 0.15                      
        result = inverse.run_inverse(initial_angle, g)
        
        # Create a list to store all inverse calculations
        results = [result]
        
        initial_angle = numpy.copy(self.arm.setpoint_jp())
        g[1,3] = 0.15
        g[0,3] = -0.15
        inter = np.linspace(np.array([0.15,0.15]),np.array([-0.15,0.15]),21)
        for i, point in enumerate(inter):
            g[0,3] = point[0]
            g[1,3] = point[1]
            new_result = inverse.run_inverse(initial_angle, g)
            results.append(new_result)  # Store in the list
            initial_angle = numpy.copy(self.arm.setpoint_jp()) 

        result = inverse.run_inverse(initial_angle, g)  
        results.append(result)

        initial_angle = numpy.copy(self.arm.setpoint_jp())
        g[1,3] = -0.15
        g[0,3] = -0.15
        inter = np.linspace(np.array([-0.15,0.15]),np.array([-0.15,-0.15]),21)
        for i, point in enumerate(inter):
            g[0,3] = point[0]
            g[1,3] = point[1]
            new_result = inverse.run_inverse(initial_angle, g)
            results.append(new_result)  # Store in the list
            initial_angle = numpy.copy(self.arm.setpoint_jp()) 

        result = inverse.run_inverse(initial_angle, g)    
        results.append(result)

        initial_angle = numpy.copy(self.arm.setpoint_jp())
        g[1,3] = -0.15
        g[0,3] = 0.15
        inter = np.linspace(np.array([-0.15,-0.15]),np.array([0.15,-0.15]),21)
        for i, point in enumerate(inter):
            g[0,3] = point[0]
            g[1,3] = point[1]
            new_result = inverse.run_inverse(initial_angle, g)
            results.append(new_result)  # Store in the list
            initial_angle = numpy.copy(self.arm.setpoint_jp())
        result = inverse.run_inverse(initial_angle, g)
        results.append(result)

        initial_angle = numpy.copy(self.arm.setpoint_jp())
        g[1,3] = -0.15
        g[0,3] = 0.15
        inter = np.linspace(np.array([0.15,-0.15]),np.array([0.15,0.15]),21)
        for i, point in enumerate(inter):
            g[0,3] = point[0]
            g[1,3] = point[1]
            new_result = inverse.run_inverse(initial_angle, g)
            results.append(new_result)  # Store in the list
            initial_angle = numpy.copy(self.arm.setpoint_jp())
        result = inverse.run_inverse(initial_angle, g)
        results.append(result)    
        # word_list = []    
        # for word in results:
        #     word_list.append(word)
        # # Save the position data into a CSV file
           
        # # Now that we have all results, we move them one by one
        # rospy.init_node('aurora_client', anonymous=True, log_level=rospy.WARN)

        # # Instantiate the NDIsensor with the EM tracker's serial number
        # t1 = NDIsensor('/NDI/01_3ACC9000_610066___T6D0_S01296')

        # tip_cp = PyKDL.Frame()
        # tip_cp_list = []  # The list to store the recorded base positions
        for res in results:
            self.arm.move_jp(res).wait()
        with open('results.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(results) 
        #     tip_cp = PyKDL.Frame()
        #     tip_cp.p = t1.measured_cp().p
        #     tip_cp.M = t1.measured_cp().M
        #     tip_cp_list.append(tip_cp)
        # # Initialize two lists to store position and rotation data
        # position_data = []
        # rotation_data = []
        # for frame in tip_cp_list:
        #     # Position
        #     p = [frame.p[i] for i in range(3)]
        #     position_data.append(p)

        #     # Rotation matrix
        #     R = [frame.M[i, j] for i in range(3) for j in range(3)]
        #     rotation_data.append(R)

        # # Save the position data into a CSV file
        # with open('position_data_file.csv', 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerows(position_data)

        # # Save the rotation data into another CSV file
        # with open('rotation_data_file.csv', 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerows(rotation_data)n_data)

        # # Save the rotation data into another CSV file
        # with open('rotation_data_file.csv', 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerows(rotation_data)

        print_id('move_jp complete')


    # main method
    def run(self):
        self.home()
        self.run_get()
        # self.run_move_jp()
        self.video()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_arm_test', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()

