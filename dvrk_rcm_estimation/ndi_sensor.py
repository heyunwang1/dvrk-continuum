"""
ndi_sensor.py

This module contains the NDIsensor class, which provides a simple API for interacting with NDI sensors.

Author: Jaspor Jiang
Email: sjiang44@jh.edu
Date: 2023-05-05
"""
import crtk

class NDIsensor(object):
    """Simple sensor API wrapping around ROS messages
    """
    # initialize the sensor
    def __init__(self, sensor_name, ros_namespace = '', expected_interval = 0.2):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_sensor(sensor_name, ros_namespace, expected_interval)

    def __init_sensor(self, sensor_name, ros_namespace, expected_interval):
        """Constructor.  This initializes a few data members.It
        requires a sensor name, this will be used to find the ROS
        topics for the sensor being controlled."""
        # data members, event based
        self.__sensor_name = sensor_name
        self.__ros_namespace = ros_namespace
        self.__full_ros_namespace = ros_namespace + sensor_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__full_ros_namespace, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_measured_cp()
