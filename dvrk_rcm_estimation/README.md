# dVRK RCM Registration

This package is designed for registering the NDI Aurora Electromagnetic (EM) sensor on the da Vinci Surgical Robot, specifically for the Remote Center of Motion (RCM) of the PSM3 arm. The package measures the RCM point (or center point) in the frame of the NDI Aurora global base using least squares optimization.

## Workflow

1. The EM tracker is attached to a fixed element on the PSM3 arm, which is close to the RCM point.
2. The sensor is rotated to collect a sequence of sensor frame data.
3. The RCM point is calculated using the method of pointing device calibration, based on the collected data.

The calculated RCM point can then be used for registration in subsequent steps.

## Usage

To use the `dvrk_rcm_estimation` package, follow these steps:

1. Import the necessary modules and classes from the package.
2. Instantiate the `NDIsensor` class with the appropriate sensor name and ROS namespace.
3. Follow the workflow described above to collect sensor frame data and compute the RCM point.

For more details on the specific modules and classes, please refer to the source code and documentation within the package.

