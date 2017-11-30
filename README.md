# matlab-sclam

This is an algorithm to solve the extrinsic calibration problem for a mobile robot with a monocular camera model.
The input measurements include the odometric measurements and visual measurements.
An auto initialization algorithm is embedded, so the initial guess of the extrinsic parameters are not necessarily required.

# Dataset

The user should provide a dataset including both measurements from odometry and the camera.
The file structure should follows the format as the files *odo.rec* and *mk.rec* in the folder *sample*.

# Configure

The user should manually set some configures according to their system, including the measurement error, threshold for data prunning, camera intrinsic parameters, landmark info, and the path of the dataset.

# Quick Start

The user could have an easy start by run the script file *main_calib.m*, which use the sample data and the configure file in the folder *sample*.
