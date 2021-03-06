#!/bin/bash

# Installs packages for ROS Kinetic
# For more information, or if this script does not run, refer to:
# http://wiki.ros.org/kinetic/Installation/Ubuntu


### Install ROS Packages
#apt-cache search ros-kinetic # view available packages
#sudo apt-get install ros-kinetic-PACKAGE # install PACKAGE

# joy
sudo apt-get install ros-kinetic-joy -y # ROS driver for a generic Linux joystick
sudo apt-get install ros-kinetic-joy-listener -y  # Translates joy msgs
sudo apt-get install ros-kinetic-joy-teleop -y # A (to be) generic joystick interface to control a robot
sudo apt-get install ros-kinetic-ps3joy -y # Playstation 3 SIXAXIS or DUAL SHOCK 3 joystick driver

# rviz
sudo apt-get install ros-kinetic-agni-tf-tools -y # This package provides a gui program as well as a rviz plugin to publish static transforms.
sudo apt-get install ros-kinetic-cartographer-rviz -y # Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.
sudo apt-get install ros-kinetic-grid-map-rviz-plugin -y # RViz plugin for displaying grid map messages.
sudo apt-get install ros-kinetic-object-recognition-ros-visualization -y # object_recognition_ros_visualization contains rviz plugins to visualize ork detection results
sudo apt-get install ros-kinetic-rqt-rviz -y # rqt_rviz provides a GUI plugin embedding RViz.
sudo apt-get install ros-kinetic-rviz -y # 3D visualization tool for ROS.
sudo apt-get install ros-kinetic-rviz-imu-plugin -y # RVIZ plugin for IMU visualization
sudo apt-get install ros-kinetic-rviz-plugin-tutorials -y # Tutorials showing how to write plugins for RViz.
sudo apt-get install ros-kinetic-rviz-python-tutorial -y # Tutorials showing how to call into rviz internals from python scripts.
sudo apt-get install ros-kinetic-rviz-visual-tools -y # Utility functions for displaying and debugging data in Rviz via published markers
sudo apt-get install ros-kinetic-visualization-msgs -y # visualization_msgs is a set of messages used by higher level packages, such as rviz, that deal in visualization-specific data.
sudo apt-get install ros-kinetic-visualization-tutorials -y # Metapackage referencing tutorials related to rviz and visualization.
sudo apt-get install ros-kinetic-xpp-quadrotor -y # The URDF file for a quadrotor to be used with the xpp packages and a simple rviz publisher of quadrotor tfs.

# transformations
sudo apt-get install ros-kinetic-tf2-geometry-msgs -y # Transformation bindings for tf library

# Ackermann
sudo apt-get install ros-kinetic-ackermann-controller -y
sudo apt-get install ros-kinetic-ackermann-msgs -y

# Ackermann Serial
sudo apt-get install ros-kinetic-ackermann-serial -y
sudo apt-get install ros-kinetic-ackermann-serial-utils -y

# Camera drivers
sudo apt-get install ros-kinetic-libuvc -y # USB Video Class driver library
sudo apt-get install ros-kinetic-libuvc-camera -y # USB Video Class camera driver
sudo apt-get install ros-kinetic-libuvc-ros -y # libuvc_ros metapackage
sudo apt-get install ros-kinetic-uvc-camera -y # A collection of node(let)s that stream images from USB cameras (UVC) and provide CameraInf
sudo apt-get install ros-kinetic-usb-cam -y # for V4L USB camera devices

#AMCL Ros navigation stack
sudo apt-get install ros-kinetic-amcl -y

# serial 
sudo apt-get install ros-kinetic-serial -y
sudo apt-get install ros-kinetic-serial-utils -y

# Rosserial and arduino 
sudo apt-get install ros-kinetic-rosserial -y
sudo apt-get install ros-kinetic-rosserial-arduino -y
sudo apt-get install ros-kinetic-rosserial-python -y

# image view
sudo apt-get install ros-kinetic-image-view -y









