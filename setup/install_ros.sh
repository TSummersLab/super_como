#!/bin/bash

# Installs the following software on the Jetson TX2 
# ROS Kinetic
# ROS Kinetic packages
# Catkin_tools


sudo apt-get update

# Installing ROS-Kinetic
cd ~/super_como/setup
chmod +x ./helper_scripts/install_ros_kinetic.sh
chmod +x ./helper_scripts/install_ros_kinetic_packages.sh
rosversion ros || ./helper_scripts/install_ros_kinetic.sh # install ROS Kinetic if not installed
./helper_scripts/install_ros_kinetic_packages.sh # install packages for ROS Kinetic

# Installing catkin_tools
sudo apt-get install python-catkin-tools -y

