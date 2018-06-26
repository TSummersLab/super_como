#!/bin/bash

# Installs the following software on the Jetson TX2 
# Nano
# Pluma
# VLC
# ROS Kinetic
# OpenCV 3.4
# Python 3


# Install nano text editor
sudo apt-get install nano -y

# Install Pluma text editor
sudo apt-get install pluma -y

# Install VLC
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) universe"
sudo apt-get install vlc -y

# Installing ROS-Kinetic
./helper_scripts/install_ros_kinetic.sh # install ROS Kinetic
./helper_scripts/install_ros_kinetic_packages.sh # install packages for ROS Kinetic

