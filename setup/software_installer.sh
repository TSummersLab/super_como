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
cd ~/super_como/setup
rosversion ros || ./helper_scripts/install_ros_kinetic.sh # install ROS Kinetic if not installed
./helper_scripts/install_ros_kinetic_packages.sh # install packages for ROS Kinetic

# Installing CUDA
cuda_installation=1
nvcc --version || cuda_installation=0
if [ $cuda_installation = 0 ]
then
#    cd ~/Downloads
    echo "Installing CUDA"
    
 
fi
