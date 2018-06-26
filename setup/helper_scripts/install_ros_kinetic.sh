#!/bin/bash

# Installs ROS Kinetic on Jetson TX2 (Ubuntu 16.04, ARM64) 
# For more information, or if this script does not run, refer to:
# http://wiki.ros.org/kinetic/Installation/Ubuntu



### Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

### Setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

### Installation
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base -y 

### Initialize rosdep
sudo rosdep init
rosdep update

### Environment setup
grep -q -F "source /opt/ros/kinetic/setup.bash" ~/.bashrc || echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc # add path to ROS setup.bash only if it was not already added
source ~/.bashrc

### Dependencies for building packages
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y



