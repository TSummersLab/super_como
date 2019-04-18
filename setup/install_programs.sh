#!/bin/bash

# Installs the following software on the Jetson TX2 
# Nano
# Pluma
# Terminator (terminal application)
# VLC
# V4L

sudo apt-get update

# Install nano text editor
sudo apt-get install nano -y

# Install Pluma text editor
sudo apt-get install pluma -y

# Install Terminator
sudo apt-get install terminator -y

# Install VLC
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) universe"
sudo apt-get install vlc -y

#installs python visiual (used for IMU)
sudo apt-get install python-visual -y

#installs arduino libraries
sudo apt-get install arduino arduino-core -y

# Video for Linux (V4L)
sudo apt-get install v4l-utils -y
# if this does not run, rerun it after running the following:
# sudo apt-get install -f


