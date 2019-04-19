#!/bin/bash

# build script
# builds or re-builds the catkin workspace 
# also fetches the submodules

ROOT=$PWD

git submodule init
git submodule update


# Get the ZED ROS wrapper 
cd workspace/src/zed-ros-wrapper
git submodule init
git submodule update
cd $ROOT

# Get laser_proc 
cd workspace/src/laser_proc
git submodule init
git submodule update
cd $ROOT

# Get urg_c 
cd workspace/src/urg_c
git submodule init
git submodule update
cd $ROOT

# Get urg_node
cd workspace/src/urg_node
git submodule init
git submodule update
cd $ROOT

# Get rp_lidar package
cd workspace/src/rplidar_ros
git submodule init
git submodule update
cd $ROOT

# Get razor_imu package
cd workspace/src/razor_imu_9dof
git submodule init
git submodule update
cd $ROOT

# Get darknet_ros package
cd workspace/src/darknet_ros
git submodule init
git submodule update
cd $ROOT

# build the workspace
cd workspace
catkin_make -DCMAKE_BUILD_TYPE=Release -k
#catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -k
catkin_make
cd $ROOT 


## install Yolo and Darknet for object detection 
#cd catkin_workspace/src
#git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
#cd ../
#catkin_make -DCMAKE_BUILD_TYPE=Release
