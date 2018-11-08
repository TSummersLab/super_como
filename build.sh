#!/bin/bash

# build script
# builds or re-builds the catkin workspace 
# also fetches the submodules

ROOT=$PWD

# Get the ZED ROS wrapper 
cd workspace/src/camera/zed/zed-ros-wrapper
git submodule init
git submodule update
cd $ROOT

# Get laser_proc 
cd workspace/src/lidar/hokuyo/laser_proc
git submodule init
git submodule update
cd $ROOT

# Get urg_c 
cd workspace/src/lidar/hokuyo/urg_c
git submodule init
git submodule update
cd $ROOT

# Get urg_node
cd workspace/src/lidar/hokuyo/urg_node
git submodule init
git submodule update
cd $ROOT

# Get rp_lidar package
cd workspace/src/lidar/rplidar/rplidar_ros
git submodule init
git submodule update
cd $ROOT


# Get razor_imu package
cd workspace/src/imu/razor_imu_9dof
git submodule init
git submodule update
cd $ROOT


# build the workspace
cd workspace
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -k
catkin_make
cd $ROOT 
