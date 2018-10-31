#!/bin/bash

# build script
# builds or re-builds the catkin workspace 
# also fetches the submodules

ROOT=$PWD

# Get the ZED ROS wrapper 
cd workspace/src/zed-ros-wrapper
git submodule init
git submodule update
cd $ROOT


# build the workspace
cd workspace
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -k
catkin_make
cd $ROOT 
