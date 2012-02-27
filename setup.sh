#!/bin/sh
source ~/ros/setup.bash
export ROS_ROOT=~/ros/ros
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
export ROS_PACKAGE_PATH=~/RoboTower:~/ros:$ROS_PACKAGE_PATH
