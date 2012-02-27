#!/bin/sh
source /home/marcello/ros/setup.bash
export ROS_ROOT=/home/marcello/ros/ros
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
export ROS_PACKAGE_PATH=~/robotower:/home/marcello/ros:$ROS_PACKAGE_PATH
