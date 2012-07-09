 #!/bin/sh
 source ~/ros/setup.bash
 cwd="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
 export ROS_ROOT=~/ros/ros
 export PATH=$ROS_ROOT/bin:$PATH
 export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
 export ROS_PACKAGE_PATH=$cwd:~/ros/stacks:$ROS_PACKAGE_PATH
