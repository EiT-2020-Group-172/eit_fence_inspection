#!/bin/bash

echo FWDIR=$FWDIR
echo ROSWS=$ROSWS

source /opt/ros/melodic/setup.bash
source $ROSWS/devel/setup.bash
source $FWDIR/Tools/setup_gazebo.bash $FWDIR $FWDIR/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FWDIR
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$FWDIR/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROSWS/src/eit_playground/models
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROSWS/src/eit_fence_inspection/models


export PX4_HOME_LAT=55.4719762
export PX4_HOME_LON=10.3248095
export PX4_HOME_ALT=7.4000000

echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH
