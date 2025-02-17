# Fence inspection repository of ROS packages

This project requires Ubuntu 18, ROS Melodic, and MAVROS to be installed and PX4 Autopilot Firmware and EIT Playground to be downloaded and built.
The *eit_playground* folder must be in the same *src* folder as this repository, inside your ROS workspace folder.

Structure should be the following:
```
$ROSWS/src/eit_playground
$ROSWS/src/eit_fence_inspection
```

ROS packages are folders in the root of this repository.
Adding new package:
```
# format
catkin_create_pkg <package name> <dependencies[]>
# example
catkin_create_pkg my-super-package std_msgs rospy roscpp
```

Useful *.bashrc* setup for the project. Change FWDIR and ROSWS with your own values. This will source, export, and build everything when you call *eitbuild* in bash.
```
function eitsetup {
  FWDIR=~/Firmware
  ROSWS=~/catkin_ws
  source $ROSWS/src/eit_fence_inspection/setup_gazebo.bash
  echo eitsetup
}
export -f eitsetup

function eitbuild {
  eitsetup
  cd $ROSWS
  catkin build
  eitsetup
  echo eitbuild
}
export -f eitbuild
```

Set up symlinks for the project
```
ln -s $ROSWS/src/eit_playground/init.d-posix/* $FWDIR/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s $ROSWS/src/eit_playground/mixers/* $FWDIR/ROMFS/px4fmu_common/mixers/
```
