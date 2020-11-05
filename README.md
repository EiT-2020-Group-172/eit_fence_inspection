# Fence inspection repository of ROS packages

For this project to work, you must have the *eit_playground* folder in the same *src* folder as this repository.

ROS packages are folders in the root of this repository.
Adding new package:
```
# format
catkin_create_pkg <package name> <dependencies[]>
# example
catkin_create_pkg my-super-package std_msgs rospy roscpp
```

Useful *.bashrc* setup for the project, if you put this repository inside *~/catkin_ws/src/*
```
alias source_eit="
source /opt/ros/melodic/setup.bash &&
source ~/catkin_ws/devel/setup.bash &&
source ~/catkin_ws/src/eit_playground/setup_gazebo.bash && 
source /home/$USER/Firmware/Tools/setup_gazebo.bash /home/$USER/Firmware /home/$USER/Firmware/build/px4_sitl_default &&
echo sourced \
"

alias export_eit="
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo &&
export PX4_HOME_LAT=55.4719762 &&
export PX4_HOME_LON=10.3248095 &&
export PX4_HOME_ALT=7.4000000 &&
echo exported \
"

alias build_eit="
source_eit &&
export_eit &&
cd ~/catkin_ws/ &&
catkin build &&
source_eit &&
echo built \
"
```

Set up symlinks for the project
```
ln -s /home/$USER/catkin_ws/src/eit_playground/init.d-posix/* /home/$USER/Firmware/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s /home/$USER/catkin_ws/src/eit_playground/mixers/* /home/$USER/Firmware/ROMFS/px4fmu_common/mixers/
```
