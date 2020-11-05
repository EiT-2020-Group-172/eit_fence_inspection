# Fence inspection repository of ROS packages

ROS packages are folders in the root of this repository.
Adding new package:
```
# format
catkin_create_pkg <package name> <dependencies[]>
# example
catkin_create_pkg my-super-package std_msgs rospy roscpp
```
