#!/bin/bash
source /opt/ros/noetic/setup.bash;
cd ~/catkin_ws/; catkin_make;
cd ~/catkin_ws/;
source devel/setup.bash;
roscd map_data_structure/launch;
roslaunch map_data_structure map_data_structure.launch;
