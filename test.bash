#!/bin/bash
(cd ~/catkin_ws/; source ~/catkin_ws/devel/setup.bash)
(cd ~/catkin_ws/; catkin_make)
(cd ~/catkin_ws/src/map_data_structure/launch/; . ~/catkin_ws/devel/setup.bash)
(cd ~/catkin_ws/src/map_data_structure/launch/; roslaunch map_data_structure map_data_structure.launch)
