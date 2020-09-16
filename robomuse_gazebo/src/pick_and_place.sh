#!/bin/bash
source /home/manuj/catkin_ws/devel/setup.bash
rosrun robomuse_gazebo move_group_python_initial.py
rosrun robomuse_gazebo pos_node.py
rosrun robomuse_gazebo move_group_python.py

