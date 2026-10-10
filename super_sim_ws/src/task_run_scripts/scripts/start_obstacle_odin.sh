#!/bin/bash
# ????(??A: Odin) ??????, ?? odin_px4_and_others ???
source /opt/ros/noetic/setup.bash
source /home/fouruav/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
exec roslaunch task_run_scripts obstacle_to_px4.launch drone:=odin
