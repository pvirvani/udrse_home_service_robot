#!/bin/sh
xterm  -e  " roslaunch turtlebot turtlebot_world.launch " &
sleep 10
xterm  -e  " roslaunch turtlebot amcl_demo.launch " &
sleep 10
xterm  -e  " roslaunch turtlebot view_navigation.launch " &
sleep 10
xterm  -e  " rosrun add_markers add_markers_pk " &
sleep 10
xterm  -e  " rosrun pick_objects pick_objects_pk "