#!/bin/sh

xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/myhsrworld.world" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch "&
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/HomeServiceRobot/catkin/src/map/map.yaml " &
sleep 5
xterm  -e  " source devel/setup.bash; rosrun add_markers add_markers " &
sleep 5
xterm  -e  " source devel/setup.bash; rosrun pick_objects pick_objects " &
