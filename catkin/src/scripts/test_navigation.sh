#!/bin/sh

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/myhsrworld.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "&
#xterm  -e  " rosrun rviz rviz -d /home/workspace/HomeServiceRobot/catkin/src/rvizConfig/rvizConfig.rviz"&
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/HomeServiceRobot/catkin/src/map/map.yaml "

