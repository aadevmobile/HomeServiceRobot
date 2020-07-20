#!/bin/sh


xterm  -e  " roslaunch turtlebot_gazebo  turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/myhsrworld.world " &
#xterm  -e  " roslaunch my_robot  world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
#xterm  -e  " rosrun gmapping slam_gmapping scan:=scan " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "&
#xterm  -e  " roslaunch /home/workspace/HomeServiceRobot/catkin/src/view_navigation.launch "&
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "
#xterm  -e  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py "
