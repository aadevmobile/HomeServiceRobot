#!/bin/sh
xterm  -e  " turtlebot_world.launch " &
sleep 5
xterm  -e  " gmapping_demo.launch" & 
sleep 5
xterm  -e  " view_navigation.launch " &
sleep 5
xterm  -e  " keyboard_teleop.launch "
