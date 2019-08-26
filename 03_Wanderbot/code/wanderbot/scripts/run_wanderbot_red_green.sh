#!/bin/bash

# https://stackoverflow.com/questions/3512055/avoid-gnome-terminal-close-after-script-execution

MYPKG="wanderbot"
SCRIPT1="roslaunch turtlebot_gazebo turtlebot_world.launch"
SCRIPT2="rosrun wanderbot red_light_green_light.py cmd_vel:=cmd_vel_mux/input/teleop"

gnome-terminal \
--tab --title "ROSLAUNCH" --working-directory=${HOME} -e "bash -ic \"export HISTFILE=${HOME}/.bash_history_junk1; $SCRIPT1; history -s $SCRIPT1; exec bash\"" \
--tab --title "REDGREEN" --working-directory=${HOME}/catkin_ws/src/${MYPKG}/scripts -e "bash -ic \"sleep 15s; export HISTFILE=${HOME}/.bash_history_junk2; $SCRIPT2; history -s $SCRIPT2; exec bash\"" 
 
