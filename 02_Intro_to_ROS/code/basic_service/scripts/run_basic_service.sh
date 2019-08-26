#!/bin/bash

# https://stackoverflow.com/questions/3512055/avoid-gnome-terminal-close-after-script-execution

MYPKG="basic_service"
SCRIPT1="roscore"
SCRIPT2="rosrun basic_service client.py blah blah"
SCRIPT3="rosrun basic_service server.py"

gnome-terminal \
--tab --title "ROSCORE" --working-directory=${HOME} -e "bash -ic \"export HISTFILE=${HOME}/.bash_history_junk1; $SCRIPT1; history -s $SCRIPT1; exec bash\"" \
--tab --title "CLIENT" --working-directory=${HOME}/catkin_ws/src/${MYPKG}/scripts -e "bash -ic \"sleep 7s; export HISTFILE=${HOME}/.bash_history_junk2; $SCRIPT2; history -s $SCRIPT2; exec bash\"" \
--tab --title "SERVER" --working-directory=${HOME}/catkin_ws/src/${MYPKG}/scripts -e "bash -ic \"sleep 7s; export HISTFILE=${HOME}/.bash_history_junk3; $SCRIPT3; history -s $SCRIPT3; exec bash\"" 
