#!/bin/bash

# ----------------------------------
# An installer script for the
# talklisten
# package.
# ----------------------------------

set -e

# ----------------------------------
MYPKG="talklisten"
# ----------------------------------

# See https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux
# for more info on text color in terminal.
RED='\033[1;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "This script overwrites the following directory (and its subdirectories):"
echo "     ~/catkin_ws/src/${MYPKG}/"
echo -e "${YELLOW}A backup copy of this directory will be created on the Desktop.${NC}"
echo ""

echo -n "Do you wish to continue (y/n)? "
read answer
if echo "$answer" | grep -iq "^y" ;then
    echo "OK.  Starting Installation..."
    echo ""

	# Get a "unique" timestamp:
	timestamp=$(date "+%Y-%m-%dT%H%M%S")
	# echo ${timestamp}	# 2017-11-16T063639

	# Define the name of the .tar archive:
	MYFILE="${HOME}/Desktop/${MYPKG}_archive_${timestamp}.tar.bz2"
	# echo ${MYFILE}

	# Create the archive (but only if at least one directory exists):
	myPKGdir=""
	SomethingToDo="False"
	if [ -d "${HOME}/catkin_ws/src/${MYPKG}" ]; then	
		myPKGdir="${HOME}/catkin_ws/src/${MYPKG}"
		SomethingToDo="True"
	fi
	
	if [ ${SomethingToDo} == "True" ]; then
		# Create the archive:
		tar -chjvf ${MYFILE} ${myPKGdir}
		echo -e "${YELLOW}Your backup archive is saved as ${MYFILE}.${NC}"
		echo ""
	else
		echo "You don't have any content to backup."
		echo ""
	fi	

	# Store the present working directory:
	myPWD=$PWD

	# Delete any old package content from catkin
	rm -rf ${HOME}/catkin_ws/src/${MYPKG}

	# Rebuild catkin (thus completely removing the package)
	cd ${HOME}/catkin_ws
	catkin_make
		
	# Create an empty package
	cd ${HOME}/catkin_ws/src
	catkin_create_pkg ${MYPKG}
	
	# Copy the catkin contents from the archive to the new package
	cp -a ${myPWD}/code/${MYPKG}/. ${HOME}/catkin_ws/src/${MYPKG}
	
	# Change the permissions on scripts
	cd ${HOME}/catkin_ws/src/${MYPKG}/scripts
	chmod +x *.py
	chmod +x *.sh
	 
	# Rebuild catkin 
	cd ${HOME}/catkin_ws
	catkin_make
			

	echo ""
	echo "The installation script is done."
	echo ""
	if [ ${SomethingToDo} == "True" ]; then
		echo "Your backup archive is saved as ${MYFILE}."
	else
		echo "No backup archive was required/created."
	fi	
	echo ""	
	echo "See ~/catkin_ws/src/${MYPKG}."
	# echo "There is no launcher icon." 

	
else
    echo "Installation Cancelled."
fi

	
