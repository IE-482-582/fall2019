# Sensors

**NOTE:  This is a placeholder document.  Please ignore everything below for now.  Updated materials will be posted soon.**

--- 

## Networked Version 
This version is currently incomplete.  When it is working:

### On the Tower computer (server):
1. Set master
	```
	export ROS_MASTER_URI=http://darkstar:11311
	```
	
2. Launch gazebo
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	roslaunch turtlebotrace race_murray.launch
	```
	
	NOTE: Replace `murray` with a different UBusername to run a different race.
	
3. Run tower
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace tower.py murray
	```

	NOTE: This script requires one input argument, which is the name of the race.  Replace `murray` with the UBusername matching what you used in Step 2.


### On your computer (client):
- Set master
	```
	export ROS_MASTER_URI=http://darkstar:11311
	```
- Run controller
	```
	cd ~/catkin_ws/src/turtlebotrace/scripts
	rosrun turtlebotrace my_robot_controller_testing.py 
	```

	NOTE: Each student will create their own controller script.  Replace `testing` with your UBusername to run your control algorithm.  This doesn't have to match the name of the race (you will be using your controller to race on tracks created by other users).

