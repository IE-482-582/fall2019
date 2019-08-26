# Followbot -- Chapter 12

## Installation Instructions

1. Create the Package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg followbot rospy geometry_msgs sensor_msgs
    ```
    *Note that we have added some dependency packages, which will show up in `CMakeLists.txt` and `package.xml`.*
    
2. Let's go ahead and create our `scripts` directory:
    ```
    cd ~/catkin_ws/src/followbot
    mkdir scripts
    ```
        
3. Get the source code from the course github site:
    ```
    cd ~/Downloads
    rm -rf fall2019
    git clone https://github.com/IE-482-582/fall2019.git
    ```
    
 4. Copy `package.xml` and the Python scripts to our followbot workspace
    ```
    cd fall2019/06_Followbot/code/followbot
    cp package.xml ~/catkin_ws/src/followbot/
    cp scripts/* ~/catkin_ws/src/followbot/scripts/
    ```
    
 5. Make our Python scripts executable
    ```
    cd ~/catkin_ws/src/followbot/scripts
    chmod +x *.py
    ```
    
6. Compile/make our package

    ```
    cd ~/catkin_ws
    catkin_make
    ```
        
---

## Viewing the Camera Feed

We'll start out by placing our robot in a simulated environment.  Then we'll look at the robot's camera feed.

1.  Open Gazebo and place the turtlebot in a playground:
    ```	
    cd ~/catkin_ws/src/followbot/scripts
    roslaunch turtlebot_gazebo turtlebot_world.launch
    ```

    *NOTE: We are using the default turtlebot_gazebo settings here.  In a moment, we'll customize the environment.*
   		
2.  Launch our ROS node view the robot's camera feed: 
    ```
    cd ~/catkin_ws/src/followbot/scripts
    rosrun followbot view_camera.py 
    ```	

    Within Gazebo you can move your robot around and watch the camera footage change.


Let's see how fast our camera feed is operating:
```
rostopic list
rostopic hz /camera/rgb/image_raw
```

What information is being passed by this topic?
```
rostopic echo /camera/rgb/image_raw -n 1
```   

---

## Detecting a Line

In this next example, we're going to place our robot on the ground.  There is a yellow line painted on the ground, but no obstacles.

1.  Open Gazebo and place the turtlebot on the test course:

    ```	
    cd ~/catkin_ws/src/followbot/scripts
    roslaunch followbot course.launch
    ```
    
    *NOTE: We are now using a customized .launch file.*	
	
2.  Launch a ROS node where we filter the camera image:
    ```
    cd ~/catkin_ws/src/followbot/scripts
    rosrun followbot follower_color_filter.py 
    ```	
   
    *NOTE:  I've modified this Python script slightly to show the original and filtered images.*
 
--- 
  
## Following a Line

Now, let's make the robot drive around while following the yellow line.

1.  Open Gazebo and place the turtlebot on the test course:
    ```	
    cd ~/catkin_ws/src/followbot/scripts
    roslaunch followbot course.launch
    ```

    *NOTE: We are now using a customized .launch file.*
	
2.  Launch a ROS node with the proportional controller:
    ```
    cd ~/catkin_ws/src/followbot/scripts
    rosrun followbot follower_p.py 
    ```	

---
  
## Extensions/Exercises

We will enhance our followbot in the following ways:

1. Edit the `follower_p.py` script so our robot actually stays on the line while making a turn.

2. Use lanes instead of a single line.
	- The lanes use white lines on the right and dashed yellow lines on the left.
	- We need to create a new image, and we need rules for lane width and line widths.
	- We'll also need to edit our launch files.
	
3. Can we capture the robot's heading direction?

4. Make the robot randomly turn when it reaches an intersection. 
	- Make the robot recognize the type of intersection (2-way?  3-way? 4-way?).
	
5. Add stop lines.
	- These will be red, perpendicular to the traffic lane.
	- Our robot should stop for 3 seconds before reaching this line.

6. Add stop signs.

7. Add street signs.

8. Add a second robot.

9. Add obstacles.

	


   


