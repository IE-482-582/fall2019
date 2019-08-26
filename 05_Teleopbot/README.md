# Teleopbot -- Chapter 8

## Installation Instructions

1. Create the Package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg teleopbot rospy geometry_msgs sensor_msgs
    ```
    *Note: We have added some dependency packages.  We'll take a look at `CMakeLists.txt` and `package.xml` shortly to see how that helped us.*
    
2. Let's go ahead and create our `scripts` directory:
    ```
    cd ~/catkin_ws/src/teleopbot
    mkdir scripts
    ```
        
3. Get the source code from the course github site:
    ```
    cd ~/Downloads
    rm -rf fall2019
    git clone https://github.com/IE-482-582/fall2019.git
    ```
    
 4. Copy the Python scripts to our teleopbot workspace
    ```
    cd fall2019/05_Teleopbot/code/teleopbot
    cp scripts/* ~/catkin_ws/src/teleopbot/scripts/
    ```
    
 5. Make our Python scripts executable
    ```
    cd ~/catkin_ws/src/teleopbot/scripts
    chmod +x *.py
    ```
    
6. Compile/make our package

    ```
    cd ~/catkin_ws
    catkin_make
    ```
        
---

## Basic Keyboard Controls

We'll first test out our teleoperated robot using some basic keyboard controls.

We'll need three (3) terminal windows:
1. Launch Gazebo:
   ```
   roslaunch turtlebot_gazebo turtlebot_world.launch
   ```
	
2. Run a node to capture our keyboard commands:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot key_publisher.py
   ```

	When you're done, use `Ctrl-z` (I don't think Ctrl-c works).
	
3. Run a node to listen for keyboard commands and send to our robot:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot keys_to_twist.py cmd_vel:=cmd_vel_mux/input/teleop
   ```
	
We can also open a 4th terminal window to listen for our keyboard commands:
```
rostopic echo keys
```
	
When you're done, use `Ctrl-z` (I don't think Ctrl-c works).

Note that the robot only moves for a brief duration after we press a key.  This is a safety feature found on most robots.  If you want to keep moving forward you have to keep giving a "move forward" command.
	

## Using Rates
We will use a modified script that will keep re-issuing the previous move command.

We'll need three (3) terminal windows:
1. Launch Gazebo:
   ```
   roslaunch turtlebot_gazebo turtlebot_world.launch
   ```
	
2. Run a node to capture our keyboard commands:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot key_publisher.py
   ```
   
   When you're done, use `Ctrl-z` (I don't think Ctrl-c works).
	
3. Run a node to listen for keyboard commands and send to our robot:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot keys_to_twist_using_rate.py cmd_vel:=cmd_vel_mux/input/teleop
   ```

---

## Using the Parameter Server

You might have noticed that our robot was very fast.  

Rather than manually editing our script to reduce the speed, we can use ROS's parameter server.  This allows us to change the speed at the command line.

We'll need three (3) terminal windows:
1. Launch Gazebo:
   ```
   roslaunch turtlebot_gazebo turtlebot_world.launch
   ```
	
2. Run a node to capture our keyboard commands:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot key_publisher.py
   ```
   
   When you're done, use `Ctrl-z` (I don't think Ctrl-c works).
	
3. Run a node to listen for keyboard commands and send to our robot:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot keys_to_twist_using_rate_and_params.py _linear_scale:=0.5 _angular_scale:=0.4 cmd_vel:=cmd_vel_mux/input/teleop
   ```

Notes:
- The name of this file is different from what the textbook uses.  I edited it to explicitly indicate that it also uses rates.
- The textbook forgot to include the cmd_vel definition.

---

## Controlling Velocity with Ramps

Unfortunately, we have asked to robot to accelerate instantaneously, which is impossible/dangerous.

A better approach is to ramp up/down the velocities.

While your robot is running, open another terminal window and let's get some more info:
```
rostopic info cmd_vel
rostopic info cmd_vel_mux/input/teleop
rosmsg show geometry_msgs/Twist
rostopic echo cmd_vel
rostopic echo cmd_vel_mux/input/teleop
rostopic hz cmd_vel_mux/input/teleop
```

Let's check out a plot of our linear and angular velocities:
```
rqt_plot rostopic cmd_vel_mux/input/teleop/linear/x cmd_vel_mux/input/teleop/angular/z 
```

Now, let's implement velocity ramps.  Again, we'll need 3 terminal windows:

1. Launch Gazebo:
   ```
   roslaunch turtlebot_gazebo turtlebot_world.launch
   ```
	
2. Run a node to capture our keyboard commands:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot key_publisher.py
   ```
   
   When you're done, use `Ctrl-z` (I don't think Ctrl-c works).
	
3. Run a node to listen for keyboard commands and send to our robot:
   ```
   cd ~/catkin_ws/src/teleopbot/scripts
   rosrun teleopbot keys_to_twist_with_ramps.py _linear_scale:=0.5 _angular_scale:=1.0 _linear_accel:=1.0 _angular_accel:=1.0 cmd_vel:=cmd_vel_mux/input/teleop
   ```

Let's check out a plot of our linear and angular velocities:
```
rqt_plot rostopic cmd_vel_mux/input/teleop/linear/x cmd_vel_mux/input/teleop/angular/z 
```

---


## Visualizing things with rviz

1. Start rviz:
   ```
   rosrun rviz rviz
   ```
	
2. Choose the frame of reference.  For our teleop robot, we'll choose "Fixed Frame" and "camera_depth_frame":
	- Under "Global Options", click on the cell to the right of "Fixed Frame".
	- Select "camera_depth_frame".
	
3. Add our robot to the scene:	
	- Click the "Add" button on the lower left side.
	- Scroll down until you find "RobotModel" (it's probably under the "rviz" folder).
	- You can leave the display name as "RobotModel".
	- Click "OK"

4. Now we'd like to see what the robot's sensors see.  Let's first add the depth scanner:
	- Again, click the "Add" button.
	- Select "PointCloud2", then click "OK".
	- Now, in the left rviz panel, expand the "PointCloud2" section (see the little triangle to the left of "PointCloud2").
	- For the "Topic", choose "/camera/depth/points".
	
5. Our robot also has a camera.  Let's add the camera feed:
	- Again, click the "Add" button.
	- Select "Image", then click "OK".
	- Expand the "Image" section in the left rviz panel. 
	- For the "Image Topic", choose "/camera/rgb/image_raw".  
		

---

## Capturing Collision Events

*This is not in the textbook.*

While your robot is moving around, open another terminal window:
```
rostopic list
rostopic echo /mobile_base/events/bumper
```

---

## Joystick Control

*This is not in the textbook.*

Using a joystick (or "gamepad") allows for much better control of a robot.  

The good news is that we are going to write a ROS node that will allow the use of an XBox-style gamepad.  The bad news is that our virtual machine doesn't seem to recognize this gamepad.

Fortunately, the code that can be used for XBox control will also allow you to use your mouse or trackpad.

You will have a homework assignment on this topic.  Details will be provided in class.  In the meantime, here's a summary of what needs to happen.

1.  Start your turtlebot as before:
   ```
   roslaunch turtlebot_gazebo turtlebot_world.launch
   ```
   
2.  Start a new ROS node that listens for gamepad signals:

    ```
    cd ~/catkin_ws/src/teleopbot/scripts	
    rosrun teleopbot gamepad_to_twist.py
    ```

    Make note of the topic to which this script subscribes...you'll need to publish to this topic in the next script.
   
3.  Start another ROS node that publishes signals from a gamepad:
    ```
    cd ~/catkin_ws/src/teleopbot/scripts
    rosrun teleopbot gamepad_testing_STUDENT.py
    ```

    You'll need to edit this script.  Look for all of the "FIXME" comments.  You'll probably also want to rename the file to be more descriptive.  Maybe something like `gamepad_publisher.py` (analogous to the `key_publisher.py` script).
   
4. Write another ROS node that implements rates and ramps.    

