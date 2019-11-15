# Chapter 10 -- Navigating about the World

--- 

## Preliminaries

1. Clone the `IE-482-582/fall2019` repo from Github.

2. Create a catkin package, named `navigation`:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg navigation
    ```

3. Build the package
    ```
    cd ~/catkin_ws
    catkin_make
    ```
    
4. Copy the files from the `08_Navigating_Maps` directory in the repo to the `navigation` directory we just created.

---

## Moving Turtlebot to a Goal Position/Orientation

### Method 1 -- Set Goal Manually in `rviz`

We'll need 1 terminal window.

- **TERMINAL 1** -- Stage Robot Simulator (alternative to Gazebo) and rviz:
    ```
    cd ~/Projects/IE_482-582/Chapter_10
    roslaunch turtlebot_stage turtlebot_in_stage_2.launch 
    ```
- **RVIZ** 
    - Click the "2D Nav Goal" button
    - Click (position) and Drag (orientation) on the map to give robot a goal pose.
    
    

### Method 2 -- Control Robot via Python


We'll need 2 terminal windows.

- **TERMINAL 1** -- Stage Robot Simulator (alternative to Gazebo) and rviz:
    ```
    cd ~/catkin_ws/src/navigation
    roslaunch turtlebot_stage turtlebot_in_stage_2.launch 
    ```
    
- **TERMINAL 2** -- Teleop your robot:
    ```
    cd ~/catkin_ws/src/navigation
    rosrun navigation patrol.py
    ```
    
