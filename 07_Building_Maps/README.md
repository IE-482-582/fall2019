# Chapter 9 -- Building Maps of the World

--- 

## Preliminaries

We'll need to install some new software before continuing.

1. Ubuntu has changed its privacy/security keys.  We need to update those:
    ```
    sudo apt-key del B01FA116

    sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    ```
    
2.  Install some new ROS packages:
    ```
    sudo apt-get clean && sudo apt-get update

    sudo apt-get install ros-indigo-turtlebot-rviz-launchers

    sudo apt-get install ros-indigo-turtlebot-stage
    ```

3.  We need to define some environment variables so `turtlebot_stage` will run:
    - First, open your `.bashrc` file:
        ``` 
        pico ~/.basrhc
        ```
    - Next, paste the following at the **bottom** of the file:
        ```
        export TURTLEBOT_STAGE_WORLD_FILE="/opt/ros/indigo/share/turtlebot_stage/maps/stage/maze.world"
        export TURTLEBOT_STAGE_MAP_FILE="/opt/ros/indigo/share/turtlebot_stage/maps/maze.yaml"
        ```    
    - Now, save and close your `.bashrc` file:  `ctrl-o`, `ctrl-x`
    - Finally, re-load your `.bashrc` file:
        ```
        source ~/.bashrc
        ```
    **NOTE**:  The process in Step 3 is supposed to eliminate the `Invalid <arg> tag: environment variable 'TURTLEBOT_STAGE_MAP_FILE' is not set.` error by pointing our system to look for files in `/opt/ros/indigo/share/turtlebot_stage/maps/`.
`                
4.  Let's create a directory where we'll save some data:
    ```
    mkdir -p ~/Projects/IE_482-582/Chapter_09
    ```

--- 

## Task 1 -- Scan a Room

We'll need 3 terminal windows.

- **TERMINAL 1** -- Stage Robot Simulator (alternative to Gazebo) and rviz:
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    roslaunch turtlebot_stage turtlebot_in_stage.launch 
    ```
    
- **TERMINAL 2** -- Teleop your robot:
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    roslaunch turtlebot_teleop keyboard_teleop.launch 
    ```
    
- **TERMINAL 3** -- Record data:
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    rosbag record -O data.bag /scan /tf     # (that's a capital oh)
    ```
    
When you're done, cancel out of your terminals (`ctrl-c`) in the following order:
- Kill Terminal 3
- Kill Terminal 1
- Kill Terminal 2


--- 

## Task 2 -- Build a Map

Again, we'll need 3 terminal windows.

- **TERMINAL 1** -- roscore
    ```
    roscore
    ```
    
- **TERMINAL 2** -- slam_gmapping
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    rosparam set use_sim_time true
    rosrun gmapping slam_gmapping
    ```
    
- **TERMINAL 3** -- Replay rosbag Data
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    rosbag play --clock data.bag
    rosrun map_server map_saver
    ```

When you're done, cancel out of your terminals (`ctrl-c`) in the following order:
- Kill Terminal 3
- Kill Terminal 2
- Kill Terminal 1
 

You may now look at the map image:
```
cd ~/Projects/IE_482-582/Chapter_09
eog map.pgm
```

--- 

## Task 3 -- Improve the Map (with the **same** data)

- **TERMINAL 1** -- roscore
    ```
    roscore
    ```

- **TERMINAL 2** -- slam_gmapping with improved parameters
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    
    rosparam set /slam_gmapping/angularUpdate 0.1
    rosparam set /slam_gmapping/linearUpdate 0.1
    rosparam set /slam_gmapping/lskip 10
    rosparam set /slam_gmapping/xmax 10
    rosparam set /slam_gmapping/xmin -10
    rosparam set /slam_gmapping/ymax 10
    rosparam set /slam_gmapping/ymin -10
    
    rosparam set use_sim_time true
    rosrun gmapping slam_gmapping
    ```

- **TERMINAL 3** -- Replay rosbag Data
    ```
    cd ~/Projects/IE_482-582/Chapter_09
    rosbag play --clock data.bag
    rosrun map_server map_saver
    ```
    
When you're done, cancel out of your terminals (`ctrl-c`) in the following order:
- Kill Terminal 3
- Kill Terminal 2
- Kill Terminal 1


---

## Task 4 -- Start the map server and view 	

- **TERMINAL 1** -- roscore
    ```
    roscore
    ```

- **TERMINAL 2**
    ```
    cd ~/Projects/IE_482-582/chapter9
    rosrun map_server map_server map.yaml
    ```

- **TERMINAL 3**
    ```
    rostopic list		          # (optional) 
    rostopic echo map_metadata    # (optional) 
    rosrun rviz rviz
    ```

    - Within RViz:
        - Add --> By topic --> /map

