# p3dx-guide-control
A ROS package for guiding an actor using a Pioneer 3DX mobile robot, with a laser scan 

The package can be used with ROS2 packages `ros1_bridge` and `navigation2`, but can also be used solely on ROS.

### Dependencies
#### ROS:

- `ariac_environment` (https://github.com/romb-technologies/ariac_environment)

- `navigation` (https://github.com/ros-planning/navigation)

- `obstacle_detector` (https://github.com/tysik/obstacle_detector)

- `timed_roslaunch` (https://github.com/MoriKen254/timed_roslaunch)

- `ira_laser_tools` (https://github.com/iralabdisco/ira_laser_tools)

#### ROS2:

- `ros1_bridge` (https://github.com/ros2/ros1_bridge)

- `navigation2` (https://github.com/ros-planning/navigation2/)


# Usage
## ROS
## Simulation - launch sequence 

**Note**: Before launching, the following environment variable must be set (in every terminal where nodes are launched):
```
export ROBOT_NAME=pioneer
```
Due to some hardcoded stuff, other `ROBOT_NAME` values will probably not work.

### 1st step - full_launch_local.launch

Launches all the main simulation and utility nodes, including:

- `fake_localization` node: set frame IDs accordingly
- `map_server` node: set frame ID accordingly
- `upload_actor.xml` include: set `start_x`, `start_y` and `actor_name` accordingly
- `guide_control.py` node:

   Parameters:  
   `~guide_distance`: desired distance to the guided actor  
   `~actor_tolerance`: how much the actor can move between samples (in m) and still be locked in  
   `~map_frame`: frame ID of the map  
   `~robot_frame`: frame ID of the robot  
   
   Subscribed topics:     
   `cmd_vel_dwb` (`geometry_msgs:Twist`): original navigation stack velocity to correct  
   `odom` (`nav_msgs:Odometry`): odometry topic of the robot  
   `obstacles` (`obstacle_detector:Obstacles`): list of tracked obstacles  
   
   Published topics:  
   `cmd_vel` (`geometry_msgs:Twist`): velocity to be used by the robot driver  
   `tracked_actor` (`obstacle_detector:CircleObstacle`): tracked actor position info  
   `robot_position` (`geometry_msgs:Pose`): robot position info   `

- in launch file `detect_obstacles.launch` under `laserscan_multi_merger` node set `destination_frame`, `scan_destination_topic` and `laserscan_topics` accordingly. Also play with the `obstacle_extractor_node` parameters.  

### 2nd step - actor_controller.launch

Launches the teleop-style controller for the actor spawner and driver that were launched previously.  
Argument: `actor_name`, set to the same value as the `actor_name` from the previous launch.

### 3rd step - move_base.launch

Launches the navigation stack with a remap from `cmd_vel` to `cmd_vel_dwb` so the `guide_control.py` node can correct the velocities. All the relevant parameters are set through YAML files in the navigation folder, according to `navigation` stack definitions.

### 4th step - set goal and control actor

Set a goal through the rviz tool or a publisher of your choice on the corresponding topic (navstack dependent, usually `move_base_simple/goal`). The robot should start navigating towards it and slow down as it moves away from the actor. Track the info about the actor through the terminal in which the first launch was called. Control the actor through the second therminal (`actor_controller.launch`).

## Pioneer P3-DX



Provided launch file for ROS2 requires completion of arguments `map_dir` (string, full path up to and including the `map.yaml` file) and `param_dir` (string, full path up to and including the `robot_params.yaml` file). The latter file format is as presented in https://github.com/ros-planning/navigation2/blob/master/nav2_bringup/launch/nav2_params.yaml, which can be used as-is for testing purposes.
