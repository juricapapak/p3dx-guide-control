# p3dx-guide-control
A ROS package for guiding an actor using a Pioneer 3DX mobile robot, with a laser scan 

The package is intended for usage with ROS2 packages `ros1_bridge` and `navigation2`.

### Dependencies
#### ROS:

- `ariac_environment` (https://github.com/romb-technologies/ariac_environment)

- `obstacle_detector` (https://github.com/tysik/obstacle_detector)

#### ROS2:

- `ros1_bridge` (https://github.com/ros2/ros1_bridge)

- `navigation2` (https://github.com/ros-planning/navigation2/)


Provided launch file for ROS2 requires completion of arguments `map_dir` (string, full path up to and including the `map.yaml` file) and `param_dir` (string, full path up to and including the `robot_params.yaml` file). The latter file format is as presented in https://github.com/ros-planning/navigation2/blob/master/nav2_bringup/launch/nav2_params.yaml, which can be used as-is for testing purposes.
