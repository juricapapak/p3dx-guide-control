# p3dx-guide-control
A ROS package for guiding an actor using a Pioneer 3DX mobile robot, with a laser scan 

The package is intended for usage with ROS2 packages `ros1_bridge` and `navigation2`.

### Dependencies
#### ROS:

- `ariac_environment` (https://github.com/romb-technologies/ariac_environment)

- `obstacle_detector` (https://github.com/tysik/obstacle_detector)

#### ROS2:

- `ros1_bridge` (https://github.com/ros2/ros1_bridge)

-  `navigation2` (https://github.com/ros-planning/navigation2/)
	 - Before compiling the package, in `dwb_controller.cpp` change `/cmd_vel` to `/cmd_vel_dwb`.
