import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_dir = LaunchConfiguration('map',
                                default="FULL PATH TO MAP.YAML"))

    param_dir = LaunchConfiguration('params',
                                default="FULL PATH TO ROBOT_PARAMETERS.YAML")

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_dir }]),

        Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[param_dir],
            remappings=[('cmd_vel', 'cmd_vel_dwb')]),

        Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/world_model', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/global_costmap/global_costmap', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/local_costmap/local_costmap', 'use_sim_time', use_sim_time],
            output='screen')
    ])
