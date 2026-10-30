"""
TurtleBot3 Waffle + Gazebo Harmonic + Nav2 in the custom obstacle_course world.

Wraps Nav2's official Jazzy all-in-one simulation launch (tb3_simulation_launch.py)
and swaps in this package's world, map, Nav2 params and spawn pose.

    ros2 launch tb3_nav2_sim sim_nav.launch.py
    ros2 launch tb3_nav2_sim sim_nav.launch.py headless:=True      # no Gazebo GUI
    ros2 launch tb3_nav2_sim sim_nav.launch.py slam:=True          # build the map live instead
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('tb3_nav2_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'obstacle_course.sdf.xacro')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'obstacle_course.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('headless', default_value='False', description='True = no Gazebo GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('slam', default_value='False'),
        # Spawn pose - must match amcl.initial_pose in params/nav2_params.yaml
        DeclareLaunchArgument('x_pose', default_value='-4.0'),
        DeclareLaunchArgument('y_pose', default_value='-4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    sim_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'slam': LaunchConfiguration('slam'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': '0.01',
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription(args + [sim_and_nav])
