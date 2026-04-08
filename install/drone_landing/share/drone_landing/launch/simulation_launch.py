"""
simulation_launch.py
====================
Launches the full autonomous-landing simulation:
  1. Gazebo with the landing world (models embedded in world file)
  2. Starts platform_mover, state_estimator, landing_controller (delayed 5 s)
  3. Optionally starts RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('drone_landing')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_dir, 'worlds', 'landing.world')
    rviz_cfg = os.path.join(pkg_dir, 'rviz', 'config.rviz')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # ---- Launch arguments ----
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 alongside the simulation')

    # ---- Gazebo (models are embedded in the world file) ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    # ---- Application nodes (delayed so Gazebo is ready) ----
    app_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_landing',
                executable='platform_mover',
                name='platform_mover',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='drone_landing',
                executable='state_estimator',
                name='state_estimator',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='drone_landing',
                executable='landing_controller',
                name='landing_controller',
                output='screen',
                parameters=[params_file],
            ),
        ],
    )

    # ---- RViz2 (optional) ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        gazebo,
        app_nodes,
        rviz_node,
    ])
