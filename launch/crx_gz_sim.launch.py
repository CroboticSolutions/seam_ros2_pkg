#!/usr/bin/env python3
"""Backward-compatible alias for crx_gz_sim_moveit.launch.py."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist_camera",
                default_value="gazebo_oak",
                description="gazebo_oak | realsense_d435 | none",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo GUI",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch MoveIt RViz",
            ),
            DeclareLaunchArgument(
                "launch_move_group",
                default_value="true",
                description="Launch MoveIt move_group and RViz",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("seam_ros2_pkg"),
                            "launch",
                            "crx_gz_sim_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "wrist_camera": LaunchConfiguration("wrist_camera"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "launch_move_group": LaunchConfiguration("launch_move_group"),
                }.items(),
            ),
        ]
    )
