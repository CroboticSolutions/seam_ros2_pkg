#!/usr/bin/env python3
"""Backward-compatible alias for crx_gz_sim_moveit.launch.py."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("fanuc_gazebo"),
                            "launch",
                            "crx_gz_sim_moveit.launch.py",
                        ]
                    )
                )
            )
        ]
    )
