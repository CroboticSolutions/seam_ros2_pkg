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
                description="gazebo_oak | gazebo_realsense | femto_bolt | realsense_d435 | none",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="crx10ia",
                description="crx10ia | crx10ia_l",
            ),
            DeclareLaunchArgument(
                "workpiece",
                default_value="cylinder",
                description="crx10ia_l only: cylinder | none",
            ),
            DeclareLaunchArgument("workpiece_x", default_value="0.55"),
            DeclareLaunchArgument("workpiece_y", default_value="0.0"),
            DeclareLaunchArgument("workpiece_yaw", default_value="0.0"),
            DeclareLaunchArgument("calibration_board", default_value="true"),
            DeclareLaunchArgument("camera_calibration", default_value=""),
            DeclareLaunchArgument("tcp_calibration", default_value=""),
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
                    "model": LaunchConfiguration("model"),
                    "workpiece": LaunchConfiguration("workpiece"),
                    "workpiece_x": LaunchConfiguration("workpiece_x"),
                    "workpiece_y": LaunchConfiguration("workpiece_y"),
                    "workpiece_yaw": LaunchConfiguration("workpiece_yaw"),
                    "calibration_board": LaunchConfiguration("calibration_board"),
                    "camera_calibration": LaunchConfiguration("camera_calibration"),
                    "tcp_calibration": LaunchConfiguration("tcp_calibration"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "launch_move_group": LaunchConfiguration("launch_move_group"),
                }.items(),
            ),
        ]
    )
