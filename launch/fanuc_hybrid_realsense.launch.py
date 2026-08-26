#!/usr/bin/env python3
"""Hybrid real D435: static TFs on J6 + driver + sim-time stamp sync.

J6_link → RealSense frames with RPY (rad):
  roll  =  90° about X
  pitch =  0
  yaw   =  90° about Z

so optical Z (depth) aligns with the robot / world for the hybrid cell.

RealSense driver stays on wall time. Stamp-sync + static TF use /clock.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# RealSense pointcloud / color headers use these frames (publish_tf:=false).
_D435_FRAMES = (
    "camera_depth_optical_frame",
    "camera_color_optical_frame",
    "camera_depth_frame",
    "camera_color_frame",
    "camera_link",
)

# +90° Z, +90° X (user-tuned hybrid mount orientation).
_ROLL = str(math.radians(90.0))
_PITCH = "0"
_YAW = str(math.radians(90.0))


def generate_launch_description():
    parent_frame = LaunchConfiguration("parent_frame")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    static_nodes = []
    for child in _D435_FRAMES:
        static_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"hybrid_d435_tf_{child}",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    _ROLL,
                    "--pitch",
                    _PITCH,
                    "--yaw",
                    _YAW,
                    "--frame-id",
                    parent_frame,
                    "--child-frame-id",
                    child,
                ],
                parameters=[{"use_sim_time": True}],
                output="log",
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parent_frame",
                default_value="J6_link",
                description="Robot link for RealSense frames",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description=(
                    "Enable D435 point-cloud generation. Keep false unless a "
                    "consumer subscribes to /camera_hw/camera/depth/color/points."
                ),
            ),
            *static_nodes,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_namespace": "camera_hw",
                    "publish_tf": "false",
                    "align_depth.enable": "true",
                    "pointcloud.enable": enable_pointcloud,
                    "pointcloud.ordered_pc": enable_pointcloud,
                }.items(),
            ),
            Node(
                package="seam_ros2_pkg",
                executable="hybrid_camera_stamp_sync.py",
                name="hybrid_camera_stamp_sync",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_namespace": "/camera_hw/camera",
                        "output_namespace": "/camera/camera",
                    }
                ],
            ),
        ]
    )
