#!/usr/bin/env python3
"""Identity TF aliases: OAK eye-in-hand → RealSense D435 frame names.

Use when the FANUC cell keeps the existing OAK mount/frames but apps expect
realsense2_camera frame ids (publish_tf:=false on the driver).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_D435_FRAME_ALIASES = (
    "camera_color_optical_frame",
    "camera_depth_optical_frame",
    "camera_color_frame",
    "camera_depth_frame",
    "camera_link",
)


def generate_launch_description():
    parent = LaunchConfiguration("parent_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nodes = [
        DeclareLaunchArgument(
            "parent_frame",
            default_value="oak_rgb_camera_optical_frame",
            description="Existing FANUC wrist camera optical frame (OAK mount)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Match Gazebo / hybrid use_sim_time",
        ),
    ]
    for child in _D435_FRAME_ALIASES:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"alias_{child}",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    parent,
                    "--child-frame-id",
                    child,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="log",
            )
        )
    return LaunchDescription(nodes)
