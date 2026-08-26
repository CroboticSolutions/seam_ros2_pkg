#!/usr/bin/env python3
"""Gazebo Harmonic + MoveIt for FANUC CRX-10iA (piper_sim_moveit.launch.py pattern).

wrist_camera:
  gazebo_oak (default) — Gazebo RGB-D sensor + image bridge
  realsense_d435 — URDF D435 TF only; launch realsense2_camera separately
  none — no wrist camera in URDF (hybrid: identity static TF + real D435)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _configure(context):
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1")
    wrist_camera = LaunchConfiguration("wrist_camera").perform(context).strip()
    if wrist_camera not in ("gazebo_oak", "realsense_d435", "none"):
        raise RuntimeError(
            f"unsupported wrist_camera={wrist_camera!r}; "
            "expected gazebo_oak, realsense_d435, or none"
        )
    no_gz_camera = wrist_camera in ("realsense_d435", "none")

    pkg_gazebo = get_package_share_directory("seam_ros2_pkg")
    pkg_crx_desc = get_package_share_directory("fanuc_crx_description")
    pkg_depthai_desc = get_package_share_directory("depthai_descriptions_v3")
    pkg_gazebo_share_root = os.path.dirname(pkg_gazebo)
    pkg_crx_share_root = os.path.dirname(pkg_crx_desc)
    pkg_depthai_share_root = os.path.dirname(pkg_depthai_desc)
    resource_roots = [
        pkg_depthai_share_root,
        pkg_crx_share_root,
        pkg_gazebo_share_root,
    ]
    if wrist_camera == "realsense_d435":
        try:
            resource_roots.insert(
                0, os.path.dirname(get_package_share_directory("realsense2_description"))
            )
        except Exception:
            pass
    urdf_file = os.path.join(pkg_gazebo, "urdf", "crx10ia_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers.yaml")
    bridge_yaml = os.path.join(
        pkg_gazebo,
        "config",
        "fanuc_gz_bridge_clock_only.yaml" if no_gz_camera else "fanuc_gz_bridge.yaml",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
            TextSubstitution(text=" wrist_camera:="),
            TextSubstitution(text=wrist_camera),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "crx10ia",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.0",
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                LaunchConfiguration("gazebo_gui"),
                if_value=["-r -v 4 ", LaunchConfiguration("world_file")],
                else_value=["-s -r -v 4 ", LaunchConfiguration("world_file")],
            )
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": bridge_yaml,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    actions = [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=(
                ":".join(resource_roots)
                + ":"
                + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
            ),
        ),
        gz_sim,
        gz_spawn_entity,
        robot_state_publisher_node,
        gz_bridge,
    ]

    if wrist_camera == "gazebo_oak":
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--frame-id",
                    "oak_rgb_camera_frame",
                    "--child-frame-id",
                    "crx10ia/J6_link/camera",
                ],
                parameters=[{"use_sim_time": True}],
                output="log",
            )
        )
        actions.append(
            Node(
                package="piper_gazebo",
                executable="pointcloud_reframe.py",
                output="screen",
                parameters=[
                    {"input_topic": "/crx10ia/camera/points"},
                    {"output_topic": "/crx10ia/camera/points_reframed"},
                    {"frame_id": "oak_rgb_camera_optical_frame"},
                    {"xyz_transform": "gazebo_camera_to_optical"},
                ],
            )
        )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner],
    )
    actions.append(delayed_spawners)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, "launch", "fanuc_moveit_stack.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "launch_rviz": "true" if launch_rviz else "false",
                "wrist_camera": wrist_camera,
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_move_group")),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
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
                "world_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("seam_ros2_pkg"), "worlds", "empty.sdf"]
                ),
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "launch_move_group",
                default_value="true",
                description="Launch MoveIt move_group and RViz",
            ),
            DeclareLaunchArgument(
                "wrist_camera",
                default_value="gazebo_oak",
                description="gazebo_oak | realsense_d435 | none",
            ),
            OpaqueFunction(function=_configure),
        ]
    )
