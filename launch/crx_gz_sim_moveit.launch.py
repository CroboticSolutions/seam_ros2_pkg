#!/usr/bin/env python3
"""Gazebo Harmonic + MoveIt for FANUC CRX-10iA (piper_sim_moveit.launch.py pattern)."""

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

    pkg_gazebo = get_package_share_directory("fanuc_gazebo")
    pkg_crx_desc = get_package_share_directory("fanuc_crx_description")
    pkg_crx_share_root = os.path.dirname(pkg_crx_desc)
    urdf_file = os.path.join(pkg_gazebo, "urdf", "crx10ia_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers.yaml")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            TextSubstitution(text=" "),
            TextSubstitution(text=urdf_file),
            TextSubstitution(text=" simulation_controllers:="),
            TextSubstitution(text=controllers_yaml),
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
            "0.05",
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
                "config_file": os.path.join(pkg_gazebo, "config", "fanuc_gz_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    moveit_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "fanuc_moveit_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": "true" if launch_rviz else "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_move_group")),
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner],
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=f"{pkg_crx_share_root}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}",
        ),
        gz_sim,
        gz_spawn_entity,
        robot_state_publisher_node,
        gz_bridge,
        delayed_spawners,
        moveit_stack_launch,
    ]


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
                    [FindPackageShare("fanuc_gazebo"), "worlds", "empty.sdf"]
                ),
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "launch_move_group",
                default_value="true",
                description="Launch MoveIt move_group and RViz",
            ),
            OpaqueFunction(function=_configure),
        ]
    )
