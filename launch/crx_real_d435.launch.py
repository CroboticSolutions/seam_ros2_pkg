#!/usr/bin/env python3
"""Physical FANUC CRX-10iA + WH-PP W500 + MoveIt (camera launched separately).

Site / GUI wizard entry point. Example:

  ros2 launch seam_ros2_pkg crx_real_d435.launch.py \\
    robot_ip:=192.168.1.100 use_mock:=false camera_model:=realsense_d435

  camera_model: realsense_d435 | oak_d_pro_w | oak_sr

Offline dry-run (no robot):

  ros2 launch seam_ros2_pkg crx_real_d435.launch.py use_mock:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("seam_ros2_pkg")
    urdf_file = os.path.join(pkg, "urdf", "crx10ia_real.urdf.xacro")
    moveit_controllers = os.path.join(pkg, "config", "moveit_controllers_real.yaml")
    gpio_yaml = os.path.join(pkg, "config", "weld_gpio_config.yaml")

    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_mock = LaunchConfiguration("use_mock").perform(context)
    motion_control = LaunchConfiguration("motion_control").perform(context)
    welding_cell = LaunchConfiguration("welding_cell").perform(context)
    camera_model = LaunchConfiguration("camera_model").perform(context)
    gpio_configuration = LaunchConfiguration("gpio_configuration").perform(context)
    if not gpio_configuration:
        gpio_configuration = gpio_yaml

    ros2_control_config = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_hardware_interface"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    # ros2_controllers.yaml substitutes $(var gpio_config_*) and $(var prefix).

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " robot_ip:=",
            robot_ip,
            " use_mock:=",
            use_mock,
            " motion_control:=",
            motion_control,
            " welding_cell:=",
            welding_cell,
            " gpio_configuration:=",
            gpio_configuration,
            " camera_model:=",
            camera_model,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder("crx10ia", package_name="seam_ros2_pkg")
        .robot_description(
            file_path=urdf_file,
            mappings={
                "robot_ip": robot_ip,
                "use_mock": use_mock,
                "motion_control": motion_control,
                "welding_cell": welding_cell,
                "gpio_configuration": gpio_configuration,
                "camera_model": camera_model,
            },
        )
        .robot_description_semantic(file_path="srdf/crx10ia.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path=moveit_controllers)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    nodes = []

    nodes.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                ParameterFile(ros2_control_config, allow_substs=True),
            ],
            output="both",
        )
    )
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": False}],
        )
    )

    for controller in (
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "fanuc_gpio_controller",
        "fanuc_force_sensor_broadcaster",
        "force_torque_sensor_broadcaster",
        "forward_position_controller",
    ):
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "controller_manager",
                    "spawner",
                    "--controller-manager-timeout",
                    "180",
                    controller,
                ],
                output="screen",
            )
        )

    moveit_dict = moveit_config.to_dict()
    nodes.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_dict,
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "use_sim_time": False,
                },
            ],
        )
    )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": False},
            ],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory("seam_ros2_pkg")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("FANUC_ROBOT_IP", "192.168.1.100"),
                description="FANUC controller IP (Stream Motion / RMI)",
            ),
            DeclareBooleanLaunchArg("use_mock", default_value=False),
            DeclareBooleanLaunchArg("launch_rviz", default_value=False),
            DeclareBooleanLaunchArg("welding_cell", default_value=True),
            DeclareLaunchArgument(
                "motion_control",
                default_value="1",
                description="Initial FANUC motion_control state",
            ),
            DeclareLaunchArgument(
                "gpio_configuration",
                default_value="",
                description="Full path to GPIO YAML for the hardware interface plugin",
            ),
            DeclareLaunchArgument(
                "gpio_config_package",
                default_value="seam_ros2_pkg",
                description="Package containing GPIO YAML for fanuc_gpio_controller",
            ),
            DeclareLaunchArgument(
                "gpio_config_path",
                default_value="config/weld_gpio_config.yaml",
                description="GPIO YAML path relative to gpio_config_package share",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Joint name prefix (must match URDF)",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="realsense_d435",
                description=(
                    "Wrist camera URDF mount: realsense_d435 | oak_d_pro_w | oak_sr"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg, "config", "moveit.rviz"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
