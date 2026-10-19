#!/usr/bin/env python3
"""MoveIt move_group + RViz for FANUC CRX Gazebo sim (piper_moveit_stack pattern)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("seam_ros2_pkg")
    urdf_file = os.path.join(pkg_gazebo, "urdf", "crx10ia_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers.yaml")
    moveit_controllers = os.path.join(pkg_gazebo, "config", "moveit_controllers.yaml")

    wrist_camera = LaunchConfiguration("wrist_camera")

    moveit_config = (
        MoveItConfigsBuilder("crx10ia", package_name="seam_ros2_pkg")
        .robot_description(
            file_path=urdf_file,
            mappings={
                "simulation_controllers": controllers_yaml,
                "wrist_camera": wrist_camera,
                "model": LaunchConfiguration("model"),
                "camera_calibration": LaunchConfiguration("camera_calibration"),
                "tcp_calibration": LaunchConfiguration("tcp_calibration"),
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

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock from Gazebo",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "wrist_camera",
            default_value="gazebo_oak",
            description="Must match crx_gz_sim wrist_camera "
                        "(gazebo_oak|gazebo_realsense|femto_bolt|realsense_d435|none)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "model",
            default_value="crx10ia",
            description="Must match crx_gz_sim model (crx10ia|crx10ia_l)",
        )
    )
    for name in ("camera_calibration", "tcp_calibration"):
        ld.add_action(DeclareLaunchArgument(name, default_value="",
                                            description="Must match crx_gz_sim " + name))
    ld.add_action(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch MoveIt RViz with MotionPlanning panel",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg_gazebo, "config", "moveit.rviz"),
        )
    )

    # Ensure RViz + move_group subscribe to /clock (hybrid D435 stamps are sim time).
    ld.add_action(SetParameter(name="use_sim_time", value=True))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    moveit_dict = moveit_config.to_dict()
    # move_group reads robot_description from robot_state_publisher's topic and respawns,
    # so GUI calibration Apply can push a new URDF and restart move_group alone.
    moveit_dict.pop("robot_description", None)
    use_sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    move_group_params = [
        moveit_dict,
        move_group_configuration,
        {"use_sim_time": use_sim},
    ]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            respawn=True,
            respawn_delay=1.0,
            parameters=move_group_params,
        )
    )

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": use_sim},
    ]

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=rviz_parameters,
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return ld
