#!/usr/bin/env python3
"""Gazebo Harmonic + MoveIt for FANUC CRX-10iA (piper_sim_moveit.launch.py pattern).

wrist_camera:
  gazebo_oak (default) — Gazebo RGB-D sensor + image bridge
  gazebo_realsense — simulated D435 (colour + depth sensors, D435 depth model)
  femto_bolt — simulated Orbbec Femto Bolt (colour + NFOV depth sensors)
  realsense_d435 — URDF D435 TF only; launch realsense2_camera separately
  none — no wrist camera in URDF (hybrid: identity static TF + real D435)

gazebo_realsense / femto_bolt publish /crx10ia/camera/{image_raw,camera_info,
depth/image_raw,points_reframed} (frame camera_color_optical_frame, aliased to
oak_rgb_camera_optical_frame like gazebo_oak).

model: crx10ia (default) | crx10ia_l
workpiece (crx10ia_l only): cylinder (default) | none -- static Cylinder.STL disk
  (D155 x 25 mm, meshes/cylinder_flat_m.stl) on the cell table top at
  workpiece_x / workpiece_y / workpiece_yaw in world (robot base axis = world origin)
calibration_board (crx10ia_l only): true (default) -- the PiPER sim ChArUco 6x8 board
  (25 mm squares, 18 mm markers, piper_gazebo charuco_6x8_25mm) on the cell table at
  board_x / board_y / board_yaw, for hand-eye calibration in simulation.
camera_calibration / tcp_calibration: mount and TCP YAMLs (crx_description format); '' =
  the model's active ~/.ros/calibration/<model>/.../current.yaml (GUI Apply), else the
  crx_description repo default.
"""

import os
import re

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


# GUI calibration store (hand_eye_calibration/calibration_store.py): one directory per
# robot model, Apply writes a version and repoints current.yaml.
_CRX_CAMERA_MOUNTS = {"femto_bolt": "femto_bolt_handeye", "realsense_d435": "realsense_d435_handeye"}


def _crx_calibration_file(model, mount, default_name):
    """Active calibration of this CRX for a mount, else the crx_description default."""
    store = os.path.join(
        os.path.expanduser(os.environ.get("ARMS_CALIBRATION_DIR", "~/.ros/calibration")),
        model, mount, "current.yaml")
    if os.path.exists(store):
        return store
    return os.path.join(get_package_share_directory("crx_description"),
                        "config", "calibration", default_name)


def _configure(context):
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1")
    wrist_camera = LaunchConfiguration("wrist_camera").perform(context).strip()
    if wrist_camera not in ("gazebo_oak", "gazebo_realsense", "femto_bolt", "realsense_d435", "none"):
        raise RuntimeError(
            f"unsupported wrist_camera={wrist_camera!r}; "
            "expected gazebo_oak, gazebo_realsense, femto_bolt, realsense_d435, or none"
        )
    model = LaunchConfiguration("model").perform(context).strip()
    if model not in ("crx10ia", "crx10ia_l"):
        raise RuntimeError(f"unsupported model={model!r}; expected crx10ia or crx10ia_l")
    workpiece = LaunchConfiguration("workpiece").perform(context).strip()
    if workpiece not in ("cylinder", "none"):
        raise RuntimeError(f"unsupported workpiece={workpiece!r}; expected cylinder or none")
    # Calibrated mount / TCP: explicit files, else the model's active store version.
    store_camera = {"femto_bolt": "femto_bolt", "gazebo_realsense": "realsense_d435",
                    "realsense_d435": "realsense_d435"}.get(wrist_camera)
    camera_calibration = LaunchConfiguration("camera_calibration").perform(context).strip()
    if not camera_calibration and store_camera:
        camera_calibration = _crx_calibration_file(
            model, _CRX_CAMERA_MOUNTS[store_camera], f"{store_camera}_handeye_default.yaml")
    tcp_calibration = LaunchConfiguration("tcp_calibration").perform(context).strip() or \
        _crx_calibration_file(model, "welding_tcp", "welding_tcp_default.yaml")
    no_gz_camera = wrist_camera in ("realsense_d435", "none")
    gz_rgbd_camera = wrist_camera in ("gazebo_realsense", "femto_bolt")

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
    if wrist_camera in ("realsense_d435", "gazebo_realsense"):
        try:
            resource_roots.insert(
                0, os.path.dirname(get_package_share_directory("realsense2_description"))
            )
        except Exception:
            pass
    if wrist_camera == "femto_bolt":
        resource_roots.insert(0, os.path.dirname(get_package_share_directory("orbbec_description")))
    urdf_file = os.path.join(pkg_gazebo, "urdf", "crx10ia_gz.urdf.xacro")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers.yaml")
    bridge_yaml = os.path.join(
        pkg_gazebo,
        "config",
        "fanuc_gz_bridge_clock_only.yaml" if no_gz_camera
        else "fanuc_gz_rgbd_bridge.yaml" if gz_rgbd_camera else "fanuc_gz_bridge.yaml",
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
            TextSubstitution(text=" model:="),
            TextSubstitution(text=model),
            TextSubstitution(text=f" camera_calibration:={camera_calibration}"),
            TextSubstitution(text=f" tcp_calibration:={tcp_calibration}"),
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

    if gz_rgbd_camera:
        # Same depth model/registration as the PiPER sim cameras, on the CRX topics.
        femto = wrist_camera == "femto_bolt"
        actions.append(
            Node(
                package="piper_gazebo",
                executable="d435_depth_processing.py",
                name="wrist_depth_processing",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    # Femto NFOV unbinned 0.25-5.46 m; ToF, so no stereo disparity noise.
                    "depth_min_m": 0.25 if femto else 0.28,
                    "depth_max_m": 5.46 if femto else 10.0,
                    "disparity_noise_stddev_px": 0.0 if femto else 0.08,
                }],
                remappings=[
                    ("/piper/camera/image_raw", "/crx10ia/camera/image_raw"),
                    ("/piper/camera/camera_info", "/crx10ia/camera/camera_info"),
                    ("/piper/camera/depth/camera_info", "/crx10ia/camera/depth/camera_info"),
                    ("/piper/camera/depth/ideal_image", "/crx10ia/camera/depth/ideal_image"),
                    ("/piper/camera/depth/image_raw", "/crx10ia/camera/depth/image_raw"),
                    ("/piper/camera/points_reframed", "/crx10ia/camera/points_reframed"),
                    ("/piper/camera/points", "/crx10ia/camera/points"),
                ],
            )
        )

    if model == "crx10ia_l" and workpiece == "cylinder":
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-string", _CYLINDER_WORKPIECE_SDF,
                    "-name", "cylinder_workpiece",
                    "-x", LaunchConfiguration("workpiece_x").perform(context),
                    "-y", LaunchConfiguration("workpiece_y").perform(context),
                    "-z", str(_crx10ia_l_table_top_height()),
                    "-Y", LaunchConfiguration("workpiece_yaw").perform(context),
                ],
            )
        )

    if model == "crx10ia_l" and LaunchConfiguration("calibration_board").perform(context).lower() in ("true", "1"):
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-file", os.path.join(get_package_share_directory("piper_gazebo"),
                                          "models", "charuco_6x8_25mm", "model.sdf"),
                    "-name", "charuco_board_6x8",
                    "-x", LaunchConfiguration("board_x").perform(context),
                    "-y", LaunchConfiguration("board_y").perform(context),
                    "-z", str(_crx10ia_l_table_top_height()),
                    "-Y", LaunchConfiguration("board_yaw").perform(context),
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
                "model": model,
                "camera_calibration": camera_calibration,
                "tcp_calibration": tcp_calibration,
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_move_group")),
        )
    )
    return actions


# Cylinder.STL (arm_api2_gui/models) in metres, Z up, base on z=0, centred in XY
# (same mesh as the PiPER sim workpiece).
_CYLINDER_WORKPIECE_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="cylinder_workpiece">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>model://seam_ros2_pkg/meshes/cylinder_flat_m.stl</uri></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://seam_ros2_pkg/meshes/cylinder_flat_m.stl</uri></mesh></geometry>
        <material>
          <ambient>0.65 0.68 0.72 1</ambient>
          <diffuse>0.65 0.68 0.72 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _crx10ia_l_table_top_height():
    """Table top z in world (= floor), read from the cell xacro so the workpiece follows it."""
    cell = os.path.join(
        get_package_share_directory("crx_description"), "urdf", "crx10ia_l", "crx10ia_l_cell.xacro"
    )
    with open(cell) as f:
        match = re.search(r'name="crx10ia_l_cell_table_top_height"\s+value="([0-9.]+)"', f.read())
    if match is None:
        raise RuntimeError(f"crx10ia_l_cell_table_top_height not found in {cell}")
    return float(match.group(1))


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
                description="crx10ia_l only: cylinder | none (static workpiece on the cell table)",
            ),
            DeclareLaunchArgument("workpiece_x", default_value="0.55",
                                  description="workpiece x in world [m] (table spans x 0.16..1.16)"),
            DeclareLaunchArgument("workpiece_y", default_value="0.0",
                                  description="workpiece y in world [m]"),
            DeclareLaunchArgument("workpiece_yaw", default_value="0.0",
                                  description="workpiece yaw in world [rad]"),
            DeclareLaunchArgument("calibration_board", default_value="true",
                                  description="crx10ia_l only: ChArUco 6x8 board on the cell table"),
            DeclareLaunchArgument("board_x", default_value="0.55", description="board x in world [m]"),
            DeclareLaunchArgument("board_y", default_value="0.35", description="board y in world [m]"),
            DeclareLaunchArgument("board_yaw", default_value="0.0", description="board yaw [rad]"),
            DeclareLaunchArgument("camera_calibration", default_value="",
                                  description="camera mount YAML ('' = crx_description default)"),
            DeclareLaunchArgument("tcp_calibration", default_value="",
                                  description="welding TCP YAML ('' = crx_description default)"),
            OpaqueFunction(function=_configure),
        ]
    )
