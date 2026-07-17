# seam_ros2_pkg

SEAM welding cell — FANUC CRX-10iA + ROBO WH-PP W500 torch.

- **Simulation:** Gazebo Harmonic + OAK-D (eye-in-hand) + MoveIt2 (ROS 2 Jazzy)
- **Physical:** Stream Motion via `fanuc_hardware_interface` + RealSense D435 + MoveIt2

## Dependencies

- [fanuc_description](https://github.com/CroboticSolutions/fanuc_description) (`fanuc_crx_description`)
- [fanuc_driver](https://github.com/FANUC-CORPORATION/fanuc_driver) (`fanuc_hardware_interface`)
- `realsense2_camera` / `realsense2_description`
- MoveIt2; for sim also `ros-jazzy-ros-gz-sim`, `ros-jazzy-gz-ros2-control`

## Simulation

```bash
cd ~/arms_ws && source install/setup.bash
ros2 launch seam_ros2_pkg crx_gz_sim.launch.py
```

MoveIt RViz: planning group `manipulator` → Plan & Execute.

## Physical (site)

Controller needs **J519 Stream Motion + R912 Remote Motion** (or **S636**), AUTO mode, TP OFF.

```bash
# optional: source config/fanuc_real.env.example (copy to a local .env first)
export FANUC_ROBOT_IP=192.168.1.100

# Driver + MoveIt (mock dry-run: use_mock:=true)
ros2 launch seam_ros2_pkg crx_real_d435.launch.py robot_ip:=$FANUC_ROBOT_IP

# Camera (publish_tf:=false — URDF owns TF)
ros2 launch realsense2_camera rs_launch.py \
  publish_tf:=false align_depth.enable:=true \
  pointcloud.enable:=true pointcloud.ordered_pc:=true

# arm_api2
ros2 launch arm_api2 moveit2_iface.launch.py \
  mode:=advanced robot_name:=crx10ia config_profile:=real launch_joy:=false
```

Or use the GUI setup wizard: **FANUC + RealSense D435 + welding torch + real**.

Smoke check after stack is up:

```bash
ros2 run seam_ros2_pkg smoke_fanuc_real.sh
# or: bash src/seam_ros2_pkg/scripts/smoke_fanuc_real.sh
```

### Hand-eye (D435 on J6)

1. Start calibration from the GUI (effector frame `J6_link`, optical `camera_color_optical_frame`).
2. Save YAML, then bake mount:

```bash
python3 -m hand_eye_calibration.realsense_mount_cli \
  --update-xacro src/seam_ros2_pkg/urdf/include/fanuc_realsense_d435_handeye_macros.xacro \
  --joint-name fanuc_j6_realsense_d435_mount
colcon build --packages-select seam_ros2_pkg --symlink-install
# restart stack
```

### GPIO / weld I/O

Edit `config/weld_gpio_config.yaml` with real DO/DI indices from the pendant before commanding weld outputs from ROS.

## Robot model

| Mode | URDF |
|------|------|
| Sim | `urdf/crx10ia_gz.urdf.xacro` |
| Real | `urdf/crx10ia_real.urdf.xacro` |

EE link: `welding_tcp`. Planning group: `manipulator`.
