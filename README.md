# seam_ros2_pkg

SEAM welding cell — FANUC CRX-10iA + ROBO WH-PP W500 torch.

- **Simulation:** Gazebo Harmonic + OAK-D (eye-in-hand) + MoveIt2 (ROS 2 Jazzy)
- **Hybrid:** Gazebo Fanuc + ROBO WH-PP W500 + real Intel RealSense D435
- **Physical:** Stream Motion via `fanuc_hardware_interface` + wrist camera + MoveIt2

## Dependencies

- [fanuc_description](https://github.com/CroboticSolutions/fanuc_description) (`fanuc_crx_description`)
- [fanuc_driver](https://github.com/FANUC-CORPORATION/fanuc_driver) (`fanuc_hardware_interface`)
- `realsense2_camera` / `realsense2_description` (RealSense D435)
- `depthai_ros_driver_v3` / `depthai_descriptions_v3` (OAK-D Pro W / OAK-D SR)
- MoveIt2; for sim also `ros-jazzy-ros-gz-sim`, `ros-jazzy-gz-ros2-control`

## Simulation

```bash
cd ~/arms_ws && source install/setup.bash
ros2 launch seam_ros2_pkg crx_gz_sim.launch.py
```

MoveIt RViz: planning group `manipulator` → Plan & Execute.

## Hybrid (sim robot + real D435)

No Gazebo depth camera. Identity static TFs:

`J6_link` → `camera_depth_optical_frame` / `camera_color_optical_frame` / …

Real D435 images/pointclouds are stamp-synced to `/clock` so they transform into `world`.

**Time mismatch:** driver topics under `/camera_hw/...` keep wall-clock stamps.
Apps / RViz / Foxglove must use **`/camera/camera/...`** (sim-stamped) and
`use_sim_time:=true` (same `/clock` as Gazebo).

```bash
ros2 launch seam_ros2_pkg crx_gz_sim.launch.py wrist_camera:=none
ros2 launch seam_ros2_pkg fanuc_hybrid_realsense.launch.py
ros2 launch arm_api2 moveit2_iface.launch.py \
  mode:=advanced robot_name:=crx10ia use_sim_time:=true launch_joy:=false
```

Check stamp sync is alive and publishing sim time:

```bash
ros2 topic echo /camera/camera/depth/color/points --field header -n 1
# stamp should be small (sim seconds), not epoch wall time
ros2 param get /hybrid_camera_stamp_sync use_sim_time
```

Or GUI wizard: **FANUC + RealSense D435 + ROBO WH-PP W500 + Hybrid**.

## Physical (site)

Controller needs **J519 Stream Motion + R912 Remote Motion** (or **S636**), AUTO mode, TP OFF.

`camera_model` selects the URDF wrist mount: `realsense_d435` (default) | `oak_d_pro_w` | `oak_sr`.

```bash
# optional: source config/fanuc_real.env.example (copy to a local .env first)
export FANUC_ROBOT_IP=192.168.1.100

# Driver + MoveIt (mock dry-run: use_mock:=true)
ros2 launch seam_ros2_pkg crx_real_d435.launch.py \
  robot_ip:=$FANUC_ROBOT_IP camera_model:=realsense_d435

# Camera — pick one (URDF owns TF; RealSense must use publish_tf:=false)
ros2 launch realsense2_camera rs_launch.py \
  publish_tf:=false align_depth.enable:=true \
  pointcloud.enable:=true pointcloud.ordered_pc:=true
# or: ros2 launch depthai_ros_driver_v3 rgbd_pro_w_pcl.launch.py
# or: ros2 launch depthai_ros_driver_v3 sr_rgbd_pcl.launch.py

# arm_api2
ros2 launch arm_api2 moveit2_iface.launch.py \
  mode:=advanced robot_name:=crx10ia config_profile:=real launch_joy:=false
```

Or use the GUI setup wizard: **FANUC + (RealSense D435 | OAK-D Pro W | OAK-D SR) + welding torch + real**.

Smoke check after stack is up:

```bash
ros2 run seam_ros2_pkg smoke_fanuc_real.sh
# or: bash src/seam_ros2_pkg/scripts/smoke_fanuc_real.sh
```

### Hand-eye (wrist camera on J6)

1. Start calibration from the GUI (effector frame `J6_link`; optical frame depends on camera:
   D435 → `camera_color_optical_frame`, Pro W → `oak_rgb_camera_optical_frame`,
   SR → `oak_right_camera_optical_frame`).
2. Save YAML, then bake the matching mount xacro (D435 example):

```bash
python3 -m hand_eye_calibration.realsense_mount_cli \
  --update-xacro src/seam_ros2_pkg/urdf/include/fanuc_realsense_d435_handeye_macros.xacro \
  --joint-name fanuc_j6_realsense_d435_mount
colcon build --packages-select seam_ros2_pkg --symlink-install
# restart stack
```

OAK mounts start from a nominal J6 pose; re-calibrate and update
`fanuc_oak_d_pro_w_macros.xacro` / `fanuc_oak_d_sr_handeye_macros.xacro` before production use.

### GPIO / weld I/O

Edit `config/weld_gpio_config.yaml` with real DO/DI indices from the pendant before commanding weld outputs from ROS.

## Robot model

| Mode | URDF |
|------|------|
| Sim | `urdf/crx10ia_gz.urdf.xacro` |
| Real | `urdf/crx10ia_real.urdf.xacro` (`camera_model` arg) |

EE link: `welding_tcp`. Planning group: `manipulator`.
