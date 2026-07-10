# seam_ros2_pkg

SEAM robotic welding cell simulation — FANUC CRX-10iA with welding torch, OAK-D
camera and System-28 welding table + robot pedestal. Gazebo Harmonic + MoveIt2
(ROS 2 Jazzy).

## Dependencies

- [fanuc_description](https://github.com/CroboticSolutions/fanuc_description) (`fanuc_crx_description`)
- `ros-jazzy-ros-gz-sim`, `ros-jazzy-gz-ros2-control`, MoveIt2

## Launch

```bash
cd ~/arms_ws
source install/setup.bash
ros2 launch seam_ros2_pkg crx_gz_sim.launch.py
```

MoveIt RViz: planning group `manipulator` → Plan & Execute.

## Robot

Default model: **CRX-10iA** (`crx10ia_gz.urdf.xacro`).
