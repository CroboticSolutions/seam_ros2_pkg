# Welding torch mesh

Source STEP file provided locally by the user:

- `/root/Torch.STEP`

The STEP file was meshed with Gmsh and exported as binary STL for URDF visual
use in Gazebo/RViz. Collision geometry is simplified in
`urdf/welding_torch.xacro`.

The mesh units are millimeters. URDF loads it with `scale="0.001 0.001 0.001"`.
The `welding_tcp` frame is placed at the measured nozzle tip:

- `x=0.40642358`
- `y=-0.01193078`
- `z=0.0`
