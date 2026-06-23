# urdfyong — 6-Axis Robotic Arm URDF Package

ROS2 Humble robot description package for the urdfyong 6-axis arm.

## Contents

- `urdf/urdfyong.urdf` — URDF with 7 links, 6 revolute joints, realistic joint limits, ros2_control + Gazebo tags
- `meshes/` — STL visual/collision models (SolidWorks export)
- `launch/display.launch.py` — Quick visualization: RSP + joint_state_publisher_gui + RViz2

## Usage

```bash
ros2 launch urdfyong display.launch.py
```

Opens RViz2 with the full arm model. Drag joint sliders to articulate.

## URDF Notes

- Joint limits are set in radians (not degrees)
- `ros2_control` tag declares `gazebo_ros2_control/GazeboSystem` hardware plugin
- `$(find ...)` in URDF is not supported by gazebo_ros2_control — absolute path is used
- First line MUST NOT contain `<?xml?>` declaration (crashes spawn_entity.py)
