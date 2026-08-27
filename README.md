# joystick_rsu

This is a ROS 2 package for controlling and visualizing a 2-DOF RSU (Rotary Stewart Unit) ankle mechanism. It takes roll and pitch targets from a joystick and calculates the target angles of two actuators using inverse kinematics.

This repository contains preliminary research conducted for the follow-up project, [roa_controller](https://github.com/WJJJ2004/roa_controller).

## Features

- Inverse kinematics for the RSU ankle mechanism
- Joystick-based roll and pitch input
- Linkage and motion visualization in RViz
- Real-time control and debug modes

## Build and Run

Run the following commands from the root of your ROS 2 workspace:

```bash
colcon build --symlink-install --packages-select joystick_rsu
source install/setup.bash
```

Real-time control mode:

```bash
ros2 launch joystick_rsu solver_node.launch.py
```

Debug mode:

```bash
ros2 launch joystick_rsu debug.launch.py
```

## License

This project is licensed under the MIT License.
