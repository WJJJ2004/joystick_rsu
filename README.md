# humanoid_manipulator
Intelligent Robot Competition – Humanoid Robot Sports

## Outline
> ROS2 package for autonomous control of a humanoid manipulator.  
> Supports Pinocchio-based IK computation, motion generation, and Teleop/Auto modes.

---

## 📁 Table of Contents

1. [Overview](#overview)  
2. [Installation & Execution](#installation--execution)  
3. [Parameter Tuning](#parameter-tuning)  
4. [Development Environment](#development-environment)  
5. [Dependencies](#dependencies)  

---

## Overview
The **humanoid_manipulator** package provides inverse kinematics, motion control, and collision-aware manipulation for a humanoid robot.  
It integrates:
- Pinocchio for kinematics and dynamics
- SRDF/URDF reference configurations
- Teleoperation and automatic vision-based modes

https://github.com/user-attachments/assets/2fbb4578-f151-4b23-b5c8-d9b63be6aeed


[스크린캐스트 09-29-2025 06_30_50 PM.webm](https://github.com/user-attachments/assets/cf3b055e-b6f5-40f3-8d1e-f2a122e52c54)

---

## Installation & Execution

### Run Teleop Mode
```bash

ros2 launch humanoid_manipulator teleop_mode.launch.py

```
### Run Auto Mode
```bash

ros2 launch humanoid_manipulator auto_mode.launch.py

```
### Run EE Teleop Node (keyboard input)
```bash

ros2 run humanoid_manipulator ee_teleop_node

```

## Parameter Tuning

Parameters are provided in config/ros_param.yaml.
Key options:

- MAX_ITERATIONS, POSITION_TOLERANCE, SE3_TOLERANCE

- USE_REFERENCE (SRDF reference configuration)

- IK_TILT_ONLY (z-axis alignment only mode)

- Orientation weights: YAW_WEIGHT, ROLL_WEIGHT, PITCH_WEIGHT

## Development Environment

OS: Ubuntu 22.04

ROS 2: Humble

Languages: C++17, Python 3.10.12

## Dependencies

ROS2 Humble core packages

Pinocchio

hpp-fcl (coal)

Eigen3

yaml-cpp
