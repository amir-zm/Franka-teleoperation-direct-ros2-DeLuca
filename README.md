# franka_teleoperation

A ROS 2 package for teleoperation between two Franka Emika robots using Cartesian impedance control (local side) and joint-space impedance control (remote side). It provides separate nodes for local and remote control loops, each configurable via YAML parameter files.

---

## Features

- **Impedance control** on both local and remote robots  
- **Configurable stiffness** and **damping** gains per axis  
- Real‐time control via **libfranka**  
- **ROS 2 parameters** loaded from YAML files  
- Easily run **local**, **remote**  

---

## Package Contents
```text
franka_teleoperation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── local_impedance_params.yaml
│   └── remote_impedance_params.yaml
├── launch/                   # (optional) launch files
├── src/
│   ├── franka_teleoperation_local_cpp.cpps
│   ├── franka_teleoperation_remote_cpp.cpp
│   └── helper/
│       ├── FrankaLocal.cpp/.hpp
│       ├── FrankaRemote.cpp/.hpp
│       ├── convertArrayToEigenMatrix.hpp
│       ├── convertArrayToEigenVector.hpp
│       ├── jacobianMatrix.hpp
│       ├── coriolisTimesDqVector.hpp
│       ├── orientationErrorByPoses.hpp
│       ├── posErrorByPoses.hpp
│       └── jointTorquesSent.hpp
└── include/
    └── franka_teleoperation/  # public headers
```

---

## Dependencies

- **ROS 2 Humble** (or later)
- **libfranka** ≥ 0.14.1  
- **franka_msgs**, **sensor_msgs**, **std_msgs**  
- **Eigen3**  
- **ament_cmake**  

---

## Building

```bash
# From your ROS 2 workspace
cd ~/franka_ws/src
git clone <your-repo-url> franka_teleoperation
cd ~/franka_ws
colcon build --symlink-install


Configuration

Two parameter files live in config/:

    local_impedance_params.yaml

    remote_impedance_params.yaml

Each file should match the node name exactly:

# Example: for franka_teleoperation_local_node
franka_teleoperation_local_node:
  ros__parameters:
    robot_ip: "192.168.1.11"
    stiffness: [ 0, 0, 0, 0, 0, 0 ]
    damping:   [15, 15, 15, 10, 10, 10]


# Example: for franka_teleoperation_remote_node
franka_teleoperation_remote_node:
  ros__parameters:
    robot_ip: "192.168.1.12"
    stiffness: [81, 81, 81, 81, 81, 81, 81]
    damping:   [25, 25, 25, 25, 25, 25, 25]


Tip: Make sure the top‐level key matches exactly (franka_teleoperation_local_node vs franka_teleoperation_remote_node).



Usage: First, source your workspace:

sudo su
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_teleoperation --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/franka_ws/install/setup.bash

1. Local node
taskset -c 2,4,5 chrt --fifo 99 ros2 run franka_teleoperation franka_teleoperation_local_node --ros-args --params-file /home/mobilerobot/franka_ws/src/franka_teleoperation/config/local_impedance_params.yaml

2. remote node
source ~/franka_ws/install/setup.bash
taskset -c 1,3 chrt --fifo 99 ros2 run franka_teleoperation franka_teleoperation_remote_node --ros-args --params-file /home/mobilerobot/franka_ws/src/franka_teleoperation/config/remote_impedance_params.yaml


