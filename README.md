
# 🤖 UR3e-Control

This repository contains a ROS 2 Jazzy workspace to control a **Universal Robots UR3e** robotic arm with a OnRobot 2FG7 gripper and visualization using RViz2. The project enables robot motion planning, execution, and manipulation tasks, integrating MoveIt 2 and ROS 2 Control.

The program has been created for the robotic arm with an attached camera to its end-effector.

## 📁 Repository Structure

```
UR3e-Control/
├── bringup/                 # Launch files to bring up the robot
├── config/                  # Controllers, planning parameters
├── description/             # URDF, meshes, and robot description
├── onrobot_gripper/         # OnRobot 2FG7 gripper integration
├── ur_moveit_config/        # MoveIt 2 configuration package
└── README.md
```

## 🧩 Required Dependencies

To build and run this workspace, install the following dependencies.

### 🐧 System Dependencies

```bash
sudo apt update
sudo apt install   libxmlrpc-c++8-dev   libxmlrpc-core-c3-dev   ros-jazzy-joint-state-publisher-gui   ros-jazzy-xacro   ros-jazzy-rviz2   ros-jazzy-robot-state-publisher   ros-jazzy-controller-manager   ros-jazzy-forward-command-controller   ros-jazzy-joint-trajectory-controller   ros-jazzy-ros2-control   ros-jazzy-ros2-controllers   ros-jazzy-ros2-control-test-assets
```

### 📦 ROS 2 Packages

Clone the required packages in your workspace's `src` directory:

```bash
cd ~/ros2_ws/src

# Main project repository
git clone https://github.com/jonurce/UR3e-Control.git

# OnRobot gripper ROS 2 interface
git clone https://github.com/Bkyb/onrobot_gripper_ros2.git
```

### 🔧 Build Instructions

```bash
cd ~/ros2_ws
colcon build
```

## 🔌 External Control Setup

To control the robot from an external computer:

1. Connect the UR3e via Ethernet.
2. On the robot's teach pendant, go to `Settings > Network` and set the network mode to **DHCP**.
3. Install the **External Control URCap** on the robot.
4. Use the robot's IP address in your external controller scripts or ROS 2 nodes.

## 🦾 OnRobot Gripper Integration

1. Install the **OnRobot URCap** for the 2FG7 gripper model.
2. Configure the gripper node via the `onrobot_gripper_ros2` package.
3. Control the gripper using ROS 2 services or topics exposed by the driver.

## Launch the sequence test control

```bash
cd ~/ros2_ws
source install/setup.bash
bash "~/ros2_ws/src/ur_custom/ur_custom_bringup/scripts/start_sequence.sh"
```

## Launch the program control with the camera

```bash
cd ~/ros2_ws
source install/setup.bash
source ~/ros2_ws/.venv/bin/activate
bash "~/ros2_ws/src/ur_custom/ur_custom_bringup/scripts/start_sequence.sh"
```

