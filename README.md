
# 🤖 UR3e-Control

This repository contains a ROS 2 Jazzy workspace to control a **Universal Robots UR3e** robotic arm with integrated support for external motion control, OnRobot grippers, and visualization using RViz2. The project enables robot motion planning, execution, and manipulation tasks, integrating MoveIt 2 and ROS 2 Control.

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

# Universal Robots ROS 2 Driver
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# UR description package
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
```

> ℹ️ Make sure you're using the `jazzy` branch if working with ROS 2 Jazzy.

### 🔧 Build Instructions

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### Launch the UR3e Robot

```bash
ros2 launch ur3e_bringup ur3e.launch.py
```

This launches the robot with controllers, the robot state publisher, and MoveIt 2 if configured.

### Visualize in RViz

```bash
ros2 launch ur3e_bringup rviz.launch.py
```

This launches the full RViz2 visualization with interactive markers for planning and executing motion.

## 🔌 External Control Setup

To control the robot from an external computer:

1. Connect the UR3e via Ethernet.
2. On the robot's teach pendant, go to `Settings > Network` and set the network mode to **DHCP**.
3. Install the **External Control URCap** on the robot.
4. Use the robot's IP address in your external controller scripts or ROS 2 nodes.

## 🦾 OnRobot Gripper Integration

1. Install the **OnRobot URCap** for your gripper model (e.g., 2FG7).
2. Configure the gripper node via the `onrobot_gripper_ros2` package.
3. Control the gripper using ROS 2 services or topics exposed by the driver.

## 📜 License

This project is under the MIT License. See the [LICENSE](LICENSE) file for details.

## 📬 Contact

Created and maintained by [@jonurce](https://github.com/jonurce)  
For issues or contributions, please open an [issue](https://github.com/jonurce/UR3e-Control/issues) or submit a pull request.
