
# UR3e-Control

This repository contains a ROS 2 Jazzy workspace to control a **Universal Robots UR3e** robotic arm with a **OnRobot 2FG7 gripper** and visualization using RViz2. The project enables robot motion planning, execution, and manipulation tasks, integrating MoveIt 2 and ROS 2 Control.

The program has been created for the robotic arm with an **attached camera to its end-effector**. In addition, the program is designed for a specific environment, were the drone structure and station structure are placed in specific relative loacation to the robot.

To see more details about the project, find the report here: [View Report]()

## 📁 Repository Structure

```
UR3e-Control/src
├── ur_custom/
│   ├── ur_custom_bringup/       # Custom launch and scripts
│   ├── ur_custom_driver/        # Driver nodes
│   └── ur_custom_description/   # URDF and meshes
└── README.md
```

### 📦 ROS 2 Packages

Clone the required packages in your workspace's `src` directory:

```bash
cd ~/ros2_ws/

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

## 🚦 Launch the Sequence Test Control
This code is working and the robot moves.

```bash
cd ~/ros2_ws
source install/setup.bash
bash "~/ros2_ws/src/ur_custom/ur_custom_bringup/scripts/start_sequence.sh"
```

## 📷 Launch the Program Control with Camera
As commented in the report, this code is not working. There is probably an error related with the camera node.

```bash
cd ~/ros2_ws
source install/setup.bash
source ~/ros2_ws/.venv/bin/activate
bash "~/ros2_ws/src/ur_custom/ur_custom_bringup/scripts/start_program.sh"
```

