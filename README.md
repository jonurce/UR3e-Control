## Description
This package provides a ROS2 Jazzy interface for controlling the UR3e with an OnRobot 2FG7 gripper and a USB camera.  

Communication with the gripper's control box is achieved using XML-RPC, for which a repository will need to be cloned
https://github.com/Bkyb/onrobot_gripper_ros2.git


## Installation
Clone the mentioned repository and build the package in your ROS2 workspace:

```bash
# The xmlrpc-c library is used in this package
sudo apt update
sudo apt install libxmlrpc-c++8-dev libxmlrpc-core-c3-dev

# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Bkyb/onrobot_gripper_ros2.git

# Build the package
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

In a new terminal:
```bash
source ~/ros2_ws/install/setup.bash

```
