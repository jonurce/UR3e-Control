#!/bin/bash
# Single script to launch the ur3e with RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.100 launch_rviz:=false use_mock_hardware:=true description_launchfile:=/home/jon/Workspace/swap_ws/src/ur_custom/ur_custom_driver/launch/ur_custom_rsp.launch.py &

sleep 15

ros2 launch ur_custom_driver ur_custom_moveit.launch.py ur_type:=ur3e launch_rviz:=true &

sleep 10

#ros2 launch onrobot_2fg7 onrobot_launch.py ip:="192.168.0.100" &

#sleep 10

ros2 launch ur_custom_driver choice_ik_4.launch.py

# Keep the script running until Ctrl+C
wait