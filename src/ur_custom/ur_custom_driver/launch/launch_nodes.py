from launch import LaunchDescription
from launch_ros.actions import Node

# This file launches the two nodes 'joint_traj_publisher' and 'camera_server', being the main program writen into the first one
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_custom_driver',
            executable='joint_traj_publisher',
            name='joint_traj_publisher',
            output='screen'
        ),
        Node(
            package='ur_custom_driver',
            executable='camera_server',
            name='camera_server',
            output='screen'
        )
    ])