from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_custom_driver',
            executable='joint_trajectory_publisher',
            name='joint_trajectory_publisher',
            output='screen'
        ),
        Node(
            package='ur_custom_driver',
            executable='camera_server',
            name='camera_server',
            output='screen'
        )
    ])