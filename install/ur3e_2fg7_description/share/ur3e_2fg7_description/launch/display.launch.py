import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_ur3e_2fg7_description = get_package_share_directory('ur3e_2fg7_description')

    # Declare launch arguments
    urdf_file = os.path.join(pkg_ur3e_2fg7_description, 'urdf', 'ur3e_2fg7.urdf')
    rviz_config_file = os.path.join(pkg_ur3e_2fg7_description, 'rviz', 'ur3e_2fg7.rviz')

    # Create robot description from xacro
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}
    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False, 'publish_fixed_joints': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])