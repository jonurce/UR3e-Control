# Description: After a robot has been loaded, this will execute a series of trajectories.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    position_goals = PathJoinSubstitution(
        [FindPackageShare("ur_custom_driver"), "config", "goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[position_goals],
                output="screen",
            )
        ]
    )
