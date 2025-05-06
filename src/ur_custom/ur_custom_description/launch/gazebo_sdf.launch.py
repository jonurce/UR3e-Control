from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg_share = FindPackageShare('ur_custom_description').find('ur_custom_description')
    name = 'ur3e_onrobot_2fg7'
    #urdf_file = PathJoinSubstitution([pkg_share, 'urdf', f'{name}.urdf'])
    #urdf_file_path = urdf_file.perform(None)

    world_file = PathJoinSubstitution([pkg_share, 'sdf', 'world.sdf'])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    #gazebo = ExecuteProcess(
    #    cmd=['gz', 'service', '-s', '/world/empty/create',
    #         '--reqtype', 'gz.msgs.EntityFactory',
    #         '--reptype', 'gz.msgs.Boolean',
    #         '--timeout', '10000',
    #         '--req', f'sdf_filename: "{urdf_file_path}", name: "{name}"'
    #         ],
    #    output='screen'
    #)
    #)


    #gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory
    #--reptype gz.msgs.Boolean --timeout 1000
    #--req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'

    return LaunchDescription([gazebo])