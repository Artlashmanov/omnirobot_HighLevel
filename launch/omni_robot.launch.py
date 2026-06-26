import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    platform_name = LaunchConfiguration('platform_name')
    platform_config = LaunchConfiguration('platform_config')
    omni_bridge_share = get_package_share_directory('omni_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(omni_bridge_share, 'config', 'omni_bridge.params.yaml'),
            description='Path to omni bridge parameter file',
        ),
        DeclareLaunchArgument(
            'platform_name',
            default_value=os.environ.get('ROBOT_PLATFORM', 'omni4'),
            description='Robot platform profile name',
        ),
        DeclareLaunchArgument(
            'platform_config',
            default_value=os.environ.get('OMNI_PLATFORM_CONFIG', ''),
            description='Path to robot platform profile config',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(omni_bridge_share, 'launch', 'omni_bridge.launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'platform_name': platform_name,
                'platform_config': platform_config,
            }.items(),
        ),
    ])
