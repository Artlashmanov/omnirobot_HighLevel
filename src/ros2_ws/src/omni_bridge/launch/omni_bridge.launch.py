import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('omni_bridge')
    default_params_file = os.path.join(package_share, 'config', 'omni_bridge.params.yaml')
    params_file = LaunchConfiguration('params_file')
    platform_name = LaunchConfiguration('platform_name')
    platform_config = LaunchConfiguration('platform_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
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
        Node(
            package='omni_bridge',
            executable='can_bridge',
            name='omni_can_bridge',
            parameters=[params_file, {
                'platform_name': platform_name,
                'platform_config': platform_config,
            }],
            output='screen',
        ),
    ])
