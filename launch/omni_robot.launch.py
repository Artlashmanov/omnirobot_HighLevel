import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    omni_bridge_share = get_package_share_directory('omni_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(omni_bridge_share, 'config', 'omni_bridge.params.yaml'),
            description='Path to omni bridge parameter file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(omni_bridge_share, 'launch', 'omni_bridge.launch.py')
            ),
            launch_arguments={'params_file': params_file}.items(),
        ),
    ])
