from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/noob/omni-pi/src/ros2_ws/src/omni_bridge/launch/omni_bridge.launch.py'
            )
        ),
    ])
