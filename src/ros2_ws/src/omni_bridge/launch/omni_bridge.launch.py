from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/noob/omni-pi/src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml',
            description='Path to omni bridge parameter file',
        ),
        ExecuteProcess(
            cmd=[
                '/home/noob/omni-pi/.venv_ros/bin/python',
                '/home/noob/omni-pi/src/ros2_ws/build/omni_bridge/omni_bridge/can_bridge_node.py',
                '--ros-args',
                '--params-file',
                params_file,
            ],
            additional_env={
                'PYTHONPATH': '/home/noob/omni-pi/src',
            },
            output='screen',
        ),
    ])
