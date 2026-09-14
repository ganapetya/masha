import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    node = Node(
        package='proud_up',
        executable='masha_interaction_node',
        name='masha_interaction_node',
        output='screen',
        parameters=[params_file],
    )
    return [node]


def generate_launch_description():
    try:
        package_path = get_package_share_directory('proud_up')
    except Exception:
        package_path = '/home/ubuntu/ros2_ws/src/proud_up'

    params = os.path.join(package_path, 'config', 'masha_interaction.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params,
            description='YAML parameters for masha_interaction_node',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
