import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        package_path = get_package_share_directory('proud_up')
    except Exception:
        package_path = '/home/ubuntu/ros2_ws/src/proud_up'

    params = os.path.join(package_path, 'config', 'proud_up.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params,
            description='YAML parameters for proud_up_node',
        ),
        Node(
            package='proud_up',
            executable='proud_up_node',
            name='proud_up_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
