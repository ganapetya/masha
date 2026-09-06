import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    enable_walk = _as_bool(LaunchConfiguration('enable_walk').perform(context))
    dry_run = _as_bool(LaunchConfiguration('dry_run').perform(context))

    node = Node(
        package='proud_up',
        executable='follow_the_cat_node',
        name='follow_the_cat_node',
        output='screen',
        parameters=[
            params_file,
            {
                'enable_walk': enable_walk,
                'dry_run': dry_run,
            },
        ],
    )
    return [node]


def generate_launch_description():
    try:
        package_path = get_package_share_directory('proud_up')
    except Exception:
        package_path = '/home/ubuntu/ros2_ws/src/proud_up'

    params = os.path.join(package_path, 'config', 'follow_the_cat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params,
            description='YAML parameters for follow_the_cat_node',
        ),
        DeclareLaunchArgument(
            'enable_walk',
            default_value='false',
            description='If true, walk 0.05 m/s for 3 s after a 2 s centered lock',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='Log pan/tilt pulses and skip servo / cmd_vel publishes',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
