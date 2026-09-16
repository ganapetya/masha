import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    apriltag_params = LaunchConfiguration('apriltag_params').perform(context)
    enable_walk = _as_bool(LaunchConfiguration('enable_walk').perform(context))
    dry_run = _as_bool(LaunchConfiguration('dry_run').perform(context))
    enabled_targets = LaunchConfiguration('enabled_targets').perform(context)

    hunter = Node(
        package='proud_up',
        executable='masha_hunter_node',
        name='masha_hunter_node',
        output='screen',
        parameters=[
            params_file,
            {
                'enable_walk': enable_walk,
                'dry_run': dry_run,
            },
        ],
    )

    nodes = [hunter]

    # Stock apriltag_ros only when Saveli is an enabled plug. Do not also
    # launch vendor apriltag_recognition — two detectors fight.
    if 'saveli' in enabled_targets.lower():
        nodes.append(
            ComposableNodeContainer(
                name='tag_container',
                namespace='apriltag',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        name='apriltag',
                        package='apriltag_ros',
                        plugin='AprilTagNode',
                        parameters=[apriltag_params],
                        remappings=[
                            ('image', '/depth_cam/rgb/image_raw'),
                            ('/image', '/depth_cam/rgb/image_raw'),
                            ('camera_info', '/depth_cam/rgb/camera_info'),
                            ('/camera_info', '/depth_cam/rgb/camera_info'),
                        ],
                    ),
                ],
                output='screen',
            )
        )
    return nodes


def generate_launch_description():
    try:
        package_path = get_package_share_directory('proud_up')
    except Exception:
        package_path = '/home/ubuntu/ros2_ws/src/proud_up'

    params = os.path.join(package_path, 'config', 'masha_hunter.yaml')
    apriltag = os.path.join(package_path, 'config', 'apriltag_36h11_savelij.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params,
            description='YAML parameters for masha_hunter_node',
        ),
        DeclareLaunchArgument(
            'apriltag_params',
            default_value=apriltag,
            description='apriltag_ros yaml (measured tag size, not the 0.08 default)',
        ),
        DeclareLaunchArgument(
            'enable_walk',
            default_value='false',
            description='If false, HUNT/NAME only: log Twist, do not publish cmd_vel',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='Log servo / Traveling / Twist and skip publishes',
        ),
        DeclareLaunchArgument(
            'enabled_targets',
            default_value='saveli',
            description='Which plugs to run; apriltag_ros starts only if saveli is listed',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
