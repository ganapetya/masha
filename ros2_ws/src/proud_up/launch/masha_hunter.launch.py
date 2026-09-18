"""Launch the spider hunter on Masha.

Order of operations (what ROS 2 actually starts):

  1. generate_launch_description() declares arguments and an OpaqueFunction.
  2. OpaqueFunction runs _launch_setup *after* substitutions are resolved,
     so we can parse enable_walk as a real bool (LaunchConfiguration is a
     string until .perform(context)).
  3. Always start masha_hunter_node. Parameter merge order:
       yaml file  →  then the extra dict (enable_walk, crab, vx_max, …).
     Later entries win. That is why launch args override masha_hunter.yaml.
  4. If enabled_targets contains "saveli", also start apriltag_ros inside
     a component_container. That node publishes TF child saveli_tag.
     Do NOT also launch vendor apriltag_recognition — two detectors fight.

The hunter node stays Idle until:
  ros2 service call /masha_hunter_node/start std_srvs/srv/Trigger

Launch args do not hot-reload. Ctrl-C, then relaunch.
"""

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
    # .perform(context) turns a LaunchConfiguration into a Python string
    # *now*, at launch time. The Node() below then gets real bools/floats.
    params_file = LaunchConfiguration('params_file').perform(context)
    apriltag_params = LaunchConfiguration('apriltag_params').perform(context)
    enable_walk = _as_bool(LaunchConfiguration('enable_walk').perform(context))
    enable_crab = _as_bool(LaunchConfiguration('enable_crab').perform(context))
    dry_run = _as_bool(LaunchConfiguration('dry_run').perform(context))
    enabled_targets = LaunchConfiguration('enabled_targets').perform(context)
    vx_max = float(LaunchConfiguration('vx_max').perform(context))
    vy_max = float(LaunchConfiguration('vy_max').perform(context))

    target_list = [t.strip() for t in enabled_targets.split(',') if t.strip()]
    hunter = Node(
        package='proud_up',
        executable='masha_hunter_node',  # CMake target installed to lib/proud_up/
        name='masha_hunter_node',
        output='screen',
        parameters=[
            params_file,  # first: yaml defaults
            {             # second: launch args win on these keys
                'enable_walk': enable_walk,
                'enable_crab': enable_crab,
                'dry_run': dry_run,
                'enabled_targets': target_list,
                'vx_max': vx_max,
                'vy_max': vy_max,
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
                        parameters=[
                            apriltag_params,
                            {
                                'family': '36h11',
                                'size': 0.06,
                                'threads': 2,
                                'max_hamming': 1,
                                'z_up': False,
                                'image_transport': 'raw',
                                'tag_ids': [0],
                                'tag_frames': ['saveli_tag'],
                                'tag_sizes': [0.06],
                            },
                        ],
                        remappings=[
                            # Relative and absolute names: the plugin has
                            # subscribed as both depending on distro/overlay.
                            ('image', '/depth_cam/rgb/image_raw'),
                            ('/image', '/depth_cam/rgb/image_raw'),
                            ('camera_info', '/depth_cam/rgb/camera_info'),
                            ('/camera_info', '/depth_cam/rgb/camera_info'),
                            # Plugin Node("apriltag") publishes /apriltag_detections.
                            # Pin the name so it does not depend on container namespace.
                            ('apriltag_detections', '/apriltag/apriltag_detections'),
                            ('/apriltag_detections', '/apriltag/apriltag_detections'),
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
            'enable_crab',
            default_value='true',
            description='If true, side offset uses vy (crab) when |bearing|<15 deg',
        ),
        DeclareLaunchArgument(
            'vx_max',
            default_value='0.12',
            description='Forward cap m/s (voice go-forward is 0.12)',
        ),
        DeclareLaunchArgument(
            'vy_max',
            default_value='0.08',
            description='Crab cap m/s',
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
