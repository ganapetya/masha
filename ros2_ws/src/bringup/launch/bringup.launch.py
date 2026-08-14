import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

_TRUE = {'true', '1', 'yes', 'on'}
_FALSE = {'false', '0', 'no', 'off'}


def _resolve_flag(context, name, profile_is_full):
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw in _TRUE:
        return True
    if raw in _FALSE:
        return False
    return profile_is_full


def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    profile = LaunchConfiguration('profile').perform(context).strip().lower()
    if profile not in ('slim', 'full'):
        profile = 'slim'
    profile_is_full = profile == 'full'

    start_apps = _resolve_flag(context, 'start_apps', profile_is_full)
    joystick = _resolve_flag(context, 'joystick', profile_is_full)
    rosbridge = _resolve_flag(context, 'rosbridge', profile_is_full)
    web_video = _resolve_flag(context, 'web_video', profile_is_full)
    voice = _resolve_flag(context, 'voice', profile_is_full)

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
    )

    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_package_path, 'launch/init_pose.launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'action_name': 'init',
        }.items(),
    )

    startup_check_node = Node(
        package='bringup',
        executable='startup_check',
        output='screen',
        parameters=[{'enable_voice': voice}],
    )

    actions = [
        startup_check_node,
        controller_launch,
        depth_camera_launch,
        lidar_launch,
        init_pose_launch,
    ]

    if rosbridge:
        actions.append(ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen',
        ))

    if web_video:
        actions.append(Node(
            package='web_video_server',
            executable='web_video_server',
            output='screen',
        ))

    if start_apps:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(app_package_path, 'launch/start_app.launch.py')),
        ))

    if joystick:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value=os.environ.get('BRINGUP_PROFILE', 'slim'),
            description='slim (default): motion+sensors. full: vendor phone/demo tree.',
        ),
        DeclareLaunchArgument(
            'start_apps',
            default_value=os.environ.get('BRINGUP_START_APPS', 'auto'),
            description='Start start_app.launch.py demo bundle. auto follows profile.',
        ),
        DeclareLaunchArgument(
            'joystick',
            default_value=os.environ.get('BRINGUP_JOYSTICK', 'auto'),
            description='Start joystick_control. auto follows profile.',
        ),
        DeclareLaunchArgument(
            'rosbridge',
            default_value=os.environ.get('BRINGUP_ROSBRIDGE', 'auto'),
            description='Start rosbridge_websocket on :9090. auto follows profile.',
        ),
        DeclareLaunchArgument(
            'web_video',
            default_value=os.environ.get('BRINGUP_WEB_VIDEO', 'auto'),
            description='Start web_video_server on :8080. auto follows profile.',
        ),
        DeclareLaunchArgument(
            'voice',
            default_value=os.environ.get('BRINGUP_VOICE', 'auto'),
            description='Let startup_check launch the voice stack if /dev/ring_mic exists. auto follows profile.',
        ),
        OpaqueFunction(function=launch_setup),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
