import os
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

def launch_setup(context):
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    # String here. ParameterValue makes it a bool on the node.
    # Default false: gait 5 keeps kinematics.so.
    use_hexapod_kinematics = LaunchConfiguration(
        'use_hexapod_kinematics', default='false')

    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    # No node name: launch writes the parameter under /**, so both
    # nodes this process creates can declare it. StepController is
    # the one that does. The hunter launch does not own this flag.
    move_controller_node = Node(
        package='controller',
        executable='move_controller',
        output='screen',
        parameters=[{
            'use_hexapod_kinematics': ParameterValue(
                use_hexapod_kinematics, value_type=bool),
        }],
    )

    return [
        namespace_arg,
        use_namespace_arg,
        move_controller_node
    ]

def generate_launch_description():
    # Declared here, not inside the OpaqueFunction, so
    # `ros2 launch ... --show-args` lists it and the command line
    # can set it. Default false keeps kinematics.so.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_hexapod_kinematics',
            default_value='false',
            description=(
                'Gait 5 asks hexapod_kinematics instead of kinematics.so. '
                'False keeps the closed solver.')),
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
