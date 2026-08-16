from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction


def launch_setup(context):
    asr_node = Node(
        package='xf_mic_asr_offline',
        executable='asr_node.py',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
    )
    return [asr_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
