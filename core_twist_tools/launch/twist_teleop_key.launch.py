from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'teleop_key.yaml',
        )

    namespace = DeclareLaunchArgument('namespace', default_value='')
    cmd_vel = DeclareLaunchArgument('cmd_vel', default_value='cmd_vel')
    config = DeclareLaunchArgument('config', default_value=default_config)

    teleop_key_node = Node(
        package='core_twist_tools',
        executable='twist_teleop_key.py',
        name='twist_teleop_key',
        namespace=LaunchConfiguration('namespace'),
        parameters = [LaunchConfiguration('config')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel')),],
        prefix = 'xterm -e',
        output="screen"
        )

    return LaunchDescription([
        namespace,
        cmd_vel,
        config,
        teleop_key_node,
        ])