from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_joy_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'joy.yaml',
        )

    default_teleop_joy_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'teleop_joy.yaml',
        )

    namespace = DeclareLaunchArgument('namespace', default_value='')
    cmd_vel = DeclareLaunchArgument('cmd_vel', default_value='cmd_vel')
    joy_config = DeclareLaunchArgument('joy_config', default_value=default_joy_config)
    teleop_joy_config = DeclareLaunchArgument('teleop_joy_config', default_value=default_teleop_joy_config)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters = [LaunchConfiguration('joy_config')],
        output='screen',
        )

    teleop_joy_node = Node(
        package='core_twist_tools',
        executable='twist_teleop_joy.py',
        name='twist_teleop_joy',
        namespace=LaunchConfiguration('namespace'),
        parameters = [LaunchConfiguration('teleop_joy_config')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))],
        output='screen',
        )

    return LaunchDescription([
        namespace,
        cmd_vel,
        joy_config,
        teleop_joy_config,
        joy_node,
        teleop_joy_node,
        ])