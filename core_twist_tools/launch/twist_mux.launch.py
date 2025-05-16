from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    namespace = DeclareLaunchArgument('namespace', default_value='')
    cmd_vel = DeclareLaunchArgument('cmd_vel', default_value='cmd_vel')
    default_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'mux.yaml'
    )

    config = DeclareLaunchArgument('config', default_value=default_config)
    use_rl = DeclareLaunchArgument('use_rl', default_value='false')

    config_file = LaunchConfiguration('config')
    use_rl_file = LaunchConfiguration('use_rl')
    twist_mux_node = Node(
        package='core_twist_tools',
        executable='twist_mux.py',
        name='twist_mux',
        namespace=LaunchConfiguration('namespace'),
        parameters=[config_file, {'use_rl': use_rl_file}],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        namespace,
        cmd_vel,
        config,
        use_rl,
        twist_mux_node,
    ])
