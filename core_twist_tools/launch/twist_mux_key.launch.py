from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    default_mux_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'mux.yaml'
        )
    
    default_key_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config',
        'teleop_key.yaml'
        )

    namespace = DeclareLaunchArgument('namespace', default_value='')
    cmd_vel = DeclareLaunchArgument('cmd_vel', default_value='cmd_vel')
    mux_config = DeclareLaunchArgument('mux_config', default_value=default_mux_config)
    teleop_key_config = DeclareLaunchArgument('teleop_key_config', default_value=default_key_config)

    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('core_twist_tools'), 'launch', 'twist_mux.launch.py')
            ),
        launch_arguments={
            'namespace':LaunchConfiguration('namespace'),
            'cmd_vel':LaunchConfiguration('cmd_vel'),
            'config': LaunchConfiguration('mux_config'),
            }.items()
        )

    key_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('core_twist_tools'), 'launch', 'twist_teleop_key.launch.py')
            ),
        launch_arguments={
            'namespace':LaunchConfiguration('namespace'),
            'config': LaunchConfiguration('teleop_key_config'),
            'cmd_vel': 'cmd_vel_key',
            }.items()
        )
    
    return LaunchDescription([
        namespace, 
        cmd_vel,
        mux_config, 
        teleop_key_config, 
        mux_launch,
        key_launch,
        ])
