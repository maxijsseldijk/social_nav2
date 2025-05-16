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
    
    default_teleop_key_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config',
        'teleop_key.yaml'
        )
    
    default_joy_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'joy.yaml',
        )

    default_teleop_joy_config = os.path.join(
        get_package_share_directory('core_twist_tools'),
        'config', 'teleop_joy.yaml',
        )

    namespace = DeclareLaunchArgument('namespace', default_value='')
    cmd_vel = DeclareLaunchArgument('cmd_vel_key', default_value='cmd_vel')
    mux_config = DeclareLaunchArgument('mux_config', default_value=default_mux_config)
    teleop_key_config = DeclareLaunchArgument('teleop_key_config', default_value=default_teleop_key_config)
    joy_config = DeclareLaunchArgument('joy_config', default_value=default_joy_config)
    teleop_joy_config = DeclareLaunchArgument('teleop_joy_config', default_value=default_teleop_joy_config)

    mux_key_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('core_twist_tools'), 'launch', 'twist_mux_key.launch.py')
            ),
        launch_arguments={
            'namespace':LaunchConfiguration('namespace'),
            'mux_config': LaunchConfiguration('mux_config'),
            'teleop_key_config': LaunchConfiguration('teleop_key_config'),
            }.items()
        )
    
    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('core_twist_tools'), 'launch', 'twist_teleop_joy.launch.py')
            ),
        launch_arguments={
            'namespace':LaunchConfiguration('namespace'),
            'cmd_vel': 'cmd_vel_joy',
            'joy_config': LaunchConfiguration('joy_config'),
            'teleop_joy_config': LaunchConfiguration('teleop_joy_config'),            
            }.items()
        )

    return LaunchDescription([
        namespace,
        cmd_vel,
        mux_config,
        teleop_key_config,
        joy_config,
        teleop_joy_config,
        mux_key_launch,
        teleop_joy_launch,
        ])