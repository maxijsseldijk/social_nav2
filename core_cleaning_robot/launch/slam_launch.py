import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():

    bringup_dir = get_package_share_directory('core_cleaning_robot')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_map_file = LaunchConfiguration('slam_map')
    log_level = LaunchConfiguration('log_level')

    namespaced_params_file_slam = ReplaceString(
        source_file=slam_params_file,
        replacements={'<robot_namespace>': (namespace)},
        condition=IfCondition(use_namespace)),

    param_substitutions_slam = {
        'map_file_name': slam_map_file,
        'use_sim_time': use_sim_time,
    }
    configured_slam_params = ParameterFile(
        RewrittenYaml(
            source_file=namespaced_params_file_slam,
            root_key=namespace,
            param_rewrites=param_substitutions_slam,
            convert_types=True),
        allow_substs=True)

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'mapper_params_online_async.yaml'),
        description='Full path to the SLAM parameters file to use for all launched nodes')

    declare_slam_map_file_cmd = DeclareLaunchArgument(
        'slam_map',
        default_value=os.path.join(get_package_share_directory(
            'core_gazebo_world'), 'maps', 'office_map_bigger', 'office_map'),
        description='Full path to map file to load for SLAM')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    slam_toolbox = Node(
        parameters=[configured_slam_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        arguments=['--ros-args', '--log-level', log_level],
        # Very important mapping to make sure the namespace can be added in the toolbox
        remappings=[(
                '/map', 'map',
        )],
        output='screen'
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        declare_slam_map_file_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,

        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        slam_toolbox
    ])
