from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):

    namespace = LaunchConfiguration('namespace').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    log_level = LaunchConfiguration('log_level')
    main_parameters_file = LaunchConfiguration('main_parameters_file')
    utility_nodes_to_launch = LaunchConfiguration(
        'utility_nodes_to_launch').perform(context)
    utility_nodes_list = repr(utility_nodes_to_launch)
    publish_agents_velocity = Node(
        package='core_reinforcement_learning',
        executable='publish_agents_velocity',
        name='publish_agents_velocity',
        output='screen',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[main_parameters_file],
        condition=IfCondition(PythonExpression([
            '"publish_agents_velocity" in ', utility_nodes_list
        ]))
    )

    construct_safe_corridor = Node(
        package='core_reinforcement_learning',
        executable='construct_safe_corridor_node',
        name='construct_safe_corridor',
        namespace=PathJoinSubstitution([namespace, robot_name]),
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[main_parameters_file],
        condition=IfCondition(PythonExpression([
            '"construct_safe_corridor" in ', utility_nodes_list
        ]))
    )

    train_launch = Node(
        package='core_reinforcement_learning',
        executable='train_rl',
        name='train_rl',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[main_parameters_file],

    )

    return [

        publish_agents_velocity,
        construct_safe_corridor,
        train_launch,
    ]

# Function to generate the launch description


def generate_launch_description():

    package_path = get_package_share_directory('core_reinforcement_learning')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Top-level namespace')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='log level')

    declare_utility_nodes_launch_cmd = DeclareLaunchArgument(
        'utility_nodes_to_launch', default_value='',
        description='Specify list of which utility nodes to launch. Currently, the options are: \
                     all, publish_agents_velocity, construct_safe_corridor, \
                     distance_to_undisturbed_path_node')

    declare_main_parameters_file_cmd = DeclareLaunchArgument(
        'main_parameters_file',
        default_value=os.path.join(package_path, 'config', 'main_params.yaml'),
        description='Full path to the ROS2 parameters file.')

    # Launch!
    return LaunchDescription([

        declare_namespace_cmd,
        declare_robot_name_cmd,
        declare_log_level_cmd,
        declare_utility_nodes_launch_cmd,
        declare_main_parameters_file_cmd,

        OpaqueFunction(function=launch_setup),
    ])


def main():
    ls = LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
