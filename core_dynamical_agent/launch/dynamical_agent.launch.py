from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
# Define the path to the package


def generate_launch_description():

    package_path = get_package_share_directory('core_dynamical_agent')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    main_parameters_file = LaunchConfiguration('main_parameters_file')
    use_social_zone = LaunchConfiguration('use_social_zone')
    laser_enabled_agents = LaunchConfiguration('laser_enable_agents')
    agent_names = LaunchConfiguration('agent_names')
    use_telop = LaunchConfiguration('use_teleop')
    use_rl = LaunchConfiguration('use_rl')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(
            package_path, 'maps', 'office_map_bigger', 'office_map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_rl_cmd = DeclareLaunchArgument(
        'use_rl',
        default_value='False',
        description='Whether to use reinforcement learning to control the robot')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world name')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            package_path, 'config', 'nav2_params_agents.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_agent_names_cmd = DeclareLaunchArgument(
        'agent_names',
        default_value='',
        description='List of agent names to be used in the simulation. '
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='log level')

    declare_use_social_zone_cmd = DeclareLaunchArgument(
        'use_social_zone',
        default_value='False',
        description='Condition to use social zone for the human agents.'
        'If disabled the agents do not observe the robot through a social zone.')

    declare_laser_enable_agents_cmd = DeclareLaunchArgument(
        'laser_enable_agents',
        default_value='False',
        description='Condition to enable the laser for the agents.'
        'If enabled the agents have a laser scan.')

    declare_use_teleop_cmd = DeclareLaunchArgument(
        'use_teleop',
        default_value='False',
        description='Condition to enable the teleop keyboard for the robot.')

    agent_launch = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_path, 'launch', 'agent.launch.py')
            )
        )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'agent_names': agent_names,
            'autostart': autostart,
            'map_yaml_file': map_yaml_file,
            'nav2_params_file': nav2_params_file,
            'main_parameters_file': main_parameters_file,
            'use_respawn': use_respawn,
            'use_social_zone': use_social_zone,
            'world': world,
            'log_level': log_level,
            'laser_enable_agents': laser_enabled_agents,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],
        },
    )

    twist_mux = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'core_twist_tools'), 'launch', 'twist_mux.launch.py')
            )
        )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'config': os.path.join(package_path, 'config', 'mux.yaml'),
            'use_rl': use_rl,
        },
        condition=IfCondition(use_telop),

    )

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', [namespace, '/cmd_vel_key'])
        ],
        condition=IfCondition(use_telop),
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_nav2_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_agent_names_cmd,
        declare_use_rl_cmd,
        declare_world_cmd,
        declare_use_social_zone_cmd,
        declare_laser_enable_agents_cmd,
        declare_use_teleop_cmd,

        agent_launch,
        teleop_keyboard,
        twist_mux,

    ])
