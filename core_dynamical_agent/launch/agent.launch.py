import os
from ast import literal_eval

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def create_non_colliding_bitmask(agent_names: list, namespace_basename: str) -> str:
    """Create a non colliding bitmask between all the agents in a list."""
    agent_index = literal_eval(agent_names).index(namespace_basename)
    return hex(1 << agent_index)


def launch_setup(context):

    # Get the namespace and package path
    namespace = LaunchConfiguration('namespace')
    main_parameters_file = LaunchConfiguration('main_parameters_file')
    pkg_path = get_package_share_directory('core_dynamical_agent')
    launch_dir = os.path.join(pkg_path, 'launch')

    # Only get the agents namespace
    namespace_value = namespace.perform(context)
    namespace_basename = os.path.basename(os.path.normpath(namespace_value))

    radius = LaunchConfiguration('radius').perform(context)
    use_social_zone = LaunchConfiguration('use_social_zone')

    height = LaunchConfiguration('height').perform(context)
    world = LaunchConfiguration('world').perform(context)

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.1'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    agent_names = LaunchConfiguration('agent_names').perform(context)
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    laser_enable_agents = LaunchConfiguration(
        'laser_enable_agents').perform(context)

    # Assume the agents can not collide with each other to do this give the agents
    # different collide bitmasks
    # Find index of namespace in the agent_names list
    collide_bitmask = create_non_colliding_bitmask(
        agent_names, namespace_basename)

    # Get the robot description from the xacro file
    xacro_file = os.path.join(pkg_path, 'models', 'agent', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file,
                                                  mappings={
                                                      'name': namespace_value,
                                                      'radius': radius,
                                                      'height': height,
                                                      'laser_enable': laser_enable_agents,
                                                      'collide_bitmask': collide_bitmask,
                                                  }
                                                  )

    model_description = robot_description_config.toxml()

    ros_gz_sim_create_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        namespace=namespace,
        output='screen',
        parameters=[{'model_description': model_description}],
        arguments=[
            '-param', 'model_description',
            '-world', world,
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-R', pose['R'],
            '-P', pose['P'],
            '-Y', pose['Y'],
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': model_description},
            {'frame_prefix': f'{namespace_value}/'}
        ],
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace=namespace,
        arguments=[
            f'/model/{namespace_value}/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
            f'/model/{namespace_value}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/model/{namespace_value}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{namespace_value}/odometry_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{namespace_value}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/model/{namespace_value}/scan/points@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            (f'/model/{namespace_value}/pose',
             f'/{namespace_value}/pose'),
            (f'/model/{namespace_value}/cmd_vel',
             f'/{namespace_value}/cmd_vel'),
            (f'/model/{namespace_value}/odometry',
             f'/{namespace_value}/odometry'),
            (f'/model/{namespace_value}/odometry_tf', '/tf'),
            (f'/model/{namespace_value}/scan', f'/{namespace_value}/scan'),
            (f'/model/{namespace_value}/scan/points',
             f'/{namespace_value}/scan/points'),
        ],
    )

    create_static_transform = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['0', '0', '0', '0', '0', '0',
                           'world', f'{namespace_value}/map'],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0',
                           f'{namespace_value}/map', f'{namespace_value}/odom'],
                output='screen',
            ),
        ]
    )

    bringup_cmd = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_dynamic_agent_launch.py')),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'map_yaml_file': map_yaml_file,
            'use_social_zone': use_social_zone,
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params_file,
            'main_parameters_file': main_parameters_file,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,

        },
    )

    return [
        create_static_transform,
        ros_gz_sim_create_node,
        robot_state_publisher,
        ros_gz_bridge_node,
        bringup_cmd,
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('core_dynamical_agent')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world file name')

    declare_height_cmd = DeclareLaunchArgument(
        'height',
        default_value='0.08',
        description='Height of the Robot')

    declare_radius_cmd = DeclareLaunchArgument(
        'radius',
        default_value='0.13',
        description="Radius of the human agents",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='agent',
        description='Top-level namespace')

    declare_agent_names_cmd = DeclareLaunchArgument(
        'agent_names',
        default_value='',
        description='List of agent names to be used in the simulation. '
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(
            pkg_path, 'maps', 'office_map_bigger', 'office_map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

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

    return LaunchDescription([

        declare_world_cmd,
        declare_radius_cmd,
        declare_height_cmd,
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_agent_names_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_use_social_zone_cmd,
        declare_laser_enable_agents_cmd,

        OpaqueFunction(function=launch_setup),
    ])
