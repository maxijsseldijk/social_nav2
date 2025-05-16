import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, \
    GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro
# Function to setup the launch


def launch_setup(context):

    namespace_value = LaunchConfiguration('namespace').perform(context)
    width = LaunchConfiguration('width').perform(context)
    length = LaunchConfiguration('length').perform(context)
    height = LaunchConfiguration('height').perform(context)
    use_social_zone = LaunchConfiguration('use_social_zone').perform(context)
    use_social_zone_robot = LaunchConfiguration(
        'use_social_zone_robot').perform(context)

    world = LaunchConfiguration('world')

    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='1.50'),
            'z': LaunchConfiguration('z_pose', default='0.1'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    use_slam = LaunchConfiguration('use_slam')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_map_file = LaunchConfiguration('slam_map')
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    main_parameters_file = LaunchConfiguration('main_parameters_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rl = LaunchConfiguration('use_rl')
    rl_controller_alg = LaunchConfiguration('rl_controller_alg')
    rl_path_length = LaunchConfiguration('rl_path_length')
    rl_path_samples = LaunchConfiguration('rl_path_samples')
    rl_action_output = LaunchConfiguration('rl_action_output')

    pkg_path = get_package_share_directory('core_cleaning_robot')
    launch_dir = os.path.join(pkg_path, 'launch')

    xacro_file = os.path.join(
        pkg_path, 'models', 'cleaning_robot', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file,
                                                  mappings={
                                                      'name': namespace_value,
                                                      'width': width,
                                                      'length': length,
                                                      'height': height,
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
    robot_joint_state_publisher = Node(

        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},

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
            f'/model/{namespace_value}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/model/{namespace_value}/scan/points@sensor_msgs/msg/PointCloud2'
            '[ignition.msgs.PointCloudPacked',
            f'/model/{namespace_value}/odometry_IMU@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{namespace_value}/odometry_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            (f'/model/{namespace_value}/pose',
             f'/{namespace_value}/pose'),
            (f'/model/{namespace_value}/cmd_vel',
             f'/{namespace_value}/cmd_vel'),
            (f'/model/{namespace_value}/scan', f'/{namespace_value}/scan'),
            (f'/model/{namespace_value}/scan/points',
             f'/{namespace_value}/scan/points'),
            (f'/model/{namespace_value}/odometry_IMU',
             f'/{namespace_value}/odometry'),
            (f'/model/{namespace_value}/odometry_tf', '/tf'),
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
    create_people_publisher = Node(
        package='core_cleaning_robot',
        executable='publish_robot_as_people_node',
        name='publish_robot_as_people_node',
        namespace=namespace,
        output='screen',
        parameters=[main_parameters_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_social_zone),
    )

    bringup_cmd = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map_yaml_file': map_yaml_file,
            'use_sim_time': use_sim_time,
            'nav2_params_file': nav2_params_file,
            'autostart': autostart,
            'rl_controller_alg': rl_controller_alg,
            'main_parameters_file': main_parameters_file,
            'use_rl': use_rl,
            'rl_path_length': rl_path_length,
            'rl_path_samples': rl_path_samples,
            'rl_action_output': rl_action_output,
            'use_respawn': use_respawn,
            'use_social_zone_robot': use_social_zone_robot,
            'log_level': log_level,

        },
    )

    slam_cmd = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')),
                condition=IfCondition(use_slam.perform(context)),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam_params_file': slam_params_file,
            'slam_map': slam_map_file,
            'use_sim_time': use_sim_time,
            'log_level': log_level,

        },
    )

    return [
        create_static_transform,
        ros_gz_sim_create_node,
        robot_state_publisher,
        robot_joint_state_publisher,
        create_people_publisher,
        ros_gz_bridge_node,
        slam_cmd,
        bringup_cmd,

    ]


# Function to generate the launch description
def generate_launch_description():

    pkg_path = get_package_share_directory('core_cleaning_robot')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world file name')
    declare_width_cmd = DeclareLaunchArgument(
        'width',
        default_value='0.3',
        description='Width of the Robot')
    declare_length_cmd = DeclareLaunchArgument(
        'length',
        default_value='0.3',
        description='Length of the Robot')
    declare_height_cmd = DeclareLaunchArgument(
        'height',
        default_value='0.15',
        description='Height of the Robot')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        description='Whether to start the SLAM node')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_path,
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the SLAM node')

    declare_slam_map_file_cmd = DeclareLaunchArgument(
        'slam_map',
        default_value=os.path.join(
            pkg_path, 'maps', 'office_map_bigger', 'office_map'),
        description='Full path to map file to load. This is without the yaml extention')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Top-level namespace')

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
        description='Whether to start the social zone node')

    declare_use_social_zone_robot_cmd = DeclareLaunchArgument(
        'use_social_zone_robot',
        default_value='False',
        description='Whether the robot should see agents with a social cost around them')

    declare_use_rl_cmd = DeclareLaunchArgument(
        'use_rl',
        default_value='False',
        description='Whether to start the rl node')

    declare_rl_controller_alg_cmd = DeclareLaunchArgument(
        'rl_controller_alg', default_value='nav2_rl_planner/HybridRLSMACPlanner',
        description='The reinforcement learning algorithm to use')

    declare_rl_path_length_cmd = DeclareLaunchArgument(
        'rl_path_length', default_value='2.0',
        description='The path length to use for the rl planner')
    declare_rl_path_samples_cmd = DeclareLaunchArgument(
        'rl_path_samples', default_value='10',
        description='The path samples to use for the rl planner')

    declare_rl_action_output_cmd = DeclareLaunchArgument(
        'rl_action_output', default_value='diff_drive',
        description='The action output to use for the rl planner either plan or diff_drive')

    return LaunchDescription([

        declare_world_cmd,
        declare_width_cmd,
        declare_length_cmd,
        declare_height_cmd,
        declare_use_slam_cmd,
        declare_slam_params_file_cmd,
        declare_slam_map_file_cmd,
        declare_use_namespace_cmd,
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_use_social_zone_cmd,
        declare_use_social_zone_robot_cmd,
        declare_use_rl_cmd,
        declare_rl_controller_alg_cmd,
        declare_rl_path_length_cmd,
        declare_rl_path_samples_cmd,
        declare_rl_action_output_cmd,

        OpaqueFunction(function=launch_setup),
    ])
