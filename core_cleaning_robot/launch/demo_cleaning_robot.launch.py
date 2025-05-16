from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from transforms3d.euler import quat2euler
from launch_ros.actions import Node
import os
import yaml


def load_parameters_from_yaml(yaml_file_path: str) -> dict:
    with open(yaml_file_path, 'r') as file:
        params = yaml.safe_load(file)

    return params


def get_yaml_param(params: dict, key: str) -> str:
    try:
        return params['/**']['ros__parameters'][key]
    except KeyError:
        return []


def get_value(param):
    if isinstance(param, list):
        return param[0]
    elif isinstance(param, str):
        return float(param)
    elif isinstance(param, float):
        return param
    else:
        raise ValueError(f"Unsupported parameter type: {type(param)}")


def get_pose_param_from_yaml(params: dict, entity: str, task: str) -> tuple:
    """
    Retrieve the Euler pose parameters from a YAML file.

    Args:
        params (dict): The YAML parameters. The orientation can be given in either
        Euler angles or quaternions.
        entity (str): The entity to retrieve the pose parameters for.

    Returns
    -------
        tuple: The x,y,z pose and roll, pitch, yaw orientation of the entity.

    """
    x_pose = get_value(get_yaml_param(params, 'TaskGenerator')[
                       f'{task}'][entity]['position']['x_pose'])
    y_pose = get_value(get_yaml_param(params, 'TaskGenerator')[
                       f'{task}'][entity]['position']['y_pose'])
    z_pose = get_value(get_yaml_param(params, 'TaskGenerator')[
                       f'{task}'][entity]['position']['z_pose'])

    if 'roll' not in get_yaml_param(params, 'TaskGenerator')[f'{task}'][entity]['orientation']:
        x_orientation = get_value(get_yaml_param(
            params, 'TaskGenerator')[f'{task}'][entity]['orientation']['x'])
        y_orientation = get_value(get_yaml_param(
            params, 'TaskGenerator')[f'{task}'][entity]['orientation']['y'])
        z_orientation = get_value(get_yaml_param(
            params, 'TaskGenerator')[f'{task}'][entity]['orientation']['z'])
        w_orientation = get_value(get_yaml_param(
            params, 'TaskGenerator')[f'{task}'][entity]['orientation']['w'])
        roll, pitch, yaw = quat2euler(
            [w_orientation, x_orientation, y_orientation, z_orientation])
    else:
        roll = get_value(get_yaml_param(params, 'TaskGenerator')[
                         f'{task}'][entity]['orientation']['roll'])
        pitch = get_value(get_yaml_param(params, 'TaskGenerator')[
                          f'{task}'][entity]['orientation']['pitch'])
        yaw = get_value(get_yaml_param(params, 'TaskGenerator')[
                        f'{task}'][entity]['orientation']['yaw'])

    return x_pose, y_pose, z_pose, roll, pitch, yaw


def generate_launch_description():

    package_path = get_package_share_directory('core_cleaning_robot')
    package_path_world = get_package_share_directory('core_gazebo_world')
    main_parameters_file_location = os.path.join(
        package_path, 'config', 'main_params.yaml')
    params = load_parameters_from_yaml(main_parameters_file_location)

    main_parameters_file = LaunchConfiguration(
        'main_parameters_file', default=main_parameters_file_location)

    use_namespace_param = get_yaml_param(params, 'use_namespace')
    use_namespace = LaunchConfiguration(
        'use_namespace', default=use_namespace_param)

    if bool(use_namespace_param):   # Namespace Configuration
        namespace = LaunchConfiguration(
            'namespace', default=get_yaml_param(params, 'namespace'))
    else:
        namespace = ''

    # World Configuration
    world = LaunchConfiguration(
        'world', default=get_yaml_param(params, 'world'))
    width = LaunchConfiguration(
        'width', default=get_yaml_param(params, 'width'))
    length = LaunchConfiguration(
        'length', default=get_yaml_param(params, 'length'))
    height = LaunchConfiguration(
        'height', default=get_yaml_param(params, 'height'))
    init_task_number = get_yaml_param(params, 'TaskGenerator')[
        'init_task_number']
    task_list = get_yaml_param(params, 'TaskGenerator')['task_list']
    try:
        task = task_list[init_task_number]
    except IndexError:
        raise IndexError(
            f"Task number {init_task_number} is not in the task list {task_list} try a number \
            between 0 and {len(task_list)-1}")

    use_teleop = LaunchConfiguration(
        'use_teleop', default=get_yaml_param(params, 'use_teleop'))
    use_slam = LaunchConfiguration(
        'use_slam', default=get_yaml_param(params, 'use_slam'))
    slam_params_file = LaunchConfiguration('slam_params_file', default=os.path.join(
        package_path, get_yaml_param(params, 'slam_params_file')))
    slam_map = LaunchConfiguration('slam_map', default=os.path.join(
        package_path_world, get_yaml_param(params, 'slam_map')))
    map_yaml_file = LaunchConfiguration('map_yaml_file', default=os.path.join(
        package_path_world, f"{get_yaml_param(params, 'map_yaml_file')}.yaml"))
    use_sim_time = LaunchConfiguration(
        'use_sim_time', default=get_yaml_param(params, 'use_sim_time'))
    nav2_params_file_robot = LaunchConfiguration('nav2_params_file_robot', default=os.path.join(
        package_path, get_yaml_param(params, 'nav2_params_file_robot')))

    # Launch Options
    autostart = LaunchConfiguration(
        'autostart', default=get_yaml_param(params, 'autostart'))
    use_respawn = LaunchConfiguration(
        'use_respawn', default=get_yaml_param(params, 'use_respawn'))
    log_level = LaunchConfiguration(
        'log_level', default=get_yaml_param(params, 'log_level'))

    # Reinforcement Learning
    use_rl = LaunchConfiguration(
        'use_rl', default=get_yaml_param(params, 'use_rl'))
    rl_controller_alg = LaunchConfiguration(
        'rl_controller_alg', default=get_yaml_param(params, 'rl_controller_alg'))
    rl_path_length = LaunchConfiguration(
        'rl_path_length', default=get_yaml_param(params, 'rl_path_length'))
    rl_path_samples = LaunchConfiguration(
        'rl_path_samples', default=get_yaml_param(params, 'rl_path_samples'))
    rl_action_output = LaunchConfiguration(
        'rl_action_output', default=get_yaml_param(params, 'rl_action_output'))

    utility_nodes_to_launch = LaunchConfiguration(
        'utility_nodes_to_launch', default=get_yaml_param(params, 'utility_nodes_to_launch'))

    # Social Interaction
    use_social_zone = LaunchConfiguration(
        'use_social_zone', default=get_yaml_param(params, 'use_social_zone'))
    use_social_zone_robot = LaunchConfiguration(
        'use_social_zone_robot', default=get_yaml_param(params, 'use_social_zone_robot'))

    # Agent Configuration
    laser_enable_agents = LaunchConfiguration(
        'laser_enable_agents', default=get_yaml_param(params, 'laser_enable_agents'))

    # Define service groups that communicate the pauze and unpause commands in gazebo to ros2
    set_pause_unpause_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='my_world_control',
        namespace=namespace,
        arguments=[
            ['/world/', world, '/control@ros_gz_interfaces/srv/ControlWorld']],
    )

    set_model_pose_service_bridge = Node(
        package='core_ros_gz_service_bridge',
        executable='service_bridge',
        name='my_model_control',
        namespace=namespace,
        arguments=[
            ['/world/', world,
             '/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean']],
    )

    gazebo_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_path_world, 'launch',
                                 'gazebo_empty.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'world': world,
        },
    )

    office_world_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_path_world, 'launch',
                                 'environment.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'namespace': namespace,
            'world': world,
        },
    )
    # Define commands for launching the navigation instances
    agent_instances_cmd = []

    for agent in get_yaml_param(params, 'agent_names'):
        agent_pose = get_pose_param_from_yaml(
            params, agent, task)
        print("\n")
        print("Launching agent: ", agent)
        print("\n")
        print(f"x pose: {agent_pose[0]}, y pose: {agent_pose[1]},  \
                z pose: {agent_pose[2]}, roll: {agent_pose[3]},  \
                pitch: {agent_pose[4]}, yaw: {agent_pose[5]}")
        agent_launch = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory(
                            'core_dynamical_agent'), 'launch', 'dynamical_agent.launch.py')
                    )
                )
            ],
            scoped=True,
            forwarding=False,
            launch_configurations={
                'namespace': PathJoinSubstitution([namespace, TextSubstitution(text=agent)]),
                'use_namespace': use_namespace,
                'agent_names': TextSubstitution(text=str(get_yaml_param(params, 'agent_names'))),
                'use_teleop': use_teleop,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map_yaml_file': map_yaml_file,
                'main_parameters_file': main_parameters_file,
                'use_respawn': use_respawn,
                'log_level': log_level,
                'use_rl': use_rl,
                'world': world,
                'use_social_zone': use_social_zone,
                'laser_enable_agents': laser_enable_agents,
                'x_pose': TextSubstitution(text=str(agent_pose[0])),
                'y_pose': TextSubstitution(text=str(agent_pose[1])),
                'z_pose': TextSubstitution(text=str(agent_pose[2])),
                'roll': TextSubstitution(text=str(agent_pose[3])),
                'pitch': TextSubstitution(text=str(agent_pose[4])),
                'yaw': TextSubstitution(text=str(agent_pose[5])),

            },
        )
        LogInfo(
            msg=['Launching ', agent]),

        agent_instances_cmd.append(agent_launch)

    robot_instance_cmd = []
    # Theoretically there can be multiple robots, but most plugins only support one robot
    for robot in get_yaml_param(params, 'robot_names'):
        robot_pose = get_pose_param_from_yaml(
            params, robot, task)
        robot_launch = GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_path, 'launch', 'robot.launch.py')
                )
            )
            ],
            scoped=True,
            forwarding=False,
            launch_configurations={
                'namespace': PathJoinSubstitution([namespace, TextSubstitution(text=str(robot))]),
                'use_namespace': use_namespace,
                'use_slam': use_slam,
                'slam_params_file': slam_params_file,
                'slam_map': slam_map,
                'map_yaml_file': map_yaml_file,
                'use_sim_time': use_sim_time,
                'nav2_params_file': nav2_params_file_robot,
                'autostart': autostart,
                'main_parameters_file': main_parameters_file,
                'use_respawn': use_respawn,
                'world': world,
                'width': width,
                'length': length,
                'height': height,
                'rl_controller_alg': rl_controller_alg,
                'use_rl': use_rl,
                'rl_path_length': rl_path_length,
                'rl_path_samples': rl_path_samples,
                'rl_action_output': rl_action_output,
                'use_social_zone': use_social_zone,
                'use_social_zone_robot': use_social_zone_robot,
                'x_pose': TextSubstitution(text=str(robot_pose[0])),
                'y_pose': TextSubstitution(text=str(robot_pose[1])),
                'z_pose': TextSubstitution(text=str(robot_pose[2])),
                'roll': TextSubstitution(text=str(robot_pose[3])),
                'pitch': TextSubstitution(text=str(robot_pose[4])),
                'yaw': TextSubstitution(text=str(robot_pose[5])),
                'log_level': log_level
            },
        )
        LogInfo(msg=['Launching ', robot]),
        robot_instance_cmd.append(robot_launch)

    reinforcement_learning_launch = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'core_reinforcement_learning'), 'launch', 'reinforcement_learning.launch.py')
            )
        )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            # As some topics are created in the namespace of the robot the rl uses the robot here.
            # Note that this will break if there are multiple robots defined.
            'namespace': namespace,
            'robot_name': robot,
            'main_parameters_file': main_parameters_file,
            'utility_nodes_to_launch': utility_nodes_to_launch,
        },
        condition=IfCondition(use_rl)

    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(package_path, 'config', 'view_bot.rviz'),
            '--ros-args', '--log-level', log_level,
        ],
        parameters=[{'use_sim_time': use_sim_time}],

        output='screen',
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
            'namespace': PathJoinSubstitution([namespace, TextSubstitution(text=str(robot))]),
            'config': os.path.join(package_path, 'config', 'mux.yaml'),
            'use_rl': use_rl,
        },
        condition=IfCondition(use_teleop)
    )

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', [PathJoinSubstitution([namespace,
             TextSubstitution(text=str(robot))]), '/cmd_vel_key'])
        ],
        condition=IfCondition(use_teleop)
    )

    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(office_world_launch)
    for robot_launch in robot_instance_cmd:
        launch_description.add_action(robot_launch)

    for agent_launch in agent_instances_cmd:
        launch_description.add_action(agent_launch)

    launch_description.add_action(teleop_keyboard)
    launch_description.add_action(twist_mux)
    launch_description.add_action(set_pause_unpause_service_bridge)
    launch_description.add_action(set_model_pose_service_bridge)

    # Small delay to make sure the world is loaded before launching the robot
    launch_description.add_action(TimerAction(
        period=2.0, actions=[reinforcement_learning_launch]))
    launch_description.add_action(TimerAction(period=5.0, actions=[rviz_node]))

    return launch_description
