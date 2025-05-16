# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.actions import OpaqueFunction


def launch_func(context):

    namespace = LaunchConfiguration('namespace')
    namespace_value = namespace.perform(context)
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    main_parameters_file = LaunchConfiguration('main_parameters_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rl = LaunchConfiguration('use_rl')
    rl_controller_alg = LaunchConfiguration(
        'rl_controller_alg').perform(context)
    trim_controller_name = rl_controller_alg.split('::')[-1]

    rl_path_length = LaunchConfiguration('rl_path_length').perform(context)
    rl_path_samples = LaunchConfiguration('rl_path_samples').perform(context)

    rl_action_output = LaunchConfiguration('rl_action_output').perform(context)
    use_social_zone_robot = LaunchConfiguration('use_social_zone_robot')
    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother',
                       ]

    remappings = [('tf', '/tf'),
                  ('tf_static', '/tf_static')
                  ]
    namespaced_params_file_nav2 = ReplaceString(
        source_file=nav2_params_file,
        replacements={'<robot_namespace>': (namespace)}),

    namespaced_params_file_nav2 = ReplaceString(
        source_file=namespaced_params_file_nav2,
        replacements={'<rl_controller_alg>': rl_controller_alg,
                      '<trim_controller_name>': trim_controller_name,
                      '<rl_path_length>': rl_path_length,
                      '<rl_path_samples>': rl_path_samples,
                      '<rl_action_output>': rl_action_output,
                      '<use_social_zone_robot>': use_social_zone_robot},


        condition=IfCondition(use_rl)),

    param_substitutions_nav2 = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
    }

    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=namespaced_params_file_nav2,
            root_key=namespace,
            param_rewrites=param_substitutions_nav2,
            convert_types=True),
        allow_substs=True)

    load_nodes = GroupAction(
        actions=[


            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                remappings=remappings,
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_ctrl')],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),

            Node(
                package='core_nav2_navigation',
                executable='follow_single_waypoint.py',
                name='follow_single_waypoint',
                emulate_tty=True,
                output='screen',
                parameters=[main_parameters_file],
                remappings=[(f'{namespace_value}/amcl_pose',
                             f'{namespace_value}/pose')],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_nav2_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_ctrl')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
            ),

        ],
    )
    return [load_nodes]


def generate_launch_description():

    bringup_dir = get_package_share_directory('core_cleaning_robot')

    namespace = LaunchConfiguration('namespace')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file to use for all launched nodes')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(
            bringup_dir, 'maps', 'office_map_bigger', 'office_map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_waypoint_params_file_cmd = DeclareLaunchArgument(
        'waypoint_params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'waypoint_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_use_rl_cmd = DeclareLaunchArgument(
        'use_rl', default_value='False',
        description='Whether to use reinforcement learning for navigation')

    declare_rl_controller_alg_cmd = DeclareLaunchArgument(
        'rl_controller_alg', default_value='nav2_rl_planner/HybridRLSMACPlanner',
        description='The reinforcement learning algorithm to use')

    declare_use_social_zone_robot_cmd = DeclareLaunchArgument(
        'use_social_zone_robot',
        default_value='False',
        description='Whether the robot should see agents with a social cost around them')

    declare_rl_path_length_cmd = DeclareLaunchArgument(
        'rl_path_length', default_value='2.0',
        description='The path length to use for the rl planner')
    declare_rl_path_samples_cmd = DeclareLaunchArgument(
        'rl_path_samples', default_value='10',
        description='The path samples to use for the rl planner')
    declare_rl_action_output_cmd = DeclareLaunchArgument(
        'rl_action_output', default_value='diff_drive',
        description='The action output to use for the rl planner either plan or diff_drive')

    # Create the launch description and populate
    return LaunchDescription([
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_nav2_params_file_cmd,
        declare_waypoint_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_use_rl_cmd,
        declare_use_social_zone_robot_cmd,
        declare_rl_controller_alg_cmd,
        declare_rl_path_length_cmd,
        declare_rl_path_samples_cmd,
        declare_rl_action_output_cmd,

        PushRosNamespace(namespace=namespace),

        OpaqueFunction(function=launch_func)
    ])
