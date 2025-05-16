from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro
import os


def launch_setup(context):

    namespace = LaunchConfiguration('namespace').perform(context)
    world = LaunchConfiguration('world').perform(context)

    if world == 'custom_office':
        model_sdf_xacro = os.path.join(
            get_package_share_directory('core_gazebo_world'),
            'models', world, 'office_geometry.urdf.xacro'
        )
    else:
        model_sdf_xacro = os.path.join(
            get_package_share_directory('core_gazebo_world'),
            'models', world, 'model.sdf'
        )

    model_sdf = xacro.process_file(
        model_sdf_xacro,
        mappings={
            'name': LaunchConfiguration('namespace').perform(context),
            'width': LaunchConfiguration('width').perform(context),
            'length': LaunchConfiguration('length').perform(context),
            'height': LaunchConfiguration('height').perform(context),
            'enable_collision': LaunchConfiguration('enable_collision').perform(context),
            'material_ambient': LaunchConfiguration('color').perform(context),
            'material_diffuse': LaunchConfiguration('color').perform(context),
            'material_specular': '0.1 0.1 0.1 1',
        },
    )

    model_description = model_sdf.toprettyxml(indent='  ')

    ros_gz_sim_create_node = Node(
        package='ros_gz_sim',  # Replace with the actual name of your package
        executable='create',  # Replace with the actual name of your executable
        name='create',
        namespace=namespace,
        output='screen',
        parameters=[{'model_description': model_description}],
        arguments=[
            '-param', 'model_description',
            '-world', LaunchConfiguration('world'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    return [ros_gz_sim_create_node]


def generate_launch_description():

    namespace = DeclareLaunchArgument('namespace', default_value='box')
    world = DeclareLaunchArgument(
        'world', default_value='empty', description='Gazebo world name')
    width = DeclareLaunchArgument('width', default_value='1.0')
    length = DeclareLaunchArgument('length', default_value='1.0')
    height = DeclareLaunchArgument('height', default_value='1.0')
    color = DeclareLaunchArgument('color', default_value='0 0 1 1')
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.0')
    roll = DeclareLaunchArgument('roll', default_value='0.0')
    pitch = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    enable_collision = DeclareLaunchArgument(
        'enable_collision', default_value='true')

    return LaunchDescription([namespace, world, x, y, z, roll, pitch, yaw, width, length,
                              height, color, enable_collision,
                              OpaqueFunction(function=launch_setup),
                              ])
