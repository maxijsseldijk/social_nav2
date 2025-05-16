from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro
import os
import tempfile


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)

    world_sdf_xacro = os.path.join(
        get_package_share_directory('core_gazebo_world'),
        'worlds', 'empty', 'world.sdf.xacro'
    )

    world_sdf = xacro.process_file(
        world_sdf_xacro,
        mappings={
            'name': world
        }
    )

    world_description = world_sdf.toprettyxml(indent='  ')

    # Save world_description to a temporary file
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp_sdf_file:
        temp_sdf_file.write(world_description)
        world_sdf_filename = temp_sdf_file.name

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f' -r -s  {world_sdf_filename}'}.items(),
    )

    return [gz_sim_launch]


def generate_launch_description():

    namespace = DeclareLaunchArgument(
        'namespace', default_value='empty', description='Simulation namespace')
    world = DeclareLaunchArgument(
        'world', default_value='empty', description='Gazebo world name')

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    return LaunchDescription([
        namespace,
        world,
        OpaqueFunction(function=launch_setup),
        ros_gz_bridge_node,
    ])
