from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

import os

def launch_setup(context):


    world_name = LaunchConfiguration('world_name').perform(context)
    
    entity_name = LaunchConfiguration('entity_name').perform(context)
    sdf_filename = LaunchConfiguration('sdf_filename').perform(context)

    entity_pose = LaunchConfiguration('entity_pose').perform(context)


    spawn_entity_srv_node = LaunchDescription([
            Node(
                package='core_ros_gz_service_bridge',
                executable='service_bridge',
                name='set_model_pose',
                namespace=entity_name, 
                arguments=[f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity']
                ),
            ])
    
    service_name = "/world/" + world_name + "/create "

    service_param = "\"{entity_factory: {name: " + entity_name + ", sdf_filename: " + sdf_filename + ", " + entity_pose + "}}\""

    spawn_entity_srv_node.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name="ros2"),
                " service call ",
                service_name,
                "ros_gz_interfaces/srv/SpawnEntity ",
                service_param,
                ]],
            shell=True,
        )
    )
    
    return [
        spawn_entity_srv_node,
        ]

def generate_launch_description():
    
    default_model_file = os.path.join(get_package_share_directory('core_ros_gz_service_bridge'),
        'models', 'box.sdf')

    world_name = DeclareLaunchArgument('world_name', default_value='empty', 
                                       description='Name of the world launched in gazebo')
    
    entity_name = DeclareLaunchArgument('entity_name', default_value='box', 
                                        description='Name of the entity')
    

    sdf_filename = DeclareLaunchArgument('sdf_filename', default_value = default_model_file, 
                                        description='Path to SDF file of the model')
    #The new position of the entity
    entity_pose = DeclareLaunchArgument('entity_pose', default_value='pose: {position: {x: 0.0, y: 0.0, z: 0.0}}')



    return LaunchDescription([
        world_name,
        entity_name,
        sdf_filename,
        entity_pose,
        OpaqueFunction(function=launch_setup)
        ])


