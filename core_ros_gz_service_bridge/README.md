# Service Bridge communication between ROS2 and IGN-Gazebo

This package provides a network bridge for services between ROS2 and Gazebo Transport.

The following services are present: 


|   Description                          | Service type                        | Parameter types                  | Important parameter |
|----------------------------------------|:-----------------------------------:|:--------------------------------:|---------------------|
| Move and existing entity to a pose     | ros_gz_interfaces/srv/SetEntityPose | ros_gz_interfaces/Entity         | id# |
| Spawn a sdf model at a given pose      | ros_gz_interfaces/srv/SpawnEntity   | ros_gz_interfaces/EntityFactory  | name |
| Delete a model by name                 | ros_gz_interfaces/srv/DeleteEntity  | ros_gz_interfaces/EntityFactory  | name |

Service definitions and their corresponding message definitions are elaborated here: [git-ros_gz_interfaces](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_interfaces) 


Run `ros2 run core_ros_gz_service_bridge service_bridge -h` for instructions.


## Example: Service bridge

It's possible to make ROS2 service requests into ign-Gazebo. Let's try unpausing the simulation.

On terminal A, start the service bridge:

`ros2 run core_ros_gz_service_bridge service_bridge /world/<world_name>/set_pose@core_ros_gz_service_interfaces/srv/SetEntityPose`

On terminal B, start Gazebo with an entity:

`ign gazebo shapes.sdf`

On terminal C, make a ROS2 request to teleport the box to another pose:
```
ros2 service call /world/<world_name>/set_pose core_ros_gz_interfaces/srv/SetEntityPose 
"{entity:
  id: 0
  name: 'box'
  type: 0
pose:
  position:
    x: 3.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0}"
```

To run the bridge node with the above configuration:
```bash
ros2 run core_ros_gz_service_bridge service_bridge 
```

## In-pipeline
Removal of redundant files, code optimization

