# ROS2 Package: core_twist_tools
![Release status](https://img.shields.io/badge/Status-Released-dark_green)
![Documentation status](https://img.shields.io/badge/Documentation-Complete-dark_green)


## Overview
core_twist_tools is a python based packaage that comprises of multiplexer tools, Joystick tools and keyboard teleop tools for controlling robot velocity.
 

## Dependencies
- xterm 
    ```
    sudo apt-get -y install xterm
    ```
- transformations (Python Library) 
    ```
    python3 -m pip install -U transformations
    ```


## Nodes

### **`twist_mux.py`** 
A python node that acts as a multiplexer to handle multiple twist messages.

#### Running the Node 
- Using **ros2 run**
    ```
    ros2 run core_twist_tools twist_mux.py
    ```
- Using the Launch File 
    ```
    ros2 launch core_twist_tools twist_mux.launch.py
    ```
#### Node Parameters
#### Subscribed Topics
- `cmd_vel_key` (geometry_msgs/msg/Twist)
    - Twist topic input from keyboard
- `cmd_vel_joy` (geometry_msgs/msg/Twist)
    - Twist topic input from a joystick 
- `cmd_vel_ctrl` (geometry_msgs/msg/Twist)
    - Twist topic input from a controller
#### Published Topics
- `cmd_vel` (geometry_msgs/msg/Twist) \
    Publishes the selected Twist message

----

### **`twist_teleop_key.py`**  
A python based node that published Twist msgs from keyboard input 


#### Running the Node 
- Using **ros2 run**
    ```
    ros2 run core_twist_tools twist_mux_key.py
    ```
- Using the Launch File 
    ```
    ros2 launch core_twist_tools twist_mux_key.launch.py
    ```


#### Node Parameters
- `max_linear_speed`(double, default: 1.0]) 
    - Maximum linear speed
- `max_angular_speed`(double, default: 2.0) 
    - Maximum angular speed 
- `cmd_rate`(float, default: 10.0) 
    -  Publishing topic rate.
         
#### Subscribed Topics

#### Published Topics
- `cmd_vel` (geometry_msgs/msg/Twist)
    - Twist topic output from the keyboard teleop. 

----

### **`twist_teleop_joy.py`**  
A python based node that published Twist msgs from Joystick input


#### Running the Node 
- Using **ros2 run**
    ```
    ros2 run core_twist_tools twist_mux_joy.py
    ```
- Using the Launch File 
    ```
    ros2 launch core_twist_tools twist_mux_joy.launch.py
    ```


#### Node Parameters
- `max_linear_speed`(double, default: 1.0]) 
    - Maximum linear speed
- `max_angular_speed`(double, default: 2.0) 
    - Maximum angular speed 
- `cmd_rate`(float, default: 10.0) 
    -  Publishing topic rate.
- `pause_button`(int, default: 4) 
    -  Button assignment for pause action
- `cont_cmd_axes`(int, default: 2) 
    -  Button assignment for continous twist message
         
#### Subscribed Topics
- `joy` (geometry_msgs/msg/Twist)
    - Twist topic output from the keyboard teleop. 

#### Published Topics
- `cmd_vel` (geometry_msgs/msg/Twist)
    - Twist topic output from the keyboard teleop. 

----

### **`twist_integrator.py`**  
A python based node that publishes pose messages by integrating twist messages


#### Running the Node 
- Using **ros2 run**
    ```
    ros2 run core_twist_tools twist_integrator.py
    ```
#### Node Parameters
         
#### Subscribed Topics
- `pose` (geometry_msgs/msg/PoseStamped)
    - Pose topic input from the published pose topic. 
- `cmd_vel` (geometry_msgs/msg/Twist)
    - Twist topic input from the controller or teleop model. 

#### Published Topics
- `pose` (geometry_msgs/msg/PoseStamped)
    - Pose topic output after integrating. 


## Launch Files

### `twist_mux.launch.py`
Launches the `twist_mux.py` node. 

#### Arguments
- `namespace` (default: '') 
    - Namespace for the node
- `cmd_vel` (default: 'cmd_vel') 
    - Topic name for the `twist_mux.py` node to publish
- `config` (default: mux.yaml) 
    - Config file enlisting the parameters and rate

#### Configuration files

- `pose_mux.yaml`
    - Defines the input topic multiplexer and lock mechanisms for controlling an entity. 
- `position_teleop_key.yaml`
    - sets default parameter value for the node `position_teleop_key.py`.

--- 

### `twist_teleop_key.launch.py`
Launches the `twist_teleop_key.py` node. 

#### Arguments
- `namespace` (default: '') 
    - Namespace for the node
- `cmd_vel` (default: 'cmd_vel') 
    - Topic name for the `twist_teleop_key.py` node to publish
- `config` (default: teleop_key.yaml) 
    - Config file enlisting the parameters and rate

#### Configuration files
- `teleop_key.yaml`
    - Defines the parameters for the node `twist_teleop_key.py`

--- 

### `twist_teleop_joy.launch.py`
Launches the `twist_teleop_joy.py` node. 

#### Arguments
- `namespace` (default: '') 
    - Namespace for the node
- `cmd_vel` (default: 'cmd_vel') 
    - Topic name for the `twist_teleop_joy.py` node to publish
- `config` (default: teleop_joy.yaml) 
    - Config file enlisting the parameters and rate

#### Configuration files
- `teleop_joy.yaml`
    - Defines the parameters for the node `twist_teleop_joy.py`


