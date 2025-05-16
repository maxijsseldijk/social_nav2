# Rl Pure Pursuit Controlller
![Release status](https://img.shields.io/badge/Status-In_process-yellow)
![Documentation status](https://img.shields.io/badge/Documentation-In_process-yellow)

Repository for the RL pure pursuit controller. 

This package creates a NAV2 implementation of the RL pure pursuit controller by subscribing to the rl plan that is published from the `train_rl` node. The controller will follow this rl plan until an update is published on the `rl_local_plan` topic.

## Configuration
A simple Pure Pursuit base is used without additional features.
| Parameter | Description | 
|-----|----|
| `desired_linear_vel` | The desired maximum linear velocity to use. | 
| `lookahead_dist` | The lookahead distance to use to find the lookahead point |  
| `transform_tolerance` | The TF transform tolerance | 
| `max_angular_vel` | The maximum angular velocity |



## Topic Subscribers 

| Topic  | Type | Description | 
|-----|----|----|
| `rl_local_plan`  | `nav2_msg/msg/Plan` | The current RL plan, containing the lookahead pose to send to the controller. | 

## Topic Publishers

| Topic  | Type | Description | 
|-----|----|----|
| `lookahead_pose`  | `core_custom_messages/msg/PathWithLength` | The current (RL) lookahead pose which also contains information on the total path length to the goal location. | 
| `lookahead_pose_rviz`  | `geometry_msgs/PointStamped`  | The lookahead pose published as PointStamped for visualization purposes.
