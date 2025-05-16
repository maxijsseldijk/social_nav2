# Social NAV2
![Release status](https://img.shields.io/badge/Status-In_progress-yellow)
![Documentation status](https://img.shields.io/badge/Documentation-In_progress-yellow)

This repository provides a framework for simulating and learning socially-aware navigation behaviors for robots using ROS 2, Gazebo, reinforcement learning (OpenAI Gym), and imitation learning (Imitation library). The framework includes simulated "human-like" agents that react to the robot based on the Social Force Model.

The navigation stack is built on Nav2 and supports two main approaches for generating socially-aware behavior:
- **Direct velocity command learning:** The RL agent learns to output motor velocity commands, as is common in the literature.
- **Local pose/plan learning:** The RL agent learns to output a local waypoint or plan near the robot. This is implemented for multiple low-level controllers, including DWB, the Social Force Window controller ([add citation]), and a custom RlPurePursuitController. For more information on the RlPurePursuitController, refer to the paper [add reference].

The framework supports pretraining neural networks using imitation learning. A forked version of the Imitation library is used to address maintenance issues. Currently, only the DAgger algorithm is implemented, but other imitation algorithms can be easily added.

For reinforcement learning, the SAC algorithm is currently supported due to its sample efficiency. Support for additional algorithms is planned.

All simulation parameters are centralized in `core_cleaning_robot/config/main_params.yaml`. This file allows you to configure everything from reward functions to Nav2 parameters. Please refer to the instructions in that file for details.

## Folder Structure

- `core_cleaning_robot`: Main simulation setup and entry point (`demo_cleaning_robot.launch.py`).
- `core_custom_messages`: Custom messages for efficient RL data handling.
- `core_dynamical_agent`: Tools for creating dynamic agents, including a cylindrical agent using the SMAC2D planner and the social costmap plugin to mimic human behavior.
- `core_gazebo_world`: World files and SLAM maps.
- `core_nav2_navigation`: Local RL planner and waypoint following plugin.
- `core_reinforcement_learning`: RL algorithms and environment utilities.
- `core_ros_gz_service_bridge`: Custom Gazebo service bridge for teleporting entities.
- `core_twist_tools`: Twist mux functionality for the robot.
- `rl_pure_pursuit_controller`: RL-based pure pursuit controller for Nav2.


## Installation with Docker
The easiest way to set up the environment is using Docker. This method installs all required packages and loads the environment correctly every time.
1. Make sure docker is installed with the nvidia container toolkit for GPU acceleration. To do this follow the steps mentioned in
   [Nvidia Container Toolkit](   https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local
   )

2. Clone this repository:
   ```
   git clone https://gitlab.tue.nl/omurarslan/student_projects/msc_max_van_ijsseldijk.git
   ```
3. Ensure VSCode is installed with the Remote Development extension.
4. Open the cloned repo in VSCode and run the container (this may take a few minutes as it installs ROS, NAV2, etc.).
5. After installation, run the demo with:
   ```
   ros2 launch core_cleaning_robot demo_cleaning_robot.launch.py
   ```

## Manual Installation
If you prefer not to use Docker, you can install manually:

1. Install ROS2 Humble
2. Install additional required packages:
   ```
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-cyclonedds
   ```
3. Add the following to your `.bashrc` to use CycloneDDS:
   ```
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
4. Clone the following repositories into the same directory:
   - [nav2_social_costmap](https://github.com/robotics-upo/nav2_social_costmap_plugin) (humble branch)
   - [People_msgs](https://github.com/wg-perception/people/tree/ros2) (ros2 branch)
   - [imitation](https://github.com/maxijsseldijk/imitation.git)
   - [lightsfm](https://github.com/robotics-upo/lightsfm.git)
   - [social_force_window](https://github.com/maxijsseldijk/social_force_window_planner.git)


5. Install Python dependencies:
   ```
   pip3 install numpy==1.22.4
   rosdep install --from-path src -y
   ```

## Current Issues and TODO
- Implement agent velocity estimation instead of directly using odometry.
- Improve transform reliability between agent and cleaning robot.

