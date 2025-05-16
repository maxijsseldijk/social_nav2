#!/usr/bin/env python3
import time
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING
import os
import gymnasium as gym
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped, Point32
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from scipy.spatial import ConvexHull
from std_msgs.msg import Bool as BoolMsg, Int32
from nav2_simple_commander.costmap_2d import PyCostmap2D


from core_reinforcement_learning.footprint_collision_checker import FootprintCollisionChecker
from core_reinforcement_learning.publish_agents_velocity import (
    OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_LOC
)
from core_reinforcement_learning.rl_node_manager import RlSubscriptionManager
from core_reinforcement_learning.utils import (get_transform_angle,
                                               in_convex_hull,
                                               get_safe_corridor_vertices,
                                               move_point_to_convex_hull)
from core_reinforcement_learning.world_interact_node import PauseSimulation, ResetSimulation

if TYPE_CHECKING:
    from train_rl import RLsimulation


class BaseClassEnv(gym.Env, ABC):
    """
    BaseClassEnv is an base class for creating reinforcement learning environments.

    Abstract Methods (must be overridden):
        _reward_function(action: np.ndarray, info: str) -> float: Calculates the reward based
        on the action taken by the agent.
        step(action: np.ndarray) -> list[np.ndarray, np.ndarray, np.ndarray]: Executes a single
        step in the environment.
    """

    def __init__(self, node: 'RLsimulation', **kwargs):
        """
        Initialize the BaseClassEnv.

        Args:
        ----
            node (RLsimulation): The ROS node for the environment.
            **kwargs: Additional keyword arguments.

        """
        self.rl_io_manager = RlSubscriptionManager(node)
        self.rng: np.random.Generator = kwargs.get('rng', 0)
        self.seed = kwargs.get('seed', 0)
        self.node = node
        self.train = self.node.get_parameter('train').value

        self.time_step_length = self.node.get_parameter(
            'time_step_length').value
        self.rl_action_output = self.node.get_parameter(
            'rl_action_output').value
        self.grid_size = self.node.get_parameter('grid_size').value

        # Task related parameters
        self.TaskGenerator = self.node.get_parameters_by_prefix(
            'TaskGenerator')
        self.task_number = self.TaskGenerator['init_task_number'].value
        self.task_list = self.TaskGenerator['task_list'].value
        self.number_of_tasks = len(self.task_list)

        # Reward related parameters
        self.max_trial_timesteps = self.node.get_parameter(
            'max_trial_timesteps').value
        self.reward_functions = self.node.get_parameter(
            'reward_functions').value
        self.use_future_collision_reward = 'future_collision' in self.reward_functions

        # Imitation learning related parameters
        self.use_expert_demonstrations = self.node.get_parameter(
            'use_expert_demonstrations').value
        self.expert_class = self.node.get_parameter(
            'imitation_learning.expert_class').value
        # Evaluation parameters
        self.reward_averaging = self.node.get_parameter(
            'reward_averaging').value
        self.reward_samples_per_timestep = self.node.get_parameter(
            'reward_samples_per_timestep').value

        if self.reward_averaging:
            self.pause_time = round(self.time_step_length /
                                    self.reward_samples_per_timestep, 2)
        else:
            self.pause_time = self.time_step_length

        if self.pause_time < 0.1:
            self.node.get_logger().error(
                f"pause_time ({self.pause_time}) is less than 0.1, shutting down. \
                  Please make sure the timesteps are longer to prevent timing issues.")
            self.node.shutdown()

        # Robot related parameters
        self.agent_names = self.node.get_parameter(
            'agent_names').get_parameter_value().string_array_value
        self.robot_names = self.node.get_parameter(
            'robot_names').get_parameter_value().string_array_value

        self.ns_robot = os.path.join(
            self.node.get_namespace(), self.robot_names[0])
        self.footprint_collision_checker = FootprintCollisionChecker()
        self.collision_footprint_factor = self.node.get_parameter(
            'collision_footprint_factor').value

        # If the robot is square assume the radius is half the width.
        if self.node.get_parameter('width') is not None:
            self.robot_width = self.node.get_parameter('width').value/2
            self.robot_length = self.node.get_parameter('length').value/2
            self.robot_radius_sqr = self.robot_width**2 + self.robot_length**2
        elif self.node.get_parameter('radius') is not None:
            self.robot_radius_sqr = self.node.get_parameter('radius').value**2
        else:
            self.node.get_logger().error(
                "No radius or width and length specified for the robot shutting down")
            self.node.shutdown()

        # Initialize the world interaction nodes
        self.world_param = self.node.get_parameter('world').value
        self.pause_node = PauseSimulation(
            node=self.node, world=self.world_param)
        self.pause_node.change_pause_simulation(pause=False)
        self.reset_gazebo = ResetSimulation(node=self.node, robot_names=self.robot_names,
                                            agent_names=self.agent_names,
                                            TaskGenerator=self.TaskGenerator,
                                            world=self.world_param, rng=self.rng)
        # Initialize the publishers
        # TODO needs if statements for when these publishers should exist
        self.plan_publisher = self.node.create_publisher(
            Path, f'{self.ns_robot}/rl_local_plan', 1)
        self.robot_action_publisher = self.node.create_publisher(
            Twist, f'{self.ns_robot}/cmd_vel_rl', 1)

        if self.use_expert_demonstrations:
            self.check_if_imitation_running = self.node.create_publisher(
                BoolMsg, '/imitation_learning_running', 1)
        else:
            self.check_if_imitation_running = None
        self.pause_motion_publisher = self.node.create_publisher(
            BoolMsg, '/pause_motion', 1)
        self.pause_agents_publisher = self.node.create_publisher(
            BoolMsg, '/pause_agents', 1)

        # As the task may be published before the waypoint following node is started,
        # the QoS profile is set to transient local.
        qos_profile_task = QoSProfile(depth=1)
        qos_profile_task.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.task_number_publisher = self.node.create_publisher(
            Int32, '/task_number', qos_profile_task)

        self.requirements = np.empty((0,))

        # Imitation learning requirements
        if self.use_expert_demonstrations:
            self.imitation_learning_is_running = True
            if self.expert_class == 'Nav2Policy':
                if self.rl_action_output == 'plan':
                    self.__add_requirements(['last_plan', 'last_robot_odom'])
                elif self.rl_action_output == 'diff_drive':
                    self.__add_requirements(['last_nav2_input'])

            elif self.expert_class == 'SocialForcePolicy':
                if self.rl_action_output == 'plan':
                    self.__add_requirements(
                        ['last_sfm_control_point', 'last_robot_odom'])
                elif self.rl_action_output == 'diff_drive':
                    self.__add_requirements(['last_nav2_input'])
        else:
            self.imitation_learning_is_running = False

        # Output requirements
        if self.rl_action_output == 'plan':
            self.__add_requirements(['last_plan', 'last_robot_odom'])

        # Reward requirements
        for reward_functionality in self.reward_functions:
            if reward_functionality == 'future_collision':
                self.__add_requirements(
                    ['last_plan', 'last_footprint', 'last_costmap', 'last_robot_odom'])
            elif reward_functionality == 'collision':
                self.__add_requirements(['last_costmap', 'last_robot_odom'])
            elif reward_functionality == "goal_reached":
                self.__add_requirements(['last_plan'])
            elif reward_functionality == 'agent_velocity_disturbance':
                self.__add_requirements(['last_agents_global_frame'])
            elif reward_functionality == 'path_traversal':
                self.__add_requirements(['last_plan'])
            elif reward_functionality == 'action':
                self.__add_requirements(['last_action'])
            elif reward_functionality == 'social_force_sfm':
                self.__add_requirements(
                    ['last_robot_odom', 'last_agents_global_frame', ])

        self.__check_requirements(self.rl_io_manager.all_nodes_callback_names)

        self.trial_time_step = 1

        # Gym environment parameters
        self.action_bounds = self.node.get_parameter('action_bounds').value
        self.action_space = None
        self.observation_space = None

    def __add_requirements(self, requirements: np.ndarray):
        """
        Add the requirements to the list of requirements.

        Args:
        ----
            requirements (np.ndarray): The requirements to be added.

        """
        new_requirements = np.array([req for req in requirements if not any(
            np.array_equal(req, existing_req) for existing_req in self.requirements)])
        self.requirements = np.concatenate(
            (self.requirements, new_requirements))

    def __check_requirements(self, fullfilled_requirements: np.ndarray) -> bool:
        """
        Check if the requirements for running the RL environment are met.

        Args:
        ----
            fullfilled_requirements (np.ndarray): The list of fullfilled requirements.

        Returns
        -------
            bool: True if the requirements are met, False otherwise.

        """
        requirements_met = all(any(np.array_equal(
            req, full_req) for full_req in fullfilled_requirements) for req in self.requirements)

        if not requirements_met:
            unmet_requirements = self.requirements[~np.isin(
                self.requirements, fullfilled_requirements)]
            self.node.get_logger().error(
                f"Requirements not met: {unmet_requirements} Shutting down")
            self.node.shutdown()
        else:
            self.node.get_logger().error("All requirements met")
        return requirements_met

    def set_action_and_observation_space(self):
        """
        Set the action and observation space for the environment.

        As the nodes responsible for the action and observation space are not yet started,
        the action and observation space is set to None during initialization.
        This function is called after the requirements are met.

        """
        state_dict = self.get_state_name_size_dict()
        self.node.get_logger().info(f"State dict: {state_dict}")
        self.action_space = gym.spaces.Box(low=self.action_bounds[0], high=self.action_bounds[1],
                                           shape=(self.get_action_size(),), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.get_state_size(),), dtype=np.float32)

    def create_utility_functions_from_requirements(self):
        """Create utility related functionality based on the requirements that are present."""
        pass

    def set_imitation_learning_running(self, running: bool):
        """
        Set the imitation learning running parameter to the specified value.

        This is used in the rl_local_planner to prevent the reinforcement learning output
        to be used when imitation learning is running.

        Args:
        ----
            running (bool): Whether the imitation learning is learning.

        """
        self.check_if_imitation_running.publish(BoolMsg(data=running))
        self.imitation_learning_is_running = running

    def get_observation_space(self) -> gym.spaces.Box:
        """
        Get the observation space of the environment.

        Returns
        -------
            gym.spaces.Box: The observation space of the environment.

        """
        return self.observation_space

    def get_action_space(self) -> gym.spaces.Box:
        """
        Get the action space of the environment.

        Returns
        -------
            gym.spaces.Box: The action space of the environment.

        """
        return self.action_space

    def get_state_size(self) -> int:
        """
        Get the size of the state.

        Returns
        -------
            int: The size of the state.

        """
        return self.rl_io_manager.get_state_size()

    def get_state_name_size_dict(self) -> dict:
        """
        Get the dictionary of the state names and their sizes.

        Returns
        -------
            dict: The dictionary of the state names and their sizes.

        """
        return self.rl_io_manager.get_state_name_size_dict()

    @abstractmethod
    def get_action_size(self) -> int:
        """
        Get the size of the action.

        Returns
        -------
            int: The size of the action.

        """
        raise NotImplementedError(
            "Action size function not implemented for this Class")

    def close(self):
        """Close the environment."""
        self.pause_node.change_pause_simulation(pause=False)
        self.node.destroy_node()
        self.node.shutdown()

    def __check_collision_to_goal_pose(self, initial_x_position: float,
                                       initial_y_position: float,
                                       goal_x_position: float,
                                       goal_y_position: float,
                                       footprint: PolygonStamped,
                                       num_samples: int = 5) -> bool:
        """
        Check if the requested goal pose results in a collision with an obstacle.

        Args:
        ----
            initial_x_position (float): The x-coordinate of the initial pose.
            initial_y_position (float): The y-coordinate of the initial pose.
            goal_x_position (float): The x-coordinate of the goal pose.
            goal_y_position (float): The y-coordinate of the goal pose.
            num_samples (int): The number of samples to check between the initial and goal pose.

        Returns
        -------
            bool: True if there is a collision, False otherwise.

        """
        sub_positions = np.linspace([initial_x_position, initial_y_position], [
                                    goal_x_position, goal_y_position], num=num_samples)
        for sub_position in sub_positions:
            collision_at_pose = self.__check_collision_at_pose(
                sub_position[0], sub_position[1], footprint)
            if collision_at_pose:
                return True
        return False

    def __check_collision_at_pose(self, x_position: float, y_position: float,
                                  footprint: PolygonStamped) -> bool:
        """
        Check if there is a collision at the given (future) pose.

        Args:
        ----
            x_position (float): The x-coordinate of the pose.
            y_position (float): The y-coordinate of the pose.
            footprint (PolygonStamped): The footprint with respect to the robot frame.

        Returns
        -------
            bool: True if there is a collision, False otherwise.

        """
        footprint_cost = self.footprint_collision_checker.footprintCostAtPose(
            x_position, y_position, footprint.polygon)
        if footprint_cost >= 100.0:
            return True
        else:
            return False

    def _check_future_collision_(self, initial_x_position: float, initial_y_position: float,
                                 goal_x_position: float, goal_y_position: float,
                                 footprint: PolygonStamped, num_samples: int = 5) -> bool:
        """
        Check if there is a potential future collision by moving a footprint between two points.

        Args:
        ----
            initial_x_position (float): The x-coordinate of the initial pose.
            initial_y_position (float): The y-coordinate of the initial pose.
            goal_x_position (float): The x-coordinate of the goal pose.
            goal_y_position (float): The y-coordinate of the goal pose.
            footprint (PolygonStamped): The footprint with respect to the robot frame.
            num_samples (int): The number of samples to interpolate between the two poses.

        Returns
        -------
            bool: True if there is a collision, False otherwise.

        """
        if self.__check_collision_to_goal_pose(initial_x_position, initial_y_position,
                                               goal_x_position, goal_y_position,
                                               footprint, num_samples):
            return True
        else:
            return False

    def _check_if_at_goal(self) -> bool:
        """
        Check if the robot is at the goal as defined by the goal_reached.goal_threshold parameter.

        Returns
        -------
            bool: Returns true if the goal location.


        """
        goal_params = self.node.get_parameters_by_prefix('goal_reached')
        goal_threshold = goal_params['goal_threshold'].value
        if self.rl_io_manager.last_plan_length < goal_threshold:
            return True
        else:
            return False

    @abstractmethod
    def _reward_function(self, action: np.ndarray, info: dict) -> float:
        """Calculate the reward based on the action taken by the agent."""
        raise NotImplementedError(
            "Reward function not implemented for this Class")

    def _get_action_reward(self, action: np.ndarray) -> float:
        """
        Calculate the reward based on the action taken by the agent.

        Args:
        ----
            action (np.ndarray): The action taken by the agent.

        Returns
        -------
            float: The reward for the action taken.

        """
        return NotImplementedError("Action reward function not implemented")

    def _get_path_traversal_reward(self) -> float:
        """
        Get the path traversal reward based on the distance the robot has moved towards the goal.

        The reward is calculated using a tanh with the center at path_traversal_for_reward.
        The min and max values are given by -max_reward and max_reward respectively by scaling.

        Returns
        -------
            float: The calculated reward between [-max_reward, max_reward].

        """
        path_traversal_reward_param = self.node.get_parameters_by_prefix(
            'path_traversal')
        max_reward = path_traversal_reward_param['max_reward'].value
        path_traversal_for_reward = path_traversal_reward_param[
            'path_traversal_for_positive_reward'].value
        if self.last_distance_to_goal is None:
            self.last_distance_to_goal = self.rl_io_manager.last_plan_length
            return 0.0
        else:
            current_path_distance = self.rl_io_manager.last_plan_length

        if (self.last_distance_to_goal == current_path_distance and
                self.last_distance_reward is not None):
            # No update on topic
            return self.last_distance_reward

        traversed_path = self.last_distance_to_goal - current_path_distance
        self.last_distance_to_goal = current_path_distance
        normalized_path = (
            traversed_path - path_traversal_for_reward) / path_traversal_for_reward
        reward = np.tanh(normalized_path).tolist() * max_reward
        self.last_distance_reward = reward
        return reward

    def _get_at_goal_reward(self) -> float:
        """
        Get the reward for reaching the goal defined by the goal_reached.reward parameter.

        Returns
        -------
            float: The reward for reaching the goal.

        """
        goal_params = self.node.get_parameters_by_prefix('goal_reached')
        reward = goal_params['reward'].value
        return reward

    def _get_future_collision_reward(self) -> float:
        """
        Get the reward for a future collision based on the future_collision.reward parameter.

        Returns
        -------
            float: The reward for the future collision.

        """
        future_collision_reward_param = self.node.get_parameters_by_prefix(
            'future_collision')
        reward = future_collision_reward_param['reward'].value
        return reward

    def _get_collision_reward(self) -> float:
        """
        Get the reward for a collision based on the collision.reward parameter.

        Returns
        -------
            float: The reward for the collision.

        """
        collision_reward_param = self.node.get_parameters_by_prefix(
            'collision')
        reward = collision_reward_param['reward'].value
        return reward

    def _get_agent_velocity_reward(self, in_interaction_range: bool = True) -> float:
        """
        Get the reward for slowing down agents in the environment.

        This reward returns a penalty for reducing the velocity of agents that are near the robot.
        If there is an agent in the interaction_range this function looks at its current velocity
        and a predefined agents preference velocity defined in agent_velocity_disturbance.v_pref.
        Currently only agents that are in the interaction range will contribute to the reward
        function.

        Args:
        ----
            in_interaction_range (bool): Whether the agent is in the interaction range.

        Returns
        -------
            float: The reward for the agent velocity.

        """
        agent_velocity_reward_param = self.node.get_parameters_by_prefix(
            'agent_velocity_disturbance')
        max_reward = agent_velocity_reward_param['max_reward'].value
        min_reward = agent_velocity_reward_param['min_reward'].value
        v_pref = agent_velocity_reward_param['v_pref'].value

        if in_interaction_range:
            v_pref_sqr = v_pref ** 2
            velocity_array = []
            # If an agent is outside the interaction range it will have a contant values as given
            # by the global variables in publish_agents_velocity.py
            agents_in_range = [agent == np.array([OUTSIDE_RANGE_LOC, OUTSIDE_RANGE_LOC,
                                                  OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_VELOCITY]
                                                 ) for agent in self.rl_io_manager.last_agents]
            self.node.get_logger().error(
                f'Agents {self.rl_io_manager.last_agents} in range {agents_in_range} ')
            # The agents are orderned in the same way for the global frame so simple mask suffices.
            for agent in self.rl_io_manager.last_agents_global_frame:
                squared_velocity = (agent[2] ** 2 + agent[3] ** 2)
                velocity_array.append(squared_velocity)

            squared_velocity = min(velocity_array)
            if v_pref_sqr >= squared_velocity:
                squared_difference = abs(squared_velocity - v_pref_sqr)
                return (1 - squared_difference / v_pref_sqr) * \
                    (max_reward - min_reward) + min_reward

            elif v_pref_sqr < squared_velocity:
                return max_reward
        else:
            return max_reward

    def _get_proxemics_reward(self, in_interaction_range: bool = True) -> float:
        """
        Reward based on the robot being a certain distance from the agents.

        Similar to Learning Local Planners for Human-Aware Navigation
        there is a negative reward if the robot is closer than 0.85m from the agent.

        Args:
        ----
            in_interaction_range (bool): Whether the agent is in the interaction range.

        Returns
        -------
            float: The reward for the proxemics.

        """
        if not np.any(np.array([in_interaction_range])):
            return 0.0

        proxemics_reward_param = self.node.get_parameters_by_prefix(
            'proxemics')
        max_reward = proxemics_reward_param['max_reward'].value
        min_reward = proxemics_reward_param['min_reward'].value
        distance_threshold = proxemics_reward_param['distance_threshold'].value
        closest_distance = 100.0
        for agent in self.rl_io_manager.last_agents_global_frame:
            distance = np.min(np.linalg.norm(
                agent[0:2] - self.rl_io_manager.last_robot_odom[0][0:2]))
            if distance < closest_distance:
                closest_distance = distance

        if closest_distance < distance_threshold:
            return min_reward

        else:
            return max_reward

    def _get_social_force_sfm_impl_reward(self, in_interaction_range: bool = True
                                          ) -> list[list, list, float]:
        """
        Calculate the social force cost based on Social Force Model.

        Args:
        ----
            in_interaction_range (bool): Whether the agent is in the interaction range.

        Returns
        -------
            list[list, list, float]: The social force cost.

        """
        if not np.any(np.array([in_interaction_range])):
            return [[0.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]], 0.0

        social_force_reward_param = self.node.get_parameters_by_prefix(
            'social_force_sfm')
        max_reward = social_force_reward_param['max_reward'].value
        min_reward = social_force_reward_param['min_reward'].value
        n = social_force_reward_param['n'].value
        A = social_force_reward_param['A'].value
        gamma_ = social_force_reward_param['gamma_'].value
        n_prime = social_force_reward_param['n_prime'].value
        lambda_ = social_force_reward_param['lambda_'].value
        epsilon = social_force_reward_param['epsilon'].value
        min_social_force = social_force_reward_param['min_social_force'].value

        total_social_force = 1.0
        robot = self.rl_io_manager.last_robot_odom[0]
        force_deceleration_list = []
        force_evasion_list = []
        for agent in self.rl_io_manager.last_agents_global_frame:
            force_decel, force_evasion, sum_force = self._calculate_social_force_sfm_impl(
                n, A, gamma_, n_prime, lambda_, epsilon, agent, robot)
            social_force_clip = np.clip(
                (min_social_force - sum_force) / min_social_force, 0, 1)
            if social_force_clip < total_social_force:
                total_social_force = social_force_clip

            force_deceleration_list.append(force_decel)
            force_evasion_list.append(force_evasion)

        # Squeeze the output to be between min and max reward

        return (force_deceleration_list, force_evasion_list,
                total_social_force * (max_reward - min_reward) + min_reward)

    def _calculate_social_force_sfm_impl(self, n, A, gamma_, n_prime, lambda_,
                                         epsilon, agent, robot) -> list[list, list, float]:
        """
        Calculate the social force based on the Social Force Model.

        Args:
        ----
            n (float): The n parameter of the Social Force Model.
            A (float): The A parameter of the Social Force Model.
            gamma_ (float): The gamma parameter of the Social Force Model.
            n_prime (float): The n_prime parameter of the Social Force Model.
            lambda_ (float): The lambda parameter of the Social Force Model.
            epsilon (float): The epsilon parameter of the Social Force Model.
            agent (np.ndarray): The agent's state.
            robot (np.ndarray): The robot's state.

        Returns
        -------
            list[list, list, float]: The social force.

        """
        agent_global_velocity_vector = agent[2:4]
        robot_global_velocity_vector = robot[3:5]

        vel_diff = robot_global_velocity_vector - agent_global_velocity_vector

        agent_location_vector = agent[0:2]
        robot_location_vector = robot[0:2]
        location_vector = agent_location_vector - robot_location_vector

        distance_robot_agent = np.linalg.norm(location_vector)
        # Prevent division by zero
        if distance_robot_agent < 0.001:
            distance_robot_agent = 0.001
        diff_direction = location_vector / distance_robot_agent
        # self.node.get_logger().error(f"Diff direction: {diff_direction}")

        interaction_vector = lambda_ * vel_diff + diff_direction
        interaction_length = np.linalg.norm(interaction_vector)
        if interaction_length < 0.001:
            interaction_length = 0.001
        interaction_direction = interaction_vector / interaction_length
        # self.node.get_logger().error(f"Interaction direction: {interaction_direction}")
        normal_direction = np.array(
            [-interaction_direction[1], interaction_direction[0]])

        B = gamma_ * interaction_length

        theta = get_transform_angle(
            interaction_direction, diff_direction) + epsilon * B

        force_deceleration_amount = - \
            np.exp(-distance_robot_agent / B - (n_prime * B * theta)**2)
        force_evasion_amount = - np.sign(theta) * np.exp(-distance_robot_agent /
                                                         B - (n * B * theta)**2)
        force_decel = force_deceleration_amount * interaction_direction
        force_evasion = force_evasion_amount * normal_direction
        sum_force = - A * np.linalg.norm(force_decel + force_evasion)

        return force_decel.tolist(), force_evasion.tolist(), sum_force

    @abstractmethod
    def _is_done(self, *args, **kwargs) -> list[bool, bool, str]:
        """
        Abstract method to check if the current episode is done.

        Subclasses should implement this method with their specific logic.

        Args:
        ----
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns
        -------
            list[bool, bool, str]: The done status.

        """
        raise NotImplementedError(
            "is_done function not implemented for this Class")

    # Action and publish functions
    def _publish_diff_drive_action(self, action: np.ndarray):
        """
        Publish the 2D diff drive action to the robot topic for visualization.

        The function can be overwritten in the subclass to implement specific
        functionality.

        Args:
        ----
            action (np.ndarray): The action to be taken by the agent.

        """
        if np.issubdtype(action.dtype, np.integer):
            raise NotImplementedError(
                "Only continuous actions are currently supported for 2D diff drive")

        twist = Twist()
        twist.linear.x = action[0].item()
        twist.angular.z = action[1].item()
        self.robot_action_publisher.publish(twist)

    # Gym environment functions
    @abstractmethod
    def step(self, action: np.ndarray) -> list[np.ndarray, np.ndarray, np.ndarray]:
        """
        Abstract method to execute a single step in the environment.

        Args:
        ----
            action (np.ndarray): The action to be taken by the agent.

        Returns
        -------
            list[np.ndarray, np.ndarray, np.ndarray]: The next state, reward, and done status.

        """
        raise NotImplementedError(
            "Step function not implemented for this Class")

    @abstractmethod
    def reset(self, seed=None):
        """
        Reset the simulation to the initial state by calling the reset service.

        Args:
        ----
            seed (int, optional): The seed for the random number generator. Defaults to None.

        Returns
        -------
            np.ndarray: The initial state of the environment.
            dict: Additional information.

        """
        raise NotImplementedError(
            "Reset function not implemented for this Class")


class GazeboEnv(BaseClassEnv):
    def __init__(self, node: 'RLsimulation', **kwargs):
        """
        Initialize the Gazebo environment based on node parameters.

        Args:
        ----
            node (RLsimulation): The ROS node for the environment.
            **kwargs: Additional keyword arguments.

        """
        super().__init__(node, **kwargs)
        # Interaction parameters
        self.reset_on_exit_interaction_range = self.node.get_parameter(
            'reset_on_exit_interaction_range').value
        self.interaction_range = self.node.get_parameter(
            'interaction_range').value
        self.critical_interaction_range = self.node.get_parameter(
            'critical_interaction_range').value
        self.timesteps_before_switch_to_nav2 = self.node.get_parameter(
            'timesteps_before_switch_to_nav2').value
        self.timesteps_before_interaction = self.node.get_parameter(
            'timesteps_before_interaction').value
        self.timesteps_before_agents_in_range = self.node.get_parameter(
            'timesteps_before_agents_in_range').value
        self.in_interaction_range = False

        # Safe Corridor Parameters
        self.use_constant_for_distant_measurements = self.node.get_parameter(
            'use_constant_for_distant_measurements').value
        self.max_lidar_distance = self.node.get_parameter(
            'max_lidar_distance').value
        self.force_waypoint_in_corridor = self.node.get_parameter(
            'force_waypoint_in_corridor').value

        # Footprint parameters
        self.footprint_from_robot_frame = None
        self.footprint_scale = self.node.get_parameter('footprint_scale').value

        # Eval parameters
        self.eval_mode = kwargs.get('eval', False)
        self.eval_step = 0
        self.num_trials_scenario = self.node.get_parameter(
            'num_eval_per_scenario').value
        self.sim_time = 0.0

        self.publish_if_in_interaction_range = self.node.create_publisher(
            BoolMsg, '/in_interaction_range', 1)
        if not self.reset_on_exit_interaction_range:
            self.timesteps_before_interaction = 0
            self.timesteps_before_switch_to_nav2 = 0

        # Initialize first values
        self.steps_outside_interaction_range = 0
        self.steps_in_interaction_range = 0
        self.buffer_agents_in_range = 0
        self.last_distance_to_goal = None
        self.last_distance_reward = None
        self.starting_plan_length = None

        # Gym environment parameters
        self.last_nav2_input_zero = False

        # Parameter setting client
        self.controller_cli = self.node.create_client(
            SetParameters, f'{self.ns_robot}/controller_server/set_parameters')
        while not self.controller_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Control parameter service not available, waiting...')
        self.controller_req = SetParameters.Request()

        # Create a publisher for the critical points polygon
        self.safe_corridor_pub = self.node.create_publisher(
            PolygonStamped, f'{self.ns_robot}/safe_corridor', 1)

    # Utility functions

    def set_starting_plan_length(self, starting_plan_length: float):
        """
        Set the starting plan length.

        Args:
        ----
            starting_plan_length (float): The starting plan length.

        """
        self.starting_plan_length = starting_plan_length

    def create_utility_functions_from_requirements(self):
        """Create utility related functionality based on the requirements that are present."""
        if hasattr(self, "last_plan_length"):
            self.set_starting_plan_length(self.rl_io_manager.last_plan_length)

        if self.rl_io_manager.check_if_in_all_nodes_list(['last_footprint',
                                                          'last_costmap',
                                                          'last_robot_odom']):
            self.footprint_collision_checker.setCostmap(
                PyCostmap2D(self.rl_io_manager.last_costmap))
            # For each point in the robot polygon remove the odom x and y
            local_footprint = PolygonStamped()
            for point in self.rl_io_manager.last_footprint.polygon.points:
                last_odom = self.rl_io_manager.last_robot_odom
                point.x = (
                    point.x - last_odom[0][0].item()) * self.footprint_scale
                point.y = (
                    point.y - last_odom[0][1].item()) * self.footprint_scale

                local_footprint.polygon.points.append(point)

            self.footprint_from_robot_frame = local_footprint

    def get_action_size(self) -> int:
        """
        Get the size of the action.

        Returns
        -------
            int: The size of the action.

        """
        if self.rl_action_output == 'plan':
            if self.rl_io_manager.check_if_in_all_nodes_list(['last_plan']):
                action_dim = np.size(self.rl_io_manager.last_plan)
            else:
                self.node.get_logger().error(
                    "Plan output requested but no NAV2 plan received. \
                     Please check the data received from the RL node manager")
                self.node.shutdown()

        elif self.rl_action_output == 'diff_drive':
            action_dim = 2

        else:
            self.node.get_logger().error(
                "No action output specified. Either 'diff_drive' or 'plan' is required")
            self.node.shutdown()

        return action_dim

    def _reward_function(self, action: np.ndarray, info: dict) -> float:
        """
        Calculate the reward based on the action taken and the state of the environment.

        Args:
        ----
            action (np.ndarray): The action taken by the agent.
            info (dict): Information on the environment given as key-value pairs.

        Returns
        -------
            float: The reward for the action taken.

        """
        total_reward = 0.0

        if 'agent_velocity_disturbance' in self.reward_functions:
            agent_velocity_reward = self._get_agent_velocity_reward(
                info['in_interaction_range'])
            total_reward += agent_velocity_reward
            if 'velocity_reward' not in info:
                info['velocity_reward'] = []
            info['velocity_reward'].append(agent_velocity_reward)

        if 'path_traversal' in self.reward_functions:
            goal_distance_reward = self._get_path_traversal_reward()
            total_reward += goal_distance_reward
            if 'goal_distance_reward' not in info:
                info['goal_distance_reward'] = []

            info['goal_distance_reward'].append(goal_distance_reward)

        if 'action' in self.reward_functions:
            action_reward = self._get_action_reward(action)
            total_reward += action_reward
            if 'action_reward' not in info:
                info['action_reward'] = []
            info['action_reward'].append(action_reward)

        if 'proxemics' in self.reward_functions:
            proxemics_reward = self._get_proxemics_reward(
                info['in_interaction_range'])
            total_reward += proxemics_reward
            if 'proxemics_reward' not in info:
                info['proxemics_reward'] = []
            info['proxemics_reward'].append(proxemics_reward)

        if 'social_force_sfm' in self.reward_functions:
            force_decel, force_evasion, sfm_reward = self._get_social_force_sfm_impl_reward(
                info['in_interaction_range'])
            # self.node.get_logger().error(f"Social force reward: {sfm_reward}")
            total_reward += sfm_reward
            if 'social_force_sfm_reward' not in info:
                info['social_force_sfm_reward'] = []
            if 'force_deceleration' not in info:
                info['force_deceleration'] = []
            if 'force_evasion' not in info:
                info['force_evasion'] = []

            info['social_force_sfm_reward'].append(sfm_reward)
            info['force_deceleration'].append(force_decel)
            info['force_evasion'].append(force_evasion)

        if 'total_reward' not in info:
            info['total_reward'] = []
        info['total_reward'].append(total_reward)

        terminal_reward = total_reward

        # The terminal rewards are ranked based on importance as it may occur that multiple entries
        # are in the done_reason that could lead to different terminal rewards.
        if any(reason == 'current_collision' for reason in info['done_reason']):
            self.node.get_logger().info("Current collision detected")
            terminal_reward = self._get_collision_reward()

        elif (any(reason == 'at_goal' for reason in info['done_reason'])
              and 'goal_reached' in self.reward_functions):
            self.node.get_logger().info("Goal reached")
            reward_goal = self._get_at_goal_reward()
            terminal_reward = reward_goal

        elif any(reason == 'future_collision' for reason in info['done_reason']):

            self.node.get_logger().info("Future collision detected")
            reward_future_collision = self._get_future_collision_reward()
            terminal_reward = reward_future_collision

        elif (any(reason == 'outside_interaction_range' for reason in info['done_reason'])
              and 'outside_interaction_range' in self.reward_functions):
            outside_interaction_range_param = self.node.get_parameters_by_prefix(
                'outside_interaction_range')
            path_traversal_for_reward = outside_interaction_range_param[
                'path_travel_positive_interaction'].value
            max_reward = outside_interaction_range_param['max_reward'].value
            min_reward = outside_interaction_range_param['min_reward'].value
            current_plan_length = self.rl_io_manager.last_plan_length

            if self.starting_plan_length - current_plan_length >= path_traversal_for_reward:
                terminal_reward = max_reward
            elif self.starting_plan_length - current_plan_length < path_traversal_for_reward:
                terminal_reward = min_reward
                return min_reward

        elif (any(reason == 'max_timesteps_reached' for reason in info['done_reason'])
              and 'max_timesteps_reached' in self.reward_functions):
            time_reward = self.node.get_parameters_by_prefix('max_timesteps')[
                'reward'].value
            self.node.get_logger().info("Max timesteps reached")
            terminal_reward = time_reward

        if 'terminal_reward' not in info:
            info['terminal_reward'] = []
        info['terminal_reward'].append(terminal_reward)

        return terminal_reward

    # Collision and Interaction functions
    def __lidar_collision_check(self) -> bool:
        """
        Check if there is a collision with the environment based on lidar information.

        Returns
        -------
            bool: True if there is a collision, False otherwise.

        """
        lidar_points = self.rl_io_manager.last_lidar
        threshold = np.sqrt(self.robot_radius_sqr) * \
            self.collision_footprint_factor

        lidar_points = lidar_points.reshape(-1, 2)
        lidar_points_pow = np.power(lidar_points, 2)
        lidar_points_dist = np.sqrt(np.sum(lidar_points_pow, axis=1))

        return any(lidar_points_dist <= threshold)

    def __get_line_of_sight(self, robot_odom, agents_global_frame) -> np.ndarray:
        """
        Check if there is line of sight between the robot and the agents.

        Returns for each agent if they are visible to the robot.

        Args:
        ----
            robot_odom (np.ndarray): The odometry information of the robot.
            agents_global_frame (np.ndarray): The odometry information of the
            agents from the map point of view.

        Returns
        -------
            np.ndarray: True if there is line of sight for each agent.

        """
        if self.footprint_from_robot_frame is not None:
            in_sight_line_array = np.array([])
            for agent in agents_global_frame:
                location_vector = agent[0:2] - robot_odom[0][0:2]
                n_points = int(np.min([
                    np.floor(np.linalg.norm(location_vector) /
                             (2 * np.sqrt(self.robot_radius_sqr))),
                    np.floor(self.interaction_range /
                             (2 * np.sqrt(self.robot_radius_sqr)))
                ]))

                in_sight_line = not self._check_future_collision_(
                    robot_odom[0][0], robot_odom[0][1], agent[0], agent[1],
                    self.footprint_from_robot_frame, n_points
                )
                in_sight_line_array = np.append(
                    in_sight_line_array, in_sight_line)

            return in_sight_line_array

        else:
            self.node.get_logger().error(
                "Footprint collision checker not initialized. \
                 Assuming line of sight for all agents.")
            return np.array([True] * agents_global_frame.shape[0])

    def __check_interaction_conditions(self, location_vector, dot_robot_agent,
                                       dot_location_yaw):
        """
        Check the interaction conditions for each entry in the location_vector.

        Args:
        ----
            location_vector (np.ndarray): The location vector of the agents.
            dot_robot_agent (np.ndarray): The dot product vector of the agents' velocities.
            dot_location_yaw (np.ndarray): The dot product vector of the agents' locations.

        Returns
        -------
            bool: True if any of the conditions are met, False otherwise.

        """
        for i in range(location_vector.shape[0]):
            if np.linalg.norm(location_vector[i]) < self.interaction_range:
                if dot_robot_agent[i] < -0.1 and dot_location_yaw[i] > 0:
                    # Agent is in front of the agent and moving towards the robot
                    self.steps_outside_interaction_range = 0
                    return True

                elif dot_robot_agent[i] > 0.1 and dot_location_yaw[i] < 0:
                    # Agent is behind the robot and moving towards the robot.
                    # Less critical as the robot can move away from the agent.
                    self.steps_outside_interaction_range = 0
                    return True

                if np.linalg.norm(location_vector[i]) < self.critical_interaction_range:
                    # If agent is within a certain range there is an interaction.
                    self.steps_outside_interaction_range = 0
                    return True

        # No interaction within the interaction range
        return False

    def __any_agent_in_interaction_range(self) -> bool:
        """
        Check if an agent is within a certain interaction range of the robot.

        Args:
        ----
            agents_global_frame (np.ndarray): The odometry information of the agent
            from the map point of view.
            robot_odom (np.ndarray): The odometry information of the robot.

        Returns
        -------
            bool: True if the agent is within the interaction range, False otherwise.
              Also returns True if in the first few timesteps to prevent problems in initialization
            If agents_global_frame is None, the function will also return True.

        """
        agents_global_frame = self.rl_io_manager.last_agents_global_frame
        robot_odom = self.rl_io_manager.last_robot_odom

        if (self.trial_time_step < self.timesteps_before_interaction
                or agents_global_frame is None):
            return True

        agents_in_sight = self.__get_line_of_sight(
            robot_odom, agents_global_frame)

        index_agents_in_sight = np.where(agents_in_sight)[0]
        # Only consider agents that are in line of sight
        robot_yaw_vector = np.array(
            [np.cos(robot_odom[0][2]), np.sin(robot_odom[0][2])])
        agent_yaw_vector = np.array(
            [agents_global_frame[index_agents_in_sight, 2],
             agents_global_frame[index_agents_in_sight, 3]])
        # Check if the agent is moving towards the robot if so the dot product will be negative
        dot_robot_agent = np.dot(robot_yaw_vector, agent_yaw_vector)
        # Check where the agent is positioned relative to the robot
        location_vector = agents_global_frame[index_agents_in_sight,
                                              0:2] - robot_odom[0][0:2]
        # If positive means the agent location is in front of the robot
        dot_location_yaw = np.dot(location_vector, robot_yaw_vector)

        if not np.any(agents_in_sight):
            self.steps_outside_interaction_range += 1

        elif not self.__check_interaction_conditions(location_vector, dot_robot_agent,
                                                     dot_location_yaw):
            self.steps_outside_interaction_range += 1

        if self.steps_outside_interaction_range <= self.timesteps_before_switch_to_nav2:
            return True

        else:
            return False

    def _all_agents_outside_range(self):
        """
        Return true if all agents are outside the view of the robot.

        Implements a buffer based on the timesteps_before_agents_in_range which will delay when
        this function returns false.

        Returns
        -------
            bool: True if there are no agents in the interaction range, False otherwise.

        """
        agents = self.rl_io_manager.last_agents
        if np.all([agent == np.array([OUTSIDE_RANGE_LOC, OUTSIDE_RANGE_LOC,
                                      OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_VELOCITY]
                                     ) for agent in agents]):
            self.buffer_agents_in_range = 0
            return True

        else:
            self.buffer_agents_in_range += 1

        if self.buffer_agents_in_range >= self.timesteps_before_agents_in_range:
            self.buffer_agents_in_range = 0
            return False

        else:
            return True

    def _is_done(self, future_collision) -> list[bool, bool, str]:
        """
        Check if the current episode is done by checking various terminal conditions.

        Args:
        ----
            future_collision: Whether a future collision was detected.

        Returns
        -------
            list[bool, bool, str]: The done status.

        """
        current_collision = self.__lidar_collision_check()
        in_interaction_range = self.__any_agent_in_interaction_range()
        # To plot in evaluation the true in interaction range is needed
        self.in_interaction_range = in_interaction_range

        if not self.reset_on_exit_interaction_range:
            # If the agent is allowed to move outside the interaction range,
            # the episode is not terminated if the agent is outside the interaction range
            self.publish_if_in_interaction_range.publish(
                BoolMsg(data=in_interaction_range))
            # If the agent is outside the interaction range the episode is not terminated even
            # if RL yield collision path
            agents_msg_outside_range = self._all_agents_outside_range()

            if not in_interaction_range or agents_msg_outside_range:
                future_collision = False

            in_interaction_range = True

        else:
            in_interaction_range = in_interaction_range

        at_goal = self._check_if_at_goal()

        max_timesteps_reached = self.trial_time_step >= self.max_trial_timesteps
        truncated = max_timesteps_reached
        terminated = current_collision or not in_interaction_range or at_goal

        if max_timesteps_reached:
            done_reason = "max_timesteps_reached"
        elif current_collision:
            done_reason = "current_collision"
        elif future_collision:
            done_reason = "future_collision"
        elif not in_interaction_range:
            done_reason = "outside_interaction_range"
        elif at_goal:
            done_reason = "at_goal"
        else:
            done_reason = "Not done"

        return terminated, truncated, done_reason, self.in_interaction_range

    # Action and publish functions

    def _publish_diff_drive_action(self, action: np.ndarray):
        """
        Publish the 2D diff drive action to the robot topic for visualization.

        Args:
        ----
            action (np.ndarray): The action to be taken by the agent.

        """
        if np.issubdtype(action.dtype, np.integer):
            raise NotImplementedError(
                "Only continuous actions are currently supported for 2D diff drive")

        if not self.in_interaction_range:
            action = self.rl_io_manager.last_nav2_input.flatten()

        twist = Twist()
        twist.linear.x = action[0].item()
        twist.angular.z = action[1].item()
        self.robot_action_publisher.publish(twist)

    def _publish_plan_action(self, action: np.ndarray) -> bool:
        """
        Publish the plan action to the plan topic.

        Args:
        ----
            action (np.ndarray): The action to be taken by the agent.

        Returns
        -------
            bool: True if there is a future collision, False otherwise.

        """
        if np.issubdtype(action.dtype, np.integer):
            action = [float(i)*self.grid_size for i in action]

        if not self.in_interaction_range:
            action = np.array([0.0, 0.0])

        last_robot_odom_list = self.rl_io_manager.last_robot_odom.flatten().tolist()
        last_plan_list = self.rl_io_manager.last_plan.flatten().tolist()

        future_collision = False

        # Publish the robot action
        plan_cmd = Path()
        plan_cmd.header.frame_id = self.rl_io_manager.last_plan_header.frame_id
        plan_cmd.header.stamp = self.rl_io_manager.last_plan_header.stamp

        total_actions = [0, 0]

        for action_index in range(0, len(action)-1, 2):

            if action_index == 0:
                updated_x_position = last_plan_list[0] + action[action_index]
                updated_y_position = last_plan_list[1] + action[action_index+1]
                total_actions[0] += updated_x_position
                total_actions[1] += updated_y_position
            else:
                updated_x_position = total_actions[0] + action[action_index]
                updated_y_position = total_actions[1] + action[action_index+1]
                total_actions[0] += action[action_index]
                total_actions[1] += action[action_index+1]
            if (self.footprint_from_robot_frame is not None
                    and self.use_future_collision_reward
                    and not self.imitation_learning_is_running):
                collision_to_goal = self._check_future_collision_(
                    last_robot_odom_list[0], last_robot_odom_list[1],
                    updated_x_position, updated_y_position,
                    self.footprint_from_robot_frame, num_samples=5)
            else:
                # Not using the footprint collision checker
                collision_to_goal = False

            if collision_to_goal:
                future_collision = True

            # Create safe corridor if needed, currently only works when plan is in the local frame
            if self.force_waypoint_in_corridor:
                updated_pose = self._force_waypoint_inside_safe_corridor(
                    waypoint=np.array([updated_x_position, updated_y_position]))

            else:
                updated_pose = np.array(
                    [updated_x_position, updated_y_position])

            pose = PoseStamped()
            pose.header.frame_id = self.rl_io_manager.last_plan_header.frame_id
            pose.header.stamp = self.rl_io_manager.last_plan_header.stamp
            pose.pose.position.x = updated_pose[0]
            pose.pose.position.y = updated_pose[1]
            plan_cmd.poses.append(pose)

        self.plan_publisher.publish(plan_cmd)

        return future_collision

    def _publish_safe_corridor_rviz(self, vertices: np.ndarray):
        """
        Publish the safe corridor to RViz.

        Args:
        ----
            vertices (np.ndarray): The vertices of the safe corridor.

        """
        hull = ConvexHull(vertices)
        hull_vertices = vertices[hull.vertices]

        # Create a PolygonStamped message
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = self.rl_io_manager.last_plan_header.frame_id
        polygon_msg.header.stamp = self.node.get_clock().now().to_msg()

        for vertex in hull_vertices:
            pt = Point32()
            pt.x = vertex[0]
            pt.y = vertex[1]
            pt.z = 0.0
            polygon_msg.polygon.points.append(pt)
        # Publish the polygon
        self.safe_corridor_pub.publish(polygon_msg)

    def _force_waypoint_inside_safe_corridor(self, waypoint: np.ndarray):
        """
        Force the waypoint inside the safe corridor.

        Args:
        ----
            waypoint (np.ndarray): The waypoint to be forced inside the safe corridor.

        Returns
        -------
            np.ndarray: The waypoint inside the safe corridor.

        """
        # In case max_lidar_distance is both the x and y coordinate. This
        # is not a valid point, but a filler point this needs to be removed.
        raw_crit_points = self.rl_io_manager.last_lidar.reshape(-1, 2)

        mask = ~np.all(raw_crit_points == self.max_lidar_distance, axis=1)
        critical_points = raw_crit_points[mask]
        vertices = get_safe_corridor_vertices(critical_points)

        if vertices is not None and len(vertices) >= 3:

            self._publish_safe_corridor_rviz(vertices)
            if in_convex_hull(waypoint, vertices):
                self.node.get_logger().error(
                    "Waypoint in Safe corridor")
            else:
                self.node.get_logger().error(
                    "Waypoint outside Safe corridor moving it inside vertices")

                waypoint = move_point_to_convex_hull(waypoint, vertices)
        return waypoint

    def step(self, action: np.ndarray) -> list[np.ndarray, np.ndarray, np.ndarray]:
        """
        Execute a single step in the Gazebo environment.

        Uses pause functionality to stop the simulation and conserve the Markov property.
        It also contains functionality to reset the simulation if a collision is detected.
        The step allows for three types of actions.
        Either a single action, a plan of actions or a single waypoint.

        Args:
        ----
        action (np.ndarray): The action to be taken by the agent.

        Returns
        -------
            list[np.ndarray, np.ndarray, np.ndarray]: The next state, reward, and done status.

        """
        info = {}

        if self.rl_action_output == 'diff_drive':
            self._publish_diff_drive_action(action)
            future_collision = False

        elif self.rl_action_output == 'plan':
            future_collision = self._publish_plan_action(action)

        else:
            self.node.get_logger().error(
                "No action output specified. Either 'diff_drive' or 'plan' is required")
            self.node.shutdown()

        # Run the simulation for a time step
        self.pause_node.change_pause_simulation(pause=False)
        t1 = self.node.get_clock().now().nanoseconds / 1e9

        if self.reward_averaging:
            reward_list, terminated_list, truncated_list = self.collect_sub_step_samples(
                action, info, future_collision)

        else:
            self.node.get_clock().sleep_for(
                Duration(seconds=self.pause_time))
            self.pause_node.change_pause_simulation(pause=True)
            self.sim_time += self.pause_time
        t2 = self.node.get_clock().now().nanoseconds / 1e9
        self.node.get_logger().debug(f'Timestep took: {t2 - t1}')
        # Additional check to see if there are agents in the environment
        if not self.rl_io_manager.check_if_in_all_nodes_list(['last_agents_global_frame']):
            self.rl_io_manager.last_agents_global_frame = None
        terminated, truncated, done_reason, in_interaction_range = self._is_done(
            future_collision)
        if self.reward_averaging:
            reward = np.mean(reward_list)
            terminated = np.any(terminated_list) or terminated
            truncated = np.any(truncated_list) or truncated

        else:
            info['done_reason'] = done_reason
            info['in_interaction_range'] = in_interaction_range
            info['sim_time'] = round(self.sim_time, 2)
            reward = self._reward_function(action, info)
        self.node.get_logger().error(f"Reward: {reward}")

        next_state = self.rl_io_manager.get_state()

        if terminated or truncated:
            self.node.get_logger().error(
                f"done at trial step {self.trial_time_step}")
            self.trial_time_step = 1
        else:
            self.trial_time_step += 1

        return next_state, reward, terminated, truncated, info

    def collect_sub_step_samples(self, action, info, future_collision):
        """
        Collect sub-step samples for reward averaging.

        This method executes multiple sub-steps within a single timestep to collect
        reward samples, termination statuses, and additional information. It is used
        when reward averaging is enabled to compute a more robust reward signal.

        Args:
        ----
            action (np.ndarray): The action taken by the agent.
            info (dict): A dictionary to store additional information about the environment state.
            future_collision (bool): Whether a future collision is predicted.

        Returns
        -------
            tuple: A tuple containing:
                - reward_list (list[float]): A list of rewards collected during sub-steps.
                - terminated_list (list[bool]): A list of termination statuses for each sub-step.
                - truncated_list (list[bool]): A list of truncation statuses for each sub-step.

        """
        reward_list = []
        info['robot_pose'] = []
        info['agent_pose'] = []
        info['done_reason'] = []
        info['sim_time'] = []
        info['in_interaction_range'] = []
        terminated_list = []
        truncated_list = []
        for _ in range(self.reward_samples_per_timestep):
            self.node.get_clock().sleep_for(
                Duration(seconds=self.pause_time))
            self.pause_node.change_pause_simulation(pause=True)

            self.sim_time += self.pause_time

            info['sim_time'].append(round(self.sim_time, 2))

            terminated, truncated, done_reason, in_interaction_range = self._is_done(
                future_collision)
            if not self.train:
                # if done_reason != "future_collision" else "Not Done"
                info['done_reason'].append(done_reason)
            else:
                info['done_reason'].append(done_reason)
            info['in_interaction_range'].append(in_interaction_range)
            info['robot_pose'].append(
                self.rl_io_manager.last_robot_odom.flatten().tolist())
            info['agent_pose'].append(
                self.rl_io_manager.last_agents_global_frame.tolist())
            terminated_list.append(terminated)
            truncated_list.append(truncated)

            reward = self._reward_function(action, info)
            reward_list.append(reward)
            self.pause_node.change_pause_simulation(pause=False)
        return reward_list, terminated_list, truncated_list

    def reset(self, seed=None):
        """
        Reset the simulation to the initial state by calling the reset service.

        Args:
        ----
            seed (int, optional): The seed for the random number generator. Defaults to None.

        Returns
        -------
            np.ndarray: The initial state of the environment.
            dict: Additional information.

        """
        self.pause_node.change_pause_simulation(pause=False)

        if self.rl_action_output == 'diff_drive':
            self._publish_diff_drive_action(np.array([0.0, 0.0]))

        if not self.eval_mode:
            self.task_number = int(self.rng.integers(
                0, self.number_of_tasks, dtype=np.int32))

        else:
            self.task_number = int(
                np.floor(self.eval_step / self.num_trials_scenario))
            self.task_number = self.task_number % self.number_of_tasks
            self.eval_step += 1
            if self.task_number >= self.number_of_tasks:
                self.node.get_logger().error(
                    "All tasks evaluated but still in evaluation mode \
                     please check the number of tasks manually shutting down")
                self.node.shutdown()

        self.node.get_logger().error(
            f"Task number: {self.task_number} Task list: {self.task_list} \
              Number of tasks: {self.number_of_tasks}")
        self.task_number_publisher.publish(Int32(data=self.task_number))

        task_params = self.node.get_parameters_by_prefix(
            f'TaskGenerator.param_change_list.{self.task_list[self.task_number]}')
        task_params = [(param_name, param_value.value)
                       for param_name, param_value in task_params.items()]
        self.node.get_logger().error(f"Task params: {task_params}")
        self.send_controller_param_request(task_params)

        self.node.get_logger().error("Reset Requested")
        self.reset_gazebo.reset_simulation(self.task_list[self.task_number])
        time.sleep(self.time_step_length)

        self.pause_node.change_pause_simulation(pause=True)
        info = {}

        return self.rl_io_manager.get_state(), info

    def send_controller_param_request(self, params):
        """
        Set multiple parameters on the /{robot_name}/controller_server node.

        Args:
        ----
            params (list): A list of tuples where each tuple contains the parameter name and value.
                        Example: [('param1', 1.0), ('param2', 42), ('param3', 'value')]

        """
        parameter_list = []

        for param_name, param_value in params:
            if isinstance(param_value, float):
                val = ParameterValue(
                    double_value=param_value, type=ParameterType.PARAMETER_DOUBLE)
            elif isinstance(param_value, int):
                val = ParameterValue(
                    integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
            elif isinstance(param_value, str):
                val = ParameterValue(
                    string_value=param_value, type=ParameterType.PARAMETER_STRING)
            elif isinstance(param_value, bool):
                val = ParameterValue(bool_value=param_value,
                                     type=ParameterType.PARAMETER_BOOL)
            else:
                raise ValueError(
                    f"Unsupported parameter type: {type(param_value)}")

            parameter_list.append(Parameter(name=param_name, value=val))

        self.controller_req.parameters = parameter_list
        self.future = self.controller_cli.call_async(self.controller_req)

        if self.future.done():
            try:
                response = self.future.result()
                if all(result.successful for result in response.results):
                    self.node.get_logger().error(
                        f'Successfully set parameters: {params}')
                    return True
                else:
                    self.node.get_logger().error(
                        f'Failed to set parameters: {params}')
            except Exception as e:
                self.node.get_logger().error(f'Exception: {e}')
            return False
