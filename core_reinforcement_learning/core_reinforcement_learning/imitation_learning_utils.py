#!/usr/bin/env python3
from typing import Callable
import numpy as np
import torch

from rclpy.node import Node
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.buffers import ReplayBuffer
from core_reinforcement_learning.rl_node_manager import RlSubscriptionManager

from imitation.algorithms.dagger import SimpleDAggerTrainer
from imitation.algorithms import bc


class ImitationLearning:
    """
    Generic class for initializing and training imitation learning algorithms.

    For algorithm-specific parameters, they are passed with the kwargs dictionary.

    For using DAgger with the Nav2Policy, the following kwarg parameters are required:
    - rng(np.random.Generator): The random number generator for the training
    - rl_io_manager(RlSubscriptionManager): The subscription manager for the RL topics
    """

    def __init__(self, sim_env, observation_space, action_space,
                 to_train_policy: BasePolicy, node: Node, tmp_dir: str, **kwargs):
        """
        Initialize the ImitationLearning class.

        Args:
        ----
            sim_env: The simulation environment.
            observation_space: The observation space.
            action_space: The action space.
            to_train_policy (BasePolicy): The policy to train.
            node (Node): The ROS 2 node.
            tmp_dir (str): The temporary directory for storing data.
            **kwargs: Additional keyword arguments.

        """
        self.node = node
        self.sim_env = sim_env
        self.observation_space = observation_space
        self.action_space = action_space
        self.to_train_policy = to_train_policy
        self.tmp_dir = tmp_dir

        self.main_params = self.node.get_parameters_by_prefix(
            'imitation_learning')
        self.seed = self.node.get_parameter('seed').value
        self.expert_class = self.get_class_from_string(
            self.main_params['expert_class'].value, kwargs)
        self.imitation_algorithm = self.get_algo_from_string(
            self.main_params['imitation_algorithm'].value, kwargs)

    def get_class_from_string(self, class_name: str, kwargs: dict) -> Callable:
        """
        Get the class from the string name.

        Args
        ----
            class_name (str): The name of the class.
            kwargs (dict): Additional keyword arguments.

        Returns
        -------
            Callable: The class corresponding to the string name.

        """
        if class_name == 'Nav2Policy':
            params = {
                'rl_io_manager': kwargs.get('rl_io_manager'),
                'rl_action_output': self.node.get_parameter('rl_action_output').value,
                'expert_noise_std': self.main_params['expert_noise_std'].value,
                'observation_space': self.observation_space,
                'action_space': self.action_space,
                'seed': self.seed
            }
            self.node.get_logger().debug(
                f"Parameters used in expert {class_name} class: {params}")
            self.check_missing_params(params)

            return Nav2Policy(**params)

        if class_name == 'SocialForcePolicy':
            params = {
                'rl_io_manager': kwargs.get('rl_io_manager'),
                'rl_action_output': self.node.get_parameter('rl_action_output').value,
                'expert_noise_std': self.main_params['expert_noise_std'].value,
                'observation_space': self.observation_space,
                'action_space': self.action_space,
                'seed': self.seed
            }
            self.node.get_logger().debug(
                f"Parameters used in expert {class_name} class: {params}")
            self.check_missing_params(params)

            return SocialForcePolicy(**params)

    def get_algo_from_string(self, algo_name: str, kwargs: dict) -> Callable:
        """
        Get the algorithm from the string name.

        Args
        ----
            algo_name (str): The name of the algorithm.
            kwargs (dict): Additional keyword arguments.

        Returns
        -------
            Callable: The algorithm corresponding to the string name.

        """
        if algo_name.lower() == 'dagger':
            params = {
                'sim_env': self.sim_env,
                'scratch_dir': self.tmp_dir,
                'observation_space': self.observation_space,
                'action_space': self.action_space,
                'policy': self.to_train_policy,
                'rng': kwargs.get('rng')
            }
            self.node.get_logger().debug(
                f"Parameters used in {algo_name} algorithm: {params}")

            self.train_params = {
                'total_timesteps': self.main_params['dagger.expert_time_steps'].value,
                'rollout_round_min_episodes': self.main_params[
                    'dagger.rollout_round_min_episodes'].value,
                'rollout_round_min_timesteps': self.main_params[
                    'dagger.rollout_round_min_timesteps'].value
            }
            self.node.get_logger().debug(
                f"Training parameters used in {algo_name} algorithm: {self.train_params}")
            self.check_missing_params(params)

            bc_trainer = bc.BC(observation_space=params['observation_space'],
                               action_space=params['action_space'],
                               policy=params['policy'], rng=params['rng'])
            return SimpleDAggerTrainer(venv=params['sim_env'],
                                       scratch_dir=params['scratch_dir'],
                                       expert_policy=self.expert_class,
                                       bc_trainer=bc_trainer, rng=params['rng'])

    def train(self):
        """Train the imitation learning algorithm."""
        self.imitation_algorithm.train(**self.train_params)

    def get_num_rollouts(self):
        """Get the number of rollouts."""
        return self.main_params['expert_rollout_episodes'].value

    def get_policy(self):
        """Get the trained policy."""
        return self.imitation_algorithm.policy

    def check_missing_params(self, params: dict):
        """
        Check for missing parameters and log an error if any are missing.

        Args:
        ----
            params (dict): Dictionary of parameter names and their values.

        """
        missing_params = [param_name for param_name,
                          param_value in params.items() if param_value is None]
        if missing_params:
            raise ValueError(
                f"One or more parameters are missing: {', '.join(missing_params)}")


class SocialForcePolicy(BasePolicy):
    """Policy class for using expert demonstrations obtained from the Social Force Window model."""

    def __init__(self, rl_io_manager: RlSubscriptionManager,
                 rl_action_output: str, expert_noise_std: float = 0.0,
                 seed: int = 0, **kwargs):
        super(SocialForcePolicy, self).__init__(**kwargs)
        self.seed = seed
        self.rl_action_output = rl_action_output
        self.expert_noise_std = expert_noise_std
        self.rl_io_manager = rl_io_manager
        self.rng = np.random.default_rng(self.seed)

    def forward(self, obs, deterministic=False):
        """
        Forward pass in the model.

        Args
        ----
            obs: The observations.
            deterministic (bool): Whether to use deterministic actions.

        Returns
        -------
            The predicted actions.

        """
        return self.return_nav2_action()

    def _predict(self, obs, deterministic=False):
        """
        Predict the action given the observation.

        Args
        ----
            obs: The observations.
            deterministic (bool): Whether to use deterministic actions.

        Returns
        -------
            The predicted actions.

        """
        return self.forward(obs, deterministic)

    def return_nav2_action(self) -> np.ndarray:
        """Return the action from the NAV2 planner."""
        if self.rl_action_output == 'diff_drive':
            return torch.tensor(self.rng.normal([self.rl_io_manager.last_nav2_input.flatten()[0],
                                                self.rl_io_manager.last_nav2_input.flatten()[1]],
                                                self.expert_noise_std, size=2)).to(self.device)

        elif self.rl_action_output == 'plan':
            converted_plan = self.__convert_last_sfm_to_action(
                self.rl_io_manager.last_sfm_control_point)
            return torch.tensor(self.rng.normal(converted_plan, self.expert_noise_std,
                                                size=converted_plan.shape)).to(self.device)

    def __convert_last_sfm_to_action(self, last_sfm_control_point: np.ndarray) -> np.ndarray:
        """
        Convert the plan to actions if the actions are dependent on the previous actions.

        Args
        ----
            last_sfm_control_point (np.ndarray): The last SFM control point.

        Returns
        -------
            np.ndarray: The converted plan in Cartesian coordinates.

        """
        last_sfm_control_point = last_sfm_control_point.flatten()
        last_plan = self.rl_io_manager.last_plan.flatten()
        total_actions = np.zeros(2, dtype=last_sfm_control_point.dtype)

        converted_plan = np.zeros_like(last_sfm_control_point)

        total_actions[0] = last_sfm_control_point[0] - last_plan[0]
        total_actions[1] = last_sfm_control_point[1] - last_plan[1]
        converted_plan[0] = total_actions[0]
        converted_plan[1] = total_actions[1]

        for i in range(2, len(last_sfm_control_point), 2):
            action_x = last_sfm_control_point[i] - \
                last_sfm_control_point[i - 2]
            action_y = last_sfm_control_point[i +
                                              1] - last_sfm_control_point[i - 1]
            total_actions[0] += action_x
            total_actions[1] += action_y
            converted_plan[i] = action_x
            converted_plan[i + 1] = action_y

        # converted_plan = self._cartesian_to_polar(converted_plan)
        return converted_plan


class Nav2Policy(BasePolicy):
    """Policy class for using expert demonstrations obtained from following the NAV2 planner."""

    def __init__(self, rl_io_manager: RlSubscriptionManager, rl_action_output: str,
                 expert_noise_std: float = 0.0, seed: int = 0, **kwargs):
        super(Nav2Policy, self).__init__(**kwargs)
        self.seed = seed
        self.rl_action_output = rl_action_output
        self.expert_noise_std = expert_noise_std
        self.rl_io_manager = rl_io_manager
        self.rng = np.random.default_rng(self.seed)

    def forward(self, obs, deterministic=False):
        """
        Forward pass in the model.

        Args
        ----
            obs: The observations.
            deterministic (bool): Whether to use deterministic actions.

        Returns
        -------
            The predicted actions.

        """
        return self.return_nav2_action()

    def _predict(self, obs, deterministic=False):
        """
        Predict the action given the observation.

        Args
        ----
            obs: The observations.
            deterministic (bool): Whether to use deterministic actions.

        Returns
        -------
            The predicted actions.

        """
        return self.forward(obs, deterministic)

    def return_nav2_action(self) -> np.ndarray:
        """Return the action from the NAV2 planner."""
        if self.rl_action_output == 'diff_drive':
            return torch.tensor(self.rng.normal([self.rl_io_manager.last_nav2_input.flatten()[0],
                                                 self.rl_io_manager.last_nav2_input.flatten()[1]],
                                                self.expert_noise_std, size=2)).to(self.device)

        elif self.rl_action_output == 'plan':
            converted_plan = self.__convert_last_plan_to_action(
                self.rl_io_manager.last_plan)
            return torch.tensor(self.rng.normal(converted_plan, self.expert_noise_std,
                                                size=converted_plan.shape)).to(self.device)

    def __convert_last_plan_to_action(self, last_plan: np.ndarray) -> np.ndarray:
        """
        Convert the plan to actions if the actions are dependent on the previous actions.

        Args
        ----
            last_plan (np.ndarray): The last plan in Cartesian coordinates.

        Returns
        -------
            np.ndarray: The converted plan in Cartesian coordinates.

        """
        last_plan = last_plan.flatten()

        total_actions = np.zeros(2, dtype=last_plan.dtype)
        converted_plan = np.zeros_like(last_plan)

        total_actions[0] = 0
        total_actions[1] = 0
        converted_plan[0] = total_actions[0]
        converted_plan[1] = total_actions[1]

        for i in range(2, len(last_plan), 2):
            action_x = last_plan[i] - last_plan[i - 2]
            action_y = last_plan[i + 1] - last_plan[i - 1]
            total_actions[0] += action_x
            total_actions[1] += action_y
            converted_plan[i] = action_x
            converted_plan[i + 1] = action_y

        # converted_plan = self._cartesian_to_polar(converted_plan)
        return converted_plan

    def _cartesian_to_polar(self, actions: np.ndarray) -> np.ndarray:
        """
        Convert a list of actions from Cartesian to polar coordinates.

        Args
        ----
            actions (np.ndarray): The list of actions in Cartesian coordinates.

        Returns
        -------
            np.ndarray: The list of actions in polar coordinates.

        """
        for i in range(0, len(actions), 2):
            actions[i], actions[i +
                                1] = self.__cartesian_to_polar(actions[i], actions[i + 1])
        return actions

    def __cartesian_to_polar(self, x: float, y: float) -> tuple:
        """
        Convert Cartesian coordinates to polar coordinates.

        Args
        ----
            x (float): The x-coordinate in Cartesian coordinates.
            y (float): The y-coordinate in Cartesian coordinates.

        Returns
        -------
            tuple: The polar coordinates (r, theta).

        """
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        if theta < 0:
            theta += 2 * np.pi

        # Normalize the angle to be between action space limits
        theta = (theta / (2 * np.pi)) * \
            (self.action_space.high[1] -
             self.action_space.low[1]) + self.action_space.low[1]
        return r, theta


class ReplayBufferCallback(BaseCallback):
    def __init__(self, logging_node: Node, replay_buffer: ReplayBuffer,
                 log_freq: int = 20, verbose=0):
        super(ReplayBufferCallback, self).__init__(verbose)
        self.replay_buffer = replay_buffer
        self.logging_node = logging_node
        self.log_freq = log_freq

    def _on_step(self) -> bool:
        """
        Add the current transition to the replay buffer and log the buffer size.

        Returns
        -------
            bool: Whether to continue training.

        """
        self.replay_buffer.add(
            self.locals['observations'],
            self.locals['new_observations'],
            self.locals['actions'],
            self.locals['rewards'],
            self.locals['dones'],
            self.locals['infos'],
        )

        if self.n_calls % self.log_freq == 0:
            self.logging_node.get_logger().error(
                f"Current at step {self.n_calls} replay buffer size: {self.replay_buffer.size()}")
        return True

    def __call__(self, locals, globals):
        """
        Call the callback function.

        Args:
        ----
            locals: Local variables.
            globals: Global variables.

        """
        self.locals = locals
        self.globals = globals
        self._on_step()
