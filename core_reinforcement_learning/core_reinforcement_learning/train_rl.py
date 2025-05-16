#!/usr/bin/env python3

import threading
import traceback
import tempfile
from typing import TYPE_CHECKING

import numpy as np
import torch
from std_msgs.msg import Bool as BoolMsg

import rclpy
from rclpy.exceptions import ROSInterruptException
from rclpy.node import Node

from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv

from core_reinforcement_learning.imitation_learning_utils import (ImitationLearning,
                                                                  ReplayBufferCallback)
from core_reinforcement_learning.eval_utils import Evaluate_Model, eval_data_callback
from core_reinforcement_learning.ros_gazebo_env import GazeboEnv
from core_reinforcement_learning.rl_alg_utils import SaveOnInterval, return_sb3_model

if TYPE_CHECKING:
    from ros_gazebo_env import RlSubscriptionManager


class RLsimulation(Node):
    def __init__(self):
        super().__init__('train_rl', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.log_info('RL node has been started')
        self.rl_algorithm = self.get_parameter('rl_algorithm').value
        self.train_params = self.get_parameters_by_prefix(
            str(self.rl_algorithm))
        self.time_steps = self.get_parameter('time_steps').value
        self.time_step_length = self.get_parameter('time_step_length').value
        self.seed = self.get_parameter('seed').value
        self.rng = np.random.default_rng(self.seed)
        self.sim_path = self.get_parameter(
            f'{self.rl_algorithm}.save_model_path').value
        self.sim_name = self.get_parameter('sim_name').value
        self.use_reward_averaging = self.get_parameter(
            'reward_averaging').value
        self.reward_samples_per_timestep = self.get_parameter(
            'reward_samples_per_timestep').value if self.use_reward_averaging else 1
        self.load_results_only_mode = self.get_parameter(
            'load_results_only_mode').value
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
        self.use_expert_demonstrations = self.get_parameter(
            'use_expert_demonstrations').value

        self.sfm_eval_params = {
            'A': self.get_parameter('social_force_sfm.A').value,
            'n': self.get_parameter('social_force_sfm.n').value,
            'n_prime': self.get_parameter('social_force_sfm.n_prime').value,
            'lambda_': self.get_parameter('social_force_sfm.lambda_').value,
            'epsilon': self.get_parameter('social_force_sfm.epsilon').value,
            'gamma_': self.get_parameter('social_force_sfm.gamma_').value,
        }

    def log_info(self, msg):
        self.get_logger().info(msg)

    def log_error(self, msg):
        self.get_logger().error(msg)

    def shutdown(self):
        self.get_logger().error('Shutting down RL algorithm')

        for timer in self.timers:
            timer.cancel()

        for publisher in self.publishers:
            publisher.destroy()

        for subscriber in self.subscriptions:
            subscriber.destroy()
        self.destroy_node()
        rclpy.shutdown()


def spin_subs(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def evaluate_rl_model(rl_sim_node: RLsimulation, env: GazeboEnv,
                      rl_io_manager: 'RlSubscriptionManager',
                      wrap_eval_env: GazeboEnv, model):

    if rl_sim_node.use_expert_demonstrations:
        env.set_imitation_learning_running(True)

    mean_reward, std_reward = evaluate_policy(model, wrap_eval_env,
                                              env.number_of_tasks * env.num_trials_scenario,
                                              callback=lambda locals, globals: eval_data_callback(
                                                  locals, globals, rl_sim_node, rl_io_manager
                                              ),
                                              deterministic=True)

    rl_sim_node.log_error(
        f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")
    rl_sim_node.log_error(
        f"Total timesteps: {model.num_timesteps}")

    # Plot specify for Social Interaction Learning
    eval_model = Evaluate_Model(
        rl_sim_node, **rl_sim_node.sfm_eval_params)

    eval_model.plot_reward_data()
    rl_sim_node.shutdown()


def train_rl_model(rl_sim_node: RLsimulation, env: GazeboEnv,
                   wrap_eval_env: GazeboEnv,
                   model):
    rl_sim_node.log_error("Starting training")
    # Some linespace saving by defining explicitly
    train_params = rl_sim_node.train_params

    callback_save = SaveOnInterval(
        save_freq=train_params['save_model_interval'].value,
        save_path=train_params['save_model_path'].value, node=rl_sim_node)

    calback_eval = EvalCallback(wrap_eval_env,
                                n_eval_episodes=env.number_of_tasks * env.num_trials_scenario,
                                best_model_save_path=f"{train_params['save_model_path'].value}/",
                                log_path=f"{train_params['save_model_path'].value}/",
                                eval_freq=train_params['eval_freq'].value,
                                deterministic=True, render=False)

    model.learn(total_timesteps=rl_sim_node.time_steps, callback=[
        callback_save, calback_eval], log_interval=4, tb_log_name='SAC', reset_num_timesteps=True)
    model.save(
        f"{train_params['save_model_path'].value}/final_model")


def collect_expert_trajectories(rl_sim_node: RLsimulation, env: GazeboEnv,
                                rl_io_manager: 'RlSubscriptionManager',
                                wrap_env: DummyVecEnv,
                                model):
    env.set_imitation_learning_running(True)
    rl_sim_node.log_error("Using expert demonstrations")
    with tempfile.TemporaryDirectory(prefix="dagger_example_") as tmp_dir:
        imitation_kwargs = {
            'rl_io_manager': rl_io_manager,
            'rng': rl_sim_node.rng,
        }
        imitation_agent = ImitationLearning(sim_env=wrap_env, tmp_dir=tmp_dir,
                                            observation_space=wrap_env.observation_space,
                                            action_space=wrap_env.action_space,
                                            node=rl_sim_node,
                                            to_train_policy=model.policy,
                                            **imitation_kwargs)
        rl_sim_node.log_error(
            "Expert loaded starting training")

        imitation_agent.train()
        model.policy = imitation_agent.get_policy()

        model.save(
            f"{rl_sim_node.train_params['save_model_path'].value}/model_expert")

        buffer_callback = ReplayBufferCallback(
            rl_sim_node, model.replay_buffer)
        env.set_imitation_learning_running(False)

        rl_sim_node.log_error(
            f"Old replay buffer size: {model.replay_buffer.size()}")
        mean_reward, std_reward = evaluate_policy(model, model.get_env(),
                                                  imitation_agent.get_num_rollouts(),
                                                  callback=buffer_callback, deterministic=True)
        rl_sim_node.log_error(
            f"Mean reward model policy: {mean_reward:.2f} +/- {std_reward:.2f}")

        model.save_replay_buffer(
            f"{rl_sim_node.train_params['save_model_path'].value}/replay_buffer_expert")
        rl_sim_node.log_error(
            f"New replay buffer size: {model.replay_buffer.size()}")


def main():

    rclpy.init(args=None)
    rl_sim_node = RLsimulation()

    # Set the seed for reproducibility
    torch.manual_seed(rl_sim_node.seed)
    env = GazeboEnv(node=rl_sim_node, rng=rl_sim_node.rng,
                    seed=rl_sim_node.seed, eval=False)
    env_eval = GazeboEnv(node=rl_sim_node, rng=rl_sim_node.rng,
                         seed=rl_sim_node.seed, eval=True)

    rl_io_manager = env.rl_io_manager

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(rl_sim_node)

    # Spin the subscriptions in a separate thread
    spin_thread = threading.Thread(target=spin_subs, args=(executor,))
    spin_thread.start()

    rl_sim_node.log_error(f"Device: {rl_sim_node.device}")
    if rl_sim_node.load_results_only_mode:
        eval_model = Evaluate_Model(rl_sim_node, **rl_sim_node.sfm_eval_params)
        eval_model.plot_social_interaction()
        eval_model.plot_reward_data()
        rl_sim_node.shutdown()

    rl_io_manager.wait_till_all_data_received()
    rl_sim_node.log_info("All data received for running RL")

    # Get the initial length of the plan if it exists for use in the reward function
    env.create_utility_functions_from_requirements()
    env_eval.create_utility_functions_from_requirements()

    state_dim = rl_io_manager.get_state_size()
    action_dim = env.get_action_size()

    env.set_action_and_observation_space()
    env_eval.set_action_and_observation_space()
    rl_sim_node.log_error(
        f"State dim: {state_dim}, Action dim: {action_dim}")

    wrap_env = DummyVecEnv([lambda: env])
    wrap_eval_env = DummyVecEnv([lambda: env_eval])

    env.pause_node.set_gym_pause(True)

    model = return_sb3_model(rl_sim_node, wrap_env)

    try:
        while rclpy.ok():
            env.pause_node.set_gym_pause(False)
            env.pause_motion_publisher.publish(BoolMsg(data=False))
            if rl_sim_node.use_expert_demonstrations and env.train:
                collect_expert_trajectories(
                    rl_sim_node, env, rl_io_manager, wrap_env, model)

            if env.train:
                train_rl_model(rl_sim_node, env, wrap_eval_env, model)

            else:
                evaluate_rl_model(rl_sim_node, env,
                                  rl_io_manager, wrap_eval_env, model)

    except ROSInterruptException as e:
        rl_sim_node.log_error(f"ROS interrupt exception caught: {e}")
    except Exception as e:
        rl_sim_node.log_error(
            f"General exception caught:{e} {traceback.format_exc()}")

    finally:
        # Ensure all subscribers are destroyed properly
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
