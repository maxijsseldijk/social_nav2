import os
import traceback
from typing import Callable, TYPE_CHECKING

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import HParam
from stable_baselines3.sac import SAC

if TYPE_CHECKING:
    from train_rl import RLsimulation
    from stable_baselines3.common.vec_env import DummyVecEnv


class SaveOnInterval(BaseCallback):
    """Callback for saving a model every ``save_freq`` steps."""

    def __init__(self, save_freq: int, save_path: str, node, verbose=1):
        """
        Initialize the SaveOnInterval callback.

        Args:
        ----
            save_freq (int): Frequency of saving the model.
            save_path (str): Path to save the model.
            node: The ROS 2 node.
            verbose (int): Verbosity level.

        """
        super(SaveOnInterval, self).__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.node = node

    def _init_callback(self) -> None:
        """Initialize the callback by creating the save directory if it doesn't exist."""
        os.makedirs(self.save_path, exist_ok=True)

    def _on_training_start(self) -> None:
        """Record hyperparameters and metrics at the start of training."""
        env = self.model.get_env().envs[0].unwrapped

        hparam_dict = {
            "algorithm": self.model.__class__.__name__,
            "gamma": self.model.gamma,
            "learning starts": self.model.learning_starts,
            "batch size": self.model.batch_size,
            "ent coef": self.model.ent_coef,
            "target entropy": self.model.target_entropy,
            "Action output": env.rl_action_output,
            "Max timesteps": env.max_trial_timesteps,
            "Time step length": env.time_step_length,
            "Action bounds": str(env.action_bounds),
            "Action space": str(env.action_space),
            "Seed": env.seed
        }

        # define the metrics that will appear in the `HPARAMS` Tensorboard tab
        metric_dict = {
            "rollout/ep_len_mean": 0,
            "train/value_loss": 0.0,
        }
        self.logger.record(
            "hparams",
            HParam(hparam_dict, metric_dict),
            exclude=("stdout", "log", "json", "csv"),
        )

    def _on_rollout_end(self) -> None:
        """Log the mean and standard deviation of rewards at the end of each rollout."""
        local = self.locals["infos"][0]
        if 'total_reward' in local:
            # Calculate mean and standard deviation
            total_reward_mean = np.mean(local["total_reward"])
            terminal_reward_mean = np.mean(
                local["terminal_reward"]) if "terminal_reward" in local else 0
            goal_distance_reward_mean = np.mean(
                local["goal_distance_reward"]) if "goal_distance_reward" in local else 0
            velocity_reward_mean = np.mean(
                local["velocity_reward"]) if "velocity_reward" in local else 0
            social_force_reward = np.mean(
                local["social_force_reward"]) if "social_force_reward" in local else 0
            social_force_sfm_reward = np.mean(
                local["social_force_sfm_reward"]) if "social_force_sfm_reward" in local else 0
            social_force_deceleration = np.mean(
                local["force_deceleration"]) if "force_deceleration" in local else 0
            social_force_evasion = np.mean(
                local["force_evasion"]) if "force_evasion" in local else 0
            proxemics_reward = np.mean(
                local["proxemics_reward"]) if "proxemics_reward" in local else 0

            # Log the mean and std of the total reward
            self.logger.record("reward/total_reward_mean", total_reward_mean)
            self.logger.record("reward/terminal_reward_mean",
                               terminal_reward_mean)
            self.logger.record(
                "reward/goal_distance_reward_mean", goal_distance_reward_mean)
            self.logger.record("reward/velocity_reward_mean",
                               velocity_reward_mean)
            self.logger.record("reward/social_force_reward",
                               social_force_reward)
            self.logger.record(
                "reward/social_force_sfm_reward", social_force_sfm_reward)
            self.logger.record(
                "reward/social_force_deceleration", social_force_deceleration)
            self.logger.record("reward/social_force_evasion",
                               social_force_evasion)
            self.logger.record("reward/proxemics_reward", proxemics_reward)

        return True

    def _on_step(self) -> bool:
        """
        Save the model and replay buffer at specified intervals.

        Returns
        -------
            bool: Whether to continue training.

        """
        if self.n_calls % self.save_freq == 0:
            self.model.save(f"{self.save_path}/model_{self.n_calls}")
            self.model.save_replay_buffer(
                f"{self.save_path}/replay_buffer_{self.n_calls}")
        return True


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """Create a simple linear learning rate schedule."""
    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func


def return_sb3_model(rl_sim_node: 'RLsimulation', wrap_env: 'DummyVecEnv'):
    train_params = rl_sim_node.train_params
    match rl_sim_node.rl_algorithm:
        case 'SAC':
            net_arch = train_params['net_arch'].value
            policy_kwargs = dict(net_arch=net_arch)
            rl_sim_node.log_error(
                f"RL model path: {train_params['save_model_path'].value}")

            model = SAC('MlpPolicy', wrap_env, gamma=train_params['gamma'].value,
                        learning_rate=linear_schedule(
                            train_params['policy_lr'].value),
                        buffer_size=train_params['replay_buffer_size'].value,
                        learning_starts=train_params['learning_starts'].value,
                        batch_size=train_params['batch_size'].value,
                        use_sde=train_params['use_sde'].value,
                        sde_sample_freq=train_params['sde_sample_freq'].value,
                        use_sde_at_warmup=train_params['use_sde_at_warmup'].value,
                        tau=train_params['tau'].value, ent_coef=train_params['ent_coef'].value,
                        train_freq=train_params['train_freq'].value,
                        target_update_interval=train_params['target_update_interval'].value,
                        gradient_steps=train_params['gradient_steps'].value,
                        target_entropy=train_params["target_entropy"].value,
                        action_noise=None, verbose=train_params['verbose'].value,
                        tensorboard_log=f'runs/{train_params["save_model_path"].value}',
                        policy_kwargs=policy_kwargs, seed=rl_sim_node.seed, device='auto',
                        _init_setup_model=True)

            if train_params['load_model'].value and train_params['load_model_path'] is not None:
                try:
                    model = SAC.load(train_params['load_model_path'].value,
                                     wrap_env, weights_only=True)
                    model.ent_coef = train_params['ent_coef'].value
                    rl_sim_node.log_error(
                        f"Loaded {train_params['load_model_path'].value}")
                except FileNotFoundError:
                    rl_sim_node.log_error(
                        f"No model found {traceback.format_exc()}")
                    rl_sim_node.shutdown()
                except Exception:
                    rl_sim_node.log_error(
                        f"General exception: {traceback.format_exc()}")
                    rl_sim_node.shutdown()

            else:
                rl_sim_node.log_error(
                    "No model loaded initializing randomly")

    if train_params['load_replay_buffer'].value:
        try:
            model.load_replay_buffer(
                train_params['load_replay_buffer_path'].value)
            rl_sim_node.log_error(
                f"Replay buffer loaded from {train_params['load_replay_buffer_path'].value}")
        except FileNotFoundError:
            rl_sim_node.log_error(
                f"Replay buffer not found at {train_params['load_replay_buffer_path'].value}")
            rl_sim_node.shutdown()

    return model
