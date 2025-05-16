import unittest
from unittest.mock import MagicMock
from core_reinforcement_learning.ros_gazebo_env import GazeboEnv
import numpy as np
from core_reinforcement_learning.publish_agents_velocity import (
    OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_LOC
)


class FakeParameter:
    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        if isinstance(self.value, list):
            return FakeParameterValue(self.value)
        return self.value


class FakeParameterValue:
    def __init__(self, value):
        self.string_array_value = value


class FakeNode:
    def __init__(self, parameters):
        self.parameters = parameters
        self.logger = FakeLogger()
        self.subscriptions = []
        self.clients = []
        self.publishers = []

    def get_parameter(self, name):
        return self.parameters.get(name, FakeParameter(None))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self

    def now(self):
        return self

    def to_msg(self):
        return self

    def get_namespace(self):
        return 'Test'

    def log_info(self, msg):
        self.get_logger().info(msg)

    def create_subscription(self, msg_type, topic, callback, qos_profile,
                            callback_group=None, event_callbacks=None, raw=False):
        subscription = MagicMock()
        self.subscriptions.append(subscription)
        return subscription

    def create_publisher(self, msg_type, topic, qos_profile,
                         callback_group=None, event_callbacks=None):
        publisher = MagicMock()
        self.publishers.append(publisher)
        return publisher

    def create_client(self, srv_type, srv_name, *, qos_profile=None, callback_group=None):
        client = MagicMock()
        self.clients.append(client)
        return client

    def get_parameters_by_prefix(self, prefix: str):
        return self.parameters[prefix]

    def log_error(self, msg):
        self.get_logger().error(msg)

    def shutdown(self):
        return self


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)

    def info(self, msg):
        print(msg)

    def assert_error_called_with(self, msg):
        assert msg in self.errors, f"Expected error message '{msg}' not found in {self.errors}"


class TestGazeboEnv(unittest.TestCase):

    def setUp(self):
        self.parameters = {

            'rl_algorithm': FakeParameter('SAC'),
            'time_steps': FakeParameter(1000),
            'time_step_length': FakeParameter(0.1),
            'seed': FakeParameter(42),
            'SAC.save_model_path': FakeParameter('/path/to/save/model'),
            'sim_name': FakeParameter('test_sim'),
            'load_results_only_mode': FakeParameter(False),
            'use_expert_demonstrations': FakeParameter(False),
            'num_eval_per_scenario': FakeParameter(5),
            'max_trial_timesteps': FakeParameter(1000),
            'reward_functions': FakeParameter(['placeholder']),
            'agent_velocity_disturbance': {'max_reward': FakeParameter(0.0),
                                           'min_reward': FakeParameter(-1.0),
                                           'v_pref': FakeParameter(0.4)},
            'path_traversal': {
                'max_reward': FakeParameter(0.2),
                'path_traversal_for_positive_reward': FakeParameter(0.2),
            },
            'social_force_sfm': {
                'max_reward': FakeParameter(0.0),
                'min_reward': FakeParameter(-3.0),
                'min_social_force': FakeParameter(-3.0),
                'lambda_': FakeParameter(2.0),
                'gamma_': FakeParameter(0.85),
                'n': FakeParameter(2.0),
                'A': FakeParameter(4.5),
                'n_prime': FakeParameter(3.0),
                'epsilon': FakeParameter(0.005),
            },
            'goal_reached': {
                'goal_threshold': FakeParameter(0.7),
                'reward': FakeParameter(1.0),
            },
            'collision': {
                'reward': FakeParameter(-3.0),
            },
            'proxemics': {
                'min_reward': FakeParameter(-3.0),
                'max_reward': FakeParameter(0.0),
                'distance_threshold': FakeParameter(0.85),
            },
            'future_collision': {
                'reward': FakeParameter(-3.0),
                'future_collision_distance': FakeParameter(0.5),
            },
            'outside_interaction_range': {
                'max_reward': FakeParameter(1.0),
                'min_reward': FakeParameter(-1.0),
                'path_travel_positive_interaction': FakeParameter(2.0),
            },
            'max_timesteps': {
                'reward': FakeParameter(-1.0)
            },
            'imitation_learning.expert_class': FakeParameter('Nav2Policy'),
            'reward_averaging': FakeParameter(True),
            'reward_samples_per_timestep': FakeParameter(10),
            'train': FakeParameter(True),
            'rl_action_output': FakeParameter('plan'),
            'grid_size': FakeParameter(0.5),
            'rl_node_manager.callback_frequency': FakeParameter(10),
            'rl_node_manager.state_nodes': FakeParameter(['plan', 'nav2_input']),
            'rl_node_manager.utility_nodes': FakeParameter(['utility_node_1', 'utility_node_2']),
            'rl_node_manager.plan.topic': FakeParameter('/plan'),
            'rl_node_manager.plan.type': FakeParameter('PathWithLength'),
            'rl_node_manager.plan.callback': FakeParameter('plan_with_length_callback'),
            'rl_node_manager.nav2_input.topic': FakeParameter('/nav2_input'),
            'rl_node_manager.nav2_input.type': FakeParameter('Twist'),
            'rl_node_manager.nav2_input.callback': FakeParameter('nav2_input_callback'),
            'rl_node_manager.utility_node_1.topic': FakeParameter('/utility_node_1_topic'),
            'rl_node_manager.utility_node_1.type': FakeParameter('LaserScan'),
            'rl_node_manager.utility_node_1.callback': FakeParameter('lidar_raw_callback'),
            'rl_node_manager.utility_node_2.topic': FakeParameter('/utility_node_2_topic'),
            'rl_node_manager.utility_node_2.type': FakeParameter('People'),
            'rl_node_manager.utility_node_2.callback': FakeParameter('agents_callback'),
            'agent_names': FakeParameter(['agent1', 'agent2']),
            'robot_names': FakeParameter(['robot1']),
            'width': FakeParameter(0.5),
            'length': FakeParameter(1.0),
            'radius': FakeParameter(None),
            'world': FakeParameter('world_name'),
            'action_bounds': FakeParameter([-1.0, 1.0]),
            'reset_on_exit_interaction_range': FakeParameter(True),
            'interaction_range': FakeParameter(5.0),
            'critical_interaction_range': FakeParameter(2.0),
            'timesteps_before_switch_to_nav2': FakeParameter(10),
            'timesteps_before_interaction': FakeParameter(5),
            'timesteps_before_agents_in_range': FakeParameter(3),
            'use_constant_for_distant_measurements': FakeParameter(True),
            'max_lidar_distance': FakeParameter(10.0),
            'footprint_scale': FakeParameter(1.0),
            'TaskGenerator': {
                'task_list': FakeParameter(
                    ['corridor_passing_south']),
                'init_task_number': FakeParameter(0), },

        }

        self.node = FakeNode(self.parameters)
        self.env = GazeboEnv(node=self.node)
        self.env.rl_io_manager.last_plan = MagicMock()
        self.env.rl_io_manager.last_nav2_input = MagicMock()

    def test_initialization(self):
        """Test that the environment initializes correctly."""
        self.assertIsNotNone(self.env)
        self.assertEqual(self.env.time_step_length, 0.1)
        self.assertEqual(self.env.rl_action_output, 'plan')
        self.assertEqual(self.env.rl_io_manager.state_callback_names, [
                         'last_plan', 'last_nav2_input'])

    def test_set_action_and_observation_space(self):
        """Test setting action and observation spaces."""
        self.env.get_action_size = MagicMock(return_value=2)
        self.env.get_state_size = MagicMock(return_value=5)
        self.env.set_action_and_observation_space()

        self.assertIsNotNone(self.env.action_space)
        self.assertIsNotNone(self.env.observation_space)
        self.assertEqual(self.env.action_space.shape[0], 2)
        self.assertEqual(self.env.observation_space.shape[0], 5)

    # Testing of reward function related functions
    def test_agent_velocity_disturbance_reward(self):

        self.env.reward_functions = ['agent_velocity_disturbance']
        # Test 1 is for when agent stops while in the neighboorhood
        # of the robot. This should result in the min reward
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_agents = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        # Mock action value as it is not needed here
        action = np.array([0.0, 0.0])
        info = {'in_interaction_range': True, 'done_reason': []}

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, -1.0)

        # Test 2 is for when agent goes half the speed of V_pref while in the neighboorhood
        # of the robot.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.2, 0.0]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, -0.75)

        # Test 3 is for when agent goes at the speed of V_pref while in the neighboorhood
        # of the robot.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.4, 0.0]])
        reward = self.env._reward_function(action, info)

        self.assertAlmostEqual(reward, 0.0)

        # Test 4 is for when agent goes faster than the speed of V_pref while in the neighboorhood
        # of the robot.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.4, 0.7]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, 0.0)

        # Test 4 is for when agent goes faster than the speed of V_pref while in the neighboorhood
        # of the robot.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.4, 0.7]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, 0.0)

        # Test 5 is when there are multiple robots in the environment
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.4, 0.7]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, 0.0)

    def test_proxemics_reward(self):

        self.env.reward_functions = ['proxemics']
        # Mock action value as it is not needed here
        action = np.array([0.0, 0.0])

        # Assume the robot and agent are in interaction range.
        info = {'in_interaction_range': True, 'done_reason': []}

        # Test 1 is for checking the reward function when a robot and agent at the same location.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, -3.0)

        # Test 2 is for checking the reward function when a robot and agent at the threshold dist.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.85, 0.0, 0.0, 0.0]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, 0.0)

        # Test 3 is for checking the reward function when a robot and agent distance
        # is slightly below the threshold.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.8499, 0.0, 0.0, 0.0]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, -3.0)

        # Test 4 is for checking the reward function with multiple robots in the environment.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0], [0.0, 0.5, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.88, 0.0, 0.0, 0.0]])
        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, -3.0)

    def test_path_traversal_reward(self):

        self.env.reward_functions = ['path_traversal']
        # Mock action value as it is not needed here
        action = np.array([0.0, 0.0])

        # Assume the robot and agent are in interaction range.
        info = {'in_interaction_range': True, 'done_reason': []}
        # Test 1 is for checking the reward function when a robot does not move toward the goal.
        self.env.last_distance_to_goal = 0.8
        self.env.rl_io_manager.last_plan_length = 0.8

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), -0.15)

        # Test 2 is for checking the reward function when a
        # robot moves the path_traversal_for_positive_reward distance.
        self.env.last_distance_to_goal = 0.8
        self.env.rl_io_manager.last_plan_length = 0.6

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(reward, 0.0)

        # Test 3 is for checking the reward function when a
        # robot moves away from the goal
        self.env.last_distance_to_goal = 0.8
        self.env.rl_io_manager.last_plan_length = 1.0

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), -0.19)

        # Test 4 is for checking the reward function when first starting to return 0
        self.env.last_distance_to_goal = None
        self.env.rl_io_manager.last_plan_length = 0.4

        reward = self.env._reward_function(action, info)

        self.assertEqual(self.env.last_distance_to_goal,
                         self.env.rl_io_manager.last_plan_length)
        self.assertAlmostEqual(round(reward, 2), 0.0)

    def test_sfm_reward(self):

        self.env.reward_functions = ['social_force_sfm']
        # Mock action value as it is not needed here
        action = np.array([0.0, 0.0])

        # Assume the robot and agent are in interaction range.
        info = {'in_interaction_range': True, 'done_reason': []}

        # Test 1 is for checking the reward function when a robot and agent at the same location
        # And standing still (Edge case likely a faulty sensor).
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), 0.0)

        # Test 2 is for checking the reward function when a robot and agent at the same location
        # But there is a velocity difference (Edge case).
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.0, 0.0, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.22, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), -3.0)

        # Test 3 is for checking the reward function when a robot and agent move in the same line
        #  but the agent moves away much more quickly (No disturbance case).
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, 0.92, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.22, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), 0.0)

        # Test 4 is for checking the reward function when a robot and agent move in the same line
        # at the same speed (Small disturbance case).
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, 0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.22, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), -0.97)

        # Test 5 is for checking the reward function when a robot and agent move in the same line
        # towards each other (Large disturbance case).
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, -0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.22, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), -2.34)

        # Test 6 is for checking the reward function when a large unrealistic value
        # is present in the data. In this case there should be no disturbance as it is likely
        # a measurment error.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, -0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 10.0, 0.0]])

        reward = self.env._reward_function(action, info)
        self.assertAlmostEqual(round(reward, 2), 0.0)

        # Test 7 is for checking if the interaction direction is correctly calculated.
        # By comparing two similar scenarios.

        # No collision predicted using vector
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, -0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.8, 0.0, 0.4, 0.0]])

        reward1 = self.env._reward_function(action, info)

        # Likely will be collision.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.6, 0.0, -0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.8, 0.0, 0.4, -0.2]])

        reward2 = self.env._reward_function(action, info)

        self.assertGreater(reward1, reward2)

        # Test 8 is for checking if the interaction direction is correctly calculated.
        # By comparing two similar scenarios.

        # No collision predicted using vector
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.8, 0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[1.6, 0.0, 0.0, -0.4, 0.0]])

        reward1 = self.env._reward_function(action, info)

        # Likely will be collision.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.8, 0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[1.6, 0.0, 0.0, -0.4, 0.2]])

        reward2 = self.env._reward_function(action, info)

        self.assertGreater(reward1, reward2)

        # Test 9 is for checking the reward when there are multiple agents in the environment.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.8, 0.22, 0.0], [0.0, 1.1, 0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[1.6, 0.0, 0.0, -0.4, 0.0]])

        reward = self.env._reward_function(action, info)

        self.assertAlmostEqual(round(reward, 2), -0.78)

    # Terminal condition check of reward function
    def test_terminal_condition(self):

        action = np.array([0.0, 0.0])
        self.env.reward_functions = [
            'agent_velocity_disturbance',
            'social_force_sfm',
            'proxemics',

        ]
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.0, 0.8, 0.22, 0.0]])
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[1.6, 0.0, 0.0, -0.4, 0.0]])
        self.env.last_distance_to_goal = 0.8
        self.env.rl_io_manager.last_plan_length = 0.8

        # Test 1 outside interaction range for the distance based rewards.
        info = {'in_interaction_range': False, 'done_reason': []}
        reward = self.env._reward_function(action, info)

        self.assertEqual(reward, 0.0)

        # Test 2 collision with obstacle. Also test if the priority of the current_collision works
        # with the other conditions also active.
        self.env.reward_functions = ['goal_reached',
                                     'outside_interaction_range',
                                     'max_timesteps_reached']
        info = {'in_interaction_range': True, 'done_reason': [
            'current_collision', 'at_goal', 'future_collision',
            'outside_interaction_range', 'max_timesteps_reached']}
        reward = self.env._reward_function(action, info)
        self.assertEqual(reward, -3.0)

        # Test 3 at goal condition. Also test if the priority works
        # with the other conditions also active.
        info = {'in_interaction_range': True, 'done_reason': [
            'at_goal', 'future_collision',
            'outside_interaction_range', 'max_timesteps_reached']}
        reward = self.env._reward_function(action, info)
        self.assertEqual(reward, 1.0)

        # Test 4 future collision. Also test if the priority works
        # with the other conditions also active.
        info = {'in_interaction_range': True, 'done_reason': [
            'future_collision',
            'outside_interaction_range', 'max_timesteps_reached']}
        reward = self.env._reward_function(action, info)
        self.assertEqual(reward, -3.0)

        # Test 5 outside interaction range. Also test if the priority works
        # with the other conditions also active.
        info = {'in_interaction_range': False, 'done_reason': [
            'outside_interaction_range', 'max_timesteps_reached']}

        self.env.starting_plan_length = 3.0
        self.env.rl_io_manager.last_plan_length = 0.8

        reward = self.env._reward_function(action, info)

        # Did not move enough towards the goal
        self.env.starting_plan_length = 3.0
        self.env.rl_io_manager.last_plan_length = 1.1

        reward = self.env._reward_function(action, info)
        self.assertEqual(reward, -1.0)

        # Test 6
        info = {'in_interaction_range': True, 'done_reason': [
            'max_timesteps_reached']}
        reward = self.env._reward_function(action, info)

        self.assertEqual(reward, -1.0)

    def test_is_agent_in_interaction_range(self):
        """Test the __any_agent_in_interaction_range function."""
        self.env.interaction_range = 2.0  # Set interaction range
        self.env.critical_interaction_range = 1.0  # Set critical interaction range
        # Assume immediate interaction.
        self.env.timesteps_before_interaction = 0
        self.env.timesteps_before_switch_to_nav2 = 0

        # Test 1: Agent is within interaction range but not moving
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.0, 1.0, 0.0, 0.0]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertFalse(result)

        # Test 2: Agent is outside interaction range
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[3.0, 3.0, 0.0, 0.0]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertFalse(result)

        # Test 3: No agents in the environment
        self.env.rl_io_manager.last_agents_global_frame = None
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertTrue(result)

        # Test 4: Agent is in critical interaction range
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[0.5, 0.5, 0.0, 0.0]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertTrue(result)

        # Test 5: Agent is outside critical range but moving towards robot.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.0, 1.0, -0.2, -0.2]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.5, 0.5]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertTrue(result)

        # Test 6: Agent is in line of sight but not moving towards the robot
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.0, 1.0, 0.5, 0.5]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, -0.5, -0.5]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertFalse(result)

        # Test 7: see if steps before switch to NAV2 works as expected.
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[1.0, 1.0, -0.2, -0.2]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.5, 0.5]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertTrue(result)

        # Test 8: Check override if timesteps is smaller than timesteps_before_interaction
        self.env.timesteps_before_interaction = 10
        self.env.rl_io_manager.last_agents_global_frame = np.array(
            [[10.5, 10.5, 0.0, 0.0]])  # Agent position and velocity
        self.env.rl_io_manager.last_robot_odom = np.array(
            [[0.0, 0.0, 0.0, 0.0, 0.0]])  # Robot position and velocity
        result = self.env._GazeboEnv__any_agent_in_interaction_range()
        self.assertTrue(result)

    def test_lidar_collision_check(self):
        """Test the __lidar_collision_check function."""
        self.env.robot_radius_sqr = 0.5  # Robot radius squared
        self.env.collision_footprint_factor = 1.42

        threshold = np.sqrt(self.env.robot_radius_sqr) * \
            self.env.collision_footprint_factor

        # Test 1: Collision detected (point inside the threshold)
        self.env.rl_io_manager.last_lidar = np.array([[0.1, 0.1]])
        result = self.env._GazeboEnv__lidar_collision_check()
        self.assertTrue(result)

        # Test 2: No collision (all points outside the threshold)
        self.env.rl_io_manager.last_lidar = np.array([[2.0, 2.0], [3.0, 0.1]])
        result = self.env._GazeboEnv__lidar_collision_check()
        self.assertFalse(result)

        # Test 3: Collision detected (point exactly on the threshold)
        self.env.rl_io_manager.last_lidar = np.array(
            [[threshold / np.sqrt(2), threshold / np.sqrt(2)]])
        result = self.env._GazeboEnv__lidar_collision_check()
        self.assertTrue(result)

        # Test 4: Empty lidar points (no collision)
        self.env.rl_io_manager.last_lidar = np.array([])
        result = self.env._GazeboEnv__lidar_collision_check()
        self.assertFalse(result)

    def test_all_agents_outside_range(self):
        """Test the _all_agents_outside_range function."""
        self.env.rl_io_manager.last_agents = np.array([
            [OUTSIDE_RANGE_LOC, OUTSIDE_RANGE_LOC,
                OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_VELOCITY],
            [OUTSIDE_RANGE_LOC, OUTSIDE_RANGE_LOC,
                OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_VELOCITY]
        ])
        self.env.timesteps_before_agents_in_range = 0
        self.env.buffer_agents_in_range = 0

        # Test 1: All agents are outside the range
        result = self.env._all_agents_outside_range()
        self.assertTrue(result)

        # Test 2: Some agents are within the range.
        self.env.rl_io_manager.last_agents = np.array([
            [OUTSIDE_RANGE_LOC, OUTSIDE_RANGE_LOC,
                OUTSIDE_RANGE_VELOCITY, OUTSIDE_RANGE_VELOCITY],
            [1.0, 1.0, 0.5, 0.5]
        ])
        result = self.env._all_agents_outside_range()
        self.assertFalse(result)

        # Test 3: Buffer exceeds timesteps_before_agents_in_range
        self.env.rl_io_manager.last_agents = np.array([
            [1.0, 1.0, 0.5, 0.5],
            [1.5, 1.5, 0.2, 0.2]
        ])

        self.env.timesteps_before_agents_in_range = 3
        self.env.buffer_agents_in_range = 3
        result = self.env._all_agents_outside_range()
        self.assertFalse(result)
        self.assertEqual(self.env.buffer_agents_in_range, 0)

    # def test_is_done(self):


if __name__ == '__main__':
    unittest.main()
