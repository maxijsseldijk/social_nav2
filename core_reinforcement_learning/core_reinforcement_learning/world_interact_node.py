#!/usr/bin/env python3
import os
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from ros_gz_interfaces.msg import Entity
import numpy as np
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from train_rl import RLsimulation


class PauseSimulation:
    """A class to manage the pausing and unpausing of a Gazebo simulation."""

    def __init__(self, node: 'RLsimulation', world: str = 'default'):
        """
        Initialize the PauseSimulation class.

        Args:
        ----
            node (RLsimulation): The ROS node for the environment.
            world (str): The name of the world to control. Defaults to 'default'.

        """
        self.node = node
        self.node.log_error("Pause simulation node initialized")
        self.pause = self.node.create_client(
            ControlWorld, f"/world/{world}/control")
        self.critical_pause = False
        self.req = ControlWorld.Request()

    def change_pause_simulation(self, pause: bool):
        """
        Change the pause state of the simulation.

        Args:
        ----
            pause (bool): True to pause the simulation, False to unpause.

        """
        if self.critical_pause is True:
            pause = self.critical_pause
        else:
            pause = pause

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Pause sim service not available, waiting...')
        try:
            self.req.world_control.pause = pause
            self.pause.call_async(self.req)
        except Exception as e:
            self.node.log_error(f"/pause_physics service call failed: {e}")

    def set_gym_pause(self, pause: bool):
        """
        Set a critical pause for the simulation.

        Args:
        ----
            pause (bool): True to set a critical pause, False to unset.

        """
        self.critical_pause = pause
        if self.critical_pause is True:
            self.change_pause_simulation(self.critical_pause)

    def get_pause_condition(self):
        """
        Get the current pause condition of the simulation.

        Returns
        -------
            bool: True if the simulation is paused, False otherwise.

        """
        return self.req.world_control.pause


class ResetSimulation:
    """A class to manage the resetting of the simulation."""

    def __init__(self, node: 'RLsimulation', robot_names: list, agent_names: list,
                 TaskGenerator: dict, world: str = 'default', rng: np.random.Generator = None):
        """
        Initialize the ResetSimulation class.

        Args:
        ----
            node (RLsimulation): The ROS node for the environment.
            robot_names (list): The list of robot names to reset.
            agent_names (list): The list of agent names to reset.
            TaskGenerator (dict): The task generator parameters.
            world (str): The name of the world to control. Defaults to 'default'.
            rng (np.random.Generator): The random number generator. Defaults to None.

        """
        self.node = node
        self.rng = rng
        self.node.log_error("Reset simulation node initialized")

        self.reset = self.node.create_client(
            SetEntityPose, f"/world/{world}/set_pose")
        self.req = SetEntityPose.Request()
        self.robot_names = robot_names
        self.agent_names = agent_names
        self.TaskGenerator = TaskGenerator

    def get_value(self, param):
        """
        Get the value of a parameter.

        Args:
        ----
            param: The parameter to get the value of.

        Returns
        -------
            list: The value of the parameter as a list.

        Raises
        ------
            ValueError: If the parameter type is unsupported.

        """
        if isinstance(param, list):
            return param
        elif isinstance(param, str):

            return [float(param)]
        elif isinstance(param, float):
            return [param]
        else:
            raise ValueError(f"Unsupported parameter type: {type(param)}")

    def request_reset_entity(self, entity_name: str, task: str):
        """
        Request to reset the pose of an entity.

        Args:
        ----
            entity_name (str): The name of the entity to reset possibly namespaced.
            task (str): The task to reset the entity for.

        """
        try:
            entity = Entity()
            entity.name = entity_name.strip('/')
            self.req.entity = entity

            entity_basename = os.path.basename(os.path.normpath(entity_name))

            # Extract limits and generate random position values
            x_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.position.x_pose'].value)
            y_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.position.y_pose'].value)
            z_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.position.z_pose'].value)

            self.req.pose.position.x = self.rng.uniform(
                x_limits[0], x_limits[1]) if len(x_limits) == 2 else x_limits[0]
            self.req.pose.position.y = self.rng.uniform(
                y_limits[0], y_limits[1]) if len(y_limits) == 2 else y_limits[0]
            self.req.pose.position.z = self.rng.uniform(
                z_limits[0], z_limits[1]) if len(z_limits) == 2 else z_limits[0]

            # Extract limits and generate random orientation values
            ox_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.orientation.x'].value)
            oy_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.orientation.y'].value)
            oz_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.orientation.z'].value)
            ow_limits = self.get_value(
                self.TaskGenerator[f'{task}.{entity_basename}.orientation.w'].value)

            self.req.pose.orientation.x = self.rng.uniform(
                ox_limits[0], ox_limits[1]) if len(ox_limits) == 2 else ox_limits[0]
            self.req.pose.orientation.y = self.rng.uniform(
                oy_limits[0], oy_limits[1]) if len(oy_limits) == 2 else oy_limits[0]
            self.req.pose.orientation.z = self.rng.uniform(
                oz_limits[0], oz_limits[1]) if len(oz_limits) == 2 else oz_limits[0]
            self.req.pose.orientation.w = self.rng.uniform(
                ow_limits[0], ow_limits[1]) if len(ow_limits) == 2 else ow_limits[0]

            self.reset.call_async(self.req)
        except Exception as e:
            self.node.log_error(f"/reset_simulation service call failed: {e}")

    def reset_simulation(self, task: str):
        """
        Reset the simulation for a given task.

        Args:
        ----
            task (str): The task to reset the simulation for.

        """
        while not self.reset.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Reset sim service not available, waiting...')

        for robot_name in self.robot_names:
            self.node.log_error(f" Resetting: {robot_name} to initial pose")

            # Add namespace if needed
            ns_robot = os.path.join(self.node.get_namespace(), robot_name)
            self.request_reset_entity(ns_robot, task)

        for agent_name in self.agent_names:
            self.node.log_error(f"Resetting: {agent_name} to initial pose ")
            # Add namespace if needed
            ns_agent = os.path.join(self.node.get_namespace(), agent_name)
            self.request_reset_entity(ns_agent, task)
