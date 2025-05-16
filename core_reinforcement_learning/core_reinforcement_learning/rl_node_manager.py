#!/usr/bin/env python3
import numpy as np
import os
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped, PointStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from people_msgs.msg import People
from std_msgs.msg import Bool as BoolMsg, Header
from core_custom_messages.msg import PointArray, Float32Array, PathWithLength
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from core_reinforcement_learning.utils import convert_twist_to_vector3
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from train_rl import RLsimulation


class RlSubscriptionManager():
    """
    A ROS2 node that subscribes to user-defined state and utility nodes.

    It concatenates the state vector and publishes it to the last_rl_state_vector topic.
    It also republishes the utility data to the "last_{utility_data}" topic for easy integration
    in the RL training script.
    A user-defined list of state and utility nodes can be defined in the main_params.yaml file.

    """

    def __init__(self, node: 'RLsimulation'):
        self.node = node
        self.node.log_info("RlSubscriptionManager node initialized")
        self.name = "rl_node_manager"
        self.callback_frequency = self.node.get_parameter(
            f"{self.name}.callback_frequency").value
        state_node_list = self.node.get_parameter(
            f"{self.name}.state_nodes").value
        utility_node_list = self.node.get_parameter(
            f"{self.name}.utility_nodes").value
        self.node.log_error(f"State nodes: {state_node_list}")
        self.state_callback_names = self.create_param_vector(state_node_list)
        self.utility_callback_names = self.create_param_vector(
            utility_node_list)

        self.all_nodes = state_node_list + utility_node_list
        self.all_nodes_callback_names = self.state_callback_names + \
            self.utility_callback_names
        self.sub_list = {}
        self.nodes_dict = {}
        self.create_subscribers(self.all_nodes)

    def create_param_vector(self, state_node_list):
        """
        Create a list of parameter names for the given state nodes.

        Args:
        ----
            state_node_list (list): List of state node names.

        Returns
        -------
            list: List of parameter names.

        """
        names = []
        for state in state_node_list:
            self.create_class_parameter(f"last_{state}", None)
            names.append(f"last_{state}")
        return names

    def create_class_parameter(self, name, value):
        """
        Create a class parameter with the given name and value.

        Args:
        ----
            name (str): Name of the parameter.
            value: Value of the parameter.

        """
        setattr(self, name, value)

    def create_subscribers(self, node_list):
        """
        Create subscribers for the given list of nodes.

        Args:
        ----
            node_list (list): List of node names.

        """
        for node in node_list:
            if self.node.get_parameter(f"{self.name}.{node}.topic").value is False:
                continue
            self.nodes_dict[node] = {
                "type": self.type_from_string(
                    self.node.get_parameter(f"{self.name}.{node}.type").value
                ),
                "topic": self.node.get_parameter(
                    f"{self.name}.{node}.topic"
                ).value,
                "callback": self.callback_from_string(
                    self.node.get_parameter(
                        f"{self.name}.{node}.callback").value
                )
            }
            node_type = self.nodes_dict[node]["type"]
            node_topic = self.nodes_dict[node]["topic"]
            node_callback = self.nodes_dict[node]["callback"]

            if not node_topic.startswith("/"):
                node_topic = os.path.join(
                    self.node.get_namespace(), node_topic)

            else:
                node_topic = f"{node_topic}"

            self.sub_list[node] = self.node.create_subscription(
                node_type, node_topic, node_callback, 1,
                callback_group=MutuallyExclusiveCallbackGroup())

    def get_state_size(self) -> int:
        """
        Calculate the size of the state vector.

        Returns
        -------
            int: state_dim size.

        """
        state_dim = 0
        for state in self.state_callback_names:
            state_dim += len(getattr(self, state).flatten())
        return state_dim

    def get_state_name_size_dict(self):
        """
        Get the dictionary of state names and their sizes.

        Returns
        -------
            dict: Dictionary of state names and their sizes.

        """
        state_dim = {}
        for state in self.state_callback_names:
            state_dim[state] = len(getattr(self, state).flatten())
        return state_dim

    def get_state(self):
        """
        Get the state vector.

        Returns
        -------
            np.array: The state vector.

        """
        state_vector = np.empty((0,))
        for state in self.state_callback_names:
            state_array = getattr(self, state).reshape(1, -1)
            if state_vector.size == 0:
                state_vector = state_array
            else:
                state_vector = np.concatenate(
                    (state_vector, state_array), axis=1, dtype=np.float32)
        return state_vector.flatten()

    def get_state_names(self):
        """
        Get the state names.

        Returns
        -------
            list: List of state names.

        """
        return np.array(self.state_callback_names).flatten()

    def check_if_all_in_state_list(self, state_list: list[str]):
        """
        Check if the given state list is in the state callback names.

        Args:
        ----
            state_list (list): List of state names.

        Returns
        -------
            bool: True if the state list is in the state callback names, otherwise False.

        """
        return all(state in self.state_callback_names for state in state_list)

    def get_utility_data(self):
        """
        Get the utility data.

        Returns
        -------
            dict: Dictionary of utility data.

        """
        utility_data = {}
        for utility in self.utility_callback_names:
            utility_data[utility] = getattr(self, utility)
        return utility_data

    def check_if_all_in_utility_list(self, utility_list: list[str]):
        """
        Check if the given utility list is in the utility callback names.

        Args:
        ----
            utility_list (list): List of utility names.

        Returns
        -------
            bool: True if the utility list is in the utility callback names, otherwise False.

        """
        return all(utility in self.utility_callback_names for utility in utility_list)

    def check_if_in_all_nodes_list(self, requested_nodes: list[str]):
        """
        Check if the given requested nodes list is in the all nodes callback names.

        Args:
        ----
            requested_nodes (list): List of requested nodes names.

        Returns
        -------
            bool: True if the requested nodes list is in the all nodes callback names.

        """
        return all(node in self.all_nodes_callback_names for node in requested_nodes)

    def lookup_param_state(self, state: str):
        """
        Lookup the state parameter value.

        Args:
        ----
            state (str): Name of the state parameter.

        Returns
        -------
            The value of the state parameter if it exists, otherwise None.

        """
        if hasattr(self, state):
            self.node.log_info(f"The state variable '{state}' exists.")
            return getattr(self, state)
        else:
            self.node.get_logger().warning(
                f"The state variable '{state}' does not exist.")
            return None

    def wait_till_all_data_received(self):
        """
        Get the callback function from the callback string.

        Args:
        ----
            callback_string (str): Name of the callback function.

        Returns
        -------
            function: The callback function.

        Raises
        ------
            ValueError: If the callback function is not recognized.

        """
        while True:
            missing_data = [
                state for state in self.state_callback_names
                if self.lookup_param_state(state) is None]
            missing_data += [
                utility for utility in self.utility_callback_names
                if self.lookup_param_state(utility) is None]

            if not missing_data:
                break
            else:
                self.node.log_error(f"Waiting for: {', '.join(missing_data)}")
                time.sleep(1)

    def callback_from_string(self, callback_string: str):
        """
        Get the callback function from the callback string.

        Args:
        ----
            callback_string (str): Name of the callback function.

        Returns
        -------
            function: The callback function.

        Raises
        ------
            ValueError: If the callback function is not recognized.

        """
        callbacks = {
            "costmap_callback": self.costmap_callback,
            "footprint_callback": self.footprint_callback,
            "odometry_callback": self.odometry_callback,
            "nav2_input_callback": self.nav2_input_callback,
            "plan_with_length_callback": self.plan_with_length_callback,
            "lidar_point_array_callback": self.lidar_point_array_callback,
            "lidar_raw_callback": self.lidar_raw_callback,
            "agents_callback": self.agents_callback,
            "agents_callback_global": self.agents_callback_global,
            "sfm_control_point_callback": self.sfm_control_point_callback,
        }
        if callback_string in callbacks:
            return callbacks[callback_string]
        else:
            raise ValueError(
                f"Callback {callback_string} not recognized. Ensure all callbacks are correct.")

    def type_from_string(self, type_string: str):
        """
        Get the message type from the type string.

        Args:
        ----
            type_string (str): Name of the message type.

        Returns
        -------
            type: The message type.

        Raises
        ------
            ValueError: If the message type is not recognized.

        """
        types = {
            "Twist": Twist,
            "PoseStamped": PoseStamped,
            "PolygonStamped": PolygonStamped,
            "Odometry": Odometry,
            "Path": Path,
            "OccupancyGrid": OccupancyGrid,
            "People": People,
            "LaserScan": LaserScan,
            "Bool": BoolMsg,
            "PointArray": PointArray,
            "Float32Array": Float32Array,
            "PathWithLength": PathWithLength,
            "Header": Header,
            "PointStamped": PointStamped
        }
        if type_string in types:
            return types[type_string]
        else:
            raise ValueError(
                f"Type {type_string} not recognized. Ensure all types are correct.")

    def costmap_callback(self, costmap: OccupancyGrid):
        """
        Process the costmap data.

        Args:
        ----
            costmap (OccupancyGrid): Costmap data.

        """
        self.last_costmap = costmap

    def footprint_callback(self, footprint: PolygonStamped):
        """
        Process footprint data.

        Args:
        ----
            footprint (PolygonStamped): Footprint data.

        """
        self.last_footprint = footprint

    def odometry_callback(self, od_data: Odometry):
        """
        Process odometry data.

        Args:
        ----
            od_data (Odometry): Odometry data.

        """
        _, _, yaw_data = quat2euler([od_data.pose.pose.orientation.w,
                                     od_data.pose.pose.orientation.x,
                                     od_data.pose.pose.orientation.y,
                                     od_data.pose.pose.orientation.z])
        velocity_global = convert_twist_to_vector3(od_data)
        self.last_robot_odom = np.array([od_data.pose.pose.position.x,
                                         od_data.pose.pose.position.y,
                                         yaw_data, velocity_global.vector.x,
                                         velocity_global.vector.y]).reshape(1, -1)

    def nav2_input_callback(self, nav2_input: Twist):
        """
        Process nav2 twist data.

        Args:
        ----
            nav2_input (Twist): Navigation input data.

        """
        self.last_nav2_input = np.array(
            [nav2_input.linear.x, nav2_input.angular.z]).reshape(1, -1)

    def plan_with_length_callback(self, plan_data: PathWithLength):
        """
        Process plan with length data.

        Args:
        ----
            plan_data (PathWithLength): Plan with length data.

        """
        if not hasattr(self, "last_plan_header") and not hasattr(self, "last_plan_length"):
            self.last_plan_header = None
            self.last_plan_length = None
        self.last_plan_header = plan_data.header
        self.last_plan_length = plan_data.path_length
        plan_2d = [[pose.pose.position.x, pose.pose.position.y]
                   for pose in plan_data.poses]
        self.last_plan = np.array(plan_2d).reshape(1, -1)

    def lidar_point_array_callback(self, lidar_data: PointArray):
        """
        Process lidar data as a point array.

        Args:
        ----
            lidar_data (PointArray): Lidar point array data.

        """
        points_xy = [[point.x, point.y] for point in lidar_data.points]
        self.last_lidar = np.array(points_xy).reshape(1, -1)

    def lidar_raw_callback(self, lidar_data: LaserScan):
        """
        Process lidardata as raw LaserScan data.

        Args:
        ----
            lidar_data (LaserScan): Lidar raw data.

        """
        self.last_lidar_raw = np.array(lidar_data.ranges).reshape(1, -1)

    def agents_callback(self, od_data: People):
        """
        Process people data.

        Args:
        ----
            od_data (People): Agents data.

        """
        closest_agent = [agent.position.x**2 +
                         agent.position.y**2 for agent in od_data.people]
        closest_agent_index = np.argmin(closest_agent)
        closest_agent_position = od_data.people[closest_agent_index]
        self.last_agents = np.array([closest_agent_position.position.x,
                                     closest_agent_position.position.y,
                                     closest_agent_position.velocity.x,
                                     closest_agent_position.velocity.y]).reshape(1, -1)

    def agents_callback_global(self, od_data: People):
        """
        Process global agents data.

        Args:
        ----
            od_data (People): Agents data.

        """
        odom_placeholder = [[person.position.x, person.position.y,
                             person.velocity.x, person.velocity.y] for person in od_data.people]
        self.last_agents_global_frame = np.array(odom_placeholder)

    def sfm_control_point_callback(self, control_point_data: Path):
        """
        Process sfm control point data.

        Args:
        ----
            control_point_data (Path): Sfm control point data.

        """
        last_timestamp = 0.0
        # Check if control point data is older then timestep
        if not hasattr(self, "last_sfm_control_point"):
            last_timestamp = control_point_data.header.stamp
            control_point_xy = [[pose.pose.position.x, pose.pose.position.y]
                                for pose in control_point_data.poses]
            self.last_sfm_control_point = np.array(
                control_point_xy).reshape(1, -1)
        elif self.node.get_clock().now().to_msg().sec - last_timestamp >= 1:
            control_point_xy = [[pose.pose.position.x, pose.pose.position.y]
                                for pose in control_point_data.poses]
            self.last_sfm_control_point = np.array(
                control_point_xy).reshape(1, -1)
        else:
            pass
