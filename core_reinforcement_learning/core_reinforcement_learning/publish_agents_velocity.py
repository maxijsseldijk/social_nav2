#!/usr/bin/env python3

import rclpy
import os
from rclpy.duration import Duration
import random
from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool as BoolMsg
from people_msgs.msg import People, Person
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_vector3
from core_reinforcement_learning.utils import convert_twist_to_vector3

OUTSIDE_RANGE_LOC = 6.0
OUTSIDE_RANGE_VELOCITY = 0.0


class PublishAgentsVelocity(Node):
    """
    A class that publishes the velocity of multiple agents.

    This class subscribes to the odometry messages of multiple agents
    and publishes their velocities after applying noise and transforming
    them to a specified coordinate frame.
    Additionally, it averages the velocity of each agent over time using
    an exponential smoothing filter. The class also listens to a topic
    that indicates whether the robot is in the interaction range of the agents.
    If the robot is outside of range, constant values are used for the people measurements.

    """

    def __init__(self):
        """Initialize the PublishAgentsVelocity node."""
        super().__init__('publish_agents_velocity', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Obtain data from the main parameter script
        self.agents = self.get_parameter(
            'agent_names').get_parameter_value().string_array_value

        # Get the high level namespace as not the full namespace is passed to this node
        # See the launch file for more details.
        self.top_ns = self.get_namespace()
        self.num_people = len(self.agents)
        self.buffer_size = self.get_parameter('buffer_size').value
        self.noise = self.get_parameter('sensor_noise').value
        self.callback_frequency = self.get_parameter(
            'callback_frequency').value
        robot_name = self.get_parameter('robot_names').value
        local_robot_frame = self.get_parameter('robot_frame').value
        self.average_velocity = self.get_parameter('average_velocity').value
        self.alpha = self.get_parameter('smoothing_alpha').value

        self.previous_velocities = {}

        if len(robot_name) == 1 and robot_name is not None:
            self.robot_frame = os.path.join(self.top_ns.strip(
                '/'), robot_name[0], local_robot_frame)
        else:
            self.get_logger().fatal("Only one robot name is supported for this node.")
            rclpy.shutdown()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1 / self.callback_frequency  # seconds

        self.received_messages = {}
        self.untransformed_messages = {}
        self.in_interaction_range = True

        self.create_subscription(
            BoolMsg, '/in_interaction_range', self.in_interaction_range_callback, 1)

        for i, agent in enumerate(self.agents):
            self.received_messages[i + 1] = []
            self.untransformed_messages[i + 1] = []
            self.create_subscription(
                Odometry,
                os.path.join(self.top_ns, agent, 'odometry'),
                lambda msg, agent_id=i +
                1: self.odometry_callback(msg, agent_id),
                1
            )

        self.people_msg = People()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher = self.create_publisher(People, '/agents', 1)
        self.publisher_untransformed = self.create_publisher(
            People, '/agents_global_frame', 1)

    def timer_callback(self):
        """Publish the velocities of agents at regular intervals."""
        if all(len(buffer) == self.buffer_size for buffer in self.received_messages.values()):
            self.people_msg.people = []
            self.people_msg.header.stamp = self.get_clock().now().to_msg()
            self.people_msg.header.frame_id = self.robot_frame
            list_message = list(self.received_messages.values())
            for buffer in list_message:
                for msg in buffer:
                    if msg[0] == 0 or msg[1] == 0:
                        # Skip the message if the transform is not available
                        continue
                    if not self.in_interaction_range:
                        msg_pose_x = OUTSIDE_RANGE_LOC
                        msg_pose_y = OUTSIDE_RANGE_LOC
                        msg_vel_x = OUTSIDE_RANGE_VELOCITY
                        msg_vel_y = OUTSIDE_RANGE_VELOCITY
                        noise = 0.0
                    else:
                        msg_pose_x = msg[0].pose.position.x
                        msg_pose_y = msg[0].pose.position.y
                        msg_vel_x = msg[1].vector.x
                        msg_vel_y = msg[1].vector.y
                        noise = self.noise
                    person = Person()
                    person.name = f'/agent{list_message.index(buffer) + 1}'
                    person.position = Point()
                    person.position.x = msg_pose_x + \
                        random.uniform(-noise, noise)
                    person.position.y = msg_pose_y + \
                        random.uniform(-noise, noise)
                    person.position.z = 0.0  # z position is not used

                    # Set the velocity of the person
                    person.velocity = Point()
                    person.velocity.x = msg_vel_x + \
                        random.uniform(-noise, noise)
                    person.velocity.y = msg_vel_y + \
                        random.uniform(-noise, noise)
                    person.velocity.z = 0.0  # z velocity is not used
                    person.reliability = 1.0

                    person.tagnames = []
                    person.tags = []

                    # Populate tagnames and tags
                    person.tagnames.append('id')
                    person.tagnames.append('group id')
                    person.tags.append(str(list_message.index(buffer) + 1))
                    person.tags.append('-1')

                    self.people_msg.people.append(person)
            self.publisher.publish(self.people_msg)

        if all(len(buffer) == self.buffer_size for buffer in self.untransformed_messages.values()):
            self.people_msg.people = []
            self.people_msg.header.stamp = self.get_clock().now().to_msg()
            list_message = list(self.untransformed_messages.values())
            # Set the frame id to that of the message which should be in odom frame
            self.people_msg.header.frame_id = list_message[0][0][0].header.frame_id
            for buffer in list_message:
                for msg in buffer:
                    person = Person()

                    person.name = f'/agent{list_message.index(buffer) + 1}'
                    person.position = Point()
                    person.position.x = msg[0].pose.position.x
                    person.position.y = msg[0].pose.position.y
                    person.position.z = 0.0
                    person.reliability = 1.0

                    # Set the velocity of the person
                    person.velocity = Point()
                    person.velocity.x = msg[1].vector.x
                    person.velocity.y = msg[1].vector.y
                    person.velocity.z = 0.0
                    person.tagnames = []
                    person.tags = []

                    # Populate tagnames and tags
                    person.tagnames.append('id')
                    person.tagnames.append('group id')
                    person.tags.append(str(list_message.index(buffer) + 1))
                    person.tags.append('-1')

                    self.people_msg.people.append(person)
            self.publisher_untransformed.publish(self.people_msg)

    def in_interaction_range_callback(self, msg: BoolMsg):
        """
        Process message to update the interaction range status.

        Args:
        ----
            msg (BoolMsg): Message indicating whether the robot is in interaction range.

        """
        self.in_interaction_range = msg.data

    def append_msg_to_buffer(self, buffer: dict,
                             msg: tuple[PoseStamped, Vector3Stamped], agent_id: int):
        """
        Append a message to the buffer for a specific agent.

        Args:
        ----
            buffer (dict): Buffer to store messages.
            msg (tuple[PoseStamped, Vector3Stamped]): Message to append.
            agent_id (int): ID of the agent.

        """
        if msg[0] is not None and msg[1] is not None:
            buffer[agent_id].append(msg)
            if len(buffer[agent_id]) > self.buffer_size:
                buffer[agent_id].pop(0)

    def odometry_callback(self, msg: Odometry, agent_id: int):
        """
        Append a message to the buffer for a specific agent.

        Args:
        ----
            buffer (dict): Buffer to store messages.
            msg (tuple[PoseStamped, Vector3Stamped]): Message to append.
            agent_id (int): ID of the agent.

        """
        msg_pose, msg_velocity = self.get_pose_and_velocity_vector(msg)

        # Only transform the pose as the velocity is not correctly calculated with moving frames
        transformed_pose, _ = self.change_coordinate_frame(
            self.robot_frame, msg)
        if self.average_velocity:
            msg_velocity = self.calculate_average_velocity(
                agent_id, msg_velocity)

        self.append_msg_to_buffer(
            self.received_messages, (transformed_pose, msg_velocity), agent_id)
        self.append_msg_to_buffer(
            self.untransformed_messages, (msg_pose, msg_velocity), agent_id)

    def calculate_average_velocity(self, agent_id: int,
                                   msg_velocity: Vector3Stamped) -> Vector3Stamped:
        """
        Calculate the average velocity for a specific agent using exponential smoothing.

        Args:
        ----
            agent_id (int): ID of the agent.
            msg_velocity (Vector3Stamped): Current velocity message.

        Returns
        -------
            Vector3Stamped: Smoothed velocity message.

        """
        if agent_id not in self.previous_velocities:
            self.previous_velocities[agent_id] = msg_velocity
        else:
            prev_velocity = self.previous_velocities[agent_id].vector
            self.previous_velocities[agent_id].vector.x = self.alpha * \
                msg_velocity.vector.x + (1 - self.alpha) * prev_velocity.x
            self.previous_velocities[agent_id].vector.y = self.alpha * \
                msg_velocity.vector.y + (1 - self.alpha) * prev_velocity.y

        return self.previous_velocities[agent_id]

    def get_pose_and_velocity_vector(self, msg: Odometry) -> tuple[PoseStamped, Vector3Stamped]:
        """
        Extract the pose and velocity vector from an odometry message.

        Args:
        ----
            msg (Odometry): Odometry message.

        Returns
        -------
            tuple[PoseStamped, Vector3Stamped]: Pose and velocity vector.

        """
        msg_pose = PoseStamped()
        msg_pose.header = msg.header
        msg_pose.pose = msg.pose.pose
        msg_velocity = convert_twist_to_vector3(msg)

        return msg_pose, msg_velocity

    def change_coordinate_frame(self, new_frame: str,
                                msg_to_transform: Odometry) -> list[PoseStamped, Vector3Stamped]:
        """
        Transform the pose and velocity vector to a new coordinate frame.

        Args:
        ----
            new_frame (str): Target coordinate frame.
            msg_to_transform (Odometry): Odometry message to transform.

        Returns
        -------
            list[PoseStamped, Vector3Stamped]: Transformed pose and velocity vector.

        """
        msg_pose, vel_vector = self.get_pose_and_velocity_vector(
            msg_to_transform)
        try:
            tfs = self.tf_buffer.lookup_transform_full(new_frame, rclpy.time.Time(),
                                                       msg_to_transform.header.frame_id,
                                                       rclpy.time.Time(), new_frame,
                                                       timeout=Duration(seconds=2.0))
            transformed_pose = do_transform_pose_stamped(msg_pose, tfs)
            transform_velocity = do_transform_vector3(vel_vector, tfs)

            return transformed_pose, transform_velocity

        except Exception as e:
            self.get_logger().error(f"Error in getting transform {e}")
            return None, None


def main(args=None):
    rclpy.init(args=args)
    people_publisher = None
    try:
        people_publisher = PublishAgentsVelocity()
        rclpy.spin(people_publisher)
    except ROSInterruptException as e:
        people_publisher.get_logger().warn(
            f"ROS interrupt exception caught: {e}")
    except Exception as e:
        people_publisher.get_logger().warn(f"General exception caught: {e}")
    finally:
        if people_publisher is not None:
            people_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
