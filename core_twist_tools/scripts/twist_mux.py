#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node


class TwistMux(Node):
    def __init__(self):
        """
        Initialization
        """
        super().__init__(
            node_name="twist_mux",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.topics = None
        self.locks = None
        self.locks_time = {}
        self.topics_time = {}
        self.topic_priority = 0.0
        self.lock_priority = 255
        self.last_topic_time = 0.0
        self.last_topic_timeout = 0.0

        # If reinforcement learning is used, pause motion should pause immediately. For normal running mode, pause motion should be False
        self.use_rl = self.get_parameter('use_rl').value
        self.pause_motion = True if self.use_rl else False

        # Create a twist publisher for cmd_vel
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        # Get topics parameters
        self.topics_parameters = self.get_parameters_by_prefix('topics')
        self.topics = {}
        for key, value in self.topics_parameters.items():
            key_elements = key.split('.')
            topic_temp = self.topics.setdefault(key_elements[0], {})
            topic_temp[key_elements[1]] = value

        for key, value in self.topics.items():
            # Topic time initialization
            self.topics_time[key] = self.get_clock().now().nanoseconds/1e9
            # Create and register joy topic subscriber with the node
            self.create_subscription(
                Twist,
                value['topic'].value,
                lambda msg, topic=key: self.callback_topic(msg, topic),
                1
            )

        self.locks_parameters = self.get_parameters_by_prefix('locks')
        self.locks = {}
        for key, value in self.locks_parameters.items():
            key_elements = key.split('.')
            topic_temp = self.locks.setdefault(key_elements[0], {})
            topic_temp[key_elements[1]] = value

        for key, value in self.locks.items():
            # Lock time initialization
            self.locks_time[key] = self.get_clock().now().nanoseconds/1e9
            # Create and register joy topic subscriber with the node
            self.create_subscription(
                Bool,
                value['topic'].value,
                lambda msg, lock=key: self.callback_lock(msg, lock),
                1
            )

        # Get rate parameter for zero twist
        try:
            self.declare_parameter('rate', 10.0)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.rate = self.get_parameter('rate').value
        self.cmd_vel_zero = Twist()
        self.timer = self.create_timer(1/self.rate, self.timer_callback)

    def callback_topic(self, data, topic):
        """
        Callback for pose mux topics
        """
        # Determine maximum active lock priority
        self.lock_priority = 0.0
        for key, value in self.locks.items():
            if getattr(value.get('toggle', None), 'value', None):
                if self.pause_motion:
                    self.lock_priority = max(
                        self.lock_priority, value['priority'].value)
                else:
                    self.lock_priority = 0.0

            elif not getattr(value.get('toggle', None), 'value', None) and (self.get_clock().now().nanoseconds/1e9 - self.locks_time[key]) > value['timeout'].value:
                self.lock_priority = max(
                    self.lock_priority, value['priority'].value)
                self.get_logger().error(f"Lock {key} timeout")

        # Determine maximum active topic priority
        self.topic_priority = self.lock_priority
        for key, value in self.topics.items():
            if (self.get_clock().now().nanoseconds/1e9 - self.topics_time[key]) < self.topics[key]['timeout'].value:
                self.topic_priority = max(
                    self.topic_priority, self.topics[key]['priority'].value)

        # Publish topic message if it satisfies the maximum topic priority
        if self.topics[topic]['priority'].value >= self.topic_priority:
            self.topics_time[topic] = self.get_clock().now().nanoseconds/1e9
            self.twist_pub.publish(data)
            self.last_topic_time = self.topics_time[topic]
            self.last_topic_timeout = self.topics[topic]['timeout'].value

    def callback_lock(self, data, lock):
        """
        Callback for pose mux locks
        """
        self.pause_motion = data.data
        self.locks_time[lock] = self.get_clock().now().nanoseconds/1e9

    def timer_callback(self):
        # 1e-9 to convert nanoseconds -> seconds
        if (self.get_clock().now().nanoseconds/1e9 - self.last_topic_time) > self.last_topic_timeout:
            self.twist_pub.publish(self.cmd_vel_zero)
        if not rclpy.ok():
            self.timer.cancel()


def main(args=None):
    """
    Start a twist mux node
    """
    rclpy.init(args=args)
    twist_mux = TwistMux()
    rclpy.spin(twist_mux)
    twist_mux.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
