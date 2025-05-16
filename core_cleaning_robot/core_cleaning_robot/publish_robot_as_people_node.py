#! /usr/bin/env python3

import rclpy
import math
import random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point
from transforms3d.euler import quat2euler
from rclpy.exceptions import ROSInterruptException


class PeoplePublisher(Node):
    def __init__(self):
        super().__init__('people_publisher', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.publisher = self.create_publisher(People, '/people', 10)
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'odometry',
            self.odometry_callback,
            10
        )

        self.callback_count = 0
        self.avg_velocity = 0.0
        self.avg_yaw = 0.0
        self.alpha = self.get_parameter('smoothing_alpha').value
        self.average_velocity = self.get_parameter('average_velocity').value
        self.noise = self.get_parameter('sensor_noise').value

    def velocity_average(self, velocity, yaw):
        self.avg_velocity = self.alpha * velocity + \
            (1 - self.alpha) * self.avg_velocity
        self.avg_yaw = self.alpha * yaw + (1 - self.alpha) * self.avg_yaw
        return self.avg_velocity, self.avg_yaw

    def odometry_callback(self, msg):

        msg_to_pub = People()
        msg_to_pub.header.stamp = self.get_clock().now().to_msg()
        msg_to_pub.header.frame_id = msg.header.frame_id
        person = Person()

        # Set the name of the person
        person.name = self.get_namespace()

        # Set the position of the person
        person.position = Point()
        person.position.x = msg.pose.pose.position.x + \
            random.uniform(-self.noise, self.noise)
        person.position.y = msg.pose.pose.position.y + \
            random.uniform(-self.noise, self.noise)
        person.position.z = msg.pose.pose.position.z + \
            random.uniform(-self.noise, self.noise)

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = quat2euler(
            [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])

        speed = abs(msg.twist.twist.linear.x)

        if self.average_velocity:
            speed, yaw = self.velocity_average(speed, yaw)

        # Set the velocity of the person
        person.velocity = Point()
        person.velocity.x = speed * \
            math.cos(yaw) + random.uniform(-self.noise, self.noise)
        person.velocity.y = speed * \
            math.sin(yaw) + random.uniform(-self.noise, self.noise)
        person.velocity.z = 0.0  # z velocity is not used
        person.tagnames = []
        person.reliability = 1.0

        person.tags = []

        # Populate tagnames and tags
        person.tagnames.append('id')
        person.tagnames.append('group id')
        person.tags.append('0')
        person.tags.append('-1')

        # Add the person to the people message
        msg_to_pub.people.append(person)

        # Publish the people message
        self.publisher.publish(msg_to_pub)


def main(args=None):
    try:
        rclpy.init(args=args)
        people_publisher = PeoplePublisher()
        rclpy.spin(people_publisher)
    except ROSInterruptException as e:
        people_publisher.get_logger().warn(
            f"ROS interrupt exception caught: {e}")
    except Exception as e:
        people_publisher.get_logger().warn(f"General exception caught: {e}")
    finally:
        people_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
