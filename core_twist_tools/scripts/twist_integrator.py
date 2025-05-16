#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from transformations import euler_from_quaternion

class TwistIntegrator(Node):

    def __init__(self):
        """
        Initialization
        """

        super().__init__('twist_integrator')

        self.position = [0.0, 0.0, 0.0]           # Input Robot Position in 3D
        self.orientation = 0.0                    # Input Robot Orientation
        self.linear_velocity = [0.0, 0.0, 0.0]     # Input Linear Velocity
        self.angular_velocity = [0.0, 0.0, 0.0]     # Input Angular Velocity

        self.last_twist_input_time = 0.0

        # Create and register position publisher with the master
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 1)
        self.pose_msg = PoseStamped()

        # Create and register input pose topic subscriber with the master
        self.create_subscription(PoseStamped, 'pose', self.callback_pose, 1)


        # Create and register velocity topic subscriber with the master
        self.create_subscription(Twist, 'cmd_vel', self.callback_twist, 1)

        self.declare_parameter('rate', 10.0)
        self.rate = self.get_parameter('rate').value
        self.declare_parameter('decay_rate', 0.25)
        self.decay_rate = self.get_parameter('decay_rate').value

        self.timer = self.create_timer(1/self.rate, self.timer_callback)    

    def callback_pose(self, data):
        """
        Callback for pose updates receieved in the geometry_msgs/PoseStamped format
        """
        pose = data.pose
        self.position = [pose.position.x, pose.position.y, pose.position.z]
        quat_angles = [pose.orientation.x, pose.orientation.y, pose.orientation.z,
                       pose.orientation.w]
        self.orientation = euler_from_quaternion(quat_angles)


    def callback_twist(self, data):
        """
        Callback for velocity updates receieved in the geometry_msgs/Twist format
        """
        self.linear_velocity = [data.linear.x, data.linear.y, data.linear.z]
        self.angular_velocity = [data.angular.x, data.angular.y, data.angular.z]

        self.last_twist_input_time = self.get_clock().now().nanoseconds/1e9
        # Log/print input velocity
        self.get_logger().info(f"Input Linear Velocity: x = {self.linear_velocity[0]:.2f}, y = {self.linear_velocity[1]:.2f}, z = {self.linear_velocity[2]:.2f}")

    def timer_callback(self):
        """
        Node timer loop
        """
        if rclpy.ok():
            current_time = self.get_clock().now().nanoseconds/1e9
            elapsed_time = current_time - self.last_twist_input_time
            remain = (1.0-self.decay_rate)**(elapsed_time*self.rate)
            dt = 1.0/self.rate

            remaining_velocity_x = self.linear_velocity[0] * remain
            remaining_velocity_y = self.linear_velocity[1] * remain
            remaining_velocity_z = self.linear_velocity[2] * remain

            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.pose.position.x = self.position[0] + remaining_velocity_x*dt
            self.pose_msg.pose.position.y = self.position[1] + remaining_velocity_y*dt
            self.pose_msg.pose.position.z = self.position[2] + remaining_velocity_z*dt

            self.pose_pub.publish(self.pose_msg)
            

def main(args=None):
    """
    Start a twist integrator node
    """
    rclpy.init(args=args)
    twist_integrator = TwistIntegrator()
    rclpy.spin(twist_integrator)
    twist_integrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()