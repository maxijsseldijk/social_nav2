#!/usr/bin/env python3

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node




class Twist_Teleop_Joy(Node):
    def __init__(self):
        """
        Initialization
        """
        super().__init__("teleop_twist_joy")
        # Initialize publisher and subscriber
        self.pause_motion = True
        self.cont_ctrl = False # Continuous Command Publication
        self.twist_cmd = Twist()

    def callback_joy(self, joy):
        """
        Callback for joypad updates received in the sensor_msgs/Joy format
        """

        self.pause_motion = not(bool(joy.buttons[self.pause_button]))
        self.cont_ctrl = joy.axes[self.cont_ctrl_axes] < 0.0

        if self.pause_motion:
            linear_speed = 0.0
            angular_speed = 0.0
        else:
            if abs(joy.axes[7]) > abs(joy.axes[1]):
                linear_speed = self.max_linear_speed*joy.axes[7]
            else:
                linear_speed = self.max_linear_speed*joy.axes[1]

            if abs(joy.axes[6]) > abs(joy.axes[0]):
                angular_speed = self.max_angular_speed*joy.axes[6]
            else:
                angular_speed = self.max_angular_speed*joy.axes[0]

        self.twist_cmd.linear.x = linear_speed
        self.twist_cmd.angular.z = angular_speed
        if not(self.cont_ctrl) and not(self.pause_motion):
            self.twist_pub.publish(self.twist_cmd)

    def publish_loop(self):
        if not(self.pause_motion):
            self.pause_cmd.data = self.pause_motion
            self.pause_pub.publish(self.pause_cmd)
        if not(self.pause_motion) and self.cont_ctrl:
            self.twist_pub.publish(self.twist_cmd)


    def start(self):
        """
    	Start the twist_teleop_key node
    	"""

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 1.0),
                ('max_angular_speed', 2.0),
                ('pause_button', 4),
                ('cont_ctrl_axes', 2),
                ('cmd_rate', 20.0),
            ]
        )

        # Create and register control command publisher with the master
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_cmd = Twist()

        # Create and register joy priority publisher with the master
        self.pause_pub = self.create_publisher(Bool, '/pause_motion', 10);
        self.pause_cmd = Bool()

        # Load parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value  # Maximum linear speed
        self.max_angular_speed = self.get_parameter('max_angular_speed').value  # Maximum angular speed
        self.pause_button = self.get_parameter('pause_button').value
        self.cont_ctrl_axes = self.get_parameter('cont_ctrl_axes').value
        cmd_rate = self.get_parameter('cmd_rate').value # Command Publication Rate Parameter

        subscriber = self.create_subscription(
            Joy,
            'joy',
            self.callback_joy,
            1
        )

        # Command publication loop
        self.timer = self.create_timer(1 / cmd_rate, self.publish_loop)



def main():
    rclpy.init()
    twist_teleop_joy = Twist_Teleop_Joy()
    twist_teleop_joy.start()

    rclpy.spin(twist_teleop_joy)

    # Destroy the node explicitly
    twist_teleop_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
