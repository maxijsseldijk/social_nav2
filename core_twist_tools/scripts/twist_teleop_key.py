#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import sys
import select
import termios
import tty
from threading import Thread
import rclpy
from rclpy.node import Node


class Twist_Teleop_Key(Node):
    def __init__(self):
        """
        Initialization
        """
        super().__init__("teleop_twist_key")
        self.linear_speeds = [0.0, 0.0, 0.0]  # [X vel, Y vel, Z vel]
        self.angular_speeds = [0.0, 0.0, 0.0]  # [Roll vel, Pitch vel, Yaw vel]
        self.cont_ctrl = True # Continuous Command Publication
        self.pause_motion = True
        self.mode = 'pause'
        self.ispressed = False
        self.exit = False
        self.keySettings = termios.tcgetattr(sys.stdin)

    def print_instruction(self):
        """
        Print Control Instructions
        """

        print('***********************************************************')
        print('Twist keyboard control to change linear speed and angular speed\n')
        print('     w      t y u  i   w/x : increase/descrease x linear speed')
        print('  a  s  d   g h j  k   a/d : increase/descrease y linear speed')
        print('     x                 i/k : increase/descrease z linear speed')
        print(' s : force stop        t/g : increase/decrease x angular speed')
        print(' p/m : pause/ move     y/h : increase/decrease y angular speed')
        print(' r : repeat            u/j : increase/decrease z angular speed')
        print('                       \n')
        print('Press <ctrl-c> or <q> to exit')
        print('***********************************************************')

    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.keySettings)
        return key 
    
    def key_input(self):
        self.print_instruction()
        print('Current Mode: {}, Linear Speed: {:.2f} {:.2f} {:.2f}, Angular Speed: {:.2f} {:.2f} {:.2f}'.format(self.mode, *self.linear_speeds, *self.angular_speeds))

        kInput = 0
        while rclpy.ok() and not self.exit:
            key = self.getkey()
            ischanged = False # Flag variable if the steering input is changed

            # -- Linear velocity --
            if key == 'w':
                self.linear_speeds[0] = self.linear_speeds[0] + self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[0] = min(self.linear_speeds[0], self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'x':
                self.linear_speeds[0] = self.linear_speeds[0] - self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[0] = max(self.linear_speeds[0], -self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'a':
                self.linear_speeds[1] = self.linear_speeds[1] + self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[1] = min(self.linear_speeds[1], self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'd':
                self.linear_speeds[1] = self.linear_speeds[1] - self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[1] = max(self.linear_speeds[1], -self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'i':
                self.linear_speeds[2] = self.linear_speeds[2] + self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[2] = min(self.linear_speeds[2], self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'k':
                self.linear_speeds[2] = self.linear_speeds[2] - self.velocity_change_rate*self.max_linear_speed
                self.linear_speeds[2] = max(self.linear_speeds[2], -self.max_linear_speed)
                self.ispressed = True
                ischanged = True

            # -- Angular velocity --
            if key == 't':
                self.angular_speeds[0] = self.angular_speeds[0] + self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[0] = min(self.angular_speeds[0], self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'g':
                self.angular_speeds[0] = self.angular_speeds[0] - self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[0] = max(self.angular_speeds[0], -self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'y':
                self.angular_speeds[1] = self.angular_speeds[1] + self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[1] = min(self.angular_speeds[1], self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'h':
                self.angular_speeds[1] = self.angular_speeds[1] - self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[1] = max(self.angular_speeds[1], -self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'u':
                self.angular_speeds[2] = self.angular_speeds[2] + self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[2] = min(self.angular_speeds[2], self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'j':
                self.angular_speeds[2] = self.angular_speeds[2] - self.velocity_change_rate*self.max_angular_speed
                self.angular_speeds[2] = max(self.angular_speeds[2], -self.max_angular_speed)
                self.ispressed = True
                ischanged = True

            # -- Additional function keys --
            if key == 's':
                self.linear_speeds = [0.0, 0.0, 0.0]
                self.angular_speeds = [0.0, 0.0, 0.0]
                self.ispressed = True
                ischanged = True
            if key == 'p':
                self.pause_motion= True
                self.mode = 'pause'
                self.ispressed = False
                ischanged = True
            if key == 'm':
                self.pause_motion = False
                self.mode = 'move '
                self.ispressed = False
                ischanged = True
            if key == 'r':
                self.ispressed = True
            if (key == 'q') or (key == '\x03'):
                self.exit = True 

            if ischanged:
                kInput = kInput + 1
                if kInput > 10:
                    kInput = 0
                    self.print_instruction()    
                print('Current Mode: {}, Linear Speed: {:.2f} {:.2f} {:.2f}, Angular Speed: {:.2f} {:.2f} {:.2f}'.format(self.mode, *self.linear_speeds, *self.angular_speeds))

    def publish_loop(self):
        if not (self.pause_motion):
            self.pause_cmd.data = self.pause_motion
            self.pause_pub.publish(self.pause_cmd)
        if not (self.pause_motion) and self.ispressed:
            self.ispressed = False
            self.twist_cmd.linear.x = self.linear_speeds[0]
            self.twist_cmd.linear.y = self.linear_speeds[1]
            self.twist_cmd.linear.z = self.linear_speeds[2]
            self.twist_cmd.angular.x = self.angular_speeds[0]
            self.twist_cmd.angular.y = self.angular_speeds[1]
            self.twist_cmd.angular.z = self.angular_speeds[2]
            self.twist_pub.publish(self.twist_cmd)
        if not rclpy.ok() or self.exit:
            self.timer.cancel()

        
    def start(self):
        """
    	Start the twist_teleop_key node
    	"""

        self.declare_parameters(
                namespace='',
                parameters=[
                    ('max_linear_speed', 1.0),
                    ('max_angular_speed', 2.0),
                    ('velocity_change_rate', 0.1),
                    ('cmd_rate', 20.0),
                ]
            )

        # Create and register control command publisher with the master
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_cmd = Twist()

        # Create and register joy priority publisher with the master
        self.pause_pub = self.create_publisher(Bool, '/pause_motion', 10)
        self.pause_cmd = Bool()

        # Load parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value # Maximum linear speed
        self.max_angular_speed = self.get_parameter('max_angular_speed').value  # Maximum angular speed
        self.velocity_change_rate = self.get_parameter('velocity_change_rate').value  # Maximum angular speed
        cmd_rate = self.get_parameter('cmd_rate').value # Command Publication Rate Parameter

        # Start a thread for keyboard inputs
        key_thread = Thread(target=self.key_input, daemon=True)   
        key_thread.start()

        # Command publication loop
        self.timer = self.create_timer(1/cmd_rate, self.publish_loop)


def main():
    rclpy.init()
    twist_teleop_key = Twist_Teleop_Key()
    twist_teleop_key.start()

    rclpy.spin(twist_teleop_key)

    # Destroy the node explicitly
    twist_teleop_key.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
