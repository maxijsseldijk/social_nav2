import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time


class PublishOdomFromMocapAndVel(Node):
    """
    Node that publishes to 'odometry' topic using position from mocap and velocity from odom.
    Only publishes if both sources are recent and valid.
    """

    def __init__(self):
        super().__init__('publish_odom_from_mocap_and_vel', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.ns = self.get_namespace().strip('/')
        self.get_logger().info('PublishOdomFromMocapAndVel node has been initialized.')
        self.create_subscription(Odometry, 'odomtest', self.odom_callback, 10)
        self.create_subscription(
            PoseStamped, f'/mocap/{self.ns}/pose', self.mocap_pose_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odometrytest', 10)

        self.latest_pose = None
        self.latest_pose_time = None
        self.latest_twist = None
        self.latest_twist_time = None
        self.data_timeout_sec = 0.5

        self.timer = self.create_timer(0.05, self.publish_fused_odom)

    def odom_callback(self, msg):
        self.latest_twist = msg.twist
        self.latest_twist_time = self.get_clock().now()

    def mocap_pose_callback(self, msg):
        self.latest_pose = msg.pose
        self.latest_pose_time = self.get_clock().now()

    def publish_fused_odom(self):
        now = self.get_clock().now()
        if (self.latest_pose is not None and self.latest_twist is not None and
            (now - self.latest_pose_time).nanoseconds * 1e-9 < self.data_timeout_sec and
                (now - self.latest_twist_time).nanoseconds * 1e-9 < self.data_timeout_sec):

            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose = self.latest_pose
            odom_msg.twist = self.latest_twist
            self.odom_pub.publish(odom_msg)
        else:
            self.get_logger().warn("Waiting for reliable mocap and odom data...")
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = PublishOdomFromMocapAndVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
