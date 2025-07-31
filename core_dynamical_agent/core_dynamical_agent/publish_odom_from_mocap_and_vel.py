import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, Point
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import math
from transforms3d.euler import quat2euler


class PublishOdomFromMocapAndVel(Node):
    """
    Node that publishes to 'odometry' topic using position from mocap and velocity from odom.
    Only publishes if both sources are recent and valid.
    """

    def __init__(self):
        super().__init__('publish_odom_from_mocap_and_vel')
        self.ns = self.get_namespace().strip('/')
        self.declare_parameter('motion_model', 'differential')
        self.motion_model = self.get_parameter('motion_model').value
        self.get_logger().info('PublishOdomFromMocapAndVel node has been initialized.')
        qos_profile_task = QoSProfile(depth=3)
        qos_profile_task.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile_task.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PoseStamped, f'/mocap/{self.ns}/pose', self.mocap_pose_callback, qos_profile_task)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.latest_pose = None
        self.latest_pose_time = None
        self.previous_pose_time = None
        self.latest_twist = None
        self.latest_twist_time = None
        self.data_timeout_sec = 0.5

        self.alpha = 0.8  # Higher alpha = more responsive to measurements
        self.beta = 0.2   # Lower beta = smoother velocity estimates
        self.filter_initialized = False

        self.get_logger().info(
            f'Alpha-Beta filter: alpha={self.alpha}, beta={self.beta}')

        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 2.0
        self.agent_pos_k_1: Point = None
        self.agent_vel_k_1: Point = None

        self.previous_orientation = None
        self.filtered_angular_velocity = 0.0
        self.filtered_orientation = None

        self.tf_broadcaster = TransformBroadcaster(self)

    def _initialize_filter(self, position: Point, time: Time):
        """Initialize the alpha-beta filter with the first position measurement."""
        self.agent_pos_k_1 = Point()
        self.agent_pos_k_1.x = position.x
        self.agent_pos_k_1.y = position.y
        self.agent_pos_k_1.z = position.z

        self.agent_vel_k_1 = Point()
        self.agent_vel_k_1.x = 0.0
        self.agent_vel_k_1.y = 0.0
        self.agent_vel_k_1.z = 0.0

        self.previous_time = time
        self.filter_initialized = True

        self.get_logger().info(
            f'Filter initialized at position: '
            f'({position.x:.3f}, {position.y:.3f}, {position.z:.3f})')

    def _apply_alpha_beta_filter(self, measured_pos: Point, dt: float):
        """Apply alpha-beta filter for position smoothing and velocity estimation."""
        pos_predicted = Point()
        pos_predicted.x = self.agent_pos_k_1.x + (self.agent_vel_k_1.x * dt)
        pos_predicted.y = self.agent_pos_k_1.y + (self.agent_vel_k_1.y * dt)
        pos_predicted.z = self.agent_pos_k_1.z  # Assume no z movement

        residual = Point()
        residual.x = measured_pos.x - pos_predicted.x
        residual.y = measured_pos.y - pos_predicted.y
        residual.z = 0.0

        self.agent_pos_k_1.x = pos_predicted.x + self.alpha * residual.x
        self.agent_pos_k_1.y = pos_predicted.y + self.alpha * residual.y
        self.agent_pos_k_1.z = pos_predicted.z

        if dt > 0.001:
            new_vel_x = self.agent_vel_k_1.x + (self.beta * residual.x) / dt
            new_vel_y = self.agent_vel_k_1.y + (self.beta * residual.y) / dt

            self.agent_vel_k_1.x = max(-self.max_linear_velocity,
                                       min(self.max_linear_velocity, new_vel_x))
            self.agent_vel_k_1.y = max(-self.max_linear_velocity,
                                       min(self.max_linear_velocity, new_vel_y))
            self.agent_vel_k_1.z = 0.0

    def publish_transform(self, msg: PoseStamped, timestamp: Time):
        if not self.filter_initialized:
            return

        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = f'{self.ns}/fake_odom'
        t.child_frame_id = f'{self.ns}/base_footprint'

        t.transform.translation.x = self.agent_pos_k_1.x
        t.transform.translation.y = self.agent_pos_k_1.y
        t.transform.translation.z = self.agent_pos_k_1.z

        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _calculate_angular_velocity(self, orientation, dt):
        """Calculate angular velocity using yaw difference and alpha-beta filtering."""
        _, _, current_yaw = quat2euler(
            [orientation.w, orientation.x, orientation.y, orientation.z])

        if self.previous_orientation is None:
            self.previous_orientation = current_yaw
            return 0.0

        yaw_diff = current_yaw - self.previous_orientation
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        raw_angular_vel = yaw_diff / dt

        raw_filtered_angular_vel = (
            self.alpha * raw_angular_vel +
            (1 - self.alpha) * self.filtered_angular_velocity
        )

        self.filtered_angular_velocity = max(-self.max_angular_velocity,
                                             min(self.max_angular_velocity,
                                                 raw_filtered_angular_vel))

        self.previous_orientation = current_yaw
        return self.filtered_angular_velocity

    def _create_odometry_message(self, msg: PoseStamped, timestamp: Time, dt: float) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = f'{self.ns}/fake_odom'
        odom_msg.child_frame_id = f'{self.ns}/base_footprint'

        odom_msg.pose.pose.position.x = self.agent_pos_k_1.x
        odom_msg.pose.pose.position.y = self.agent_pos_k_1.y
        odom_msg.pose.pose.position.z = self.agent_pos_k_1.z
        odom_msg.pose.pose.orientation = msg.pose.orientation

        twist_msg = TwistWithCovariance()
        if self.motion_model == 'omnidirectional':
            twist_msg.twist.linear.x = self.agent_vel_k_1.x
            twist_msg.twist.linear.y = self.agent_vel_k_1.y
        elif self.motion_model == 'differential':
            twist_msg.twist.linear.x = math.sqrt(
                self.agent_vel_k_1.x**2 + self.agent_vel_k_1.y**2)
            twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0

        angular_vel = self._calculate_angular_velocity(
            msg.pose.orientation, dt)
        twist_msg.twist.angular.z = angular_vel

        odom_msg.pose.covariance[0] = 0.01   # x position variance
        odom_msg.pose.covariance[7] = 0.01   # y position variance
        odom_msg.pose.covariance[35] = 0.05  # yaw variance
        twist_msg.covariance[0] = 0.1        # x velocity variance
        twist_msg.covariance[7] = 0.1        # y velocity variance
        twist_msg.covariance[35] = 0.2       # angular velocity variance

        odom_msg.twist = twist_msg
        return odom_msg

    def mocap_pose_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now()
        current_pos = msg.pose.position

        if not self.filter_initialized:
            self._initialize_filter(current_pos, current_time)
            return

        dt = (current_time - self.previous_time).nanoseconds * 1e-9
        if dt <= 0.001:
            return
        self._apply_alpha_beta_filter(current_pos, dt)

        odom_msg = self._create_odometry_message(msg, current_time, dt)
        self.odom_pub.publish(odom_msg)

        self.previous_time = current_time

        self.publish_transform(msg, current_time)


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
