#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from visualization_msgs.msg import Marker
import time
from geometry_msgs.msg import Point, PolygonStamped
from core_custom_messages.msg import PointArray
from rclpy.exceptions import ROSInterruptException


class SafeCorridor(Node):
    """
    A ROS 2 node that constructs a safe corridor based on LiDAR data.

    This node subscribes to LiDAR data, processes it to find boundary points,
    and publishes the safe corridor for visualization and further processing.
    """

    def __init__(self):
        """Initialize the SafeCorridor node."""
        super().__init__('safe_corridor_node')
        self.get_logger().error('Safe Corridor Node has been started')
        self.create_subscription(
            LaserScan, f'{self.get_namespace()}/scan', self.lidar_callback, 10)
        self.boundary_publisher = self.create_publisher(
            PointArray, f'{self.get_namespace()}/critical_points', 10)
        self.boundary_publisher_rviz = self.create_publisher(
            Marker, f'{self.get_namespace()}/critical_points_rviz', 10)
        self.safe_corridor_rviz_pub_ = self.create_publisher(
            PolygonStamped, f'{self.get_namespace()}/safe_corridor', 10)

        self.declare_parameter('number_of_boundary_points', 15)
        self.number_of_boundary_points = self.get_parameter(
            'number_of_boundary_points').value

        self.declare_parameter('callback_frequency', 10)
        self.callback_frequency = self.get_parameter(
            'callback_frequency').value

        # If True, the safe corridor will yield a constant distance for points outside max_distance
        self.declare_parameter('use_constant_for_distant_measurements', True)
        self.use_constant_for_distant_measurements = self.get_parameter(
            'use_constant_for_distant_measurements').value

        # The maximum distance to consider for the safe corridor
        self.declare_parameter('max_lidar_distance', 4.0)
        self.max_distance = self.get_parameter('max_lidar_distance').value

        timer_period = 1 / self.callback_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.laser_data = None

    def return_max_dist_point(self):
        """Fuction that returns a constant for a point outside the max_distance."""
        return (self.max_distance, self.max_distance)

    def timer_callback(self):
        """Process the LiDAR data to find boundary points and publish the safe corridor."""
        if self.laser_data is None:
            self.get_logger().error('No LiDAR data received yet')
            time.sleep(1)
            return

        boundary_points = []
        remaining_points = self.laser_data.copy()

        closest_obstacle = min(
            remaining_points, key=lambda x: x[0]**2 + x[1]**2)
        remaining_points.remove(closest_obstacle)
        boundary_points.append(closest_obstacle)
        remaining_points_temp = remaining_points.copy()
        current_bp = closest_obstacle

        while len(remaining_points) > 0 and len(boundary_points) < self.number_of_boundary_points:

            # Check all the points if they fall inside the hyperplane created
            # with (x-boudary_point).T * boudary_point <= 0.
            # To overcome multiple points on the same line, we add a small margin of 0.05
            for scan_point in remaining_points:
                diff_scan_bp = [
                    scan_point[0] - current_bp[0], scan_point[1] - current_bp[1]]
                if -diff_scan_bp[0] * current_bp[0] - diff_scan_bp[1] * current_bp[1] >= 0.05:
                    pass
                else:
                    remaining_points_temp.remove(scan_point)

            remaining_points = remaining_points_temp.copy()
            if len(remaining_points) > 0:
                # Add another boundary point
                closest_obstacle = min(
                    remaining_points, key=lambda x: x[0]**2 + x[1]**2)
                remaining_points.remove(closest_obstacle)
                remaining_points_temp.remove(closest_obstacle)
                boundary_points.append(closest_obstacle)
                current_bp = closest_obstacle

        else:
            if len(boundary_points) < self.number_of_boundary_points:
                points_to_add = self.number_of_boundary_points - \
                    len(boundary_points)
                self.get_logger().info(
                    f'Not enough boundary points found. \
                    Adding constant far measurement, {points_to_add} times.')
                for _ in range(points_to_add):
                    boundary_points.append(self.return_max_dist_point())
        # Copy data for visualization as the boundary_points will be modified
        boundary_point_viz = boundary_points.copy()

        if self.use_constant_for_distant_measurements:
            for i, point in enumerate(reversed(boundary_points)):
                index = len(boundary_points) - 1 - i
                if point[0] ** 2 + point[1] ** 2 > self.max_distance ** 2:
                    boundary_points[index] = self.return_max_dist_point()
                else:
                    break

        # Publish the boundary points for visualization
        boundary_marker = Marker()
        boundary_marker.header.frame_id = self.raw_laser_data_header.frame_id
        boundary_marker.header.stamp = self.raw_laser_data_header.stamp
        boundary_marker.ns = 'safe_corridor'
        boundary_marker.id = 0
        boundary_marker.type = Marker.CUBE_LIST
        boundary_marker.action = Marker.ADD
        boundary_marker.pose.orientation.w = 1.0
        boundary_marker.scale.x = 0.2
        boundary_marker.scale.y = 0.2
        boundary_marker.color.r = 0.0
        boundary_marker.color.g = 0.0
        boundary_marker.color.b = 0.0
        boundary_marker.color.a = 1.0
        boundary_marker.points = [
            Point(x=point[0], y=point[1], z=0.0) for point in boundary_point_viz]
        boundary_marker.points.append(boundary_marker.points[0])
        self.boundary_publisher_rviz.publish(boundary_marker)

        # Publish the boundary points for training
        boundary_points_array = PointArray()
        boundary_points_array.points = [
            Point(x=point[0], y=point[1], z=0.0) for point in boundary_points]
        self.boundary_publisher.publish(boundary_points_array)

    def polar_to_cartesian(self, radius: float, angle: float) -> list[float, float]:
        """
        Convert a polar coordinate to a cartesian coordinate.

        Args
        ----
            radius (float): The radius in polar coordinates.
            angle (float): The angle in polar coordinates.

        Returns
        -------
            list[float, float]: The x and y coordinates in cartesian coordinates.

        """
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        return x, y

    def lidar_callback(self, msg: LaserScan):
        """
        Process calbacks for processing LiDAR data.

        This method is called whenever a new LaserScan message is received.
        It filters out 'inf' values and converts the valid points to Cartesian coordinates.

        Args:
        ----
            msg (LaserScan): The LaserScan message containing LiDAR data.

        """
        self.raw_laser_data_header = msg.header
        # Filter out 'inf' values and convert to Cartesian coordinates
        valid_points = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if not math.isinf(distance):  # Check if the distance is valid
                valid_points.append(self.polar_to_cartesian(distance, angle))
            angle += msg.angle_increment

        # Now valid_points contains the filtered and converted LiDAR data
        self.laser_data = valid_points


def main():
    rclpy.init(args=None)
    safe_corridor = None
    try:
        safe_corridor = SafeCorridor()
        rclpy.spin(safe_corridor)
    except ROSInterruptException as e:
        safe_corridor.get_logger().error(
            f"ROS interrupt exception caught: {e}")
    except Exception as e:
        safe_corridor.get_logger().error(f"General exception caught: {e}")
    finally:
        if safe_corridor is not None:
            safe_corridor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
