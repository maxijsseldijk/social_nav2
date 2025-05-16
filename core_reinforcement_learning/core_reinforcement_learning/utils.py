#! /usr/bin/env python3

import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from transforms3d.euler import quat2euler
from scipy.spatial import ConvexHull, Delaunay, QhullError


def convert_twist_to_vector3(msg: Odometry) -> Vector3Stamped:
    """
    Convert the twist message from the odometry to a Vector3Stamped message.

    Args:
    ----
        msg (Odometry): The odometry message containing the twist information.

    Returns
    -------
        Vector3Stamped: The converted twist message as a Vector3Stamped message.

    """
    local_speed = math.sqrt(msg.twist.twist.linear.x **
                            2 + msg.twist.twist.linear.y ** 2)

    # Calculate the angle of the velocity vector in the robot's local frame
    local_angle = math.atan2(msg.twist.twist.linear.y,
                             msg.twist.twist.linear.x)

    orientation_q = msg.pose.pose.orientation
    _, _, yaw = quat2euler(
        [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])

    global_angle = yaw + local_angle
    velocity_x = local_speed * math.cos(global_angle)
    velocity_y = local_speed * math.sin(global_angle)
    msg_velocity = Vector3Stamped()
    msg_velocity.header = msg.header
    msg_velocity.vector.x = velocity_x
    msg_velocity.vector.y = velocity_y
    msg_velocity.vector.z = 0.0

    return msg_velocity


def get_transform_angle(vector1: np.ndarray, vector2: np.ndarray) -> float:
    """
    Get the angle between two vectors and return the angle in radians within the range [-pi, pi].

    Args:
    ----
        vector1 (np.ndarray): The first vector.
        vector2 (np.ndarray): The second vector.

    Returns
    -------
        float: The angle between the two vectors in radians [-pi, pi].

    """
    angle1 = np.arctan2(vector1[1], vector1[0])
    angle2 = np.arctan2(vector2[1], vector2[0])
    angle_diff = angle2 - angle1

    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

    return angle_diff


def get_safe_corridor_vertices(critical_points: np.ndarray) -> np.ndarray:
    """
    Compute the intersection points of the hyperplanes.

    Args:
    ----
        critical_points (np.ndarray): The critical points.

    Returns
    -------
        np.ndarray: The vertices of the safe corridor. or None if no safe corridor can be spanned.

    """
    # Compute the intersection points of the hyperplanes
    A = critical_points
    b = np.array([np.dot(bp, bp) for bp in critical_points])

    # Find all vertices that satisfy A x <= b
    vertices = []
    for i in range(len(A)):
        for j in range(i + 1, len(A)):
            A_sub = np.array([A[i], A[j]])
            if np.linalg.det(A_sub) != 0:
                x = np.linalg.solve(A_sub, np.array([b[i], b[j]]))
                if np.all(A @ x <= b * 1.01):
                    vertices.append(x)

    try:
        ConvexHull(vertices)
    except QhullError:
        return None

    return np.array(vertices)


def in_convex_hull(waypoint: np.ndarray, vertices: np.ndarray) -> bool:
    """
    Check if the waypoint is inside the convex hull.

    Args:
    ----
        waypoint (np.ndarray): The waypoint to be checked.
        vertices (np.ndarray): The vertices of the convex hull.

    Returns
    -------
        bool: True if the waypoint is inside the convex hull, False otherwise.

    """
    hull = Delaunay(vertices)
    return hull.find_simplex(waypoint) >= 0


def move_point_to_convex_hull(point: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """
    Move the point to the convex hull.

    Args:
    ----
        point (np.ndarray): The point to be moved.
        vertices (np.ndarray): The vertices of the convex hull.

    Returns
    -------
        np.ndarray: The point inside the convex hull.

    """
    hull = ConvexHull(vertices)

    # Find the closest point on the convex hull
    closest_point = None
    min_distance = float('inf')

    for simplex in hull.simplices:
        simplex_points = vertices[simplex]
        proj_point = project_point_to_line_segment(
            point, simplex_points[0], simplex_points[1])
        distance = np.linalg.norm(proj_point - point)
        if distance < min_distance:
            min_distance = distance
            closest_point = proj_point

    return closest_point


def project_point_to_line_segment(point: np.ndarray, start: np.ndarray,
                                  end: np.ndarray) -> np.ndarray:
    """
    Project the point to the line segment.

    Args:
    ----
        point (np.ndarray): The point to be projected.
        start (np.ndarray): The start of the line segment.
        end (np.ndarray): The end of the line segment.

    Returns
    -------
        np.ndarray: The projected point.

    """
    line_vec = end - start
    point_vec = point - start
    line_len = np.dot(line_vec, line_vec)
    if line_len == 0:
        return start
    projection = np.dot(point_vec, line_vec) / line_len
    projection = np.clip(projection, 0, 1)
    return start + projection * line_vec
