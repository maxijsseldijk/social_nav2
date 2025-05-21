import unittest
import numpy as np
from core_reinforcement_learning.utils import (in_convex_hull,
                                               get_safe_corridor_vertices,
                                               project_point_to_line_segment,
                                               create_offset_critical_points,
                                               move_point_to_convex_hull)


class TestUtils(unittest.TestCase):

    def test_waypoint_on_edge(self):
        critical_points = np.array([[0, 0], [4, 0], [4, 4], [0, 4]])
        waypoint = np.array([2, 0])
        self.assertTrue(in_convex_hull(waypoint, critical_points))

    def test_waypoint_inside_complex_convex(self):
        critical_points = np.array([[0, 0], [4, 0], [4, 4], [2, 6], [0, 4]])
        waypoint = np.array([2, 3])
        self.assertTrue(in_convex_hull(waypoint, critical_points))

    def test_waypoint_outside_complex_convex(self):
        critical_points = np.array([[0, 0], [4, 0], [4, 4], [2, 6], [0, 4]])
        waypoint = np.array([5, 5])
        self.assertFalse(in_convex_hull(waypoint, critical_points))

    def test_move_point_to_convex_hull(self):
        critical_points = np.array([[0, 0], [4, 0], [4, 4], [0, 4]])
        point = np.array([5, 5])
        moved_point = move_point_to_convex_hull(point, critical_points)
        self.assertTrue(in_convex_hull(moved_point, critical_points))

    def test_project_point_to_line_segment(self):
        point = np.array([2, 2])
        start = np.array([0, 0])
        end = np.array([4, 0])
        projected_point = project_point_to_line_segment(point, start, end)
        expected_point = np.array([2, 0])
        np.testing.assert_array_almost_equal(projected_point, expected_point)

    def test_create_offset_critical_points(self):
        critical_points_square = np.array([
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [0.0, -1.0]
        ])
        offset = 0.1

        offset_points = create_offset_critical_points(
            critical_points_square, offset)
        offset = 0.1
        expect_points = np.array([
            [0.9, 0.0],
            [0.0, 0.9],
            [-0.9, 0.0],
            [0.0, -0.9]
        ])
        np.testing.assert_array_almost_equal(offset_points, expect_points)

        hull = get_safe_corridor_vertices(
            critical_points_square)
        hull_offset = get_safe_corridor_vertices(offset_points)

        # In this case the convex polygon vertices are at a 45 degree angle
        # So it is expected that the planes should be offset by offset * sqrt(2)
        self.assertTrue(
            np.allclose(np.linalg.norm(hull, axis=1) - np.linalg.norm(hull_offset, axis=1),
                        offset * np.sqrt(2)))

        # Test with too large offset leading to no change
        offset = 1.5
        offset_points = create_offset_critical_points(
            critical_points_square, offset)
        np.testing.assert_array_almost_equal(
            offset_points, critical_points_square)

        # Test both positive and negative values
        critical_points_square = np.array([
            [1.0, -0.3]
        ])

        offset = 0.1
        offset_points = create_offset_critical_points(
            critical_points_square, offset)
        expect_points = np.array([[0.904217, -0.271265]])
        np.testing.assert_array_almost_equal(offset_points, expect_points)

        # Test value raise
        offset = -0.1
        np.testing.assert_raises(
            ValueError, create_offset_critical_points, critical_points_square, offset)

    def test_get_safe_corridor_vertices(self):
        critical_points_square = np.array([
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [0.0, -1.0]
        ])

        expected_vertices_square = np.array([
            [1.0, 1.0],
            [1.0, -1.0],
            [-1.0, 1.0],
            [-1.0, -1.0]
        ])

        critical_points_triangle = np.array([
            [0., -0.5],
            [0.5, 0.],
            [0.5, -0.5]
        ])

        # Get the vertices from the function
        vertices_square = get_safe_corridor_vertices(critical_points_square)
        vertices_triangle = get_safe_corridor_vertices(
            critical_points_triangle)

        # Check if the vertices match the expected vertices
        self.assertTrue(np.allclose(
            vertices_square, expected_vertices_square, atol=1e-2))
        self.assertTrue(
            np.all([in_convex_hull(vertex, vertices_square) for vertex in vertices_square]))

        self.assertTrue(vertices_triangle is None)


if __name__ == '__main__':
    unittest.main()
