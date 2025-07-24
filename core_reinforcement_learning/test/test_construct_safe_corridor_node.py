import unittest
from unittest.mock import MagicMock
from core_reinforcement_learning.construct_safe_corridor_node import SafeCorridor

# Fake classes to mock ROS2 node and parameter behavior


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def error(self, msg):
        print(msg)

    def info(self, msg):
        print(msg)

    def warn(self, msg):
        print(msg)


class FakeNode:
    def __init__(self, parameters=None):
        if not isinstance(parameters, dict):
            self.parameters = {}
        else:
            self.parameters = parameters
        self.logger = FakeLogger()
        self.subscriptions = []
        self.publishers = []

    def get_parameter(self, name):
        return self.parameters.get(name, FakeParameter(None))

    def get_logger(self):
        return self.logger

    def create_subscription(self, *args, **kwargs):
        sub = MagicMock()
        self.subscriptions.append(sub)
        return sub

    def create_publisher(self, *args, **kwargs):
        pub = MagicMock()
        self.publishers.append(pub)
        return pub

    def declare_parameter(self, name, value):
        self.parameters[name] = FakeParameter(value)

    def get_namespace(self):
        return "test_ns"

    def create_timer(self, period, callback):
        # Do nothing for timer in test
        return MagicMock()


def fake_init(self, parameters):
    FakeNode.__init__(self, parameters)
    self.append_square_zone = self.get_parameter('append_square_zone').value
    self.square_half_length = self.get_parameter('square_half_length').value
    self.mid_points_offset = self.get_parameter('mid_points_offset').value
    self.num_square_zone_points = self.get_parameter(
        'num_square_zone_points').value
    self.number_of_boundary_points = self.get_parameter(
        'number_of_boundary_points').value
    self.callback_frequency = self.get_parameter('callback_frequency').value
    self.use_constant_for_distant_measurements = self.get_parameter(
        'use_constant_for_distant_measurements').value
    self.max_distance = self.get_parameter('max_lidar_distance').value


class TestSafeCorridor(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            'append_square_zone': FakeParameter(False),
            'square_half_length': FakeParameter(2.0),
            'mid_points_offset': FakeParameter(0.2),
            'num_square_zone_points': FakeParameter(8),
            'number_of_boundary_points': FakeParameter(10),
            'callback_frequency': FakeParameter(10),
            'use_constant_for_distant_measurements': FakeParameter(True),
            'max_lidar_distance': FakeParameter(4.0),
        }
        # Patch Node base class to use FakeNode
        SafeCorridor.__bases__ = (FakeNode,)
        SafeCorridor.__init__ = fake_init

        self.node = SafeCorridor(self.parameters)

    def test_initialization(self):
        self.assertFalse(self.node.append_square_zone)
        self.assertEqual(self.node.square_half_length, 2.0)
        self.assertEqual(self.node.num_square_zone_points, 8)

    def test_insert_square_zone(self):
        # Should add num_square_zone_points points to the list
        points = []
        result = self.node.insert_square_zone(points)
        self.assertEqual(len(result), self.node.num_square_zone_points)
        # Check that points are tuples of length 2
        for pt in result:
            self.assertEqual(len(pt), 2)
            self.assertIsInstance(pt[0], float)
            self.assertIsInstance(pt[1], float)

        # check that if we insert 20 points the midle points is in the middle
        self.node.num_square_zone_points = 16
        self.node.square_half_length = 3.0
        points = []
        result = self.node.insert_square_zone(points)
        # Check if scaling works as expected
        self.assertAlmostEqual(result[2][0], 0.0)
        self.assertAlmostEqual(result[2][1], -2.4)

    def test_zero_points(self):
        self.node.num_square_zone_points = 0
        points = []
        result = self.node.insert_square_zone(points)
        self.assertEqual(result, [])

    def test_not_divisible_by_four(self):
        self.node.num_square_zone_points = 7
        points = []
        result = self.node.insert_square_zone(points)
        self.assertEqual(len(result), 7)

    def test_zero_and_negative_half_length(self):
        self.node.square_half_length = 0
        self.node.num_square_zone_points = 4
        points = []
        result = self.node.insert_square_zone(points)
        for pt in result:
            self.assertAlmostEqual(pt[0], 0.0)
            self.assertAlmostEqual(pt[1], 0.0)
        self.node.square_half_length = -2.0
        points = []
        result = self.node.insert_square_zone(points)
        for pt in result:
            self.assertIsInstance(pt[0], float)
            self.assertIsInstance(pt[1], float)

    def test_mid_points_offset_zero_and_one(self):
        self.node.square_half_length = 2.0
        self.node.num_square_zone_points = 8
        # Offset = 0 (middle points on edge)
        self.node.mid_points_offset = 0.0
        points = []
        result = self.node.insert_square_zone(points)
        self.assertIn((0.0, -2.0), result)
        self.node.mid_points_offset = 1.0

        points = []
        result = self.node.insert_square_zone(points)
        self.assertIn((0.0, 0.0), result)

    def test_return_max_dist_point(self):
        pt = self.node.return_max_dist_point()
        self.assertEqual(pt, (self.node.max_distance, self.node.max_distance))


if __name__ == '__main__':
    unittest.main()
