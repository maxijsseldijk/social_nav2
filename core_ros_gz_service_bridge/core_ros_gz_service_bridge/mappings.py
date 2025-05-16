

from collections import namedtuple

Mapping = namedtuple('Mapping', ('ros_type', 'gz_type'))

# List of known mappings
#
# The pattern for adding a new mapping
#
#   'ros2_package_name': [
#       Mapping('ros2_message_name', 'ignition_message_name'),
#   ],
MAPPINGS = {
    'builtin_interfaces': [
        Mapping('Time', 'Time'),
    ],
    'geometry_msgs': [
        Mapping('Point', 'Vector3d'),
        Mapping('Pose', 'Pose'),
        Mapping('PoseArray', 'Pose_V'),
        Mapping('PoseStamped', 'Pose'),
        Mapping('PoseWithCovariance', 'PoseWithCovariance'),
        Mapping('Quaternion', 'Quaternion'),
        Mapping('Transform', 'Pose'),
        Mapping('TransformStamped', 'Pose'),
        Mapping('Twist', 'Twist'),
        Mapping('TwistStamped', 'Twist'),
        Mapping('TwistWithCovariance', 'TwistWithCovariance'),
        Mapping('Wrench', 'Wrench'),
        Mapping('WrenchStamped', 'Wrench'),
        Mapping('Vector3', 'Vector3d'),
    ],
    'rcl_interfaces': [
        Mapping('ParameterValue', 'Any'),
    ],
    'ros_gz_interfaces': [
        Mapping('Entity', 'Entity'),
        Mapping('EntityFactory', 'EntityFactory'),
    ],
    'std_msgs': [
        Mapping('Bool', 'Boolean'),
        Mapping('ColorRGBA', 'Color'),
        Mapping('Empty', 'Empty'),
        Mapping('Float32', 'Float'),
        Mapping('Float64', 'Double'),
        Mapping('Header', 'Header'),
        Mapping('Int32', 'Int32'),
        Mapping('UInt32', 'UInt32'),
        Mapping('String', 'StringMsg'),
    ],
}
