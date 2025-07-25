from glob import glob
import os

from setuptools import setup
package_name = 'core_dynamical_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models', 'agent'), glob('models/agent/*')),
        (os.path.join('share', package_name, 'models', 'turtle_bot', 'core_turtlebot3_waffle_pi'),
         glob('models/turtle_bot/core_turtlebot3_waffle_pi/*')),
        (os.path.join('share', package_name, 'models', 'turtle_bot', 'turtlebot3'),
            glob('models/turtle_bot/turtlebot3/*')),
        (os.path.join('share', package_name, 'models', 'turtle_bot', 'turtlebot3_waffle_pi'),
            glob('models/turtle_bot/turtlebot3_waffle_pi/*')),
        (os.path.join('share', package_name, 'models', 'turtle_bot', 'turtlebot3_common', 'meshes'),
         glob('models/turtle_bot/turtlebot3_common/meshes/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maxvanijsseldijk@gmail.com',
    description='An package containing tools to generate agents with human-like features',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_odom_from_mocap_and_vel = core_dynamical_agent.publish_odom_from_mocap_and_vel:main',
            'agent_test_node = core_dynamical_agent.agent_test_node:main'

        ],
    },
)
