from glob import glob
import os

from setuptools import setup
package_name = 'core_reinforcement_learning'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maxvanijsseldijk@gmail.com',
    description='An package containing reinforcement learning tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'construct_safe_corridor_node = \
                  core_reinforcement_learning.construct_safe_corridor_node:main',
            'publish_agents_velocity = core_reinforcement_learning.publish_agents_velocity:main',
            'train_rl = core_reinforcement_learning.train_rl:main'
        ],
    },
)
