from glob import glob
import os

from setuptools import setup
package_name = 'core_cleaning_robot'

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
        (os.path.join('share', package_name, 'models', 'cleaning_robot'),
            glob('models/cleaning_robot/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maxvanijsseldijk@gmail.com',
    description='An package containing tools to generate the'
                'simulation environment with the cleaning robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_robot_as_people_node = core_cleaning_robot.publish_robot_as_people_node:main',
        ],
    },
)
