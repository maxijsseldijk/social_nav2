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

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maxvanijsseldijk@gmail.com',
    description='An package containing tools to generate agents with human-like features',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={

    },
)
