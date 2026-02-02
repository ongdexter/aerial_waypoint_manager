from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waypoint_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dexter Ong',
    maintainer_email='dexterong94@gmail.com',
    description='Waypoint planning interface for PX4-based UAVs using MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_planner_node = waypoint_planner.waypoint_planner_node:main',
            'waypoint_gui = waypoint_planner.waypoint_gui:main',
            'request_example = waypoint_planner.request_example:main',
        ],
    },
)
