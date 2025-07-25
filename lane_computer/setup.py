import os
from setuptools import find_packages, setup

package_name = 'lane_computer'

if not os.path.exists('resource'):
    os.makedirs('resource')
with open(os.path.join('resource', package_name), 'w') as f:
    pass

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@user.com',
    description='A ROS 2 package for lane computation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_computer_node = lane_computer.lane_computer_node:main',
        ],
    },
)