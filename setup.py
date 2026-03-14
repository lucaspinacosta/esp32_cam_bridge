import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'esp32_cam_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=[
        'setuptools',
        'requests',
    ],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='lucas@ingeniarius.pt',
    description='Bridge an ESP32-CAM HTTP stream and LED API into ROS 2 topics.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32_cam_bridge = esp32_cam_bridge.esp32_cam_bridge_node:main',
            'human_follower = esp32_cam_bridge.human_follower_node:main',
        ],
    },
)
