from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'realtime_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack0682@github.com',
    description='AprilTag-based 6DOF box pose detection and tracking pipeline',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_box_pose_node=realtime_detect.apriltag_box_pose_node:main',
        ],
    },
)
