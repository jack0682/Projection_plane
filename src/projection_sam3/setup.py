from setuptools import find_packages, setup

package_name = 'projection_sam3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/projection_sam3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack0682@github.com',
    description='SAM3 text-based semantic segmentation node for projection plane images',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'projection_sam3_node = projection_sam3.node:main',
        ],
    },
)
