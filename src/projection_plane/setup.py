from setuptools import setup, find_packages

package_name = 'projection_plane'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/projection.launch.py']),
        ('share/' + package_name + '/config', ['config/projection_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jack',
    maintainer_email='jack@example.com',
    description='Orthographic projection of point clouds onto a plane',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projection_node = projection_plane.projection_node:main',
        ],
    },
)
