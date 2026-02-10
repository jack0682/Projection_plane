
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('projection_plane')
    config_file = os.path.join(pkg_share, 'config', 'projection_params.yaml')

    # Load defaults from YAML
    config_defaults = {
        'input_file': '',
        'output_file': '',
        'plane_n': '[0.0, 0.0, 1.0]',
        'plane_d': '0.0'
    }
    
    try:
        with open(config_file, 'r') as f:
            params = yaml.safe_load(f)
            ros_params = params['projection_node']['ros__parameters']
            
            if 'input_file' in ros_params:
                config_defaults['input_file'] = str(ros_params['input_file'])
            if 'output_file' in ros_params:
                config_defaults['output_file'] = str(ros_params['output_file'])
            if 'plane_n' in ros_params:
                config_defaults['plane_n'] = str(ros_params['plane_n'])
            if 'plane_d' in ros_params:
                config_defaults['plane_d'] = str(ros_params['plane_d'])
    except Exception as e:
        print(f"Warning: Failed to load defaults from config file: {e}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_file',
            default_value=config_defaults['input_file'],
            description='Path to input PLY file'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value=config_defaults['output_file'],
            description='Path to output PNG file (optional)'
        ),
        DeclareLaunchArgument(
            'plane_n',
            default_value=config_defaults['plane_n'],
            description='Plane normal vector [nx, ny, nz]'
        ),
        DeclareLaunchArgument(
            'plane_d',
            default_value=config_defaults['plane_d'],
            description='Plane distance constant'
        ),
        
        Node(
            package='projection_plane',
            executable='projection_plane_node',
            name='projection_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'input_file': LaunchConfiguration('input_file'),
                    'output_file': LaunchConfiguration('output_file'),
                    'plane_n': LaunchConfiguration('plane_n'),
                    'plane_d': LaunchConfiguration('plane_d'),
                }
            ]
        )
    ])
