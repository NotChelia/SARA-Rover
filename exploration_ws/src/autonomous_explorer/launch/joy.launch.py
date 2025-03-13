from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('autonomous_explorer')
    
    joy_params = os.path.join(package_dir, 'config', 'joy_params.yaml')
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params],
        output='screen',
    )
    
    return LaunchDescription([
        joy_node
    ])
