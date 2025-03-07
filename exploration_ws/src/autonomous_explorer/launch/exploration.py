import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('autonomous_explorer')
    
    # Exploration node - simplified parameters
    exploration_node = Node(
        package='autonomous_explorer',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'point_cloud_topic': '/pointlio/cloud_registered'},
            {'odom_topic': '/pointlio/odometry'},
            {'map_frame': 'camera_init'},
            {'robot_frame': 'body'},
            {'min_frontier_size': 10}
        ]
    )
    
    return LaunchDescription([
        exploration_node
    ])
