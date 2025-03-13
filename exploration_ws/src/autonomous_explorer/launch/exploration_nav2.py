from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import tf2_ros

def generate_launch_description():
    package_dir = get_package_share_directory('autonomous_explorer')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'nav2.launch.py')
        )
    )
    
    exploration_node = Node(
        package='autonomous_explorer',
        executable='exploration_node_nav2',
        name='exploration_node_nav2',
        output='screen',
        parameters=[{
            'map_topic': '/pointlio/cloud_registered',
            'pointcloud_topic': '/unilidar/cloud',
            'odom_topic': '/pointlio/odom',
            'cmd_vel_topic': '/cmd_vel_explorer',
            'global_frame': 'camera_init',
            'robot_frame': 'base_link',
            'exploration_frequency': 2.0,
            'visualization_frequency': 5.0,
            'min_frontier_size': 10,
            'frontier_detection_range': 5.0,
            'goal_tolerance': 0.5,
            'potential_scale': 5.0,
            'gain_scale': 1.0,
            'obstacle_cost_scale': 0.5,
            'rotation_threshold': 0.9,
            'max_goal_distance': 5.0,
            'min_goal_distance': 1.0,
            'recovery_behaviors': ['clear_costmap']
        }]
    )
    
    static_transform_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_transform_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_unilidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar']
    )
    
    rviz_config_file = os.path.join(package_dir, 'rviz', 'exploration_nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    cmd_vel_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'cmd_vel_mux.py')
        )
    )
    
    return LaunchDescription([
        nav2_launch,
        exploration_node,
        cmd_vel_mux_launch,
        static_transform_publisher1,
        static_transform_publisher2,
        rviz_node
    ])
