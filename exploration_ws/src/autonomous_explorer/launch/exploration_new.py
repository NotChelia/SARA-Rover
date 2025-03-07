from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_explorer',
            executable='exploration_node_new',
            name='exploration_node_new',
            output='screen',
            parameters=[{
                'map_topic': '/pointlio/cloud_registered',
                'pointcloud_topic': '/unilidar/cloud',
                'odom_topic': '/pointlio/odom',
                'cmd_vel_topic': '/cmd_vel',
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
    ])
