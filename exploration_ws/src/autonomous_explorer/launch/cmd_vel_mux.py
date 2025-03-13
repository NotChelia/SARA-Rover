from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    joystick_topic_arg = DeclareLaunchArgument(
        'joystick_topic',
        default_value='/joy',
        description='Topic for joystick input'
    )
    
    explorer_cmd_vel_topic_arg = DeclareLaunchArgument(
        'explorer_cmd_vel_topic',
        default_value='/cmd_vel_explorer',
        description='Topic for explorer cmd_vel input'
    )
    
    output_cmd_vel_topic_arg = DeclareLaunchArgument(
        'output_cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic for output cmd_vel'
    )
    
    joystick_threshold_arg = DeclareLaunchArgument(
        'joystick_threshold',
        default_value='0.1',
        description='Threshold for joystick activation'
    )
    
    timeout_duration_arg = DeclareLaunchArgument(
        'timeout_duration',
        default_value='0.5',
        description='Timeout duration in seconds'
    )
    
    cmd_vel_mux_node = Node(
        package='autonomous_explorer',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        parameters=[{
            'joystick_topic': LaunchConfiguration('joystick_topic'),
            'explorer_cmd_vel_topic': LaunchConfiguration('explorer_cmd_vel_topic'),
            'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic'),
            'joystick_threshold': LaunchConfiguration('joystick_threshold'),
            'timeout_duration': LaunchConfiguration('timeout_duration'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        joystick_topic_arg,
        explorer_cmd_vel_topic_arg,
        output_cmd_vel_topic_arg,
        joystick_threshold_arg,
        timeout_duration_arg,
        cmd_vel_mux_node
    ])
