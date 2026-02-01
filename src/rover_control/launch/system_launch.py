from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
import os

def generate_launch_description():

    # Launch file to start:  
    # 1. Joy node (controller input)
    # 2. Control node (processing and publishing to rover)
    # 3. Micro-ROS agent (communication bridge to ESP32)
    
    # Get micro-ROS port from environment variable, default to 8888
    micro_ros_port = os.environ.get('MICRO_ROS_PORT', '8888')
    
    return LaunchDescription([
        # Log startup information
        LogInfo(msg='Starting Rover Ground Control System... '),
        
        # 1. Joy Node - reads controller input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # 2. Controller Node - processes Xbox controller for drive control
        Node(
            package='rover_control',
            executable='controller_node',
            name='xbox_controller',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # 3. Joystick Node - processes joystick for arm control
        Node(
            package='rover_control',
            executable='joystick_node',
            name='joystick',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # 4. Micro-ROS Agent - UDP bridge to ESP32
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'udp4', '--port', micro_ros_port
            ],
            output='screen',
            name='micro_ros_agent',
        ),
        
        LogInfo(msg=f'All nodes launched!  Micro-ROS agent on UDP port {micro_ros_port}'),
    ])