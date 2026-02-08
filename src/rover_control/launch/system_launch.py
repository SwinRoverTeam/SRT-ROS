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
    
    # Get joystick device numbers from environment variables
    xbox_device = os.environ.get('XBOX_DEVICE', '2')
    joystick_device = os.environ.get('JOYSTICK_DEVICE', '1')
    
    return LaunchDescription([
        # Log startup information
        LogInfo(msg='Starting Rover Ground Control System... '),
        
        # 1a. Joy Node for Xbox Controller - reads Xbox controller input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_xbox',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'device_id': int(xbox_device),
                'autorepeat_rate': 20.0,
            }],
            remappings=[
                ('joy', 'joy_xbox'),
            ],
        ),
        
        # 1b. Joy Node for Joystick - reads Joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_joystick',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'device_id': int(joystick_device),
                'autorepeat_rate': 20.0,
            }],
            remappings=[
                ('joy', 'joy_joystick'),
            ],
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
        
        # 3. Joystick Node - processes Joystick for arm control
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
        
        LogInfo(msg=f'All nodes launched! Xbox: js{xbox_device}, Joystick: js{joystick_device}, Micro-ROS port {micro_ros_port}'),
    ])