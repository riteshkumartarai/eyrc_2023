import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        
        Node(
            package='bot_control5',
            executable='mini_theme',  # Replace with the actual name of your webcam publisher node script
            name='minitheme'
        ),
        Node(
            package='bot_control5',
            executable='stopflag',  # Replace with the actual name of your camera node script
            name='stop_service_server'
        ),
        
    ])
