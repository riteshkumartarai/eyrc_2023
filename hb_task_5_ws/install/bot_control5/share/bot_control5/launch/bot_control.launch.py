import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        
        Node(
            package='bot_control5',
            executable='control_node1',  # Replace with the actual name of your webcam publisher node script
            name='hex'
        ),
        Node(
            package='bot_control5',
            executable='control_node2',  # Replace with the actual name of your camera node script
            name='rec'
        ),
        Node(
            package='bot_control5',
            executable='control_node3',  # Replace with the actual name of your camera node script
            name='tri'
        ),
        
    ])
