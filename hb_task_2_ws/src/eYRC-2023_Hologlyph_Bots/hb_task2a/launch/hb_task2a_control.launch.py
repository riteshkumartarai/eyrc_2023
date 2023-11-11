from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='hb_task2a',
            executable='feedback',
        
        ),
        Node(
            package='hb_task2a',
            executable='control',
        
        )
    ])

