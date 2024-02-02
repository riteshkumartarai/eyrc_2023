import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        
        Node(
            package='feedback5',
            executable='image_proc',  # Replace with the actual name of your webcam publisher node script
            name='image_transformation'
        ),
        Node(
            package='feedback5',
            executable='feedback_node',  # Replace with the actual name of your camera node script
            name='feedback_node'
        ),
        Node(
            package='feedback5',
            executable='viz_pose',  # Replace with the actual name of your visualization node script
            name='visualization'
        ),
    ])
