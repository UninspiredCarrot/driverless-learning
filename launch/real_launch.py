from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_input',
            executable='zed_camera_rectify',
            name='camera'
        ),
        Node(
            package='perception_process',
            executable='cone_estimate',
            name='depth',
        )
    ])