from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reference_square',
            executable='image_pub',
        ),
        Node(
            package='reference_square',
            executable='odom_pub',
        ),
        Node(
            package='reference_square',
            executable='camerainfo_pub',
        ),
        Node(
            package='reference_square',
            executable='reference_square',
        ),
    ])