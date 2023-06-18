import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description():
    namespace = 'solaster'

    config = os.path.join(
        get_package_share_directory('reference_square'),
        'config',
        'config.yaml'
        )


    reference_square_node = Node(package='reference_square',
                                 executable='reference_square',
                                 output='screen',
                                 emulate_tty=True,
                                 name='reference_square',
                                 parameters=[{"debug": True}, config]
                                 )

    test_odom_pub = Node(package='reference_square',
                         executable='odom_pub',
                         output='screen',
                         emulate_tty=True,
                         parameters=[{"start_pos": [0., 0., 0.2]},
                                     {"step_pos": [0., 0., 0.2]}
                                     ]
                         )

    test_camera_info_pub = Node(package='reference_square',
                                executable='camerainfo_pub',
                                output='screen',
                                emulate_tty=True
                                )

    test_image_pub = Node(package='reference_square',
                          executable='image_pub',
                          output='screen',
                          emulate_tty=True
                          )

    group = GroupAction([
        PushRosNamespace(namespace),
        reference_square_node,
        test_odom_pub,
        test_camera_info_pub,
        test_image_pub
    ])

    return LaunchDescription([group])
