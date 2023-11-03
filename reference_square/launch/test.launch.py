import time
from launch import LaunchDescription, actions
import os
from ament_index_python.packages import get_package_share_directory
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
                                 parameters=[{"debug": True},
                                             {"period_image_publish_ms": 1},
                                             config]
                                 )

    test_odom_pub = Node(package='reference_square',
                         executable='odom_pub',
                         output='screen',
                         emulate_tty=True,
                         parameters=[{"start_pos": [0., 0., 0.2]},
                                     {"step_pos": [0., 0., 0.2]},
                                     {"debug": True}
                                     ]
                         )

    test_camera_info_pub = Node(package='reference_square',
                                executable='camerainfo_pub',
                                output='screen',
                                emulate_tty=True,
                                parameters=[{"debug": True}]
                                )

    test_image_pub = Node(package='reference_square',
                          executable='image_pub',
                          output='screen',
                          emulate_tty=True,
                          parameters=[{"debug": True}]
                          )



    # Список топиков, которые необходимо записать в лог
    bag_record_masks = [
        # Тестовые топики
        '/solaster/odom',
        '/solaster/image'
        ]

    # Динамическое создание папки с логами в нужном месте
    timestamp = time.strftime("%Y_%m_%d-%H_%M_%S")
    bag_file_path = '../bagfiles/' + 'rosbag2_' + timestamp

    bag_record_masks_str = '|'.join(bag_record_masks)

    data_record = actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '--max-bag-size', '1000000000',
                 '--storage', 'mcap',
                 '--regex', bag_record_masks_str,
                 '-o', bag_file_path
                 ],
    )

    group = GroupAction([
        PushRosNamespace(namespace),
        reference_square_node,
        test_odom_pub,
        test_camera_info_pub,
        test_image_pub,
        data_record
    ])

    # Запуск всех модулей
    return LaunchDescription([
        group
    ])
