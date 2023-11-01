import time
from launch import LaunchDescription, actions


def generate_launch_description():
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

    # Запуск всех модулей
    return LaunchDescription([
        data_record
    ])
