"""Файл публикации тестового изображения."""
import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge


class ImagePub(Node):
    """
    Класс ноды публикации тестового изображения.

    Args
    ----
        Node (Node): ROS2 нода.
    """

    def __init__(self):
        """
        Инициализация объектов и запись изображения в cv2.

        Raises
        ------
            Exception: Исключение на существование файла изображения.
            Exception: Исключение на корректное открытие изображения.

        """
        super().__init__('test_image_publisher')
        self.br = CvBridge()

        self._namespace = self.get_namespace()
        if self._namespace != '':
            if self._namespace[-1] != '/':
                self._namespace += '/'

            if self._namespace[0] != '/':
                self._namespace = '/' + self._namespace

        self._publisher = self.create_publisher(Image, self._namespace + 'image', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        install_path = get_package_share_directory('reference_square')
        image_path = os.path.join(install_path, 'resource', 'desk.jpg')

        # Проверить существование файла с тестовым изображением
        if cv.haveImageReader(image_path) is False:
            self.get_logger().fatal("Test image is not available, origin path: {}"
                                    .format(image_path))
            raise Exception("Test image is not available")

        self._test_image = cv.imread(image_path)

        # Проверить что тестовый файл корректно открыт
        if self._test_image is None:
            self.get_logger().fatal("Test image is corrupt")
            raise Exception("Test image is corrupt")

    def timer_callback(self):
        """Публикация изображения."""
        self._publisher.publish(self.br.cv2_to_imgmsg(self._test_image, encoding='rgb8'))
        self.get_logger().info('Send test image')


def main(args=None):
    """
    Инициализация ноды и запуск.

    Args
    ----
        args (any, optional): Входящие аргументы. Defaults to None.
    """
    rclpy.init(args=args)

    try:
        image_pub = ImagePub()
    except Exception as e:
        print('ImagePub::Exception ' + str(e))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(image_pub)
    except Exception as e:
        print('ImagePub::Exception ' + str(e))

    # Заглушка что бы нода не вылетала с ошибкой после Ctrl+C
    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            image_pub.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
