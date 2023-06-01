import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge


class ImagePub(Node):
    def __init__(self):
        self.br = CvBridge()
        super().__init__('test_image_publisher')
        self._publisher = self.create_publisher(Image, '/solaster/front_camera/image', 10)
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
        self.get_logger().info(f'Send test image')
        self._publisher.publish(self.br.cv2_to_imgmsg(self._test_image))


def main(args=None):
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
