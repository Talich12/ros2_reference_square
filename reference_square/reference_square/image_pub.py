import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge


class ImagePub(Node):
    def __init__(self):
        self.br = CvBridge()
        super().__init__('image_pub')
        self._publisher_ = self.create_publisher(Image, '/solaster/front_camera/image', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        image_path = '/home/mamba/ros2_reference_square/reference_square/reference_square/desk.jpg'
        img = cv.imread(image_path)
        self.get_logger().info(f'send image')
        self._publisher_.publish(self.br.cv2_to_imgmsg(img))


def main(args=None):
    rclpy.init(args=args)

    try:
        image_pub = ImagePub()
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
