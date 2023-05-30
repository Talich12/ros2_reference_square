import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge

class OdomPub(Node):
    def __init__(self):
        self.br = CvBridge()
        super().__init__('odom_pub')
        self._publisher_ = self.create_publisher(Image, '/solaster/front_camera/image', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        img = cv.imread('/home/mamba/ros2_fiducial_square/src/reference_square/reference_square/desk.jpg')
        self.get_logger().info(f'send image')
        self._publisher_.publish(self.br.cv2_to_imgmsg(img))


def main(args=None):
    rclpy.init(args=args)

    odom_pub = OdomPub()

    rclpy.spin(odom_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()