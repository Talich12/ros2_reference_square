"""Файл публикации навигационных данных."""
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomPub(Node):
    """Класс ноды публикации данных позиции и ориенации.

    Args:
        Node (Node): ROS2 нода.
    """

    def __init__(self):
        """Инициализация объектов."""
        super().__init__('test_odom_publisher')
        self._publisher_ = self.create_publisher(Odometry, '/solaster/odom', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Публикация данных позиции и ориентации."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.
        msg.pose.pose.position.y = 2.
        msg.pose.pose.position.z = 2.
        msg.pose.pose.orientation.x = 0.1727661
        msg.pose.pose.orientation.y = 0.1727661
        msg.pose.pose.orientation.z = 0.
        msg.pose.pose.orientation.w = 0.9696926
        self._publisher_.publish(msg)
        self.get_logger().info('Send test odometry data')


def main(args=None):
    """Инициализация ноды и запуск.

    Args:
        args (any, optional): Входящие аругменты. Defaults to None.
    """
    rclpy.init(args=args)

    try:
        odom_pub = OdomPub()
        rclpy.spin(odom_pub)
    except Exception as e:
        print('OdomPub::Exception ' + str(e))

    # Заглушка что бы нода не вылетала с ошибкой после Ctrl+C
    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            odom_pub.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
