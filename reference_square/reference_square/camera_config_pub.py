"""Файл публикации данных математической модели камеры."""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo


class CameraConfigPub(Node):
    """
    Класс ноды публикации данных математической модели камеры.

    Args:
    ----
        Node (Node): ROS2 нода.

    """

    def __init__(self):
        """Инициализация объектов."""
        super().__init__('test_config_publisher',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self._namespace = self.get_namespace()
        if self._namespace != '':
            if self._namespace[-1] != '/':
                self._namespace += '/'

            if self._namespace[0] != '/':
                self._namespace = '/' + self._namespace

        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        self._camera_info_publisher = self.create_publisher(CameraInfo,
                                                            self._namespace + 'config', 10)

        timer_period = 1.0  # [c]
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Публикация данных математической модели камеры."""
        msg = CameraInfo()
        msg.k = [3143.707075133142,
                 0.0,
                 1472.5466182777634,
                 0.0,
                 3156.761966184623,
                 1956.168580192791,
                 0.0,
                 0.0,
                 1.0
                 ]

        msg.d = [
            0.16875903232491504,
            -1.6926366325911733,
            -0.0027783866020407743,
            0.0013668182615799744,
            4.3384439380592745
        ]
        self._camera_info_publisher.publish(msg)

        if self._debug:
            self.get_logger().info('Send test camera info')


def main(args=None):
    """
    Инициализация ноды и запуск.

    Args:
    ----
        args (any, optional): входные параметры. Defaults to None.

    """
    rclpy.init(args=args)

    try:
        odom_pub = CameraConfigPub()
        rclpy.spin(odom_pub)
    except Exception as e:
        print('CameraConfigPub::Exception ' + str(e))

    # Заглушка что бы нода не вылетала с ошибкой после Ctrl+C
    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            odom_pub.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
