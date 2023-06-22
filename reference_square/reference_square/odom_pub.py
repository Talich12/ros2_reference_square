"""Файл публикации навигационных данных."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from transforms3d import euler, quaternions
import math


class OdomPub(Node):
    """
    Класс ноды публикации данных позиции и ориенации.

    Args:
    ----
        Node (Node): ROS2 нода.

    """

    def __init__(self):
        """Инициализация объектов."""
        super().__init__('test_odom_publisher',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self._namespace = self.get_namespace()
        if self._namespace != '':
            if self._namespace[-1] != '/':
                self._namespace += '/'

            if self._namespace[0] != '/':
                self._namespace = '/' + self._namespace

        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        self._publisher_ = self.create_publisher(Odometry,
                                                 self._namespace + 'odom',
                                                 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        # Принимаем параметры из reference_square.launch.py
        self._start_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value

        if len(self._start_pos) != 3:
            self.get_logger().info("Параметр start_pos не задан, устанавливается значение [0, 0, 0]")
            self._start_pos = [0., 0., 0.]

        self._step_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value

        if len(self._step_pos) != 3:
            self.get_logger().info("Параметр step_pos не задан, устанавливается значение [0, 0, 0]")
            self._step_pos = [0., 0., 0.]

        # Перевод градусов в радианы
        for i in range(0, 2):
            self._start_pos[i] = math.radians(self._start_pos[i])
            self._step_pos[i] = math.radians(self._step_pos[i])

        # Перевод поворота в кватернион
        self._start_orientation_quat = euler.euler2quat(self._start_pos[0],
                                                        self._start_pos[1],
                                                        0)

        self._step_orientation_quat = euler.euler2quat(self._step_pos[0],
                                                       self._step_pos[1],
                                                       0)

    def timer_callback(self):
        """Публикация данных позиции и ориентации."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.
        msg.pose.pose.position.y = 2.
        msg.pose.pose.position.z = self._start_pos[2]
        msg.pose.pose.orientation.x = self._start_orientation_quat[1]
        msg.pose.pose.orientation.y = self._start_orientation_quat[2]
        msg.pose.pose.orientation.z = self._start_orientation_quat[3]
        msg.pose.pose.orientation.w = self._start_orientation_quat[0]

        # Умножение кватернионов чтобы сохранить результат шага
        self._start_orientation_quat = quaternions.qmult(self._start_orientation_quat,
                                                         self._step_orientation_quat)

        self._start_pos[2] += self._step_pos[2]

        self._publisher_.publish(msg)

        if self._debug:
            self.get_logger().info('Send test odometry data')


def main(args=None):
    """
    Инициализация ноды и запуск.

    Args:
    ----
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
