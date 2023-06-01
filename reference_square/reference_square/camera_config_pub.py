import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo


class OdomPub(Node):
    def __init__(self):
        super().__init__('test_odom_publisher')
        self._publisher_ = self.create_publisher(CameraInfo, '/solaster/front_camera/config', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
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
        self.get_logger().info(f'{msg}')
        self._publisher_.publish(msg)


def main(args=None):
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
