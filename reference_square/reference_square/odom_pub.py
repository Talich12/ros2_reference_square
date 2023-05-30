import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry



class OdomPub(Node):
    def __init__(self):
        super().__init__('odom_pub')
        self._publisher_ = self.create_publisher(Odometry, '/solaster/odom', 10)
        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.
        msg.pose.pose.position.y = 2.
        msg.pose.pose.position.z = 2.
        msg.pose.pose.orientation.x = 0.1727661
        msg.pose.pose.orientation.y = 0.1727661
        msg.pose.pose.orientation.z = 0.
        msg.pose.pose.orientation.w = 0.9696926
        self.get_logger().info(f'{msg}')
        self._publisher_.publish(msg)



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