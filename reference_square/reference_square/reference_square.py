import rclpy
from . import calibration

from transforms3d import euler
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Polygon, Point32
from nav_msgs.msg import Odometry

import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge


class ReferenceSquareNode(Node):
    def __init__(self):
        super().__init__('reference_square')
        self._br = CvBridge()
        self._projection_square_side = 1  # В метрах
        self._id = id

        self._сamera_info_subscription = self.create_subscription(
            CameraInfo, '/solaster/front_camera/config', self.camera_info_callback, 10)
        self._image_subscription = self.create_subscription(
            Image, '/solaster/front_camera/image', self.image_callback, 10)
        self._odom_subscription = self.create_subscription(
            Odometry, '/solaster/odom', self.odom_callback, 10)

        self._image_publisher = self.create_publisher(Image, "topic_image", 10)
        self._polygon_publisher = self.create_publisher(Polygon, 'topic_polygon', 10)

        timer_period = 1  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        self._euler_angles = []
        self._convert_mtx = []
        self._image = []

    def odom_callback(self, msg: Odometry):
        """
        Получить данные положения апарата в пространстве

        Args:
            msg (Odometry): Данные о позиции и ориентации
        """

        pose_data = msg.pose.pose
        # Переводим позицию в массив в формате [x,y,z]
        pose = [pose_data.position.x, pose_data.position.y, pose_data.position.z]
        self._pose = pose
        # Переводим ориентацию в массив в формате [w,x,y,z]
        # чтобы перевести кватернион в повороты Эйлера
        orientation = [pose_data.orientation.w, pose_data.orientation.x,
                       pose_data.orientation.y, pose_data.orientation.z]
        degrees = []
        # Преобразовываем ориентацию в повороты Эйлера и переводим из радиан в градусы
        try:
            angles = euler.quat2euler(orientation)
        except Exception:
            return

        for angle in angles:
            degrees.append(math.degrees(angle))

        self._euler_angles = degrees

        self._odom_timestump = msg._header._stamp.nanosec + msg._header._stamp.sec*(10**9)

    def camera_info_callback(self, msg: CameraInfo):
        """
        Получить данные конфига камеры

        Args:
            msg (CameraInfo): Данные конфига камеры
        """

        # Переводим массив искажения msg.k [float[9]] в матрицу 3x3
        mtx = [[msg.k[0], msg.k[1], msg.k[2]],
               [msg.k[3], msg.k[4], msg.k[5]],
               [msg.k[6], msg.k[7], msg.k[8]]]
        # Принимаем дисторсию msg.d [float[5]]
        self._distortion = msg.d
        self._convert_mtx = mtx

    def image_callback(self, msg: Image):
        """
        Получить изображение

        Args:
            msg (Image): _description_
        """

        # Преобразовываем msg (Image) в cv2 объект
        try:
            self._image = self._br.imgmsg_to_cv2(msg)
        except Exception:
            return

    def timer_callback(self):
        """Построение проекционного квадрата"""

        if len(self._euler_angles) == 0 or len(self._convert_mtx) == 0 or len(self._image) == 0:
            print("error")
            return

        time = self.get_clock().now().nanoseconds
        if self._odom_timestump + 10**9 < time:
            print("error")
            return

        msg = Polygon()

        half_size = self._projection_square_side / 2

        # Задаем точки квадрата в мировых координатах
        r0_world = [0, 0, 0]
        r1_world = [-half_size, -half_size, 0]
        r2_world = [half_size, -half_size, 0]
        r3_world = [half_size, half_size, 0]
        r4_world = [-half_size, half_size, 0]

        # Преобразовываем углы Эйлера в матрицы поворота
        Rx = calibration.get_Rx(self._euler_angles[0])
        Ry = calibration.get_Ry(self._euler_angles[1])

        # Переводим в расстояние до дна в матрицу 1x3
        T = np.array([0, 0, self._pose[2], ])[:, None]

        # Строим полную матрицу преоброзований
        R = calibration.get_RT(Rx, Ry, T)

        camera_calibration = calibration.CameraCalibFile()
        camera_calibration.load_calibration(self._convert_mtx, self._distortion)

        # Вытаскиваем элементы смещение из матрицы искажения камеры
        c_frame = camera_calibration.get_Cxy()

        # Преобразуем матрицу искажений в матрицу 3x4 для работы с гомогенными коорданатами
        camera_calibration.create_conversion_mtx()

        # Преобразуем мировые координаты квадрата в координаты камеры
        # путем перемножения матриц искажения и преоброзований
        r0_frame = camera_calibration.coords_conversion(r0_world, R)
        r1_frame = camera_calibration.coords_conversion(r1_world, R)
        r2_frame = camera_calibration.coords_conversion(r2_world, R)
        r3_frame = camera_calibration.coords_conversion(r3_world, R)
        r4_frame = camera_calibration.coords_conversion(r4_world, R)

        polyline_pts = [r1_frame, r2_frame, r3_frame, r4_frame]

        # Вычисляем переменную для правильного отображения квадрата на фото
        # (В этом случае квадрат будет ровно по центру изображения)
        trans_vec = r0_frame - c_frame

        # Переводим все точки в формат Polygon[Point32[]]
        for point in polyline_pts:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.

            msg.points.append(p)

        self._polygon_publisher.publish(msg)
        self.get_logger().info(f'send_polygon')

        # Колибруем все точки
        polyline_pts = [(r_frame - trans_vec).astype(int) for r_frame in polyline_pts]

        polyline_pts = np.array(polyline_pts)

        # Рисуем проекционный квадрат на изображении
        try:
            img = self._image
        except Exception:
            return
        img = cv.polylines(img, [polyline_pts], True, (0, 255, 0), 2)
        cv.imwrite('reference_square.jpg', img)

        self.get_logger().info(f'send_reference_square')
        self._image_publisher.publish(self._br.cv2_to_imgmsg(img, encoding='rgb8'))

        self._euler_angles = []
        self._convert_mtx = []
        self._image = []


def main(args=None):
    rclpy.init(args=args)

    try:
        reference_square = ReferenceSquareNode()
        rclpy.spin(reference_square)
    except Exception as e:
        print('ReferenceSquareNode::Exception ' + str(e))

    # Заглушка что бы нода не вылетала с ошибкой после Ctrl+C
    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            reference_square.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
