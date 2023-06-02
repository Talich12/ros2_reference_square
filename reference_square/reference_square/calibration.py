import copy
import numpy as np


class CameraCalibFile:
    """Класс реализующий чтение файла каллибровки камеры с помощью шахматной доски."""

    def __init__(self):
        self._mtx = None
        self._dist = None
        self._conv_mtx = None

    def load_calibration(self, mtx, dist):
        """
        Сохранение матрицы проекционной модели камеры и коэффициенты дисторсии

        Args:
            mtx (float[3][3]): Матрица искажения
            dist (float[5]): Коэффициенты дисторсии
        """

        self._mtx = mtx
        self._dist = dist

    def get_mtx(self):
        """
        Получить матрицу описывающую проекционную модель камеры.

        Returns:
            float[3][3]: Матрица проекционной модели камеры
        """

        return self._mtx

    def get_dist(self):
        """
        Получить коэффициенты дистросии камеры

        Returns:
            float[5]: Коэффициенты дисторсии
        """

        return self._dist

    def create_conversion_mtx(self):
        """
        Перевести матрицу 3x3 в матрицу 3x4 для работы в гомогенными координатами
        """

        # копируем матрицу камеры
        self._conv_mtx = copy.deepcopy(self._mtx)
        for row in range(len(self._conv_mtx)):
            self._conv_mtx[row].append(0)

        self._conv_mtx = np.array(self._conv_mtx)

    def coords_conversion(self, world_coords, R=None):
        """
        Перевод мировых координат в координаты камеры

        Args:
            world_coords (float[3]): Координаты точки в мировых координатах (x, y, z)
            R (float[4][4], optional): Общая матрица преоброзаваний. Defaults to None.

        Returns:
            float[2]: Координаты точки в координатах камеры_
        """

        world_coords.append(1)
        homo_word_coords = np.array(world_coords)[:, None]

        if R is not None:
            homo_word_coords = np.dot(R, homo_word_coords)

        image_coords = np.dot(self._conv_mtx, homo_word_coords).flatten()
        image_coords /= image_coords[-1]
        image_coords = image_coords[:2].astype(int)

        return image_coords

    def get_Cxy(self):
        """
        Получить коэффициенты смещения из матрицы проекционной модели камеры

        Returns:
            float[2]: Массив с коэффициентами смещения
        """

        Cxy = np.array([self._mtx[0][2], self._mtx[1][2]])

        return Cxy


def get_Rx(x_axis_angle):
    """
    Получить матрицу поворота по оси x

    Args:
        x_axis_angle (float): Значение угла поворота по оси x в градусах

    Returns:
        float[3][3]: Матрица поворота по оси x 
    """

    x_axis_angle = np.radians(x_axis_angle)

    Rx = [[1, 0, 0],
          [0, np.cos(x_axis_angle), -np.sin(x_axis_angle)],
          [0, np.sin(x_axis_angle), np.cos(x_axis_angle)]]

    Rx = np.array(Rx)

    return Rx


def get_Ry(y_axis_angle):
    """
    Получить матрицу поворота по оси y

    Args:
        y_axis_angle (float): Значение угла поворота по оси y в градусах

    Returns:
        float[3][3]: Матрица поворота по оси y
    """

    y_axis_angle = np.radians(y_axis_angle)

    Ry = [[np.cos(y_axis_angle), 0, np.sin(y_axis_angle)],
          [0, 1, 0],
          [-np.sin(y_axis_angle), 0, np.cos(y_axis_angle)],
          ]

    Ry = np.array(Ry)

    return Ry


def get_RT(Rx, Ry, T):
    """
    Получить общую матрицу преоброзований в гомогенных координатах

    Args:
        Rx (float[3][3]): Матрица поворота по оси x
        Ry (float[3][3]): Матрица поворота по оси y
        T (float[3][1]): Дополнительная матрица смещения

    Returns:
        float[4][4]: Общая матрица преоброзований в гомогенных координатах
    """

    R = Rx.dot(Ry)
    R = np.hstack([R, T])
    row = [0, 0, 0, 1]
    R = np.vstack([R, row])
    return R
