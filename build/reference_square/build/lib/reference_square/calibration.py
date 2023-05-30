import json
import copy
import numpy as np


class CameraCalibFile:
    """Класс реализующий чтение файла каллибровки камеры с помощью 
    шахматной доски.
    """

    def __init__(self):
        self._mtx = None
        self._dist = None
        self._conv_mtx = None

    def load_calibration(self, mtx, dist):
        """Загрузить данные калибровки из файла формата pickle или json.

        Parameters
        ----------
        path_to_calibration :str
            строка содержащая путь к файлу с калибровкой камеры(формата
            pickle или json).
        """

        self._mtx = mtx
        self._dist = dist


    def get_mtx(self):
        """Получить матрицу описывающую проекционную модель камеры.
        (см. http://opencv.jp/opencv-2.1_org/py/
        camera_calibration_and_3d_reconstruction.html)

        Returns
        -------
        list
            двумерная матрица, описывающая модель камеры.
        """
        return self._mtx

    def get_dist(self):
        """Получить вектор дисперсий, описывающих искажения нашей камеры.
        (см. http://opencv.jp/opencv-2.1_org/py/
        camera_calibration_and_3d_reconstruction.html)

        Returns
        -------
        list
            вектор дисперсий, описывающих искажения нашей камеры
        """
        return self._dist

    def create_conversion_mtx(self):
        # копируем матрицу камеры
        self._conv_mtx = copy.deepcopy(self._mtx)
        for row in range(len(self._conv_mtx)):
            self._conv_mtx[row].append(0)
        
        self._conv_mtx = np.array(self._conv_mtx)


        
    def coords_conversion(self, world_coords, R=None):
        world_coords.append(1)
        homo_word_coords = np.array(world_coords)[:,None]
        
        if R is not None:
            homo_word_coords = np.dot(R, homo_word_coords)
        
        image_coords = np.dot(self._conv_mtx, homo_word_coords).flatten()
        image_coords /= image_coords[-1]
        image_coords = image_coords[:2].astype(int)

        
        return image_coords
    
    def get_Cxy(self):
        Cxy = np.array([self._mtx[0][2],
                            self._mtx[1][2]])
                            
        
        return Cxy
    
def get_Rx(x_axis_angle):
    x_axis_angle = np.radians(x_axis_angle)
    
    Rx = [[1, 0 , 0],
          [0, np.cos(x_axis_angle), -np.sin(x_axis_angle)],
          [0, np.sin(x_axis_angle), np.cos(x_axis_angle)]]
    
    Rx = np.array(Rx)
    
    return Rx

def get_Ry(y_axis_angle):
    y_axis_angle = np.radians(y_axis_angle)

    Ry = [[np.cos(y_axis_angle), 0, np.sin(y_axis_angle)],
          [0, 1, 0],
          [-np.sin(y_axis_angle), 0, np.cos(y_axis_angle)],
    ]
    
    Ry = np.array(Ry)

    return Ry

def get_RT(Rx, Ry, T):

    R = Rx.dot(Ry)
    R = np.hstack([R, T])
    row = [0,0,0,1]
    R = np.vstack([R, row])
    return R
    
        