# Построение проекционного квадрата на основе ROS2
---
## Подготовка
Для того чтобы спроекцировать квадрат из мировых координат в координаты камеры нам понадобится:
1. Растояние от камеры до плоскости (В данном случае этим является растояние аппарата до дна)
    ```python
    pose = [pose_data.position.x, pose_data.position.y, pose_data.position.z]
    ```
    pose_data.position.z - расстояние до дна
    
2. Положение аппарата в пространстве
    ```python
    # Переводим ориентацию в массив в формате [w,x,y,z] чтобы перевести кватернион в повороты Эйлера
        orientation = [pose_data.orientation.w, pose_data.orientation.x, pose_data.orientation.y, pose_data.orientation.z]
        degrees = []
        # Преобразовываем орентацию в повороты Эйлера и переводим из радиан в градусы
        angles = euler.quat2euler(orientation)
        for angle in angles:
            degrees.append(math.degrees(angle))

        self._euler_angles = degrees
    ```
    Для работы с матрицей поворота мы должны преброзовать кватернион в повороты Эйлера и указать их в градусах
    
3. Матрица искожения и дисторсия камеры
    ```python
    # Переводим массив искажения msg.k [float[9]] в матрицу 3x3 
        mtx = [[msg.k[0], msg.k[1], msg.k[2]],
               [msg.k[3], msg.k[4], msg.k[5]],
               [msg.k[6], msg.k[7], msg.k[8]]]
        # Принимаем дисторсию msg.d [float[5]]
        self._distortion = msg.d
        self._convert_mtx = mtx
    ```
    
4. Само изображение с камеры
    ```python
    # Преобразовываем msg (Image) в cv2 объект
        self._image = self._br.imgmsg_to_cv2(msg)
    ```
    Для работы с изображением нужно преоброзовать его в объект cv2
    
5. Координаты квадрата
    ```python
    self._projection_square_side = 1
    
    half_size = self._projection_square_side / 2

        # Задаем точки квадрата в мировых координатах
        r0_world = [0, 0, 0]
        r1_world = [-half_size, -half_size, 0]
        r2_world = [half_size, -half_size, 0]
        r3_world = [half_size, half_size, 0]
        r4_world = [-half_size, half_size, 0]
    ```
    
## Построение матрицы преоброзований

Для дальнейшего понимания нужно знать теорию гомогенных (однородных) координат 
Подробнее [здесь](http://opengl-tutorial.blogspot.com/p/3.html)

### Порядок действий:

1. Построить матрицы поворота по **x** и **y** (В этом примере именно по этим координатым так как камера находится перпендикулярно вертикальной оси, поэтому поворот по **z** можно не учитывать)
    ```python
        Rx = calibration.get_Rx(self._euler_angles[0])
        Ry = calibration.get_Ry(self._euler_angles[1])
    ```
    Функиции выглядят следущим образом
    ```python
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
    ```
    
2. Построить дополнительную матрицу перемещений (в данном случае простио вызуализация расстояния до дна ввиде матрицы)
    ```python
    # Переводим в расстояние до дна в матрицу 1x3
    T = np.array([0, 0, self._pose[2],])[:, None]
    ```
    ![Image](https://habrastorage.org/files/68c/2ae/5bc/68c2ae5bca1d40438498f01c063b4fcd.png "icon")
3. Построение общей матрицы преоброзований
    ```python
    R = calibration.get_RT(Rx, Ry, T)
    
    def get_RT(Rx, Ry, T):

    R = Rx.dot(Ry)
    R = np.hstack([R, T])
    row = [0,0,0,1]
    R = np.vstack([R, row])
    return R
    ```
    Если учитывать что мы не используем Rz наша матрица будет выглядить следущим образом:
    ![Image](https://habrastorage.org/files/d63/846/156/d638461566f14757b25aa70528d3509f.png "icon")
    
    В данном случае перемножаем обе матрицы поворота, и присоединям справа матрицу перемещений. В конце переводим матрицу к гомогенному ввиду, тоесть снизу ряд [0,0,0,1]
    
4. Построение матрицы искожений
    ```python
    camera_calibration.load_calibration(self._convert_mtx, self._distortion)
    
    camera_calibration.create_conversion_mtx()
    
    def create_conversion_mtx(self):
        # копируем матрицу камеры
        self._conv_mtx = copy.deepcopy(self._mtx)
        for row in range(len(self._conv_mtx)):
            self._conv_mtx[row].append(0)
        
        self._conv_mtx = np.array(self._conv_mtx)
    ```
    Копируем нашу матрицу искажений и добавляем нули
    
5. Преоброзование всех точек квадрата с помощью матричного умножения
    ```python
    # Преобразуем мировые координаты квадрата в координаты камеры путем перемножения матриц искажения и преоброзований
    r0_frame = camera_calibration.coords_conversion(r0_world, R)
    r1_frame = camera_calibration.coords_conversion(r1_world, R)
    r2_frame = camera_calibration.coords_conversion(r2_world, R)
    r3_frame = camera_calibration.coords_conversion(r3_world, R)
    r4_frame = camera_calibration.coords_conversion(r4_world, R)
    
    def coords_conversion(self, world_coords, R=None):
    world_coords.append(1)
    homo_word_coords = np.array(world_coords)[:,None]
    
    if R is not None:
        homo_word_coords = np.dot(R, homo_word_coords)
    
    image_coords = np.dot(self._conv_mtx, homo_word_coords).flatten()
    image_coords /= image_coords[-1]
    image_coords = image_coords[:2].astype(int)

    
    return image_coords
    ```
    
    image_coords /= image_coords[-1]
    После перемножения мы делим все координаты на w чтобы привести их к стандартизированному виду
    
6. Калибруем точки чтобы они находились относительно центра изображения
    ```python
    c_frame = camera_calibration.get_Cxy()
    
    def get_Cxy(self):
        Cxy = np.array([self._mtx[0][2],
                            self._mtx[1][2]])
                            
        
        return Cxy
    ```
    Находим центр изображения изходя из матрицы искажения
    ```python
     # Колибруем все точки
        polyline_pts = [(r_frame - trans_vec).astype(int) for r_frame in polyline_pts]
    ```
    
7. Отображение квадрата
    ```python
    polyline_pts = np.array(polyline_pts)

        # Рисуем проекционный квадрат на изображении
        img = self._image
        img = cv.polylines(img, [polyline_pts], True, (0, 255, 0), 2)
        cv.imwrite('reference_square.jpg', img)
    ```





    
