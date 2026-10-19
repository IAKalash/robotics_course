# Курс «Робототехника · ROS 2»

Репозиторий учебных практических работ по курсу «Робототехника» (НГУ).

## Среда выполнения
- **ОС:** Ubuntu 24.04.4 LTS (Noble Numbat), x86_64
- **ROS 2:** Jazzy Jalisco (`desktop`)
- **RMW:** `rmw_fastrtps_cpp`
- **Gazebo:** Harmonic 8.11.0

---

## ПР06. Модель робота, tf2 и QoS преобразований

### Цель работы
Сборка параметризованной модели Ackermann-робота в формате URDF/Xacro, объединение корпуса (`base_link`), рулевых кулаков, колес и сенсоров (`camera_link`, `lidar_link`, `imu_link`) в единое связное дерево `tf2`. Исследование поведения топика статических трансформаций `/tf_static` при различных политиках QoS (`transient_local` против `volatile`), воспроизведение дефекта некорректного родительского фрейма (`camera_link` вместо `base_link`) и его исправление.

### Структура пакета `course_description`
- `urdf/vehicle.urdf.xacro` — параметризованное Xacro-описание модели Ackermann (колесная база 0.8 м, колея 0.62 м, радиус колеса 0.12 м, корпус, сенсоры, рулевые кулаки и колеса);
- `launch/display.launch.py` — запуск `robot_state_publisher` с разворачиванием Xacro через `Command` и опциональным открытием `rviz2`;
- `test/test_urdf.py` — автоматические тесты:
  - корректность разворачивания Xacro в URDF XML;
  - проверка через `check_urdf`;
  - проверка родительских связей сенсоров (`base_link`);
  - проверка геометрических параметров (wheelbase, track_width).

### Сборка и проверка
1. **Сборка пакета:**
   ```bash
   colcon build --symlink-install --packages-select course_description
   source install/setup.bash
   ```

2. **Проверка развертывания модели:**
   ```bash
   xacro src/course_description/urdf/vehicle.urdf.xacro > /tmp/vehicle.urdf
   check_urdf /tmp/vehicle.urdf
   ```

3. **Запуск тестов:**
   ```bash
   colcon test --packages-select course_description
   colcon test-result --verbose
   ```

4. **Запуск `robot_state_publisher`:**
   ```bash
   ros2 launch course_description display.launch.py
   ```

5. **Проверка связей и QoS дерева tf2:**
   ```bash
   ros2 run tf2_ros tf2_echo base_link lidar_link
   ros2 topic echo /tf_static --once
   ```

---

## Проверка сдачи (Course Kit Checker)
```bash
python3 .course-kit/v1/tools/check_practice.py PR06 --submission .
```

