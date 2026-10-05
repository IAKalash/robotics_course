# Курс «Робототехника · ROS 2»

Репозиторий учебных практических работ по курсу «Робототехника» (НГУ).

## Среда выполнения
- **ОС:** Ubuntu 24.04.4 LTS (Noble Numbat), x86_64
- **ROS 2:** Jazzy Jalisco (`desktop`)
- **RMW:** `rmw_fastrtps_cpp`
- **Gazebo:** Harmonic 8.11.0

---

## ПР04. Параметризуем движение

### Цель работы
Расширение ноды `patrol` динамическими параметрами ROS 2 (`linear_speed`, `turn_rate`, `publish_hz`). Добавление параметрического callback для валидации входящих значений без разрушения активного состояния ноды, динамическое пересоздание таймера при изменении частоты публикации, воспроизведение отказа при отключенной валидации (деление на ноль) и доказательство сохранения работоспособности при включенной валидации. Исследование интерфейсов сервисов (`/clear`) и экшенов (`/turtle1/rotate_absolute`).

### Структура пакета `patrol`
- `package.xml` — манифест пакета и зависимости (`rclpy`, `geometry_msgs`, `turtlesim`).
- `setup.py` — правила установки пакета и точки входа скриптов (`patrol`, `draw_route`).
- `patrol/patrol_node.py` — нода `PatrolNode` с объявлением параметров и callback валидации.
- `test/test_patrol.py` — модульные тесты для чистых функций валидации и вычисления скорости.

### Сборка и запуск
1. **Сборка пакета и запуск тестов:**
   ```bash
   colcon build --symlink-install --packages-select patrol
   source install/setup.bash
   python3 -m pytest src/patrol/test
   ```

2. **Запуск симулятора:**
   ```bash
   ros2 launch turtle_bringup sim.launch.py
   ```

3. **Запуск ноды патрулирования (с переназначением топика):**
   ```bash
   ros2 run patrol patrol --ros-args -r cmd_vel:=/turtle1/cmd_vel
   ```

4. **Динамическое изменение параметров в рантайме:**
   ```bash
   # Изменение частоты публикации на 5 Гц:
   ros2 param set /patrol publish_hz 5.0

   # Попытка установки недопустимого значения (будет отклонена):
   ros2 param set /patrol publish_hz 0.0
   ```

---

## Проверка сдачи (Course Kit Checker)
```bash
python3 .course-kit/v1/tools/check_practice.py PR04 --submission .
```
