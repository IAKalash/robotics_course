# Курс «Робототехника · ROS 2»

Репозиторий учебных практических работ по курсу «Робототехника» (НГУ).

## Среда выполнения
- **ОС:** Ubuntu 24.04.4 LTS (Noble Numbat), x86_64
- **ROS 2:** Jazzy Jalisco (`desktop`)
- **RMW:** `rmw_fastrtps_cpp`
- **Gazebo:** Harmonic 8.11.0

---

## ПР03. Первая нода: поза и команда

### Цель работы
Реализация собственной ноды `patrol` на Python (`rclpy`), подписывающейся на позу черепахи (`/turtle1/pose`) и публикующей команды скорости (`geometry_msgs/msg/Twist`) по таймеру с частотой 10 Гц в относительный топик `cmd_vel`. Вынос логики принятия решений в чистую функцию, покрытие её юнит-тестами (`pytest`), воспроизведение дефекта разрыва связи из-за относительного имени топика и доказательство восстановления управления через remap (`-r cmd_vel:=/turtle1/cmd_vel`).

### Структура пакета `patrol`
- `package.xml` — метаданные пакета и зависимости (`rclpy`, `geometry_msgs`, `turtlesim`).
- `setup.py` — конфигурация установки и точка входа консольного скрипта `patrol`.
- `patrol/patrol_node.py` — нода `PatrolNode` и чистая функция `compute_cmd_vel`.
- `test/test_patrol.py` — модульные тесты для чистой функции.

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

4. **Проверка частоты публикации (10 Гц):**
   ```bash
   ros2 topic hz /turtle1/cmd_vel --window 100
   ```

### Дополнительно: Маршрут мышью (`draw_route`)
Запуск графического интерфейса рисования траектории:
```bash
ros2 run patrol draw_route
```

---

## Проверка сдачи (Course Kit Checker)
```bash
python3 .course-kit/v1/tools/check_practice.py PR03 --submission .
```
