# Курс «Робототехника · ROS 2»

Репозиторий учебных практических работ по курсу «Робототехника» (НГУ).

## Среда выполнения
- **ОС:** Ubuntu 24.04.4 LTS (Noble Numbat), x86_64
- **ROS 2:** Jazzy Jalisco (`desktop`)
- **RMW:** `rmw_fastrtps_cpp`
- **Gazebo:** Harmonic 8.11.0

---

## ПР05. Launch, QoS и повтор записи

### Цель работы
Поднятие системы одной launch-командой в пространстве имён `/robot_a`, запись сенсорного потока в формат MCAP и подтверждение обнаружения и исправления несовместимых профилей QoS (`reliable` подписчик против `best_effort` издателя) на живом потоке и при повторе записи (replay).

### Структура пакетов
- `course_bringup` — пакет запуска и ретрансляции данных:
  - `launch/patrol.launch.py` — запуск системы с аргументами `namespace`, `subscriber_reliability`, `record`, `bag_dir`, группировкой по namespace, переназначениями топиков и условным запуском recorder;
  - `course_bringup/pose_relay.py` — нода `pose_relay`: читает позу из `input_pose` и публикует в относительный `sensor_pose` с QoS `best_effort`;
  - `config/qos-overrides.yaml` — профиль переопределения QoS для воспроизведения rosbag2.
- `patrol` — пакет управления движением:
  - `patrol/patrol_node.py` — нода `patrol` с настраиваемой надёжностью подписки (`subscriber_reliability`) и относительными топиками (`pose`, `cmd_vel`).

### Сборка и запуск
1. **Сборка пакетов:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Запуск симулятора turtlesim (глобально):**
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

3. **Запуск системы до исправления (несовместимый QoS: `subscriber_reliability:=reliable`):**
   ```bash
   ros2 launch course_bringup patrol.launch.py namespace:=robot_a subscriber_reliability:=reliable record:=false
   ```
   Проверка отсутствия приёма сообщений:
   ```bash
   timeout 3s ros2 topic echo /robot_a/sensor_pose --qos-reliability reliable --once
   ```

4. **Запуск системы после исправления (совместимый QoS: `subscriber_reliability:=best_effort`) и запись в MCAP:**
   ```bash
   ros2 launch course_bringup patrol.launch.py namespace:=robot_a subscriber_reliability:=best_effort record:=false
   ```
   Проверка успешного приёма сообщений:
   ```bash
   timeout 3s ros2 topic echo /robot_a/sensor_pose --qos-reliability best_effort --once
   ```
   Запись топика в MCAP:
   ```bash
   ros2 bag record -s mcap -o evidence/pr05/bag --topics /robot_a/sensor_pose
   ```

5. **Воспроизведение записи (Replay) с переопределением QoS:**
   ```bash
   ros2 bag play evidence/pr05/bag --qos-profile-overrides-path config/qos-overrides.yaml
   ros2 bag info evidence/pr05/bag
   ```

---

## Проверка сдачи (Course Kit Checker)
```bash
python3 .course-kit/v1/tools/check_practice.py PR05 --submission .
```
