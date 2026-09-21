# Курс «Робототехника · ROS 2»

Репозиторий учебных практических работ по курсу «Робототехника» (НГУ).

## Среда выполнения
- **ОС:** Ubuntu 24.04.4 LTS (Noble Numbat), x86_64, Linux 7.0.0-31-generic
- **ROS 2:** Jazzy Jalisco (`desktop`)
- **RMW:** `rmw_fastrtps_cpp`
- **Gazebo:** Harmonic 8.11.0

---

## ПР01. Окружение и граф ROS 2

### Цель работы
Запуск готовой системы `turtlesim`, исследование взаимодействия узлов и топиков, измерение частоты публикации позы, воспроизведение дефекта изоляции при разделении доменов (`ROS_DOMAIN_ID`), доказательство восстановления связи и формирование отчёта evidence.

### Порядок запуска и воспроизведения

#### 1. Подготовка терминалов и настройка домена
В каждом терминале выполняется подключение окружения ROS 2 Jazzy и установка базового домена:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=16
```

#### 2. Запуск симулятора и телеуправления
- **Терминал 1 (симулятор):**
  ```bash
  ros2 run turtlesim turtlesim_node
  ```
- **Терминал 2 (клавиатурное управление):**
  ```bash
  ros2 run turtlesim turtle_teleop_key
  ```

#### 3. Наблюдение и исследование графа (Терминал 3)
```bash
ros2 node list --no-daemon --spin-time 2
ros2 topic list -t
ros2 node info /turtlesim
ros2 topic type /turtle1/pose
ros2 topic echo /turtle1/pose --once
ros2 topic hz /turtle1/pose
```

#### 4. Разрыв связи (воспроизведение дефекта)
Остановить `turtle_teleop_key` в Терминале 2 и перезапустить его в домене 17:
```bash
export ROS_DOMAIN_ID=17
ros2 run turtlesim turtle_teleop_key
```

В Терминале 3 проверить отсутствие связи с симулятором (таймаут 5 секунд, `exit=124`):
```bash
export ROS_DOMAIN_ID=17
ros2 node list --no-daemon --spin-time 2
timeout 5s ros2 topic echo /turtle1/pose "turtlesim/msg/Pose" --once
printf 'exit=%s\n' "$?"
```

#### 5. Восстановление связи (доказательство устранения)
Перезапустить `turtle_teleop_key` в исходном домене 16. В Терминале 3 проверить успешное получение данных (`exit=0`):
```bash
export ROS_DOMAIN_ID=16
ros2 node list --no-daemon --spin-time 2
timeout 5s ros2 topic echo /turtle1/pose "turtlesim/msg/Pose" --once
printf 'exit=%s\n' "$?"
```

---

## Автоматическая проверка (Course Kit Checker)
Локальная проверка evidence с помощью чекера курса:
```bash
python3 .course-kit/v1/tools/check_practice.py PR01 --submission .
```
Проверка корректности JSON:
```bash
python3 -m json.tool evidence/pr01/environment.json > /dev/null
python3 -m json.tool evidence/pr01/report.json > /dev/null
```
