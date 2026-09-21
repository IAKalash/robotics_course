# ПР01: Граф ROS 2 и изоляция доменов

## 1. Исправный граф (домен 16)

### Список нод и их назначение
Команда:
```bash
ros2 node list --no-daemon --spin-time 2
```
Вывод:
```text
/teleop_turtle
/turtlesim
```

- `/turtlesim` — окно симулятора с черепахой, принимает команды движения, считает физику и публикует позу.
- `/teleop_turtle` — узел управления с клавиатуры, ловит стрелки в терминале и шлет скорость в `/turtle1/cmd_vel`.

---

### Топики и типы
Команда:
```bash
ros2 topic list -t
```
Вывод:
```text
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

- `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) — управляющие команды скорости (телеоп -> черепаха).
- `/turtle1/pose` (`turtlesim/msg/Pose`) — координаты x, y, угол theta и скорость черепахи.
- `/turtle1/color_sensor` (`turtlesim/msg/Color`) — цвет фона под черепахой.
- `/rosout` и `/parameter_events` — стандартные служебные топики ROS 2.

---

### Инфо по ноде /turtlesim
Команда:
```bash
ros2 node info /turtlesim
```
Вывод:
```text
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

---

### Проверка позы и частоты
Тип:
```bash
ros2 topic type /turtle1/pose
```
Вывод:
```text
turtlesim/msg/Pose
```

Одно сообщение:
```bash
ros2 topic echo /turtle1/pose --once
```
Вывод:
```text
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

Замер частоты (крутилась около 12 секунд):
```bash
ros2 topic hz /turtle1/pose
```
Вывод:
```text
average rate: 62.502
	min: 0.013s max: 0.019s std dev: 0.00053s window: 631
```
Частота получилась стабильно ~62.5 Гц (в turtlesim таймер тикает каждые 16 мс: 1 / 0.016 = 62.5 Гц).

---

## 2. Разрыв связи и восстановление

### Сравнение состояний: До / Сбой / После

| Стадия | Домен симулятора | Домен телеопа | Домен терминала C | Видимые ноды в C | Чтение /turtle1/pose | Код выхода |
|---|---|---|---|---|---|---|
| **До** | 16 | 16 | 16 | `/teleop_turtle`, `/turtlesim` | Получено | 0 |
| **Сбой** | 16 | 17 | 17 | `/teleop_turtle` | Таймаут 5с | 124 |
| **После** | 16 | 16 | 16 | `/teleop_turtle`, `/turtlesim` | Получено | 0 |

---

### Сбой (домен 17)
В терминале B перезапустили teleop с `export ROS_DOMAIN_ID=17`.
В терминале C:
```bash
export ROS_DOMAIN_ID=17
ros2 node list --no-daemon --spin-time 2
timeout 5s ros2 topic echo /turtle1/pose "turtlesim/msg/Pose" --once
printf 'exit=%s\n' "$?"
```
Вывод:
```text
exit=124
```
В домене 17 виден только телеоп, черепаху не видно, данных нет, отвалилось по таймауту с кодом 124.

---

### Восстановление (домен 16)
В терминале B перезапустили teleop обратно в домен 16.
В терминале C:
```bash
export ROS_DOMAIN_ID=16
ros2 node list --no-daemon --spin-time 2
timeout 5s ros2 topic echo /turtle1/pose "turtlesim/msg/Pose" --once
printf 'exit=%s\n' "$?"
```
Вывод:
```text
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
exit=0
```
Обе ноды снова видны, поза сразу пришла, код 0. Черепаха снова слушается стрелок.

---

### Почему так произошло (Root cause)
DDS discovery ищет другие ноды только в рамках своего `ROS_DOMAIN_ID`. Ноды с разными domain id друг друга в сети не видят.
Переменная `ROS_DOMAIN_ID` применяется только в момент старта процесса ноды при инициализации DDS. Команда `export` в терминале меняет переменную среды только для будущих команд этого терминала, но не может на лету переключить уже работающий процесс. Поэтому teleop пришлось перезапустить с новым доменом, а turtlesim все это время спокойно работал в домене 16.
