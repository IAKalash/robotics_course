# Дерево систем координат (tf2 frames) модели Ackermann-робота

## Обзор структуры дерева преобразований

Корень дерева: `base_link` (REP-105).
Все сенсоры (`camera_link`, `lidar_link`, `imu_link`) жестко закреплены на несущем корпусе `base_link`.
Рулевое управление реализовано по схеме Аккермана: поворотные кулаки (`front_left_knuckle`, `front_right_knuckle`) вращаются вокруг вертикальной оси $Z$, а передние колёса вращаются вокруг горизонтальной оси $Y$ внутри кулаков. Задние колёса закреплены непосредственно на `base_link` и вращаются вокруг оси $Y$.

---

## Таблица связей и владельцев публикации

| Родительский фрейм (`parent`) | Дочерний фрейм (`child`) | Тип сочленения (`joint`) | Ожидаемое смещение $[x, y, z]$ (м) | Ожидаемая ориентация $[r, p, y]$ (рад) | Владелец публикации (Node) | Топик публикации |
|---|---|---|---|---|---|---|
| `base_link` | `camera_link` | `fixed` | `[0.28, 0.0, 0.38]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` | `/tf_static` |
| `base_link` | `lidar_link` | `fixed` | `[0.28, 0.0, 0.38]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` | `/tf_static` |
| `base_link` | `imu_link` | `fixed` | `[0.28, 0.0, 0.38]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` | `/tf_static` |
| `base_link` | `front_left_knuckle` | `revolute` | `[0.40, 0.31, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |
| `base_link` | `front_right_knuckle` | `revolute` | `[0.40, -0.31, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |
| `front_left_knuckle` | `front_left_wheel` | `continuous` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |
| `front_right_knuckle` | `front_right_wheel` | `continuous` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |
| `base_link` | `rear_left_wheel` | `continuous` | `[-0.40, 0.31, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |
| `base_link` | `rear_right_wheel` | `continuous` | `[-0.40, -0.31, 0.0]` | `[0.0, 0.0, 0.0]` | `robot_state_publisher` (по `/joint_states`) | `/tf` |

---

## Правила владения и предотвращения конфликтов
1. **Единственный владелец (Single Publisher Rule)**:
   Каждое статическое преобразование фиксированных звеньев сенсоров (`camera_link`, `lidar_link`, `imu_link`) публикуется исключительно узлом `robot_state_publisher` на топике `/tf_static`. Запрещено одновременное использование сторонних статических бродкастеров (`tf2_ros static_transform_publisher`), приводящее к конфликтам в буфере `tf2`.
2. **Динамические сочленения**:
   Подвижные звенья колес и рулевых кулаков транслируются в `/tf` узлом `robot_state_publisher` только при наличии входящих данных `/joint_states`.
