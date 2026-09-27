# Команды и протокол экспериментов ПР02

## 1. Разбор команд Linux

### Команда 1: `mkdir -p src evidence/pr02`
- **Точная команда**: `mkdir -p src evidence/pr02`
- **Назначение**: Создание каталогов для исходного кода пакетов (`src`) и артефактов отчёта (`evidence/pr02`). Флаг `-p` (`--parents`) позволяет создавать всю цепочку недостающих родительских директорий и не возвращает ошибку, если директория уже существует.
- **Результат**: Созданы необходимые каталоги в корне репозитория без ошибок.

### Команда 2: `ros2 pkg prefix turtlesim`
- **Точная команда**: `ros2 pkg prefix turtlesim`
- **Назначение**: Определение пути к префиксу установки пакета `turtlesim` (где установлены его исполняемые файлы, разделяемые ресурсы и файлы описания).
- **Результат**: Выведен абсолютный путь установки: `/opt/ros/jazzy` (системная установка ROS 2).

### Команда 3: `colcon build --symlink-install --packages-select turtle_bringup 2>&1 | tee evidence/pr02/build.txt`
- **Точная команда**: `colcon build --symlink-install --packages-select turtle_bringup 2>&1 | tee evidence/pr02/build.txt`
- **Назначение**: Сборка только пакета `turtle_bringup` с использованием символических ссылок для Python-скриптов и ресурсов (`--symlink-install`). Перенаправление потока ошибок в стандартный вывод (`2>&1`) и передача через конвейер (`|`) утилите `tee` для одновременного отображения вывода в терминале и записи в лог-файл.
- **Результат**: Пакет собран успешно (Summary: 1 package finished), полный лог сохранён в `evidence/pr02/build.txt`.

---

## 2. Различия механизмов оболочки

### Отличие перенаправления (`>`) от конвейера (`|`):
- Оператор перенаправления `>` связывает стандартный вывод команды с **файлом** на диске. Дескриптор stdout процесса подменяется на дескриптор открытого файла. Данные пишутся напрямую на диск, перезаписывая файл.
- Оператор конвейера `|` (pipe) связывает стандартный вывод левого процесса со стандартным вводом правого **процесса** через IPC-буфер ядра Linux (inter-process communication / анонимный канал). Оба процесса выполняются параллельно, обмениваясь байтовым потоком в оперативной памяти без создания промежуточного файла.

### Отличие `source` (или `.`) от запуска новой программы:
- При обычном запуске скрипта или бинарника (например, `./script.sh` или `python3 app.py`) оболочка вызывает системные вызовы `fork()` и `execve()`, создавая **новый изолированный дочерний процесс**. Переменные окружения, экспортированные внутри дочернего процесса, уничтожаются при его завершении и не влияют на родительский терминал.
- Команда `source file.bash` (встроенная команда bash) выполняет команды из файла **в контексте текущего процесса оболочки** без вызова `fork()`. Поэтому все переменные (`export ROS_DOMAIN_ID=...`, пути в `PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`), экспортируемые файлом `setup.bash`, остаются активными в текущей сессии терминала.

---

## 3. Запуск и жизненный цикл `sim.launch.py`

### Команда запуска:
```bash
ros2 launch turtle_bringup sim.launch.py
```

### Проверка активных нод в графе:
```bash
ros2 node list --no-daemon --spin-time 2
```
Вывод:
```text
/turtlesim
```

### Остановка:
При нажатии `Ctrl+C` в терминале с launch-файлом система launch передает сигнал `SIGINT` дочернему процессу `turtlesim_node`. Процесс корректно освобождает ресурсы и завершается. Окно симулятора закрывается, нода исчезает из графа ROS 2.

---

## 4. Управление движением (нормальная доставка)

### Начальное состояние позы:
```bash
ros2 topic echo /turtle1/pose --once
```
Вывод:
```yaml
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
```

### Отправка команды управления:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

### Конечное состояние позы:
```bash
ros2 topic echo /turtle1/pose --once
```
Вывод:
```yaml
x: 6.544444561004639
y: 5.544444561004639
theta: 0.5
linear_velocity: 0.0
angular_velocity: 0.0
```
*Наблюдение*: Черепаха сместилась вперед и повернулась. После завершения импульса скорости черепаха остановилась, линейная и угловая скорости вернулись в 0.

---

## 5. Воспроизведение ошибки и доказательство исправления

### Шаг 1: Воспроизведение сбоя (ошибочное имя топика)
Запуск публикации с неверным именем топика `/cmd_vel` без namespace черепахи:
```bash
ros2 topic pub --rate 1 --wait-matching-subscriptions 0 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

Проверка информации о топике `/cmd_vel`:
```bash
ros2 topic info /cmd_vel --verbose
```
Вывод:
```text
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: _ros2cli_53625
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Topic type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
Endpoint type: PUBLISHER
GID: 01.0f.93.03.79.d1.0c.cf.00.00.00.00.00.00.07.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
```

Проверка информации о целевом топике черепахи `/turtle1/cmd_vel`:
```bash
ros2 topic info /turtle1/cmd_vel --verbose
```
Вывод:
```text
Type: geometry_msgs/msg/Twist

Publisher count: 0

Subscription count: 1

Node name: turtlesim
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Topic type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
Endpoint type: SUBSCRIPTION
GID: 01.0f.93.03.e0.ca.c6.a5.00.00.00.00.00.00.1d.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

*Результат*: Черепаха остаётся неподвижной. Издатель успешно публикует сообщения, но у топика `/cmd_vel` **0 подписчиков** (`Subscription count: 0`), а нода `/turtlesim` слушает топик `/turtle1/cmd_vel`.

### Шаг 2: Доказательство исправления
Останавливаем ошибочного издателя (`Ctrl+C`) и публикуем в правильный топик `/turtle1/cmd_vel`:
```bash
ros2 topic pub --rate 1 --wait-matching-subscriptions 0 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

Проверка информации о топике:
```bash
ros2 topic info /turtle1/cmd_vel --verbose
```
Вывод:
```text
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: _ros2cli_54550
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Topic type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
Endpoint type: PUBLISHER
GID: 01.0f.93.03.16.d5.51.b8.00.00.00.00.00.00.07.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: turtlesim
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Topic type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
Endpoint type: SUBSCRIPTION
GID: 01.0f.93.03.e0.ca.c6.a5.00.00.00.00.00.00.1d.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

*Результат*: Черепаха непрерывно движется по дуге окружности.

---

## 6. Почему правильного типа сообщения недостаточно (Discovery vs Delivery)

В DDS / ROS 2 сопряжение (matching) конечных точек требует совпадения нескольких критериев:
1. **Полное имя топика (Fully Qualified Topic Name)**: Включает namespace и базовое имя топика.
2. **Тип сообщения (Message Type)**: Хеш и определение структуры данных (например, `geometry_msgs/msg/Twist`).
3. **Совместимость QoS (Quality of Service)**: Надежность, долговечность и др.

В фазе **Discovery (Обнаружение)** участники DDS объявляют свои Reader и Writer. Нода `/turtlesim` создала подписчика на имя `/turtle1/cmd_vel`. CLI-издатель опубликовал данные в топик `/cmd_vel`.
Несмотря на то, что типы сообщений (`geometry_msgs/msg/Twist`) и QoS полностью совпадали, DDS рассматривает `/cmd_vel` и `/turtle1/cmd_vel` как **два абсолютно независимых канала связи (разные топики в шине)**.
Следовательно, сопоставления (endpoint match) не произошло, и сообщения физически не доставляются (**Delivery failure**) в ноду симуляции.
