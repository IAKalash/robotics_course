# Параметризация движения и управление нодой (ПР04)

## 1. Объявленные параметры ноды `patrol`

| Имя параметра | Тип | Значение по умолчанию | Допустимый диапазон | Назначение |
|---|---|---|---|---|
| `linear_speed` | `double` | `0.5` м/с | `[0.0, 1.0]` | Линейная скорость движения черепахи вперед |
| `turn_rate` | `double` | `0.3` рад/с | `[-1.0, 1.0]` | Угловая скорость поворота вокруг оси Z |
| `publish_hz` | `double` | `10.0` Гц | `[1.0, 30.0]` | Частота публикации команд и срабатывания таймера |

Все параметры валидируются до изменения внутреннего состояния ноды и таймера. Нечисловые (NaN, Inf) и выходящие за пределы значения отклоняются.

---

## 2. Динамическое изменение параметров и частоты таймера (10 Гц → 5 Гц)

### Проверка списка параметров работающей ноды:
```bash
ros2 param list /patrol
```
Вывод:
```text
/patrol:
  linear_speed
  publish_hz
  start_type_description_service
  turn_rate
  use_sim_time
```

### Запрос текущего значения `publish_hz`:
```bash
ros2 param get /patrol publish_hz
```
Вывод:
```text
Double value is: 10.0
```

### Изменение частоты публикации на 5.0 Гц:
```bash
ros2 param set /patrol publish_hz 5.0
```
Вывод:
```text
Set parameter successful
```

### Подтверждение изменения частоты потока команд за 10 секунд:
```bash
ros2 topic hz /turtle1/cmd_vel
```
Вывод:
```text
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00006s window: 6
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00006s window: 12
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00010s window: 18
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00009s window: 23
average rate: 5.000
	min: 0.200s max: 0.201s std dev: 0.00015s window: 29
average rate: 5.000
	min: 0.199s max: 0.201s std dev: 0.00019s window: 35
average rate: 5.000
	min: 0.199s max: 0.201s std dev: 0.00018s window: 40
```

---

## 3. Эксперимент со сбоем: воспроизведение дефекта и доказательство исправления

### Стадия 2 · Сломать (дефект при отсутствии валидации):
При отключенной предварительной проверке частоты установка значения `0.0`:
```bash
ros2 param set /patrol publish_hz 0.0
```
приводит к попытке вычисления периода $T = 1.0 / 0.0$, вызывающей `ZeroDivisionError: float division by zero`, либо передаче неположительного периода в `create_timer(period)`, что приводит к исключению `ValueError: timer period must be greater than zero` и падению процесса ноды.

### Стадия 3 · Доказать (валидация активна):
Попытка установки недопустимого значения частоты 0.0 при включенной валидации:
```bash
ros2 param set /patrol publish_hz 0.0
```
Вывод:
```text
Setting parameter failed: publish_hz 0.0 is out of allowed range [1.0, 30.0]
```

### Проверка сохранности активного состояния:
Повторный запрос параметра:
```bash
ros2 param get /patrol publish_hz
```
Вывод:
```text
Double value is: 5.0
```

Замер частоты топика:
```bash
ros2 topic hz /turtle1/cmd_vel
```
Вывод:
```text
ros2 topic hz /turtle1/cmd_vel
average rate: 5.002
	min: 0.200s max: 0.200s std dev: 0.00011s window: 7
average rate: 5.001
	min: 0.200s max: 0.200s std dev: 0.00015s window: 12
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00013s window: 18
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00012s window: 24
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00015s window: 30
```
*Результат*: Недопустимое значение `0.0` успешно отклонено на этапе валидации до модификации активного состояния. Нода не упала, предыдущий таймер продолжает штатно работать на частоте 5.0 Гц.

---

## 4. Результаты модульных тестов чистых функций валидации

В `src/patrol/test/test_patrol.py` протестированы граничные и недопустимые случаи:
- Допустимые значения: `10.0 Гц`, `5.0 Гц`, граничные `1.0 Гц` и `30.0 Гц` (`assert ok is True`).
- Недопустимый ноль: `0.0 Гц` (`assert ok is False`, отказ `out of allowed range`).
- Отрицательное значение: `-5.0 Гц` (`assert ok is False`, отказ `out of allowed range`).
- Превышение максимума: `35.0 Гц` (`assert ok is False`, отказ `out of allowed range`).
- Нечисловые значения: `float('nan')`, `float('inf')` (`assert ok is False`, отказ `must be a finite number`).
- Линейная скорость `linear_speed`: проверка допустимости `[0.0, 1.0]` и отклонение отрицательных значений и NaN.
- Угловая скорость `turn_rate`: проверка диапазона `[-1.0, 1.0]` и отклонение выхода за границы.

Все 13 модульных тестов пакета успешно пройдены.

---

## 5. Исследование сервисов и экшенов (Services vs Actions)

### Вызов сервиса очистки экрана `/clear`:
```bash
ros2 service call /clear std_srvs/srv/Empty {}
```
Вывод:
```text
waiting for service to become available...
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```
*Результат*: Вызов сервиса очищает все нарисованные черепахой линии в окне симулятора `turtlesim`, оставляя саму черепаху в текущей позе.

### Обнаружение Action-интерфейсов:
```bash
ros2 action list -t
```
Вывод:
```text
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

### Различие сервисного запроса (Service Request) и цели действия (Action Goal):
1. **Сервисный запрос (Service Request)** представляет собой короткую атомарную операцию вида «запрос–ответ» (RPC), выполняемую синхронно или асинхронно без предоставления промежуточной обратной связи и без возможности штатной отмены в процессе выполнения.
2. **Цель действия (Action Goal)** предназначена для длительных задач управления роботом, позволяя клиенту непрерывно получать промежуточный прогресс выполнения (**feedback**), при необходимости асинхронно отменять задачу (**cancel**) и получать финальный статус по завершении (**result**).
