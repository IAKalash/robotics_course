"""Утилиты геометрии и управления для маршрута мышью (ПР03 Дополнительно)."""

from __future__ import annotations

import math


def canvas_to_turtlesim(u: float, v: float, width: float, height: float) -> tuple[float, float]:
    """Преобразовать экранные координаты холста (u, v) в координаты turtlesim (x, y).

    Холст: u in [0, width], v in [0, height], ось v направлена вниз.
    Turtlesim: x in [1, 10], y in [1, 10], ось y направлена вверх.
    Формулы:
        x = 1 + 9 * (u / width)
        y = 10 - 9 * (v / height)
    """
    if width <= 0 or height <= 0:
        return 5.5, 5.5
    u_clamped = max(0.0, min(float(u), float(width)))
    v_clamped = max(0.0, min(float(v), float(height)))
    x = 1.0 + 9.0 * (u_clamped / float(width))
    y = 10.0 - 9.0 * (v_clamped / float(height))
    return round(x, 4), round(y, 4)


def turtlesim_to_canvas(x: float, y: float, width: float, height: float) -> tuple[float, float]:
    """Обратное преобразование из координат turtlesim (x, y) в координаты холста (u, v)."""
    if width <= 0 or height <= 0:
        return 0.0, 0.0
    x_clamped = max(1.0, min(float(x), 10.0))
    y_clamped = max(1.0, min(float(y), 10.0))
    u = (x_clamped - 1.0) / 9.0 * float(width)
    v = (10.0 - y_clamped) / 9.0 * float(height)
    return round(u, 2), round(v, 2)


def normalize_angle(angle: float) -> float:
    """Нормализовать угол в диапазон [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def filter_and_interpolate_points(
    raw_points: list[tuple[float, float]],
    min_dist: float = 0.1,
    max_dist: float = 0.25,
) -> list[tuple[float, float]]:
    """Фильтровать дубликаты и интерполировать редкие точки мыши.

    - Убирает соседние точки ближе чем min_dist.
    - Добавляет промежуточные точки, если шаг превышает max_dist.
    - Сохраняет конечную точку.
    """
    if not raw_points:
        return []
    if len(raw_points) == 1:
        return list(raw_points)

    result: list[tuple[float, float]] = [raw_points[0]]

    for next_pt in raw_points[1:]:
        prev_pt = result[-1]
        dist = math.hypot(next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1])

        if dist < min_dist:
            # Слишком близко — пропускаем промежуточный шум
            continue

        if dist > max_dist:
            # Слишком далеко (быстрое движение мыши) — интерполируем
            steps = int(math.ceil(dist / max_dist))
            for s in range(1, steps):
                alpha = s / steps
                ix = prev_pt[0] + alpha * (next_pt[0] - prev_pt[0])
                iy = prev_pt[1] + alpha * (next_pt[1] - prev_pt[1])
                result.append((round(ix, 4), round(iy, 4)))

        result.append(next_pt)

    # Убеждаемся, что самая последняя точка зафиксирована
    if result[-1] != raw_points[-1]:
        result.append(raw_points[-1])

    return result


def point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Расстояние от точки P до отрезка AB."""
    abx = bx - ax
    aby = by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    # Проекция точки P на прямую AB
    t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
    t_clamped = max(0.0, min(1.0, t))
    proj_x = ax + t_clamped * abx
    proj_y = ay + t_clamped * aby
    return math.hypot(px - proj_x, py - proj_y)


def compute_cross_track_error(
    cur_x: float, cur_y: float, route_points: list[tuple[float, float]]
) -> float:
    """Вычислить минимальное расстояние от текущей позиции до ломаной линии маршрута."""
    if not route_points:
        return 0.0
    if len(route_points) == 1:
        return math.hypot(cur_x - route_points[0][0], cur_y - route_points[0][1])

    min_dist = float('inf')
    for i in range(len(route_points) - 1):
        ax, ay = route_points[i]
        bx, by = route_points[i + 1]
        dist = point_to_segment_distance(cur_x, cur_y, ax, ay, bx, by)
        if dist < min_dist:
            min_dist = dist
    return round(min_dist, 4)


def validate_max_speed(val: float) -> tuple[bool, str]:
    """Валидация максимальной скорости контроллера: (0.0, 2.0] м/с."""
    if not isinstance(val, (int, float)) or isinstance(val, bool):
        return False, "max_speed must be a number"
    if not math.isfinite(val):
        return False, "max_speed must be a finite number"
    if val <= 0.0:
        return False, f"max_speed must be positive, got {val}"
    if val > 2.0:
        return False, f"max_speed {val} is out of allowed range (0.0, 2.0]"
    return True, "ok"


def validate_goal_tolerance(val: float) -> tuple[bool, str]:
    """Валидация точности достижения цели: (0.0, 1.0] м.

    Нулевая и отрицательная точность строго отклоняется.
    """
    if not isinstance(val, (int, float)) or isinstance(val, bool):
        return False, "goal_tolerance must be a number"
    if not math.isfinite(val):
        return False, "goal_tolerance must be a finite number"
    if val <= 0.0:
        return False, f"goal_tolerance must be positive, got {val}"
    if val > 1.0:
        return False, f"goal_tolerance {val} is out of allowed range (0.0, 1.0]"
    return True, "ok"


def validate_turn_gain(val: float) -> tuple[bool, str]:
    """Валидация коэффициента поворота: [0.1, 10.0]."""
    if not isinstance(val, (int, float)) or isinstance(val, bool):
        return False, "turn_gain must be a number"
    if not math.isfinite(val):
        return False, "turn_gain must be a finite number"
    if not (0.1 <= val <= 10.0):
        return False, f"turn_gain {val} is out of allowed range [0.1, 10.0]"
    return True, "ok"


def compute_route_step(
    cur_x: float,
    cur_y: float,
    cur_theta: float,
    target_x: float,
    target_y: float,
    reach_dist: float = 0.1,
    max_speed: float = 0.5,
    turn_gain: float = 2.0,
    goal_tolerance: float | None = None,
) -> tuple[float, float, float, bool]:
    """Вычислить шаг управления к целевой точке с учётом параметров контроллера.

    Параметры:
        reach_dist / goal_tolerance: радиус попадания в точку (м)
        max_speed: верхний предел линейной скорости (м/с)
        turn_gain: коэффициент пропорционального регулятора по углу

    Возвращает:
        (linear_velocity, angular_velocity, distance_to_target, is_reached)
    """
    effective_tolerance = goal_tolerance if goal_tolerance is not None else reach_dist
    dx = target_x - cur_x
    dy = target_y - cur_y
    dist = math.hypot(dx, dy)

    if dist < effective_tolerance:
        return 0.0, 0.0, dist, True

    target_angle = math.atan2(dy, dx)
    angle_err = normalize_angle(target_angle - cur_theta)

    # При большой ошибке угла разворачиваемся на месте
    if abs(angle_err) > 0.4:
        lin_x = 0.0
        ang_z = max(-1.0, min(1.0, (turn_gain + 0.5) * angle_err))
    else:
        # При малом угле движемся вперед и корректируем направление
        # Линейная скорость масштабируется в зависимости от max_speed
        lin_x = max(0.05, min(max_speed, (max_speed / 0.5) * 0.6 * dist))
        ang_z = max(-1.0, min(1.0, turn_gain * angle_err))

    return round(lin_x, 3), round(ang_z, 3), dist, False

