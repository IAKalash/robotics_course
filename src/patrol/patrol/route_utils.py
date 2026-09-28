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


def compute_route_step(
    cur_x: float,
    cur_y: float,
    cur_theta: float,
    target_x: float,
    target_y: float,
    reach_dist: float = 0.1,
) -> tuple[float, float, float, bool]:
    """Вычислить шаг управления к целевой точке.

    Возвращает:
        (linear_velocity, angular_velocity, distance_to_target, is_reached)
    Ограничения:
        linear.x in [0.0, 0.5]
        angular.z in [-1.0, 1.0]
    """
    dx = target_x - cur_x
    dy = target_y - cur_y
    dist = math.hypot(dx, dy)

    if dist < reach_dist:
        return 0.0, 0.0, dist, True

    target_angle = math.atan2(dy, dx)
    angle_err = normalize_angle(target_angle - cur_theta)

    # При большой ошибке угла разворачиваемся на месте
    if abs(angle_err) > 0.4:
        lin_x = 0.0
        ang_z = max(-1.0, min(1.0, 2.5 * angle_err))
    else:
        # При малом угле движемся вперед и корректируем направление
        lin_x = max(0.1, min(0.5, 0.6 * dist))
        ang_z = max(-1.0, min(1.0, 2.0 * angle_err))

    return round(lin_x, 3), round(ang_z, 3), dist, False
