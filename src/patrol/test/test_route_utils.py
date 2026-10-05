import math

from patrol.route_utils import (
    canvas_to_turtlesim,
    compute_cross_track_error,
    compute_route_step,
    filter_and_interpolate_points,
    normalize_angle,
    point_to_segment_distance,
    turtlesim_to_canvas,
    validate_goal_tolerance,
    validate_max_speed,
    validate_turn_gain,
)

W = 500
H = 500


def test_canvas_to_turtlesim_corners():
    # Левый верхний угол холста (0, 0) -> верхний левый угол turtlesim (1.0, 10.0)
    tx, ty = canvas_to_turtlesim(0, 0, W, H)
    assert math.isclose(tx, 1.0, abs_tol=1e-3)
    assert math.isclose(ty, 10.0, abs_tol=1e-3)

    # Правый нижний угол холста (W, H) -> правый нижний угол turtlesim (10.0, 1.0)
    tx, ty = canvas_to_turtlesim(W, H, W, H)
    assert math.isclose(tx, 10.0, abs_tol=1e-3)
    assert math.isclose(ty, 1.0, abs_tol=1e-3)

    # Центр холста (W/2, H/2) -> центр turtlesim (5.5, 5.5)
    tx, ty = canvas_to_turtlesim(W / 2, H / 2, W, H)
    assert math.isclose(tx, 5.5, abs_tol=1e-3)
    assert math.isclose(ty, 5.5, abs_tol=1e-3)


def test_turtlesim_to_canvas_roundtrip():
    # Проверка взаимно однозначного обратного преобразования
    orig_x, orig_y = 3.25, 7.8
    u, v = turtlesim_to_canvas(orig_x, orig_y, W, H)
    rec_x, rec_y = canvas_to_turtlesim(u, v, W, H)
    assert math.isclose(orig_x, rec_x, abs_tol=0.02)
    assert math.isclose(orig_y, rec_y, abs_tol=0.02)


def test_normalize_angle():
    assert math.isclose(normalize_angle(0.0), 0.0)
    assert math.isclose(normalize_angle(math.pi), math.pi, abs_tol=1e-5)
    # Переход через pi: 3.5 rad -> ~ -2.783 rad
    wrapped = normalize_angle(3.5)
    assert -math.pi <= wrapped <= math.pi
    assert math.isclose(wrapped, 3.5 - 2 * math.pi, abs_tol=1e-5)

    # Переход через -pi: -4.0 rad -> ~ 2.283 rad
    wrapped_neg = normalize_angle(-4.0)
    assert -math.pi <= wrapped_neg <= math.pi
    assert math.isclose(wrapped_neg, -4.0 + 2 * math.pi, abs_tol=1e-5)


def test_filter_and_interpolate_points():
    # Пустой список
    assert filter_and_interpolate_points([]) == []

    # Одна точка
    assert filter_and_interpolate_points([(5.0, 5.0)]) == [(5.0, 5.0)]

    # Фильтрация близких точек (меньше 0.1)
    close_pts = [(5.0, 5.0), (5.01, 5.01), (5.02, 5.02), (5.15, 5.0)]
    filtered = filter_and_interpolate_points(close_pts, min_dist=0.1, max_dist=0.25)
    assert len(filtered) == 2
    assert filtered[0] == (5.0, 5.0)
    assert filtered[-1] == (5.15, 5.0)

    # Интерполяция слишком далеких точек (прыжок > 0.25)
    far_pts = [(1.0, 1.0), (3.0, 1.0)]
    interpolated = filter_and_interpolate_points(far_pts, max_dist=0.25)
    assert len(interpolated) > 2
    assert interpolated[0] == (1.0, 1.0)
    assert interpolated[-1] == (3.0, 1.0)


def test_compute_route_step_target_reached():
    # Когда расстояние < 0.1, цель считается достигнутой
    lin_x, ang_z, dist, is_reached = compute_route_step(
        cur_x=5.0, cur_y=5.0, cur_theta=0.0, target_x=5.05, target_y=5.0
    )
    assert is_reached is True
    assert lin_x == 0.0
    assert ang_z == 0.0
    assert dist < 0.1


def test_compute_route_step_turn_in_place():
    # Черепаха смотрит вправо (theta=0), а цель прямо назад (цель x=3.0, y=5.0, угол pi)
    # Ошибка угла ~ pi (> 0.4) -> разворот на месте
    lin_x, ang_z, dist, is_reached = compute_route_step(
        cur_x=5.0, cur_y=5.0, cur_theta=0.0, target_x=3.0, target_y=5.0
    )
    assert is_reached is False
    assert lin_x == 0.0  # поворот на месте
    assert abs(ang_z) > 0.0
    assert -1.0 <= ang_z <= 1.0


def test_compute_route_step_move_forward():
    # Черепаха смотрит на цель (theta=0, цель x=8.0, y=5.0) -> движение вперед
    lin_x, ang_z, dist, is_reached = compute_route_step(
        cur_x=5.0, cur_y=5.0, cur_theta=0.0, target_x=8.0, target_y=5.0
    )
    assert is_reached is False
    assert lin_x > 0.0
    assert lin_x <= 0.5
    assert -1.0 <= ang_z <= 1.0


def test_validate_max_speed():
    assert validate_max_speed(0.1)[0] is True
    assert validate_max_speed(0.5)[0] is True
    assert validate_max_speed(2.0)[0] is True
    # Отклонение нуля, отрицательных и слишком больших значений
    assert validate_max_speed(0.0)[0] is False
    assert validate_max_speed(-0.5)[0] is False
    assert validate_max_speed(2.5)[0] is False
    assert validate_max_speed(float('nan'))[0] is False
    assert validate_max_speed(float('inf'))[0] is False


def test_validate_goal_tolerance_rejection():
    # Допустимые значения
    assert validate_goal_tolerance(0.02)[0] is True
    assert validate_goal_tolerance(0.1)[0] is True
    assert validate_goal_tolerance(1.0)[0] is True
    # Требование задания: нулевая и отрицательная точность строго отклоняются!
    ok_zero, reason_zero = validate_goal_tolerance(0.0)
    assert ok_zero is False
    assert "positive" in reason_zero

    ok_neg, reason_neg = validate_goal_tolerance(-0.1)
    assert ok_neg is False
    assert "positive" in reason_neg

    assert validate_goal_tolerance(1.5)[0] is False
    assert validate_goal_tolerance(float('nan'))[0] is False
    assert validate_goal_tolerance(float('inf'))[0] is False


def test_validate_turn_gain():
    assert validate_turn_gain(0.1)[0] is True
    assert validate_turn_gain(2.0)[0] is True
    assert validate_turn_gain(10.0)[0] is True
    assert validate_turn_gain(0.0)[0] is False
    assert validate_turn_gain(-1.0)[0] is False
    assert validate_turn_gain(15.0)[0] is False
    assert validate_turn_gain(float('nan'))[0] is False


def test_compute_cross_track_error():
    route = [(1.0, 1.0), (5.0, 1.0), (5.0, 5.0)]
    # Точка лежит точно на отрезке
    assert math.isclose(compute_cross_track_error(3.0, 1.0, route), 0.0, abs_tol=1e-3)
    # Точка смещена по Y на 0.2
    assert math.isclose(compute_cross_track_error(3.0, 1.2, route), 0.2, abs_tol=1e-3)
    # Пустой маршрут
    assert compute_cross_track_error(3.0, 1.0, []) == 0.0


def test_compute_route_step_with_custom_controller_params():
    # С большей скоростью max_speed=1.5
    lin_x, _, _, _ = compute_route_step(
        cur_x=5.0, cur_y=5.0, cur_theta=0.0, target_x=8.0, target_y=5.0, max_speed=1.5
    )
    assert lin_x > 0.5
    assert lin_x <= 1.5

    # С большей точностью цели goal_tolerance=0.02, dist=0.05 не считается достигнутой
    _, _, _, reached = compute_route_step(
        cur_x=5.0, cur_y=5.0, cur_theta=0.0, target_x=5.05, target_y=5.0, goal_tolerance=0.02
    )
    assert reached is False


def test_draw_route_node_parameters():
    import rclpy
    from patrol.draw_route_node import DrawRouteNode
    from rclpy.parameter import Parameter

    if not rclpy.ok():
        rclpy.init()
    try:
        node = DrawRouteNode()
        # Проверка дефолтных значений
        assert node.max_speed == 0.5
        assert node.goal_tolerance == 0.1
        assert node.turn_gain == 2.0

        # Установка валидного параметра
        res = node.set_parameters([Parameter("max_speed", Parameter.Type.DOUBLE, 1.2)])
        assert res[0].successful is True
        assert node.max_speed == 1.2

        # Отклонение невалидного goal_tolerance (0.0 должно отклоняться)
        res_fail = node.set_parameters([Parameter("goal_tolerance", Parameter.Type.DOUBLE, 0.0)])
        assert res_fail[0].successful is False
        assert node.goal_tolerance == 0.1  # Состояние не нарушено
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


