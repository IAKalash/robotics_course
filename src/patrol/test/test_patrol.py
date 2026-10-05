import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from patrol.patrol_node import (
    compute_cmd_vel,
    validate_linear_speed,
    validate_publish_hz,
    validate_turn_rate,
)


def test_compute_cmd_vel_without_pose():
    """Без полученной позы нода должна выдавать нулевую команду."""
    cmd = compute_cmd_vel(None)
    assert isinstance(cmd, Twist)
    assert cmd.linear.x == 0.0
    assert cmd.linear.y == 0.0
    assert cmd.linear.z == 0.0
    assert cmd.angular.x == 0.0
    assert cmd.angular.y == 0.0
    assert cmd.angular.z == 0.0


def test_compute_cmd_vel_with_pose_and_custom_params():
    """С полученной позой нода должна учитывать параметры скорости."""
    pose = Pose(x=5.54, y=5.54, theta=0.0, linear_velocity=0.0, angular_velocity=0.0)
    # По умолчанию
    cmd_def = compute_cmd_vel(pose)
    assert cmd_def.linear.x == 0.5
    assert cmd_def.angular.z == 0.3

    # С кастомными параметрами
    cmd_custom = compute_cmd_vel(pose, linear_speed=0.8, turn_rate=-0.5)
    assert cmd_custom.linear.x == 0.8
    assert cmd_custom.angular.z == -0.5


def test_validate_publish_hz_valid():
    """Проверка допустимых значений частоты публикации (1.0..30.0 Гц)."""
    ok, _ = validate_publish_hz(10.0)
    assert ok is True

    ok, _ = validate_publish_hz(5.0)
    assert ok is True

    ok, _ = validate_publish_hz(1.0)
    assert ok is True

    ok, _ = validate_publish_hz(30.0)
    assert ok is True


def test_validate_publish_hz_invalid():
    """Проверка отклонения нуля, отрицательных значений, выхода за границы и нечисловых значений."""
    # Ноль
    ok, reason = validate_publish_hz(0.0)
    assert ok is False
    assert "out of allowed range" in reason

    # Отрицательное
    ok, reason = validate_publish_hz(-5.0)
    assert ok is False
    assert "out of allowed range" in reason

    # Выше максимума
    ok, reason = validate_publish_hz(35.0)
    assert ok is False
    assert "out of allowed range" in reason

    # NaN и Inf
    ok, reason = validate_publish_hz(float("nan"))
    assert ok is False
    assert "finite number" in reason

    ok, reason = validate_publish_hz(float("inf"))
    assert ok is False
    assert "finite number" in reason


def test_validate_linear_speed():
    """Проверка валидации линейной скорости [0.0..1.0]."""
    assert validate_linear_speed(0.0)[0] is True
    assert validate_linear_speed(0.5)[0] is True
    assert validate_linear_speed(1.0)[0] is True
    assert validate_linear_speed(-0.1)[0] is False
    assert validate_linear_speed(1.5)[0] is False
    assert validate_linear_speed(float("nan"))[0] is False


def test_validate_turn_rate():
    """Проверка валидации угловой скорости [-1.0..1.0]."""
    assert validate_turn_rate(-1.0)[0] is True
    assert validate_turn_rate(0.0)[0] is True
    assert validate_turn_rate(1.0)[0] is True
    assert validate_turn_rate(-1.5)[0] is False
    assert validate_turn_rate(1.5)[0] is False
    assert validate_turn_rate(float("nan"))[0] is False
