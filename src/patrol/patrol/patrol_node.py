from __future__ import annotations

import math
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from turtlesim.msg import Pose

if TYPE_CHECKING:
    from rclpy.parameter import Parameter


def validate_linear_speed(val: float) -> tuple[bool, str]:
    """Проверить допустимость линейной скорости [0.0, 1.0] м/с."""
    if not isinstance(val, (int, float)) or math.isnan(val) or math.isinf(val):
        return False, "linear_speed must be a finite number"
    if not (0.0 <= float(val) <= 1.0):
        return False, f"linear_speed {val} is out of allowed range [0.0, 1.0]"
    return True, ""


def validate_turn_rate(val: float) -> tuple[bool, str]:
    """Проверить допустимость угловой скорости [-1.0, 1.0] рад/с."""
    if not isinstance(val, (int, float)) or math.isnan(val) or math.isinf(val):
        return False, "turn_rate must be a finite number"
    if not (-1.0 <= float(val) <= 1.0):
        return False, f"turn_rate {val} is out of allowed range [-1.0, 1.0]"
    return True, ""


def validate_publish_hz(val: float) -> tuple[bool, str]:
    """Проверить допустимость частоты публикации [1.0, 30.0] Гц."""
    if not isinstance(val, (int, float)) or math.isnan(val) or math.isinf(val):
        return False, "publish_hz must be a finite number"
    if not (1.0 <= float(val) <= 30.0):
        return False, f"publish_hz {val} is out of allowed range [1.0, 30.0]"
    return True, ""


def compute_cmd_vel(
    latest_pose: Pose | None,
    linear_speed: float = 0.5,
    turn_rate: float = 0.3,
) -> Twist:
    """Вычислить команду скорости на основе последней полученной позы и параметров.

    Чистая функция:
    - До первой позы (latest_pose is None): нулевая скорость.
    - После получения позы: linear.x = linear_speed, angular.z = turn_rate.
    """
    cmd = Twist()
    if latest_pose is not None:
        cmd.linear.x = float(linear_speed)
        cmd.angular.z = float(turn_rate)
    return cmd


class PatrolNode(Node):
    """Нода патрулирования для turtlesim с поддержкой динамических параметров."""

    def __init__(self) -> None:
        super().__init__('patrol')
        self.latest_pose: Pose | None = None

        # Объявление параметров со значениями по умолчанию
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('turn_rate', 0.3)
        self.declare_parameter('publish_hz', 10.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        # Регистрация callback для валидации и динамического изменения параметров
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Подписка на позу черепахи
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )

        # Издатель команды скорости в относительный топик 'cmd_vel'
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        # Создание таймера с периодом 1 / publish_hz
        period = 1.0 / self.publish_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Нода patrol запущена (linear_speed={self.linear_speed}, '
            f'turn_rate={self.turn_rate}, publish_hz={self.publish_hz})'
        )

    def parameter_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Валидация входящих параметров перед применением изменений."""
        # 1. Валидация: проверяем ВСЕ входящие параметры без изменения состояния
        for param in params:
            if param.name == 'publish_hz':
                ok, reason = validate_publish_hz(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра publish_hz: {reason}')
                    return SetParametersResult(successful=False, reason=reason)
            elif param.name == 'linear_speed':
                ok, reason = validate_linear_speed(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра linear_speed: {reason}')
                    return SetParametersResult(successful=False, reason=reason)
            elif param.name == 'turn_rate':
                ok, reason = validate_turn_rate(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра turn_rate: {reason}')
                    return SetParametersResult(successful=False, reason=reason)

        # 2. Если все параметры валидны — применяем изменения
        for param in params:
            if param.name == 'publish_hz':
                new_hz = float(param.value)
                if new_hz != self.publish_hz:
                    self.publish_hz = new_hz
                    # Корректная остановка и пересоздание таймера (двух таймеров не остаётся)
                    self.destroy_timer(self.timer)
                    period = 1.0 / self.publish_hz
                    self.timer = self.create_timer(period, self.timer_callback)
                    self.get_logger().info(f'Таймер пересоздан: новая частота {self.publish_hz} Гц (период {period:.4f} с)')
            elif param.name == 'linear_speed':
                self.linear_speed = float(param.value)
                self.get_logger().info(f'Установлен linear_speed: {self.linear_speed}')
            elif param.name == 'turn_rate':
                self.turn_rate = float(param.value)
                self.get_logger().info(f'Установлен turn_rate: {self.turn_rate}')

        return SetParametersResult(successful=True)

    def pose_callback(self, msg: Pose) -> None:
        """Сохранить последнее сообщение с позой черепахи."""
        self.latest_pose = msg

    def timer_callback(self) -> None:
        """Периодически вычислять и публиковать команду скорости."""
        cmd = compute_cmd_vel(self.latest_pose, self.linear_speed, self.turn_rate)
        self.cmd_pub.publish(cmd)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
