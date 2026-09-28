from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose


def compute_cmd_vel(latest_pose: Pose | None) -> Twist:
    """Вычислить команду скорости на основе последней полученной позы.

    Чистая функция:
    - До первой позы (latest_pose is None): нулевая скорость.
    - После получения позы: linear.x = 0.5, angular.z = 0.3.
    """
    cmd = Twist()
    if latest_pose is not None:
        cmd.linear.x = 0.5
        cmd.angular.z = 0.3
    return cmd


class PatrolNode(Node):
    """Нода патрулирования для turtlesim."""

    def __init__(self) -> None:
        super().__init__('patrol')
        self.latest_pose: Pose | None = None

        # Подписка на позу черепахи (абсолютное имя топика)
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

        # Таймер с периодом 0.1 секунды (10 Гц)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Нода patrol запущена, ожидание позы...')

    def pose_callback(self, msg: Pose) -> None:
        """Сохранить последнее сообщение с позой черепахи."""
        self.latest_pose = msg

    def timer_callback(self) -> None:
        """Периодически вычислять и публиковать команду скорости."""
        cmd = compute_cmd_vel(self.latest_pose)
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
