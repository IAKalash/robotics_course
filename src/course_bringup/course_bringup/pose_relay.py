"""Нода ретрансляции позы с профилем QoS Best Effort (ПР05)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from turtlesim.msg import Pose


class PoseRelayNode(Node):
    """Читает позу из относительного топика 'input_pose' и публикует в 'sensor_pose' с best_effort."""

    def __init__(self) -> None:
        super().__init__('pose_relay')

        # Профиль QoS для датчика (Best Effort)
        self.sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Издатель сенсорной позы с best_effort (относительное имя 'sensor_pose')
        self.sensor_pub = self.create_publisher(
            Pose,
            'sensor_pose',
            self.sensor_qos,
        )

        # Подписчик на входную позу (относительное имя 'input_pose', в launch ремапится в /turtle1/pose)
        self.pose_sub = self.create_subscription(
            Pose,
            'input_pose',
            self._on_pose,
            10,
        )

        self.get_logger().info('Нода pose_relay запущена: input_pose -> sensor_pose (best_effort)')

    def _on_pose(self, msg: Pose) -> None:
        """Пересылать полученное сообщение в топик sensor_pose."""
        self.sensor_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PoseRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
