import pytest
import rclpy
from rclpy.qos import ReliabilityPolicy

from course_bringup.pose_relay import PoseRelayNode


def test_pose_relay_node_qos():
    if not rclpy.ok():
        rclpy.init()
    try:
        node = PoseRelayNode()
        # Проверяем, что издатель sensor_pose настроен на BEST_EFFORT
        assert node.sensor_qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert 'sensor_pose' in node.sensor_pub.topic_name
        assert 'input_pose' in node.pose_sub.topic_name
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_launch_description_arguments():
    # Проверка структуры patrol.launch.py
    import importlib.util
    from pathlib import Path

    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'patrol.launch.py'
    assert launch_path.is_file(), f"Не найден launch-файл: {launch_path}"

    spec = importlib.util.spec_from_file_location("patrol_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    arg_names = [arg.name for arg in ld.entities if hasattr(arg, 'name')]

    assert 'namespace' in arg_names
    assert 'subscriber_reliability' in arg_names
    assert 'record' in arg_names
    assert 'bag_dir' in arg_names
