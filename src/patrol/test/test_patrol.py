from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from patrol.patrol_node import compute_cmd_vel


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


def test_compute_cmd_vel_with_pose():
    """После получения позы нода должна выдавать linear.x=0.5 и angular.z=0.3."""
    pose = Pose(x=5.54, y=5.54, theta=0.0, linear_velocity=0.0, angular_velocity=0.0)
    cmd = compute_cmd_vel(pose)
    assert isinstance(cmd, Twist)
    assert cmd.linear.x == 0.5
    assert cmd.linear.y == 0.0
    assert cmd.linear.z == 0.0
    assert cmd.angular.x == 0.0
    assert cmd.angular.y == 0.0
    assert cmd.angular.z == 0.3
