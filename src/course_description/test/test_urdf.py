import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory


def get_xacro_path() -> Path:
    # First try from source directory
    source_xacro = Path(__file__).resolve().parents[1] / 'urdf' / 'vehicle.urdf.xacro'
    if source_xacro.is_file():
        return source_xacro
    # Fallback to installed share directory
    share_dir = Path(get_package_share_directory('course_description'))
    return share_dir / 'urdf' / 'vehicle.urdf.xacro'


def expand_xacro(xacro_path: Path) -> str:
    result = subprocess.run(
        ['xacro', str(xacro_path)],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def test_xacro_expansion():
    """Verify that xacro expands without errors and outputs valid XML."""
    xacro_file = get_xacro_path()
    assert xacro_file.is_file(), f"File {xacro_file} does not exist"
    urdf_xml = expand_xacro(xacro_file)
    root = ET.fromstring(urdf_xml)
    assert root.tag == 'robot'
    assert root.attrib.get('name') == 'course_ackermann'


def test_check_urdf_validation(tmp_path):
    """Verify that check_urdf parses the expanded robot model successfully."""
    xacro_file = get_xacro_path()
    urdf_xml = expand_xacro(xacro_file)
    urdf_file = tmp_path / 'vehicle.urdf'
    urdf_file.write_text(urdf_xml, encoding='utf-8')

    result = subprocess.run(
        ['check_urdf', str(urdf_file)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"check_urdf failed:\n{result.stderr}\n{result.stdout}"
    assert 'Successfully Parsed XML' in result.stdout
    assert 'root Link: base_link' in result.stdout


def test_lidar_parent_is_base_link():
    """Verify that lidar_link has base_link as its parent joint."""
    xacro_file = get_xacro_path()
    urdf_xml = expand_xacro(xacro_file)
    root = ET.fromstring(urdf_xml)

    lidar_parent = None
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is not None and child.attrib.get('link') == 'lidar_link':
            parent = joint.find('parent')
            if parent is not None:
                lidar_parent = parent.attrib.get('link')
            break

    assert lidar_parent is not None, "Joint for child 'lidar_link' not found"
    assert lidar_parent == 'base_link', (
        f"Expected lidar_link parent to be 'base_link', but got '{lidar_parent}'"
    )


def test_sensor_links_parent_connections():
    """Verify camera and imu links are directly attached to base_link."""
    xacro_file = get_xacro_path()
    urdf_xml = expand_xacro(xacro_file)
    root = ET.fromstring(urdf_xml)

    parent_map = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        parent = joint.find('parent')
        if child is not None and parent is not None:
            parent_map[child.attrib.get('link')] = parent.attrib.get('link')

    assert parent_map.get('camera_link') == 'base_link'
    assert parent_map.get('imu_link') == 'base_link'


def test_ackermann_geometry_parameters():
    """Verify geometry matches course ackermann specifications."""
    xacro_file = get_xacro_path()
    urdf_xml = expand_xacro(xacro_file)
    root = ET.fromstring(urdf_xml)

    joints_by_name = {j.attrib.get('name'): j for j in root.findall('joint')}

    fl_joint = joints_by_name.get('front_left_steering_joint')
    rl_joint = joints_by_name.get('rear_left_wheel_joint')
    assert fl_joint is not None and rl_joint is not None

    fl_origin = [float(v) for v in fl_joint.find('origin').attrib.get('xyz').split()]
    rl_origin = [float(v) for v in rl_joint.find('origin').attrib.get('xyz').split()]

    # Wheelbase = distance along X between front axle and rear axle
    wheelbase = fl_origin[0] - rl_origin[0]
    assert pytest.approx(wheelbase, abs=1e-3) == 0.8

    # Track width = distance along Y between left knuckle and right knuckle
    fr_joint = joints_by_name.get('front_right_steering_joint')
    fr_origin = [float(v) for v in fr_joint.find('origin').attrib.get('xyz').split()]
    track_width = fl_origin[1] - fr_origin[1]
    assert pytest.approx(track_width, abs=1e-3) == 0.62
