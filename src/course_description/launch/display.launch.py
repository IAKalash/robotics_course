#!/usr/bin/env python3
"""Launch robot_state_publisher with Ackermann vehicle URDF/Xacro."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('course_description')
    default_xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'vehicle.urdf.xacro'])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization',
    )

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            default_xacro_path,
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_arg,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
