"""Launch-файл запуска системы с namespace, параметризацией QoS и записью в MCAP (ПР05)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    # Аргументы запуска
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot_a',
        description='Пространство имён для робота (например, robot_a)',
    )
    subscriber_reliability_arg = DeclareLaunchArgument(
        'subscriber_reliability',
        default_value='reliable',
        description='Надёжность подписки на позу (reliable или best_effort)',
    )
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Записывать ли поток sensor_pose в MCAP',
    )
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value='evidence/pr05/bag',
        description='Директория для сохранения bag-файла',
    )

    namespace_conf = LaunchConfiguration('namespace')
    subscriber_reliability_conf = LaunchConfiguration('subscriber_reliability')
    record_conf = LaunchConfiguration('record')
    bag_dir_conf = LaunchConfiguration('bag_dir')

    # Вычисление полного имени топика для записи: /<namespace>/sensor_pose
    record_topic = PythonExpression([
        "'/' + '", namespace_conf, "'.strip('/') + '/sensor_pose'"
    ])

    # Группа действий внутри пространства имён
    robot_group = GroupAction(
        actions=[
            PushRosNamespace(namespace_conf),
            Node(
                package='course_bringup',
                executable='pose_relay',
                name='pose_relay',
                output='screen',
                remappings=[
                    ('input_pose', '/turtle1/pose'),
                ],
            ),
            Node(
                package='patrol',
                executable='patrol',
                name='patrol',
                output='screen',
                parameters=[
                    {'subscriber_reliability': subscriber_reliability_conf},
                ],
                remappings=[
                    ('pose', 'sensor_pose'),
                    ('cmd_vel', '/turtle1/cmd_vel'),
                ],
            ),
        ]
    )

    # Процесс записи в MCAP (запускается только при record:=true через IfCondition)
    recorder_process = ExecuteProcess(
        condition=IfCondition(record_conf),
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'mcap',
            '-o', bag_dir_conf,
            '--topics', record_topic,
        ],
        output='screen',
    )

    log_start = LogInfo(
        msg=[
            'Запуск patrol.launch.py в namespace: ', namespace_conf,
            ', subscriber_reliability: ', subscriber_reliability_conf,
            ', record: ', record_conf,
        ]
    )

    return LaunchDescription([
        namespace_arg,
        subscriber_reliability_arg,
        record_arg,
        bag_dir_arg,
        log_start,
        robot_group,
        recorder_process,
    ])
