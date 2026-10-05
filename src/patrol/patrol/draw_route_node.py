"""Нода рисования маршрута мышью и следования по нему (ПР03 + ПР04 Дополнительно).

Поддерживает динамическую параметризацию параметров контроллера:
- max_speed: максимальная линейная скорость (м/с)
- goal_tolerance: радиус захвата промежуточной цели (м)
- turn_gain: коэффициент поворота (P-регулятор угла)
А также отображает ползунки управления параметрами в окне Tkinter и считает
метрики времени и отклонения (cross-track error) для сравнения проходов.
"""

from __future__ import annotations

import math
import sys
import time
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose

from patrol.route_utils import (
    canvas_to_turtlesim,
    compute_cross_track_error,
    compute_route_step,
    filter_and_interpolate_points,
    turtlesim_to_canvas,
    validate_goal_tolerance,
    validate_max_speed,
    validate_turn_gain,
)

CANVAS_WIDTH = 500
CANVAS_HEIGHT = 500


class DrawRouteNode(Node):
    """ROS 2 нода контроллера маршрута с поддержкой динамических параметров."""

    def __init__(self) -> None:
        super().__init__('draw_route')

        # Объявление параметров со значениями по умолчанию
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('turn_gain', 2.0)

        self.max_speed: float = float(self.get_parameter('max_speed').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)
        self.turn_gain: float = float(self.get_parameter('turn_gain').value)

        # Регистрация параметрического колбэка
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'Нода draw_route запущена (max_speed={self.max_speed}, '
            f'goal_tolerance={self.goal_tolerance}, turn_gain={self.turn_gain})'
        )

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        """Валидация входящих параметров перед применением изменений."""
        # 1. Проверяем ВСЕ параметры без изменения текущего состояния
        for param in params:
            if param.name == 'max_speed':
                ok, reason = validate_max_speed(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра max_speed: {reason}')
                    return SetParametersResult(successful=False, reason=reason)
            elif param.name == 'goal_tolerance':
                ok, reason = validate_goal_tolerance(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра goal_tolerance: {reason}')
                    return SetParametersResult(successful=False, reason=reason)
            elif param.name == 'turn_gain':
                ok, reason = validate_turn_gain(param.value)
                if not ok:
                    self.get_logger().warn(f'Отклонение параметра turn_gain: {reason}')
                    return SetParametersResult(successful=False, reason=reason)

        # 2. Если все параметры валидны — применяем их
        for param in params:
            if param.name == 'max_speed':
                self.max_speed = float(param.value)
                self.get_logger().info(f'Установлен max_speed: {self.max_speed:.2f}')
            elif param.name == 'goal_tolerance':
                self.goal_tolerance = float(param.value)
                self.get_logger().info(f'Установлен goal_tolerance: {self.goal_tolerance:.2f}')
            elif param.name == 'turn_gain':
                self.turn_gain = float(param.value)
                self.get_logger().info(f'Установлен turn_gain: {self.turn_gain:.2f}')

        return SetParametersResult(successful=True)


class DrawRouteApp:
    """Графический интерфейс Tkinter с интерактивными ползунками и интеграцией с ROS 2."""

    def __init__(self, root: tk.Tk, node: DrawRouteNode) -> None:
        self.root = root
        self.node = node
        self.root.title("Управление turtlesim: Маршрут мышью и параметры (ПР04)")
        self.root.resizable(False, False)

        # Состояние ROS
        self.latest_pose: Pose | None = None
        self.last_pose_time: float = 0.0

        # Состояние маршрута
        self.raw_mouse_points: list[tuple[float, float]] = []
        self.route_waypoints: list[tuple[float, float]] = []
        self.current_waypoint_idx: int = 0
        self.is_running: bool = False

        # Метрики движения для сравнения проходов
        self.run_start_time: float = 0.0
        self.run_deviations: list[float] = []

        # Подписка и публикация
        self.pose_sub = self.node.create_subscription(
            Pose,
            '/turtle1/pose',
            self._on_pose,
            10,
        )
        self.cmd_pub = self.node.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10,
        )

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Периодический опрос ROS и обновление GUI (каждые 30 мс)
        self.root.after(30, self._update_loop)

    def _build_ui(self) -> None:
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Левая колонка: Холст и кнопки управления
        frame_left = ttk.Frame(main_frame)
        frame_left.pack(side=tk.LEFT, padx=(0, 10), fill=tk.BOTH)

        self.canvas = tk.Canvas(
            frame_left,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg="#f0f8ff",
            highlightthickness=2,
            highlightbackground="#4682b4",
        )
        self.canvas.pack(side=tk.TOP, pady=(0, 10))
        self._draw_grid()

        # События мыши на холсте
        self.canvas.bind("<Button-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)

        # Панель кнопок
        frame_ctrl = ttk.Frame(frame_left)
        frame_ctrl.pack(fill=tk.X, pady=(0, 5))

        self.btn_start = ttk.Button(frame_ctrl, text="Запустить", command=self.start_route)
        self.btn_start.pack(side=tk.LEFT, padx=3)

        self.btn_stop = ttk.Button(frame_ctrl, text="Стоп", command=self.stop_route)
        self.btn_stop.pack(side=tk.LEFT, padx=3)

        self.btn_repeat = ttk.Button(frame_ctrl, text="Повторить", command=self.repeat_route)
        self.btn_repeat.pack(side=tk.LEFT, padx=3)

        self.btn_clear = ttk.Button(frame_ctrl, text="Очистить", command=self.clear_route)
        self.btn_clear.pack(side=tk.LEFT, padx=3)

        # Метка статуса
        self.status_var = tk.StringVar(value="Статус: Ожидание позы turtlesim...")
        self.lbl_status = ttk.Label(frame_left, textvariable=self.status_var, font=("Arial", 9))
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X)

        # Правая колонка: Ползунки параметров и метрики
        frame_right = ttk.Frame(main_frame, width=280)
        frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Блок параметров
        params_group = ttk.LabelFrame(frame_right, text="Параметры контроллера", padding=10)
        params_group.pack(fill=tk.X, pady=(0, 10))

        # 1. max_speed
        lbl_s_title = ttk.Label(params_group, text="Макс. скорость (max_speed):")
        lbl_s_title.pack(anchor=tk.W)
        self.speed_val_var = tk.StringVar(value=f"{self.node.max_speed:.2f} м/с")
        lbl_s_val = ttk.Label(params_group, textvariable=self.speed_val_var, font=("Arial", 9, "bold"), foreground="#0055aa")
        lbl_s_val.pack(anchor=tk.E)

        self.scale_speed = ttk.Scale(
            params_group,
            from_=0.1,
            to=2.0,
            value=self.node.max_speed,
            command=self._on_speed_scale,
        )
        self.scale_speed.pack(fill=tk.X, pady=(0, 10))

        # 2. goal_tolerance
        lbl_t_title = ttk.Label(params_group, text="Точность цели (goal_tolerance):")
        lbl_t_title.pack(anchor=tk.W)
        self.tol_val_var = tk.StringVar(value=f"{self.node.goal_tolerance:.2f} м")
        lbl_t_val = ttk.Label(params_group, textvariable=self.tol_val_var, font=("Arial", 9, "bold"), foreground="#0055aa")
        lbl_t_val.pack(anchor=tk.E)

        self.scale_tol = ttk.Scale(
            params_group,
            from_=0.02,
            to=0.5,
            value=self.node.goal_tolerance,
            command=self._on_tol_scale,
        )
        self.scale_tol.pack(fill=tk.X, pady=(0, 10))

        # 3. turn_gain
        lbl_g_title = ttk.Label(params_group, text="Коэф. поворота (turn_gain):")
        lbl_g_title.pack(anchor=tk.W)
        self.gain_val_var = tk.StringVar(value=f"{self.node.turn_gain:.2f}")
        lbl_g_val = ttk.Label(params_group, textvariable=self.gain_val_var, font=("Arial", 9, "bold"), foreground="#0055aa")
        lbl_g_val.pack(anchor=tk.E)

        self.scale_gain = ttk.Scale(
            params_group,
            from_=0.5,
            to=5.0,
            value=self.node.turn_gain,
            command=self._on_gain_scale,
        )
        self.scale_gain.pack(fill=tk.X, pady=(0, 10))

        # Кнопка сброса параметров
        self.btn_reset = ttk.Button(params_group, text="Сбросить по умолчанию", command=self._reset_params)
        self.btn_reset.pack(fill=tk.X, pady=(5, 0))

        # Блок метрик последнего заезда
        metrics_group = ttk.LabelFrame(frame_right, text="Сравнение заездов", padding=10)
        metrics_group.pack(fill=tk.X, pady=(0, 10))

        self.metric_speed_var = tk.StringVar(value="Скорость в заезде: —")
        ttk.Label(metrics_group, textvariable=self.metric_speed_var).pack(anchor=tk.W, pady=1)

        self.metric_time_var = tk.StringVar(value="Время движения: —")
        ttk.Label(metrics_group, textvariable=self.metric_time_var).pack(anchor=tk.W, pady=1)

        self.metric_mean_dev_var = tk.StringVar(value="Среднее отклонение: —")
        ttk.Label(metrics_group, textvariable=self.metric_mean_dev_var).pack(anchor=tk.W, pady=1)

        self.metric_max_dev_var = tk.StringVar(value="Макс. отклонение: —")
        ttk.Label(metrics_group, textvariable=self.metric_max_dev_var).pack(anchor=tk.W, pady=1)

        hint_text = (
            "Подсказка: нарисуйте маршрут и пройдите его на max_speed=0.5, "
            "затем увеличьте скорость до 1.2 и нажмите «Повторить». "
            "Сравните длительность и отклонение!"
        )
        lbl_hint = ttk.Label(frame_right, text=hint_text, wraplength=260, foreground="#666666", font=("Arial", 8))
        lbl_hint.pack(fill=tk.X, pady=(5, 0))

    def _draw_grid(self) -> None:
        """Нарисовать координатную сетку 1..10 на холсте."""
        self.canvas.delete("grid")
        for i in range(1, 10):
            u = (i / 9.0) * CANVAS_WIDTH
            self.canvas.create_line(u, 0, u, CANVAS_HEIGHT, fill="#d0e0f0", tags="grid")
            v = (i / 9.0) * CANVAS_HEIGHT
            self.canvas.create_line(0, v, CANVAS_WIDTH, v, fill="#d0e0f0", tags="grid")

        self.canvas.create_text(15, 15, text="Y=10", anchor=tk.NW, fill="#888888", font=("Arial", 8), tags="grid")
        self.canvas.create_text(CANVAS_WIDTH - 15, CANVAS_HEIGHT - 15, text="X=10", anchor=tk.SE, fill="#888888", font=("Arial", 8), tags="grid")
        self.canvas.create_text(15, CANVAS_HEIGHT - 15, text="(1, 1)", anchor=tk.SW, fill="#888888", font=("Arial", 8), tags="grid")

    def _on_speed_scale(self, val_str: str) -> None:
        val = round(float(val_str), 2)
        self.speed_val_var.set(f"{val:.2f} м/с")
        self.node.set_parameters([Parameter('max_speed', Parameter.Type.DOUBLE, val)])

    def _on_tol_scale(self, val_str: str) -> None:
        val = round(float(val_str), 2)
        self.tol_val_var.set(f"{val:.2f} м")
        self.node.set_parameters([Parameter('goal_tolerance', Parameter.Type.DOUBLE, val)])

    def _on_gain_scale(self, val_str: str) -> None:
        val = round(float(val_str), 2)
        self.gain_val_var.set(f"{val:.2f}")
        self.node.set_parameters([Parameter('turn_gain', Parameter.Type.DOUBLE, val)])

    def _reset_params(self) -> None:
        self.scale_speed.set(0.5)
        self.scale_tol.set(0.1)
        self.scale_gain.set(2.0)
        self._on_speed_scale("0.5")
        self._on_tol_scale("0.1")
        self._on_gain_scale("2.0")

    def _on_pose(self, msg: Pose) -> None:
        self.latest_pose = msg
        self.last_pose_time = time.monotonic()

    def _on_mouse_down(self, event: tk.Event) -> None:
        if self.is_running:
            return
        self.clear_route()
        tx, ty = canvas_to_turtlesim(event.x, event.y, CANVAS_WIDTH, CANVAS_HEIGHT)
        self.raw_mouse_points.append((tx, ty))
        self.prev_canvas_pt = (event.x, event.y)

    def _on_mouse_drag(self, event: tk.Event) -> None:
        if self.is_running or not self.raw_mouse_points:
            return
        u = max(0, min(event.x, CANVAS_WIDTH))
        v = max(0, min(event.y, CANVAS_HEIGHT))

        tx, ty = canvas_to_turtlesim(u, v, CANVAS_WIDTH, CANVAS_HEIGHT)
        self.raw_mouse_points.append((tx, ty))

        self.canvas.create_line(
            self.prev_canvas_pt[0],
            self.prev_canvas_pt[1],
            u,
            v,
            fill="#1e90ff",
            width=3,
            capstyle=tk.ROUND,
            smooth=True,
            tags="drawn_route",
        )
        self.prev_canvas_pt = (u, v)

    def _on_mouse_up(self, event: tk.Event) -> None:
        if self.is_running or not self.raw_mouse_points:
            return
        self.route_waypoints = filter_and_interpolate_points(self.raw_mouse_points)
        count = len(self.route_waypoints)
        if count > 0:
            self.status_var.set(f"Маршрут готов: {count} точек. Нажмите «Запустить».")
        else:
            self.status_var.set("Маршрут пуст. Нарисуйте линию на холсте.")

    def start_route(self) -> None:
        if not self.route_waypoints:
            self.status_var.set("Ошибка: маршрут не нарисован!")
            return
        if self.latest_pose is None or (time.monotonic() - self.last_pose_time > 0.5):
            self.status_var.set("Ошибка: нет актуальных данных позы от turtlesim!")
            return

        self.is_running = True
        self.current_waypoint_idx = 0
        self.run_start_time = time.monotonic()
        self.run_deviations.clear()
        self.btn_start.configure(state=tk.DISABLED)
        self.btn_repeat.configure(state=tk.DISABLED)
        self.status_var.set(f"Движение к точке 1/{len(self.route_waypoints)}...")

    def repeat_route(self) -> None:
        """Повторить прохождение того же маршрута (например, с новой скоростью)."""
        if not self.route_waypoints:
            self.status_var.set("Маршрут не сохранён. Нарисуйте новый путь.")
            return
        self.start_route()

    def stop_route(self) -> None:
        self.is_running = False
        self.btn_start.configure(state=tk.NORMAL)
        self.btn_repeat.configure(state=tk.NORMAL)
        self._publish_zero_cmd()
        self.status_var.set("Движение остановлено (Стоп).")

    def clear_route(self) -> None:
        self.stop_route()
        self.raw_mouse_points.clear()
        self.route_waypoints.clear()
        self.current_waypoint_idx = 0
        self.run_deviations.clear()
        self.canvas.delete("drawn_route")
        self.canvas.delete("target_marker")
        self.status_var.set("Холст очищен. Зажмите мышь для рисования нового пути.")

    def _publish_zero_cmd(self) -> None:
        """Опубликовать нулевую команду скорости для остановки робота."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _record_finish_metrics(self) -> None:
        """Подсчитать и вывести финальные метрики прохождения маршрута."""
        duration = time.monotonic() - self.run_start_time
        mean_dev = (sum(self.run_deviations) / len(self.run_deviations)) if self.run_deviations else 0.0
        max_dev = max(self.run_deviations) if self.run_deviations else 0.0

        self.metric_speed_var.set(f"Скорость в заезде: {self.node.max_speed:.2f} м/с")
        self.metric_time_var.set(f"Время движения: {duration:.2f} с")
        self.metric_mean_dev_var.set(f"Среднее отклонение: {mean_dev:.3f} м")
        self.metric_max_dev_var.set(f"Макс. отклонение: {max_dev:.3f} м")

    def _update_loop(self) -> None:
        """Периодический опрос ROS 2 и обновление состояния окна."""
        rclpy.spin_once(self.node, timeout_sec=0)

        # Синхронизация ползунков при изменении параметров через консоль
        if abs(self.node.max_speed - self.scale_speed.get()) > 0.05:
            self.scale_speed.set(self.node.max_speed)
            self.speed_val_var.set(f"{self.node.max_speed:.2f} м/с")
        if abs(self.node.goal_tolerance - self.scale_tol.get()) > 0.01:
            self.scale_tol.set(self.node.goal_tolerance)
            self.tol_val_var.set(f"{self.node.goal_tolerance:.2f} м")
        if abs(self.node.turn_gain - self.scale_gain.get()) > 0.05:
            self.scale_gain.set(self.node.turn_gain)
            self.gain_val_var.set(f"{self.node.turn_gain:.2f}")

        # Проверка актуальности позы
        has_fresh_pose = (
            self.latest_pose is not None and (time.monotonic() - self.last_pose_time <= 0.5)
        )

        if not has_fresh_pose and self.is_running:
            self.stop_route()
            self.status_var.set("Остановка: потеряна поза turtlesim (таймаут > 0.5 c)!")

        # Отрисовка текущего положения черепахи
        if self.latest_pose is not None:
            self._draw_turtle_indicator(self.latest_pose.x, self.latest_pose.y)

        # Шаг управления при активном следовании
        if self.is_running and has_fresh_pose and self.latest_pose is not None:
            # Замер отклонения от ломаной линии маршрута
            dev = compute_cross_track_error(
                self.latest_pose.x, self.latest_pose.y, self.route_waypoints
            )
            self.run_deviations.append(dev)

            if self.current_waypoint_idx < len(self.route_waypoints):
                target_x, target_y = self.route_waypoints[self.current_waypoint_idx]
                lin_x, ang_z, dist, is_reached = compute_route_step(
                    self.latest_pose.x,
                    self.latest_pose.y,
                    self.latest_pose.theta,
                    target_x,
                    target_y,
                    max_speed=self.node.max_speed,
                    goal_tolerance=self.node.goal_tolerance,
                    turn_gain=self.node.turn_gain,
                )

                if is_reached:
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.route_waypoints):
                        # Финиш маршрута
                        self.stop_route()
                        self._record_finish_metrics()
                        self.status_var.set(
                            f"Маршрут завершён! v={self.node.max_speed:.2f} м/с, "
                            f"время: {time.monotonic() - self.run_start_time:.1f} с"
                        )
                        self.canvas.delete("target_marker")
                    else:
                        self.status_var.set(
                            f"Движение к точке {self.current_waypoint_idx + 1}/{len(self.route_waypoints)} "
                            f"(d={dist:.2f}, v={self.node.max_speed:.2f})"
                        )
                else:
                    cmd = Twist()
                    cmd.linear.x = lin_x
                    cmd.angular.z = ang_z
                    self.cmd_pub.publish(cmd)

                    # Рисуем маркер текущей цели
                    tu, tv = turtlesim_to_canvas(target_x, target_y, CANVAS_WIDTH, CANVAS_HEIGHT)
                    self.canvas.delete("target_marker")
                    self.canvas.create_oval(
                        tu - 5, tv - 5, tu + 5, tv + 5, fill="#ff4500", outline="black", tags="target_marker"
                    )

        # Следующая итерация через 30 мс
        self.root.after(30, self._update_loop)

    def _draw_turtle_indicator(self, x: float, y: float) -> None:
        """Нарисовать положение черепахи на холсте."""
        u, v = turtlesim_to_canvas(x, y, CANVAS_WIDTH, CANVAS_HEIGHT)
        self.canvas.delete("turtle_indicator")
        r = 6
        self.canvas.create_oval(
            u - r, v - r, u + r, v + r, fill="#32cd32", outline="#006400", width=2, tags="turtle_indicator"
        )

    def _on_close(self) -> None:
        """Корректное завершение при закрытии окна."""
        self.is_running = False
        try:
            self._publish_zero_cmd()
        except Exception:
            pass
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DrawRouteNode()
    root = tk.Tk()
    app = DrawRouteApp(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app._on_close()


if __name__ == "__main__":
    main()
