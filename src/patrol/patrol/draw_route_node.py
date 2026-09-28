"""Нода рисования маршрута мышью и следования по нему (ПР03 Дополнительно)."""

from __future__ import annotations

import sys
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from patrol.route_utils import (
    canvas_to_turtlesim,
    compute_route_step,
    filter_and_interpolate_points,
    turtlesim_to_canvas,
)

CANVAS_WIDTH = 500
CANVAS_HEIGHT = 500


class DrawRouteApp:
    """Графический интерфейс Tkinter с интеграцией в цикл событий ROS 2."""

    def __init__(self, root: tk.Tk, node: Node) -> None:
        self.root = root
        self.node = node
        self.root.title("Управление turtlesim: Маршрут мышью (ПР03)")
        self.root.resizable(False, False)

        # Состояние ROS
        self.latest_pose: Pose | None = None
        self.last_pose_time: float = 0.0

        # Состояние маршрута
        self.raw_mouse_points: list[tuple[float, float]] = []
        self.route_waypoints: list[tuple[float, float]] = []
        self.current_waypoint_idx: int = 0
        self.is_running: bool = False

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

        # Запуск периодического цикла опроса ROS и обновления GUI (каждые 30 мс)
        self.root.after(30, self._update_loop)

    def _build_ui(self) -> None:
        frame_top = ttk.Frame(self.root, padding=10)
        frame_top.pack(fill=tk.BOTH, expand=True)

        # Холст
        self.canvas = tk.Canvas(
            frame_top,
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

        # Панель управления и кнопок
        frame_ctrl = ttk.Frame(frame_top)
        frame_ctrl.pack(fill=tk.X)

        self.btn_start = ttk.Button(frame_ctrl, text="Запустить", command=self.start_route)
        self.btn_start.pack(side=tk.LEFT, padx=5)

        self.btn_stop = ttk.Button(frame_ctrl, text="Стоп", command=self.stop_route)
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        self.btn_clear = ttk.Button(frame_ctrl, text="Очистить", command=self.clear_route)
        self.btn_clear.pack(side=tk.LEFT, padx=5)

        # Метка статуса
        self.status_var = tk.StringVar(value="Статус: Ожидание позы черепахи...")
        self.lbl_status = ttk.Label(frame_top, textvariable=self.status_var, font=("Arial", 10))
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X, pady=(5, 0))

    def _draw_grid(self) -> None:
        """Нарисовать координатную сетку 1..10 на холсте."""
        self.canvas.delete("grid")
        for i in range(1, 10):
            # Вертикальные линии
            u = (i / 9.0) * CANVAS_WIDTH
            self.canvas.create_line(u, 0, u, CANVAS_HEIGHT, fill="#d0e0f0", tags="grid")
            # Горизонтальные линии
            v = (i / 9.0) * CANVAS_HEIGHT
            self.canvas.create_line(0, v, CANVAS_WIDTH, v, fill="#d0e0f0", tags="grid")

        # Границы рабочей области
        self.canvas.create_text(
            15, 15, text="Y=10", anchor=tk.NW, fill="#888888", font=("Arial", 8), tags="grid"
        )
        self.canvas.create_text(
            CANVAS_WIDTH - 15, CANVAS_HEIGHT - 15, text="X=10", anchor=tk.SE, fill="#888888", font=("Arial", 8), tags="grid"
        )
        self.canvas.create_text(
            15, CANVAS_HEIGHT - 15, text="(1, 1)", anchor=tk.SW, fill="#888888", font=("Arial", 8), tags="grid"
        )

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
        # Ограничиваем координаты границами холста
        u = max(0, min(event.x, CANVAS_WIDTH))
        v = max(0, min(event.y, CANVAS_HEIGHT))

        tx, ty = canvas_to_turtlesim(u, v, CANVAS_WIDTH, CANVAS_HEIGHT)
        self.raw_mouse_points.append((tx, ty))

        # Отрисовка отрезка на холсте
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
        # Фильтрация и интерполяция точек
        self.route_waypoints = filter_and_interpolate_points(self.raw_mouse_points)
        count = len(self.route_waypoints)
        if count > 0:
            self.status_var.set(f"Маршрут зафиксирован: {count} точек. Нажмите «Запустить».")
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
        self.btn_start.configure(state=tk.DISABLED)
        self.status_var.set(f"Движение к точке 1/{len(self.route_waypoints)}...")

    def stop_route(self) -> None:
        self.is_running = False
        self.btn_start.configure(state=tk.NORMAL)
        self._publish_zero_cmd()
        self.status_var.set("Движение остановлено (Стоп).")

    def clear_route(self) -> None:
        self.stop_route()
        self.raw_mouse_points.clear()
        self.route_waypoints.clear()
        self.current_waypoint_idx = 0
        self.canvas.delete("drawn_route")
        self.canvas.delete("target_marker")
        self.status_var.set("Холст очищен. Зажмите мышь для рисования нового пути.")

    def _publish_zero_cmd(self) -> None:
        """Опубликовать нулевую команду скорости для остановки робота."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _update_loop(self) -> None:
        """Периодический вызов spin_once и вычисление шага управления."""
        rclpy.spin_once(self.node, timeout_sec=0)

        # Проверка актуальности позы
        has_fresh_pose = (
            self.latest_pose is not None and (time.monotonic() - self.last_pose_time <= 0.5)
        )

        if not has_fresh_pose and self.is_running:
            self.stop_route()
            self.status_var.set("Остановка: потеряна поза turtlesim (таймаут > 0.5 c)!")

        # Отрисовка текущего положения черепахи на холсте
        if self.latest_pose is not None:
            self._draw_turtle_indicator(self.latest_pose.x, self.latest_pose.y)

        # Если активно движение — вычисляем управление
        if self.is_running and has_fresh_pose and self.latest_pose is not None:
            if self.current_waypoint_idx < len(self.route_waypoints):
                target_x, target_y = self.route_waypoints[self.current_waypoint_idx]
                lin_x, ang_z, dist, is_reached = compute_route_step(
                    self.latest_pose.x,
                    self.latest_pose.y,
                    self.latest_pose.theta,
                    target_x,
                    target_y,
                )

                if is_reached:
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.route_waypoints):
                        # Все точки пройдены
                        self.stop_route()
                        self.status_var.set("Маршрут успешно пройден!")
                        self.canvas.delete("target_marker")
                    else:
                        self.status_var.set(
                            f"Движение к точке {self.current_waypoint_idx + 1}/{len(self.route_waypoints)} (d={dist:.2f})"
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

        # Следующая итерация через 30 мс (~33 Гц)
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
    node = Node("draw_route")
    root = tk.Tk()
    app = DrawRouteApp(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app._on_close()


if __name__ == "__main__":
    main()
