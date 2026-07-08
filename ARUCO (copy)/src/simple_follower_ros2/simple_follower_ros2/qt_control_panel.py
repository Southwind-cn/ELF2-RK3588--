#!/usr/bin/env python3
# coding=utf-8

import os
import signal
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

from PyQt5.QtCore import QProcess, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QStackedWidget,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker


PACKAGE_NAME = "simple_follower_ros2"
TERMINAL_MARKER = "ARUCO_QT_TERMINAL_MARKER"
TERMINAL_PID_FILE = "/tmp/aruco_qt_terminal_pids.txt"


class RosMonitorNode(Node):
    def __init__(self, status_callback):
        super().__init__("simple_follower_qt_monitor")
        qos = QoSProfile(depth=10)
        self._status_callback = status_callback
        self._last_cmd_time = 0.0
        self._last_marker_time = 0.0
        self._stop_publisher = self.create_publisher(Twist, "/cmd_vel", qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, qos)
        self.create_subscription(Marker, "/aruco_single/marker", self._marker_callback, qos)

    def _cmd_callback(self, msg):
        self._last_cmd_time = time.time()
        self._status_callback(
            {
                "type": "cmd",
                "linear": msg.linear.x,
                "angular": msg.angular.z,
                "stamp": self._last_cmd_time,
            }
        )

    def _marker_callback(self, msg):
        self._last_marker_time = time.time()
        self._status_callback(
            {
                "type": "marker",
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "stamp": self._last_marker_time,
            }
        )

    def publish_stop(self):
        self._stop_publisher.publish(Twist())


class RosMonitorThread(QThread):
    status_changed = pyqtSignal(dict)
    ros_error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self._node = None

    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = RosMonitorNode(self.status_changed.emit)
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as exc:
            self.ros_error.emit(str(exc))
        finally:
            if self._node is not None:
                self._node.destroy_node()
                self._node = None

    def stop(self):
        self._running = False
        self.wait(1200)

    def publish_stop(self):
        if self._node is not None:
            self._node.publish_stop()


class ModeCard(QFrame):
    clicked = pyqtSignal(str)

    def __init__(self, mode_id, title, subtitle, points, parent=None):
        super().__init__(parent)
        self.mode_id = mode_id
        self.setObjectName("modeCard")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(168)
        self._selected = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(8)

        self.title = QLabel(title)
        self.title.setObjectName("cardTitle")
        self.subtitle = QLabel(subtitle)
        self.subtitle.setObjectName("cardSubtitle")
        self.subtitle.setWordWrap(True)
        layout.addWidget(self.title)
        layout.addWidget(self.subtitle)

        for text in points:
            label = QLabel(text)
            label.setObjectName("pointLabel")
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addStretch(1)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.mode_id)
        super().mousePressEvent(event)

    def set_selected(self, selected):
        self._selected = selected
        self.setProperty("selected", selected)
        self.style().unpolish(self)
        self.style().polish(self)


class FollowerControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARUCO 与视觉巡线控制台")
        self.resize(1120, 760)
        self.process = None
        self.terminal_processes = []
        self.detached_running = False
        self.active_mode = None
        self.last_cmd_stamp = 0.0
        self.last_marker_stamp = 0.0
        self.stop_publish_count = 0
        self.workspace_root = self._detect_workspace_root()

        self._build_ui()
        self._apply_style()
        self._set_mode("aruco")

        self.monitor = RosMonitorThread(self)
        self.monitor.status_changed.connect(self._update_ros_status)
        self.monitor.ros_error.connect(self._append_error)
        self.monitor.start()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status_age)
        self.status_timer.start(500)

    def _detect_workspace_root(self):
        desktop = Path.home() / "Desktop"
        candidates = [
            desktop / "auro(test)",
            desktop / "ARUCO(test)",
            desktop / "test",
            desktop / "ARUCO",
            desktop / "ARUCO (copy)",
        ]
        for candidate in candidates:
            if (candidate / "install" / "setup.bash").exists():
                return candidate
        cwd = Path.cwd()
        if (cwd / "install" / "setup.bash").exists():
            return cwd
        for parent in Path(__file__).resolve().parents:
            if (parent / "install" / "setup.bash").exists():
                return parent
        return cwd

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(24, 22, 24, 20)
        main.setSpacing(18)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("ARUCO / 视觉巡线算法可视化界面")
        title.setObjectName("pageTitle")
        subtitle = QLabel("选择功能、启动 ROS2 launch，并查看速度输出、识别状态和运行日志。")
        subtitle.setObjectName("pageSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))

        self.state_badge = QLabel("未运行")
        self.state_badge.setObjectName("stateBadge")
        self.state_badge.setAlignment(Qt.AlignCenter)
        header.addWidget(self.state_badge)
        main.addLayout(header)

        content = QHBoxLayout()
        content.setSpacing(18)
        main.addLayout(content, 1)

        left = QVBoxLayout()
        left.setSpacing(14)
        content.addLayout(left, 2)

        cards = QHBoxLayout()
        cards.setSpacing(14)
        self.aruco_card = ModeCard(
            "aruco",
            "ArUco 跟随",
            "识别指定 ArUco 标记，根据标记距离和横向偏差发布 /cmd_vel。",
            ["输入: /aruco_single/marker", "输出: /cmd_vel", "适用: 标记追踪、定距跟随"],
        )
        self.line_card = ModeCard(
            "line",
            "视觉巡线",
            "从摄像头图像提取目标颜色线条，按中心偏差控制小车转向。",
            ["输入: /image_raw 或 /camera/color/image_raw", "输出: /cmd_vel", "适用: 色带路线巡航"],
        )
        self.aruco_card.clicked.connect(self._set_mode)
        self.line_card.clicked.connect(self._set_mode)
        cards.addWidget(self.aruco_card)
        cards.addWidget(self.line_card)
        left.addLayout(cards)

        self.detail_stack = QStackedWidget()
        self.detail_stack.addWidget(self._build_aruco_detail())
        self.detail_stack.addWidget(self._build_line_detail())
        left.addWidget(self.detail_stack)

        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(210)
        log_layout.addWidget(self.log_view)
        left.addWidget(log_group, 1)

        right = QVBoxLayout()
        right.setSpacing(14)
        content.addLayout(right, 1)

        control_group = QGroupBox("启动控制")
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(12)

        row = QGridLayout()
        row.setHorizontalSpacing(10)
        row.setVerticalSpacing(10)
        row.addWidget(QLabel("摄像头类型"), 0, 0)
        self.camera_combo = QComboBox()
        self.camera_combo.addItem("USB 摄像头", "true")
        self.camera_combo.addItem("Astra 彩色相机", "false")
        row.addWidget(self.camera_combo, 0, 1)
        row.addWidget(QLabel("USB 设备"), 1, 0)
        self.video_device = QLineEdit("/dev/video21")
        row.addWidget(self.video_device, 1, 1)
        row.addWidget(QLabel("工作目录"), 2, 0)
        self.workspace_input = QLineEdit(str(self.workspace_root))
        row.addWidget(self.workspace_input, 2, 1)
        control_layout.addLayout(row)

        button_row = QHBoxLayout()
        self.start_button = QPushButton("启动当前功能")
        self.stop_button = QPushButton("停止")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self._start_current_mode)
        self.stop_button.clicked.connect(self._stop_process)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        control_layout.addLayout(button_row)

        self.command_label = QLabel("")
        self.command_label.setObjectName("commandLabel")
        self.command_label.setWordWrap(True)
        control_layout.addWidget(self.command_label)
        right.addWidget(control_group)

        telemetry = QGroupBox("实时状态")
        telemetry_layout = QGridLayout(telemetry)
        telemetry_layout.setVerticalSpacing(12)
        self.mode_value = self._metric_label("-")
        self.cmd_value = self._metric_label("linear 0.000 / angular 0.000")
        self.marker_value = self._metric_label("未检测")
        self.cmd_age_value = self._metric_label("无速度消息")
        telemetry_layout.addWidget(QLabel("当前功能"), 0, 0)
        telemetry_layout.addWidget(self.mode_value, 0, 1)
        telemetry_layout.addWidget(QLabel("速度输出"), 1, 0)
        telemetry_layout.addWidget(self.cmd_value, 1, 1)
        telemetry_layout.addWidget(QLabel("ArUco 标记"), 2, 0)
        telemetry_layout.addWidget(self.marker_value, 2, 1)
        telemetry_layout.addWidget(QLabel("话题活跃"), 3, 0)
        telemetry_layout.addWidget(self.cmd_age_value, 3, 1)
        right.addWidget(telemetry)

        flow = QGroupBox("算法流程")
        flow_layout = QVBoxLayout(flow)
        self.flow_label = QLabel()
        self.flow_label.setObjectName("flowLabel")
        self.flow_label.setWordWrap(True)
        flow_layout.addWidget(self.flow_label)
        right.addWidget(flow, 1)

    def _build_aruco_detail(self):
        widget = QGroupBox("ArUco 跟随参数展示")
        layout = QGridLayout(widget)
        items = [
            ("目标距离", "goal_x = 0.60 m"),
            ("横向死区", "lateral_deadband = 0.05 m"),
            ("最大线速度", "0.35 m/s"),
            ("最大角速度", "0.42 rad/s"),
            ("丢标停车", "0.4 s 未收到标记即停车"),
            ("Launch", "aruco_follower.launch.py"),
        ]
        for index, (name, value) in enumerate(items):
            layout.addWidget(QLabel(name), index, 0)
            layout.addWidget(self._metric_label(value), index, 1)
        return widget

    def _build_line_detail(self):
        widget = QGroupBox("视觉巡线参数展示")
        layout = QGridLayout(widget)
        items = [
            ("颜色选择", "OpenCV Adjust_hsv 窗口轨迹条"),
            ("检测区域", "图像底部 30 px"),
            ("巡线速度", "linear.x = 0.40 m/s"),
            ("转向比例", "angular.z = -error * 0.0011"),
            ("丢线策略", "未检测到目标颜色时停车"),
            ("Launch", "line_follower.launch.py"),
        ]
        for index, (name, value) in enumerate(items):
            layout.addWidget(QLabel(name), index, 0)
            layout.addWidget(self._metric_label(value), index, 1)
        return widget

    def _metric_label(self, text):
        label = QLabel(text)
        label.setObjectName("metricValue")
        label.setWordWrap(True)
        return label

    def _apply_style(self):
        self.setStyleSheet(
            """
            QWidget {
                background: #f4f6f8;
                color: #182026;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", Arial, sans-serif;
                font-size: 14px;
            }
            #pageTitle {
                font-size: 28px;
                font-weight: 700;
            }
            #pageSubtitle {
                color: #66717d;
                font-size: 14px;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d9e0e7;
                border-radius: 8px;
                margin-top: 12px;
                padding: 16px 12px 12px 12px;
                font-weight: 700;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #23313d;
            }
            #modeCard {
                background: #ffffff;
                border: 1px solid #d8e0e7;
                border-radius: 8px;
            }
            #modeCard[selected="true"] {
                border: 2px solid #2474c6;
                background: #eef6ff;
            }
            #cardTitle {
                font-size: 20px;
                font-weight: 700;
            }
            #cardSubtitle, #pointLabel, #commandLabel, #flowLabel {
                color: #5d6874;
                line-height: 150%;
            }
            #pointLabel {
                padding-top: 3px;
            }
            #stateBadge {
                background: #dfe5eb;
                color: #31404d;
                border-radius: 14px;
                padding: 7px 14px;
                min-width: 84px;
                font-weight: 700;
            }
            #stateBadge[running="true"] {
                background: #d8f0e2;
                color: #17633b;
            }
            QPushButton {
                background: #2474c6;
                color: white;
                border: 0;
                border-radius: 7px;
                padding: 10px 14px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #1d65ad;
            }
            QPushButton:disabled {
                background: #b8c3ce;
            }
            QPushButton#stopButton, QPushButton[text="停止"] {
                background: #b94747;
            }
            QLineEdit, QComboBox, QTextEdit {
                background: #ffffff;
                border: 1px solid #ccd5de;
                border-radius: 6px;
                padding: 7px;
            }
            QTextEdit {
                background: #101820;
                color: #dbe7ef;
                font-family: "DejaVu Sans Mono", Consolas, monospace;
                font-size: 12px;
            }
            #metricValue {
                color: #1f3447;
                font-weight: 600;
            }
            """
        )
        self.stop_button.setObjectName("stopButton")

    def _set_mode(self, mode_id):
        if self.process is not None or self.detached_running:
            QMessageBox.information(self, "功能运行中", "请先停止当前功能，再切换到另一个功能。")
            return
        self.active_mode = mode_id
        is_aruco = mode_id == "aruco"
        self.aruco_card.set_selected(is_aruco)
        self.line_card.set_selected(not is_aruco)
        self.detail_stack.setCurrentIndex(0 if is_aruco else 1)
        self.mode_value.setText("ArUco 跟随" if is_aruco else "视觉巡线")
        self.flow_label.setText(
            "摄像头图像 -> aruco_ros 识别标记 -> 发布 /aruco_single/marker -> ar_follow 计算速度 -> /cmd_vel 控制底盘"
            if is_aruco
            else "摄像头图像 -> HSV 颜色阈值分割 -> 计算底部目标线重心 -> line_follow 计算转向 -> /cmd_vel 控制底盘"
        )
        self._update_command_preview()

    def _launch_file_for_mode(self):
        if self.active_mode == "aruco":
            return "aruco_follower.launch.py"
        return "line_follower.launch.py"

    def _update_command_preview(self):
        command = self._build_launch_command()
        self.command_label.setText("命令: " + command)

    def _build_launch_command(self):
        if self.active_mode == "aruco":
            return (
                "终端1: ros2 launch aruco_ros aruco_recognize.launch.py | "
                "终端2: rqt | "
                "终端3: ros2 launch simple_follower_ros2 aruco_follower.launch.py"
            )
        return "ros2 launch simple_follower_ros2 line_follower.launch.py"

    def _start_current_mode(self):
        if self.process is not None or self.detached_running:
            return
        self._update_command_preview()
        self._clear_terminal_pid_file()
        workspace = Path(self.workspace_input.text().strip()).expanduser()
        if not (workspace / "install" / "setup.bash").exists():
            QMessageBox.warning(
                self,
                "环境不存在",
                f"没有找到 {workspace / 'install' / 'setup.bash'}，请确认工作目录。",
            )
            return

        self.workspace_root = workspace
        if self.active_mode == "aruco":
            commands = [
                ("ArUco 识别", "ros2 launch aruco_ros aruco_recognize.launch.py"),
                ("RQT 可视化", "rqt"),
                ("ArUco 跟随", "ros2 launch simple_follower_ros2 aruco_follower.launch.py"),
            ]
        else:
            commands = [
                ("视觉巡线", "ros2 launch simple_follower_ros2 line_follower.launch.py"),
            ]

        for index, (title, command) in enumerate(commands):
            QTimer.singleShot(
                index * 1200,
                lambda title=title, command=command: self._open_command_terminal(
                    title, command, self.workspace_root
                ),
            )

        self.detached_running = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.camera_combo.setEnabled(False)
        self.video_device.setEnabled(False)
        self.workspace_input.setEnabled(False)
        self._set_running_state(True)
        self._append_log(f"工作目录: {workspace}")
        self._append_log("已按顺序打开终端启动当前功能。")

    def _open_command_terminal(self, title, command, workspace):
        if not self.detached_running:
            return
        setup_file = workspace / "install" / "setup.bash"
        shell_command = (
            f"echo $$ >> {shlex.quote(TERMINAL_PID_FILE)}; "
            f"export {TERMINAL_MARKER}=1; "
            f"cd {shlex.quote(str(workspace))} && "
            f"source {shlex.quote(str(setup_file))} && "
            f"echo '启动: {title}' && "
            f"echo '终端标记: {TERMINAL_MARKER}' && "
            f"echo '终端PID: '$$ && "
            f"{command}; "
            "echo; echo '命令已退出，终端即将关闭'; sleep 1"
        )
        terminal = self._terminal_program()
        if terminal is None:
            self._append_log("未找到可用终端程序，请安装 gnome-terminal 或 x-terminal-emulator。")
            return
        program, args = terminal
        try:
            proc = subprocess.Popen(
                [program] + args + ["bash", "-lc", shell_command],
                start_new_session=True,
            )
            self.terminal_processes.append(proc)
            self._append_log(f"$ {command}")
            self._append_log(f"终端进程 PID: {proc.pid}")
        except OSError as exc:
            self._append_log(f"启动失败: {command} ({exc})")

    def _clear_terminal_pid_file(self):
        try:
            Path(TERMINAL_PID_FILE).unlink(missing_ok=True)
        except OSError:
            pass

    def _terminal_program(self):
        if shutil.which("xterm"):
            return "xterm", ["-T", TERMINAL_MARKER, "-e"]
        if shutil.which("gnome-terminal"):
            return "gnome-terminal", ["--title", TERMINAL_MARKER, "--"]
        if shutil.which("x-terminal-emulator"):
            return "x-terminal-emulator", ["-T", TERMINAL_MARKER, "-e"]
        if shutil.which("konsole"):
            return "konsole", ["--new-tab", "-p", f"tabtitle={TERMINAL_MARKER}", "-e"]
        if shutil.which("xfce4-terminal"):
            return "xfce4-terminal", [f"--title={TERMINAL_MARKER}", "-e"]
        return None

    def _stop_process(self):
        self._start_stop_burst(4)
        if self.detached_running and self.process is None:
            self._append_log("正在停止相关 ROS2 / rqt 进程并关闭终端...")
            QProcess.startDetached("bash", ["-lc", self._stop_detached_command()])
            self._close_started_terminals()
            self._schedule_post_stop_bursts()
            self.detached_running = False
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.camera_combo.setEnabled(True)
            self.video_device.setEnabled(True)
            self.workspace_input.setEnabled(True)
            self._set_running_state(False)
            return
        if self.process is None:
            return
        self._append_log("正在停止当前功能...")
        self.process.terminate()
        self._schedule_post_stop_bursts()
        QTimer.singleShot(2500, self._kill_if_running)

    def _schedule_post_stop_bursts(self):
        for delay in (250, 700, 1300, 2100):
            QTimer.singleShot(delay, lambda: self._start_stop_burst(10))
            QTimer.singleShot(delay + 80, self._start_cli_stop_burst)

    def _close_started_terminals(self):
        for proc in self.terminal_processes:
            if proc.poll() is not None:
                continue
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except OSError:
                try:
                    proc.terminate()
                except OSError:
                    pass
        QTimer.singleShot(800, self._force_close_started_terminals)

    def _force_close_started_terminals(self):
        alive = []
        for proc in self.terminal_processes:
            if proc.poll() is not None:
                continue
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError:
                try:
                    proc.kill()
                except OSError:
                    pass
            if proc.poll() is None:
                alive.append(proc)
        self.terminal_processes = alive

    def _stop_detached_command(self):
        return (
            f"if [ -f {shlex.quote(TERMINAL_PID_FILE)} ]; then "
            f"while read -r pid; do "
            "case \"$pid\" in ''|*[!0-9]*) continue;; esac; "
            "kill -TERM $pid 2>/dev/null || true; "
            f"done < {shlex.quote(TERMINAL_PID_FILE)}; "
            "fi; "
            "pkill -TERM -f 'ros2 launch aruco_ros aruco_recognize.launch.py'; "
            "pkill -TERM -f 'ros2 launch simple_follower_ros2 aruco_follower.launch.py'; "
            "pkill -TERM -f 'ros2 launch simple_follower_ros2 line_follower.launch.py'; "
            "pkill -TERM -f 'simple_follower_ros2.*arfollower'; "
            "pkill -TERM -f 'simple_follower_ros2.*line_follow'; "
            "pkill -TERM -f 'aruco_ros.*single'; "
            "pkill -TERM -f 'aruco_recognize.launch.py'; "
            "pkill -TERM -f '^rqt$'; "
            f"pkill -TERM -f '[{TERMINAL_MARKER[0]}]{TERMINAL_MARKER[1:]}'; "
            "sleep 0.5; "
            f"if [ -f {shlex.quote(TERMINAL_PID_FILE)} ]; then "
            f"while read -r pid; do "
            "case \"$pid\" in ''|*[!0-9]*) continue;; esac; "
            "kill -KILL $pid 2>/dev/null || true; "
            f"done < {shlex.quote(TERMINAL_PID_FILE)}; "
            f"rm -f {shlex.quote(TERMINAL_PID_FILE)}; "
            "fi; "
            "pkill -KILL -f 'ros2 launch aruco_ros aruco_recognize.launch.py'; "
            "pkill -KILL -f 'ros2 launch simple_follower_ros2 aruco_follower.launch.py'; "
            "pkill -KILL -f 'ros2 launch simple_follower_ros2 line_follower.launch.py'; "
            "pkill -KILL -f 'simple_follower_ros2.*arfollower'; "
            "pkill -KILL -f 'simple_follower_ros2.*line_follow'; "
            "pkill -KILL -f 'aruco_ros.*single'; "
            "pkill -KILL -f 'aruco_recognize.launch.py'; "
            "pkill -KILL -f '^rqt$'; "
            f"pkill -KILL -f '[{TERMINAL_MARKER[0]}]{TERMINAL_MARKER[1:]}'; "
            "true"
        )

    def _start_cli_stop_burst(self):
        workspace = Path(self.workspace_input.text().strip()).expanduser()
        setup_file = workspace / "install" / "setup.bash"
        if not setup_file.exists():
            return
        stop_command = (
            f"cd {shlex.quote(str(workspace))} && "
            f"source {shlex.quote(str(setup_file))} && "
            "timeout 2.0 ros2 topic pub -r 30 /cmd_vel geometry_msgs/msg/Twist '{}'"
        )
        QProcess.startDetached("bash", ["-lc", stop_command])
        self._append_log("已发布 /cmd_vel 零速度停车命令。")

    def _start_stop_burst(self, count=12):
        self.stop_publish_count = max(self.stop_publish_count, count)
        self._publish_stop_once()

    def _publish_stop_once(self):
        if self.stop_publish_count <= 0:
            return
        self.monitor.publish_stop()
        self.cmd_value.setText("linear 0.000 / angular 0.000")
        self.stop_publish_count -= 1
        QTimer.singleShot(100, self._publish_stop_once)

    def _kill_if_running(self):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            self._append_log("进程未正常退出，执行强制停止。")
            self.process.kill()

    def _read_process_output(self):
        if self.process is None:
            return
        data = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
        if data.strip():
            self._append_log(data.rstrip())

    def _process_finished(self, exit_code, exit_status):
        self._append_log(f"进程已退出，exit_code={exit_code}, status={int(exit_status)}")
        if self.process is not None:
            self.process.deleteLater()
        self.process = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.camera_combo.setEnabled(True)
        self.video_device.setEnabled(True)
        self.workspace_input.setEnabled(True)
        self._set_running_state(False)

    def _set_running_state(self, running):
        self.state_badge.setText("运行中" if running else "未运行")
        self.state_badge.setProperty("running", running)
        self.state_badge.style().unpolish(self.state_badge)
        self.state_badge.style().polish(self.state_badge)

    def _update_ros_status(self, status):
        if status["type"] == "cmd":
            self.last_cmd_stamp = status["stamp"]
            self.cmd_value.setText(
                f"linear {status['linear']:.3f} / angular {status['angular']:.3f}"
            )
        elif status["type"] == "marker":
            self.last_marker_stamp = status["stamp"]
            self.marker_value.setText(
                f"x {status['x']:.3f}, y {status['y']:.3f}, z {status['z']:.3f}"
            )

    def _refresh_status_age(self):
        now = time.time()
        if self.last_cmd_stamp:
            self.cmd_age_value.setText(f"/cmd_vel {now - self.last_cmd_stamp:.1f}s 前更新")
        else:
            self.cmd_age_value.setText("无速度消息")
        if self.last_marker_stamp and now - self.last_marker_stamp > 1.0:
            self.marker_value.setText("超过 1s 未检测到标记")

    def _append_log(self, text):
        timestamp = time.strftime("%H:%M:%S")
        for line in text.splitlines():
            self.log_view.append(f"[{timestamp}] {line}")
        self.log_view.moveCursor(self.log_view.textCursor().End)

    def _append_error(self, text):
        self._append_log("ROS 监控线程错误: " + text)

    def closeEvent(self, event):
        if self.process is not None or self.detached_running:
            self._stop_process()
        self.monitor.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("simple_follower_qt")
    window = FollowerControlPanel()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
