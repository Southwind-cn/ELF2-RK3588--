#!/usr/bin/env python3
# coding=utf-8

import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from PyQt5.QtCore import QEvent, QProcess, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile


TERMINAL_MARKER = "NAV2_QT_TERMINAL_MARKER"
TERMINAL_PID_FILE = "/tmp/nav2_qt_terminal_pids.txt"


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class Nav2MonitorNode(Node):
    def __init__(self, status_callback):
        super().__init__("nav2_qt_monitor")
        qos = QoSProfile(depth=10)
        self._status_callback = status_callback
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", qos)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, qos)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos)
        self.create_subscription(NavPath, "/plan", self._path_callback, qos)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav_status_callback,
            qos,
        )

    def _cmd_callback(self, msg):
        self._status_callback(
            {
                "type": "cmd",
                "linear": msg.linear.x,
                "angular": msg.angular.z,
                "stamp": time.time(),
            }
        )

    def _odom_callback(self, msg):
        self._status_callback(
            {
                "type": "odom",
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "linear": msg.twist.twist.linear.x,
                "angular": msg.twist.twist.angular.z,
                "stamp": time.time(),
            }
        )

    def _path_callback(self, msg):
        self._status_callback(
            {"type": "path", "count": len(msg.poses), "stamp": time.time()}
        )

    def _nav_status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        self._status_callback(
            {"type": "nav_status", "status": status, "stamp": time.time()}
        )

    def publish_stop(self):
        self._cmd_pub.publish(Twist())

    def publish_twist(self, linear_x, linear_y, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

    def publish_goal(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._goal_pub.publish(pose)

    def publish_initial_pose(self, x, y, yaw):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(pose)


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
            self._node = Nav2MonitorNode(self.status_changed.emit)
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

    def publish_twist(self, linear_x, linear_y, angular_z):
        if self._node is not None:
            self._node.publish_twist(linear_x, linear_y, angular_z)

    def publish_goal(self, x, y, yaw):
        if self._node is not None:
            self._node.publish_goal(x, y, yaw)

    def publish_initial_pose(self, x, y, yaw):
        if self._node is not None:
            self._node.publish_initial_pose(x, y, yaw)


class ModeCard(QFrame):
    clicked = pyqtSignal(str)

    def __init__(self, mode_id, title, subtitle, points, parent=None):
        super().__init__(parent)
        self.mode_id = mode_id
        self.setObjectName("modeCard")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(160)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(8)

        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        subtitle_label = QLabel(subtitle)
        subtitle_label.setObjectName("cardSubtitle")
        subtitle_label.setWordWrap(True)
        layout.addWidget(title_label)
        layout.addWidget(subtitle_label)

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
        self.setProperty("selected", selected)
        self.style().unpolish(self)
        self.style().polish(self)


class Nav2QtPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nav2 可视化控制台")
        self.resize(1180, 800)
        self.setFocusPolicy(Qt.StrongFocus)
        self.active_mode = "navigation"
        self.detached_running = False
        self.terminal_processes = []
        self.workspace_root = self._detect_workspace_root()
        self.last_cmd_stamp = 0.0
        self.last_odom_stamp = 0.0
        self.last_path_stamp = 0.0
        self.stop_publish_count = 0
        self.teleop_keys = set()
        self.teleop_omni = False

        self._build_ui()
        self._apply_style()
        self._set_mode("navigation")

        self.monitor = RosMonitorThread(self)
        self.monitor.status_changed.connect(self._update_ros_status)
        self.monitor.ros_error.connect(self._append_error)
        self.monitor.start()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status_age)
        self.status_timer.start(500)
        QApplication.instance().installEventFilter(self)

    def _detect_workspace_root(self):
        cwd = Path.cwd()
        if (cwd / "install" / "setup.bash").exists():
            return cwd
        for parent in Path(__file__).resolve().parents:
            if (parent / "install" / "setup.bash").exists():
                return parent
        desktop_nav2 = Path.home() / "Desktop" / "nav2"
        if (desktop_nav2 / "install" / "setup.bash").exists():
            return desktop_nav2
        return cwd

    def _default_map_dir(self):
        install_map_dir = (
            self.workspace_root / "install" / "wheeltec_nav2" / "share" / "wheeltec_nav2" / "map"
        )
        if install_map_dir.exists():
            return install_map_dir
        source_map_dir = self.workspace_root / "src" / "wheeltec_robot_nav2" / "map"
        if source_map_dir.exists():
            return source_map_dir
        return install_map_dir

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(24, 22, 24, 20)
        main.setSpacing(18)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("Nav2 导航可视化界面")
        title.setObjectName("pageTitle")
        subtitle = QLabel("启动建图、导航和 RViz，监控 /cmd_vel、/odom、/plan 和导航状态。")
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

        card_row = QHBoxLayout()
        card_row.setSpacing(14)
        self.cards = {
            "navigation": ModeCard(
                "navigation",
                "Nav2 定点导航",
                "加载已有地图并启动导航栈，可在 RViz 或本界面发送目标点。",
                ["Launch: wheeltec_nav2.launch.py", "话题: /goal_pose", "监控: /plan、导航 action"],
            ),
            "slam": ModeCard(
                "slam",
                "SLAM 建图",
                "启动 slam_toolbox 在线建图，并保存到导航启动使用的地图目录。",
                ["Launch: slam_launch.py", "地图: /map", "保存: map/WHEELTEC.yaml"],
            ),
            "rviz": ModeCard(
                "rviz",
                "RViz 可视化",
                "单独打开 Wheeltec RViz 配置，用于观察地图、激光、路径和目标点。",
                ["Launch: wheeltec_rviz.launch.py", "配置: wheeltec.rviz", "工具: 2D Pose / Nav Goal"],
            ),
        }
        for card in self.cards.values():
            card.clicked.connect(self._set_mode)
            card_row.addWidget(card)
        left.addLayout(card_row)

        target_group = QGroupBox("目标点与初始位姿")
        target_layout = QGridLayout(target_group)
        target_layout.setHorizontalSpacing(10)
        target_layout.setVerticalSpacing(10)
        self.goal_x = self._spin(-1000.0, 1000.0, 0.0)
        self.goal_y = self._spin(-1000.0, 1000.0, 0.0)
        self.goal_yaw = self._spin(-180.0, 180.0, 0.0)
        target_layout.addWidget(QLabel("X / m"), 0, 0)
        target_layout.addWidget(self.goal_x, 0, 1)
        target_layout.addWidget(QLabel("Y / m"), 0, 2)
        target_layout.addWidget(self.goal_y, 0, 3)
        target_layout.addWidget(QLabel("Yaw / deg"), 0, 4)
        target_layout.addWidget(self.goal_yaw, 0, 5)
        self.goal_button = QPushButton("发布导航目标")
        self.initial_pose_button = QPushButton("设置初始位姿")
        self.goal_button.clicked.connect(self._publish_goal)
        self.initial_pose_button.clicked.connect(self._publish_initial_pose)
        target_layout.addWidget(self.goal_button, 1, 0, 1, 3)
        target_layout.addWidget(self.initial_pose_button, 1, 3, 1, 3)
        left.addWidget(target_group)

        self.teleop_group = QGroupBox("建图键盘控制")
        teleop_layout = QVBoxLayout(self.teleop_group)
        teleop_layout.setSpacing(10)
        speed_layout = QGridLayout()
        speed_layout.setHorizontalSpacing(10)
        self.teleop_linear = self._spin(0.0, 1.5, 0.2)
        self.teleop_linear.setSingleStep(0.05)
        self.teleop_angular = self._spin(0.0, 3.0, 1.0)
        self.teleop_angular.setSingleStep(0.1)
        self.omni_check = QCheckBox("全向 / 麦轮模式")
        self.omni_check.toggled.connect(self._set_teleop_omni)
        speed_layout.addWidget(QLabel("线速度 m/s"), 0, 0)
        speed_layout.addWidget(self.teleop_linear, 0, 1)
        speed_layout.addWidget(QLabel("角速度 rad/s"), 0, 2)
        speed_layout.addWidget(self.teleop_angular, 0, 3)
        speed_layout.addWidget(self.omni_check, 1, 0, 1, 4)
        teleop_layout.addLayout(speed_layout)

        pad = QGridLayout()
        pad.setHorizontalSpacing(8)
        pad.setVerticalSpacing(8)
        for row, col, key, text in [
            (0, 0, "u", "U 左前"),
            (0, 1, "i", "I 前进"),
            (0, 2, "o", "O 右前"),
            (1, 0, "j", "J 左转"),
            (1, 1, "k", "K 停止"),
            (1, 2, "l", "L 右转"),
            (2, 0, "m", "M 左后"),
            (2, 1, ",", ", 后退"),
            (2, 2, ".", ". 右后"),
        ]:
            button = QPushButton(text)
            button.setObjectName("teleopButton")
            button.pressed.connect(lambda key=key: self._press_teleop_key(key))
            button.released.connect(lambda key=key: self._release_teleop_key(key))
            pad.addWidget(button, row, col)
        teleop_layout.addLayout(pad)

        hint = QLabel("窗口获得焦点后可直接按键控制：I/J/K/L/U/O/M/,/. 移动，空格急停。")
        hint.setObjectName("teleopHint")
        hint.setWordWrap(True)
        teleop_layout.addWidget(hint)
        left.addWidget(self.teleop_group)

        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self._publish_teleop_from_keys)
        self.teleop_timer.start(100)

        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(250)
        log_layout.addWidget(self.log_view)
        left.addWidget(log_group, 1)

        right = QVBoxLayout()
        right.setSpacing(14)
        content.addLayout(right, 1)

        control_group = QGroupBox("启动控制")
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(12)
        form = QGridLayout()
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(10)
        form.addWidget(QLabel("工作目录"), 0, 0)
        self.workspace_input = QLineEdit(str(self.workspace_root))
        form.addWidget(self.workspace_input, 0, 1)
        form.addWidget(QLabel("地图文件"), 1, 0)
        self.map_input = QLineEdit("WHEELTEC.yaml")
        form.addWidget(self.map_input, 1, 1)
        form.addWidget(QLabel("地图目录"), 2, 0)
        map_dir_row = QHBoxLayout()
        self.map_dir_input = QLineEdit(str(self._default_map_dir()))
        self.open_map_dir_button = QPushButton("打开目录")
        self.open_map_dir_button.clicked.connect(self._open_map_directory)
        map_dir_row.addWidget(self.map_dir_input, 1)
        map_dir_row.addWidget(self.open_map_dir_button)
        form.addLayout(map_dir_row, 2, 1)
        form.addWidget(QLabel("use_sim_time"), 3, 0)
        self.sim_time_combo = QComboBox()
        self.sim_time_combo.addItem("false", "false")
        self.sim_time_combo.addItem("true", "true")
        form.addWidget(self.sim_time_combo, 3, 1)
        self.open_rviz_check = QCheckBox("启动时同时打开 RViz")
        self.open_rviz_check.setChecked(True)
        form.addWidget(self.open_rviz_check, 4, 0, 1, 2)
        control_layout.addLayout(form)

        button_row = QHBoxLayout()
        self.start_button = QPushButton("启动当前功能")
        self.stop_button = QPushButton("停止")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setEnabled(False)
        self.save_map_button = QPushButton("保存地图")
        self.start_button.clicked.connect(self._start_current_mode)
        self.stop_button.clicked.connect(self._stop_processes)
        self.save_map_button.clicked.connect(self._save_map)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        control_layout.addLayout(button_row)
        control_layout.addWidget(self.save_map_button)

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
        self.odom_value = self._metric_label("未收到里程计")
        self.path_value = self._metric_label("无路径")
        self.nav_status_value = self._metric_label("未收到状态")
        self.topic_age_value = self._metric_label("等待 ROS 话题")
        telemetry_layout.addWidget(QLabel("当前功能"), 0, 0)
        telemetry_layout.addWidget(self.mode_value, 0, 1)
        telemetry_layout.addWidget(QLabel("速度输出"), 1, 0)
        telemetry_layout.addWidget(self.cmd_value, 1, 1)
        telemetry_layout.addWidget(QLabel("里程计"), 2, 0)
        telemetry_layout.addWidget(self.odom_value, 2, 1)
        telemetry_layout.addWidget(QLabel("全局路径"), 3, 0)
        telemetry_layout.addWidget(self.path_value, 3, 1)
        telemetry_layout.addWidget(QLabel("导航状态"), 4, 0)
        telemetry_layout.addWidget(self.nav_status_value, 4, 1)
        telemetry_layout.addWidget(QLabel("话题活跃"), 5, 0)
        telemetry_layout.addWidget(self.topic_age_value, 5, 1)
        right.addWidget(telemetry)

        flow = QGroupBox("功能流程")
        flow_layout = QVBoxLayout(flow)
        self.flow_label = QLabel()
        self.flow_label.setObjectName("flowLabel")
        self.flow_label.setWordWrap(True)
        flow_layout.addWidget(self.flow_label)
        right.addWidget(flow, 1)

    def _spin(self, low, high, value):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(3)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        return spin

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
            QPushButton#stopButton {
                background: #b94747;
            }
            QPushButton#teleopButton {
                background: #31404d;
                min-height: 30px;
            }
            QPushButton#teleopButton:hover {
                background: #23313d;
            }
            QLineEdit, QComboBox, QDoubleSpinBox, QTextEdit {
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
            #teleopHint {
                color: #5d6874;
            }
            """
        )

    def _set_mode(self, mode_id):
        if self.detached_running:
            QMessageBox.information(self, "功能运行中", "请先停止当前功能，再切换模式。")
            return
        self.active_mode = mode_id
        names = {
            "navigation": "Nav2 定点导航",
            "slam": "SLAM 建图",
            "rviz": "RViz 可视化",
        }
        for key, card in self.cards.items():
            card.set_selected(key == mode_id)
        self.mode_value.setText(names[mode_id])
        self.flow_label.setText(
            {
                "navigation": "底盘与激光 -> AMCL 定位 -> Planner 生成 /plan -> Controller 发布 /cmd_vel -> 到达 /goal_pose",
                "slam": "底盘与激光 -> slam_toolbox 在线建图 -> 键盘发布 /cmd_vel 控制底盘 -> RViz 观察 /map -> 保存 WHEELTEC 地图",
                "rviz": "读取 wheeltec.rviz -> 显示 TF、LaserScan、Map、Path、Goal -> 配合 Nav2 调试",
            }[mode_id]
        )
        self._set_teleop_enabled(mode_id == "slam")
        self._update_command_preview()

    def _set_teleop_enabled(self, enabled):
        self.teleop_group.setEnabled(enabled)
        if not enabled:
            self.teleop_keys.clear()
            if hasattr(self, "monitor"):
                self.monitor.publish_stop()

    def _set_teleop_omni(self, enabled):
        self.teleop_omni = enabled
        self._append_log("键盘控制切换为全向模式。" if enabled else "键盘控制切换为普通差速模式。")

    def keyPressEvent(self, event):
        key = self._event_to_teleop_key(event)
        if key is None:
            super().keyPressEvent(event)
            return
        if event.isAutoRepeat():
            event.accept()
            return
        self._press_teleop_key(key)
        event.accept()

    def keyReleaseEvent(self, event):
        key = self._event_to_teleop_key(event)
        if key is None:
            super().keyReleaseEvent(event)
            return
        if event.isAutoRepeat():
            event.accept()
            return
        self._release_teleop_key(key)
        event.accept()

    def eventFilter(self, watched, event):
        if not self.isActiveWindow() or self.active_mode != "slam":
            return super().eventFilter(watched, event)
        if event.type() == QEvent.KeyPress:
            key = self._event_to_teleop_key(event)
            if key is None:
                return super().eventFilter(watched, event)
            if not event.isAutoRepeat():
                self._press_teleop_key(key)
            return True
        if event.type() == QEvent.KeyRelease:
            key = self._event_to_teleop_key(event)
            if key is None:
                return super().eventFilter(watched, event)
            if not event.isAutoRepeat():
                self._release_teleop_key(key)
            return True
        return super().eventFilter(watched, event)

    def _event_to_teleop_key(self, event):
        if self.active_mode != "slam":
            return None
        if event.key() == Qt.Key_Space:
            return "k"
        text = event.text().lower()
        if text in {"i", "o", "j", "l", "u", "m", ",", ".", "k"}:
            return text
        return None

    def _press_teleop_key(self, key):
        if self.active_mode != "slam":
            return
        if key == "k":
            self.teleop_keys.clear()
            self.monitor.publish_stop()
            self.cmd_value.setText("linear 0.000 / angular 0.000")
            self._append_log("键盘控制: 停止")
            return
        self.teleop_keys.add(key)
        self._publish_teleop_from_keys()

    def _release_teleop_key(self, key):
        if key in self.teleop_keys:
            self.teleop_keys.remove(key)
        if not self.teleop_keys:
            self.monitor.publish_stop()
            self.cmd_value.setText("linear 0.000 / angular 0.000")

    def _publish_teleop_from_keys(self):
        if self.active_mode != "slam" or not self.teleop_keys:
            return
        linear_scale = self.teleop_linear.value()
        angular_scale = self.teleop_angular.value()
        x_axis = 0.0
        turn_axis = 0.0
        bindings = {
            "i": (1, 0),
            "o": (1, -1),
            "j": (0, 1),
            "l": (0, -1),
            "u": (1, 1),
            ",": (-1, 0),
            ".": (-1, -1 if self.teleop_omni else 1),
            "m": (-1, 1 if self.teleop_omni else -1),
        }
        for key in self.teleop_keys:
            dx, dtheta = bindings.get(key, (0, 0))
            x_axis += dx
            turn_axis += dtheta
        x_axis = max(-1.0, min(1.0, x_axis))
        turn_axis = max(-1.0, min(1.0, turn_axis))
        if self.teleop_omni:
            self.monitor.publish_twist(linear_scale * x_axis, linear_scale * turn_axis, 0.0)
            self.cmd_value.setText(
                f"linear {linear_scale * x_axis:.3f} / lateral {linear_scale * turn_axis:.3f}"
            )
        else:
            self.monitor.publish_twist(linear_scale * x_axis, 0.0, angular_scale * turn_axis)
            self.cmd_value.setText(
                f"linear {linear_scale * x_axis:.3f} / angular {angular_scale * turn_axis:.3f}"
            )

    def _update_command_preview(self):
        commands = self._commands_for_mode()
        self.command_label.setText("命令: " + " | ".join(cmd for _, cmd in commands))

    def _commands_for_mode(self):
        use_sim_time = self.sim_time_combo.currentData()
        map_arg = self._map_argument()
        if self.active_mode == "navigation":
            commands = [
                (
                    "Nav2 导航",
                    f"ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py use_sim_time:={use_sim_time}{map_arg}",
                )
            ]
            if self.open_rviz_check.isChecked():
                commands.append(("RViz", "ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py"))
            return commands
        if self.active_mode == "slam":
            commands = [
                (
                    "SLAM 建图",
                    f"ros2 launch wheeltec_nav2 slam_launch.py use_sim_time:={use_sim_time}",
                )
            ]
            if self.open_rviz_check.isChecked():
                commands.append(("RViz", "ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py"))
            return commands
        return [("RViz", "ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py")]

    def _map_argument(self):
        map_file = self._navigation_map_file()
        return " map:=" + shlex.quote(str(map_file))

    def _navigation_map_file(self):
        value = self.map_input.text().strip() or "WHEELTEC.yaml"
        map_file = Path(value).expanduser()
        if map_file.is_absolute():
            return map_file
        return self._map_directory() / map_file

    def _map_directory(self):
        value = self.map_dir_input.text().strip()
        if value:
            return Path(value).expanduser()
        return self._default_map_dir()

    def _map_save_path_without_extension(self):
        map_file = self._navigation_map_file()
        return map_file.with_suffix("")

    def _sync_default_map_dir_for_workspace(self, workspace):
        current = Path(self.map_dir_input.text().strip()).expanduser()
        old_default = self._default_map_dir()
        self.workspace_root = workspace
        new_default = self._default_map_dir()
        if not self.map_dir_input.text().strip() or current == old_default:
            self.map_dir_input.setText(str(new_default))

    def _start_current_mode(self):
        if self.detached_running:
            return
        workspace = Path(self.workspace_input.text().strip()).expanduser()
        if not (workspace / "install" / "setup.bash").exists():
            QMessageBox.warning(
                self,
                "环境不存在",
                f"没有找到 {workspace / 'install' / 'setup.bash'}，请确认工作目录。",
            )
            return
        self._clear_terminal_pid_file()
        self._sync_default_map_dir_for_workspace(workspace)
        commands = self._commands_for_mode()
        for index, (title, command) in enumerate(commands):
            QTimer.singleShot(
                index * 1200,
                lambda title=title, command=command: self._open_command_terminal(
                    title, command, self.workspace_root
                ),
            )
        self.detached_running = True
        self._set_inputs_enabled(False)
        self._set_running_state(True)
        self._append_log(f"工作目录: {workspace}")
        self._append_log("已按顺序打开终端启动当前功能。")

    def _open_command_terminal(self, title, command, workspace, require_running=True):
        if require_running and not self.detached_running:
            return
        setup_file = workspace / "install" / "setup.bash"
        wheeltec_nav_prefix = workspace / "install" / "wheeltec_nav2"
        wheeltec_nav_env = ""
        if wheeltec_nav_prefix.exists():
            quoted_prefix = shlex.quote(str(wheeltec_nav_prefix))
            wheeltec_nav_env = (
                f"export AMENT_PREFIX_PATH={quoted_prefix}:$AMENT_PREFIX_PATH; "
                f"export CMAKE_PREFIX_PATH={quoted_prefix}:$CMAKE_PREFIX_PATH; "
            )
        shell_command = (
            f"echo $$ >> {shlex.quote(TERMINAL_PID_FILE)}; "
            f"export {TERMINAL_MARKER}=1; "
            f"cd {shlex.quote(str(workspace))} && "
            f"source {shlex.quote(str(setup_file))} && "
            f"{wheeltec_nav_env}"
            f"echo '启动: {title}' && "
            f"echo '终端标记: {TERMINAL_MARKER}' && "
            f"echo '终端PID: '$$ && "
            f"{command}; "
            "echo; echo '命令已退出，终端即将关闭'; sleep 1"
        )
        terminal = self._terminal_program()
        if terminal is None:
            self._append_log("未找到可用终端程序，请安装 xterm、gnome-terminal 或 x-terminal-emulator。")
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

    def _save_map(self):
        workspace = Path(self.workspace_input.text().strip()).expanduser()
        if not (workspace / "install" / "setup.bash").exists():
            QMessageBox.warning(self, "环境不存在", "请先设置正确的工作目录。")
            return
        self._sync_default_map_dir_for_workspace(workspace)
        map_dir = self._map_directory()
        map_path = self._map_save_path_without_extension()
        self._open_command_terminal(
            "保存地图",
            f"mkdir -p {shlex.quote(str(map_dir))} && "
            f"ros2 launch wheeltec_nav2 save_map.launch.py map_path:={shlex.quote(str(map_path))}",
            workspace,
            require_running=False,
        )
        self._append_log(f"地图将保存到: {map_path}.yaml / {map_path}.pgm")

    def _open_map_directory(self):
        workspace = Path(self.workspace_input.text().strip()).expanduser()
        if (workspace / "install" / "setup.bash").exists():
            self._sync_default_map_dir_for_workspace(workspace)
        map_dir = self._map_directory()
        try:
            map_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            QMessageBox.warning(self, "无法创建目录", f"{map_dir}\n{exc}")
            return
        if shutil.which("xdg-open"):
            QProcess.startDetached("xdg-open", [str(map_dir)])
            self._append_log(f"打开地图目录: {map_dir}")
            return
        QMessageBox.information(self, "地图目录", str(map_dir))

    def _publish_goal(self):
        yaw = math.radians(self.goal_yaw.value())
        self.monitor.publish_goal(self.goal_x.value(), self.goal_y.value(), yaw)
        self._append_log(
            f"发布 /goal_pose: x={self.goal_x.value():.3f}, y={self.goal_y.value():.3f}, yaw={self.goal_yaw.value():.1f}deg"
        )

    def _publish_initial_pose(self):
        yaw = math.radians(self.goal_yaw.value())
        self.monitor.publish_initial_pose(self.goal_x.value(), self.goal_y.value(), yaw)
        self._append_log(
            f"发布 /initialpose: x={self.goal_x.value():.3f}, y={self.goal_y.value():.3f}, yaw={self.goal_yaw.value():.1f}deg"
        )

    def _stop_processes(self):
        self._start_stop_burst(4)
        self.teleop_keys.clear()
        if not self.detached_running:
            return
        self._append_log("正在停止 Nav2 / SLAM / RViz 相关进程并关闭终端...")
        QProcess.startDetached("bash", ["-lc", self._stop_detached_command()])
        self._close_started_terminals()
        self._schedule_post_stop_bursts()
        self.detached_running = False
        self._set_inputs_enabled(True)
        self._set_running_state(False)

    def _stop_detached_command(self):
        return (
            f"if [ -f {shlex.quote(TERMINAL_PID_FILE)} ]; then "
            "while read -r pid; do "
            "case \"$pid\" in ''|*[!0-9]*) continue;; esac; "
            "kill -TERM $pid 2>/dev/null || true; "
            f"done < {shlex.quote(TERMINAL_PID_FILE)}; "
            "fi; "
            "pkill -TERM -f 'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py'; "
            "pkill -TERM -f 'ros2 launch wheeltec_nav2 slam_launch.py'; "
            "pkill -TERM -f 'ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py'; "
            "pkill -TERM -f 'ros2 launch wheeltec_nav2 save_map.launch.py'; "
            "pkill -TERM -f 'rviz2'; "
            f"pkill -TERM -f '[{TERMINAL_MARKER[0]}]{TERMINAL_MARKER[1:]}'; "
            "sleep 0.5; "
            f"if [ -f {shlex.quote(TERMINAL_PID_FILE)} ]; then "
            "while read -r pid; do "
            "case \"$pid\" in ''|*[!0-9]*) continue;; esac; "
            "kill -KILL $pid 2>/dev/null || true; "
            f"done < {shlex.quote(TERMINAL_PID_FILE)}; "
            f"rm -f {shlex.quote(TERMINAL_PID_FILE)}; "
            "fi; "
            "pkill -KILL -f 'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py'; "
            "pkill -KILL -f 'ros2 launch wheeltec_nav2 slam_launch.py'; "
            "pkill -KILL -f 'ros2 launch wheeltec_rviz2 wheeltec_rviz.launch.py'; "
            f"pkill -KILL -f '[{TERMINAL_MARKER[0]}]{TERMINAL_MARKER[1:]}'; "
            "true"
        )

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

    def _schedule_post_stop_bursts(self):
        for delay in (250, 700, 1300, 2100):
            QTimer.singleShot(delay, lambda: self._start_stop_burst(10))
            QTimer.singleShot(delay + 80, self._start_cli_stop_burst)

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

    def _set_inputs_enabled(self, enabled):
        self.start_button.setEnabled(enabled)
        self.stop_button.setEnabled(not enabled)
        self.workspace_input.setEnabled(enabled)
        self.map_input.setEnabled(enabled)
        self.map_dir_input.setEnabled(enabled)
        self.open_map_dir_button.setEnabled(enabled)
        self.sim_time_combo.setEnabled(enabled)
        self.open_rviz_check.setEnabled(enabled)

    def _set_running_state(self, running):
        self.state_badge.setText("运行中" if running else "未运行")
        self.state_badge.setProperty("running", running)
        self.state_badge.style().unpolish(self.state_badge)
        self.state_badge.style().polish(self.state_badge)

    def _clear_terminal_pid_file(self):
        try:
            Path(TERMINAL_PID_FILE).unlink(missing_ok=True)
        except OSError:
            pass

    def _update_ros_status(self, status):
        if status["type"] == "cmd":
            self.last_cmd_stamp = status["stamp"]
            self.cmd_value.setText(
                f"linear {status['linear']:.3f} / angular {status['angular']:.3f}"
            )
        elif status["type"] == "odom":
            self.last_odom_stamp = status["stamp"]
            self.odom_value.setText(
                f"x {status['x']:.3f}, y {status['y']:.3f}, v {status['linear']:.3f}, w {status['angular']:.3f}"
            )
        elif status["type"] == "path":
            self.last_path_stamp = status["stamp"]
            self.path_value.setText(f"{status['count']} poses")
        elif status["type"] == "nav_status":
            self.nav_status_value.setText(self._goal_status_text(status["status"]))

    def _goal_status_text(self, status):
        mapping = {
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }
        return mapping.get(status, str(status))

    def _refresh_status_age(self):
        now = time.time()
        parts = []
        if self.last_cmd_stamp:
            parts.append(f"/cmd_vel {now - self.last_cmd_stamp:.1f}s")
        if self.last_odom_stamp:
            parts.append(f"/odom {now - self.last_odom_stamp:.1f}s")
        if self.last_path_stamp:
            parts.append(f"/plan {now - self.last_path_stamp:.1f}s")
        self.topic_age_value.setText(" / ".join(parts) if parts else "等待 ROS 话题")

    def _append_log(self, text):
        timestamp = time.strftime("%H:%M:%S")
        for line in str(text).splitlines():
            self.log_view.append(f"[{timestamp}] {line}")
        self.log_view.moveCursor(self.log_view.textCursor().End)

    def _append_error(self, text):
        self._append_log("ROS 监控线程错误: " + text)

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self)
        if self.detached_running:
            self._stop_processes()
        self.monitor.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("nav2_qt_panel")
    window = Nav2QtPanel()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
