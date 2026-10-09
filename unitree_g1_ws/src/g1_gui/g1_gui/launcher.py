#!/usr/bin/env python3
"""
G1 启动器 — 统一入口界面。
集成：进程管理 / FSM 状态切换 / Velocity 摇杆控制。
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt, QTimer, QPointF, QObject, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush


# ═══════════════════════════════════════════════════════════
#  路径配置
# ═══════════════════════════════════════════════════════════

# launcher.py 位于 unitree_g1_ws/src/g1_gui/g1_gui/，向上四级为项目根目录。
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))

CTRL_PATH = os.path.join(PROJECT_ROOT, "unitree_g1_ws2/deploy/robots/g1_23dof/build/g1_ctrl")
CTRL_CWD = os.path.join(PROJECT_ROOT, "unitree_g1_ws2/deploy/robots/g1_23dof/build")

JOYSTICK_JSON = "/dev/shm/unitree_joystick_bridge.json"

# 所有手柄按键
ALL_BUTTONS = ["a", "b", "x", "y", "lb", "rb", "back", "start",
               "ls", "rs", "up", "down", "left", "right"]


# ═══════════════════════════════════════════════════════════
#  手柄 JSON 写入器
# ═══════════════════════════════════════════════════════════

class JoystickWriter:
    """管理 /dev/shm/unitree_joystick_bridge.json 的读写（线程安全）"""

    def __init__(self):
        # 程序一启动，就立刻生成一个全零的手柄 JSON
        self._lock = threading.Lock()
        self.pulse_active = False  # True 时摇杆定时器暂停写轴
        self.state = {
            "axes": {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0},
            "buttons": {b: 0 for b in ALL_BUTTONS},
            "seq": 0,
        }
        os.makedirs(os.path.dirname(JOYSTICK_JSON), exist_ok=True)
        self._write()

    def _write(self):
        self.state["seq"] += 1
        # 原子写入：先写临时文件再 rename，防止 g1_ctrl 读到半截 JSON
        tmp = JOYSTICK_JSON + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f)
        os.rename(tmp, JOYSTICK_JSON)

    def set_axis(self, name, value):
        with self._lock:
            self.state["axes"][name] = max(-1.0, min(1.0, value))
            self._write()

    def combo(self, buttons, axes=None, duration=0.3):
        """
        发送一个按键组合（edge trigger：先松→再按→保持→松开）。
        buttons: 要按下的按键列表
        axes:    要同时设置的轴 dict，如 {"lt": 1.0}

        注意：LT/RT 在 g1_ctrl 中是 Axis 类型 (EMA smooth=0.03)，
        需要提前 ~250ms 写入才能让 pressed 达到阈值。
        因此 trigger 轴会分两阶段：先预热 → 再配合按键边沿触发。
        """
        trigger_axes = {}   # LT/RT 需要预热的轴
        normal_axes = {}    # 不需要预热的轴
        if axes:
            for k, v in axes.items():
                if k in ("lt", "rt"):
                    trigger_axes[k] = v
                else:
                    normal_axes[k] = v

        # 阶段 0: 预热 trigger 轴，让 g1_ctrl 的 EMA 滤波达到阈值
        if trigger_axes:
            with self._lock:
                for k, v in trigger_axes.items():
                    self.state["axes"][k] = v
                self._write()
            time.sleep(0.25)  # 等待 250ms → Axis 数据 > 0.5 阈值

        # 阶段 1: 先释放按键（制造上升沿），等待 50ms
        with self._lock:
            for b in buttons:
                self.state["buttons"][b] = 0
            self._write()
        time.sleep(0.05)

        # 阶段 2: 按下按键 + 保持轴值
        with self._lock:
            for b in buttons:
                self.state["buttons"][b] = 1
            # 重新写所有轴（trigger 轴已在阶段 0 预热，继续保持）
            if axes:
                for k, v in axes.items():
                    self.state["axes"][k] = v
            self._write()
        time.sleep(duration)

        # 阶段 3: 全部释放
        with self._lock:
            for b in buttons:
                self.state["buttons"][b] = 0
            if axes:
                for k in axes.keys():
                    self.state["axes"][k] = 0.0
            self._write()

    def velocity_pulse(self, lx=0.0, ly=0.0, rx=0.0, duration=2.0):
        """
        发送持续速度指令，duration 秒后自动归零。
        期间 pulse_active=True 阻止摇杆定时器覆盖。
        lx+ = 左移, ly- = 前进, rx+ = 逆时针
        """
        self.pulse_active = True
        with self._lock:
            self.state["axes"]["lx"] = lx
            self.state["axes"]["ly"] = ly
            self.state["axes"]["rx"] = rx
            self._write()
        time.sleep(duration)
        with self._lock:
            self.state["axes"]["lx"] = 0.0
            self.state["axes"]["ly"] = 0.0
            self.state["axes"]["rx"] = 0.0
            self._write()
        self.pulse_active = False


# ═══════════════════════════════════════════════════════════
#  2 轴摇杆 Widget（从 arm_teleop 改造）
# ═══════════════════════════════════════════════════════════

class Joystick2D(QWidget):
    """虚拟 2 轴摇杆，返回 (x, y) ∈ [-1, 1]"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(180, 180)
        self.setMaximumSize(240, 240)
        self._x = 0.0
        self._y = 0.0
        self._dragging = False

    def value(self):
        return (self._x, self._y)

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 12
        knob_r = int(r * 0.28)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # 背景
        p.setBrush(QColor(30, 30, 40))
        p.setPen(Qt.NoPen)
        p.drawRect(0, 0, w, h)

        # 外圈
        p.setPen(QPen(QColor(100, 100, 130), 3))
        p.setBrush(QColor(20, 20, 30))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # 十字线
        p.setPen(QPen(QColor(60, 60, 80), 1, Qt.DashLine))
        p.drawLine(cx - r, cy, cx + r, cy)
        p.drawLine(cx, cy - r, cx, cy + r)

        # 把手
        kx = cx + int(self._x * (r - knob_r))
        ky = cy - int(self._y * (r - knob_r))
        gradient = QColor(70, 140, 255) if self._dragging else QColor(90, 100, 140)
        p.setPen(QPen(gradient.darker(130), 2))
        p.setBrush(gradient)
        p.drawEllipse(QPointF(kx, ky), knob_r, knob_r)

    def _to_norm(self, px, py):
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 12
        dx = (px - cx) / r
        dy = (cy - py) / r
        d = math.hypot(dx, dy)
        if d > 1.0:
            dx /= d
            dy /= d
        return (dx, dy)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._x, self._y = self._to_norm(e.pos().x(), e.pos().y())
            self.update()

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._x, self._y = self._to_norm(e.pos().x(), e.pos().y())
            self.update()

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self._x, self._y = 0.0, 0.0
        self.update()


# ═══════════════════════════════════════════════════════════
#  单轴滑块（旋转用）
# ═══════════════════════════════════════════════════════════

class Slider1D(QWidget):
    """横向单轴滑块（旋转用），返回 value ∈ [-1, 1]，+向右"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(180, 50)
        self.setMaximumSize(240, 70)
        self._val = 0.0
        self._dragging = False

    def value(self):
        return self._val

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        track_w = w - 40
        track_x0 = 20
        thumb_r = 14

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor(30, 30, 40))
        p.setPen(Qt.NoPen)
        p.drawRect(0, 0, w, h)

        # 轨道
        p.setPen(QPen(QColor(100, 100, 130), 3))
        p.drawLine(track_x0, cy, track_x0 + track_w, cy)

        # 零线
        p.setPen(QPen(QColor(60, 60, 80), 1, Qt.DashLine))
        p.drawLine(cx, cy - 14, cx, cy + 14)

        # 滑块
        tx = cx + int(self._val * track_w / 2)
        tx = max(track_x0 + thumb_r, min(track_x0 + track_w - thumb_r, tx))
        c = QColor(180, 120, 60) if self._dragging else QColor(90, 100, 140)
        p.setPen(QPen(c.darker(130), 2))
        p.setBrush(c)
        p.drawEllipse(QPointF(tx, cy), thumb_r, thumb_r)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._val = self._pos_to_val(e.pos().x())
            self.update()

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._val = self._pos_to_val(e.pos().x())
            self.update()

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self._val = 0.0
        self.update()

    def _pos_to_val(self, px):
        w = self.width()
        track_w = w - 40
        cx = w // 2
        v = (px - cx) / (track_w / 2)
        return max(-1.0, min(1.0, v))


# ═══════════════════════════════════════════════════════════
#  FSM 控制面板
# ═══════════════════════════════════════════════════════════

class FsmControlPanel(QFrame):
    """FSM 模式切换按钮组 + 状态显示"""

    MODES = {
        "Passive": {
            "buttons": ["b"],
            "axes": {"lt": 1.0},
            "label": "LT+B → Passive (趴下)",
        },
        "FixStand": {
            "buttons": ["up"],
            "axes": {"lt": 1.0},
            "label": "LT+Up → FixStand (站立)",
        },
        "Velocity": {
            "buttons": ["rb", "x"],
            "axes": {},
            "label": "RB+X → Velocity (行走)",
        },
        "WBC": {
            # 只能从 Velocity 进入
            "buttons": ["rb", "up"],
            "axes": {},
            "label": "RB+Up → WBC (整机控制)",
        },
        "UFO": {
            # 只能从 Velocity 进入
            "buttons": ["y"],
            "axes": {"lt": 1.0},
            "label": "LT+Y → UFO (全身策略)",
        },
        "Dance": {
            # 两种进入方式，根据当前状态自动选择
            # FixStand → Dance: RB+Y
            # Velocity → Dance: LT+down
            "from_fixstand": {"buttons": ["rb", "y"], "axes": {}},
            "from_velocity": {"buttons": ["down"], "axes": {"lt": 1.0}},
            "label": "Dance (跳舞)",
        },
    }

    def __init__(self, writer: JoystickWriter):
        super().__init__()
        self.writer = writer
        self.current_mode = "—"
        self._vx = 0.0
        self._vy = 0.0
        self._vw = 0.0

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setStyleSheet(
            "FsmControlPanel { background: #252535; border: 1px solid #3a3a50; "
            "border-radius: 6px; padding: 8px; }"
        )

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # 标题行
        header = QHBoxLayout()
        title = QLabel("<b>🎮 FSM 控制</b>")
        title.setFont(QFont("Sans", 12))
        header.addWidget(title)
        header.addStretch()

        self.mode_label = QLabel("当前状态: —")
        self.mode_label.setFont(QFont("Sans", 9))
        self.mode_label.setStyleSheet("color: #888;")
        header.addWidget(self.mode_label)
        layout.addLayout(header)

        # 模式按钮
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self.buttons = {}
        for mode_name, cfg in self.MODES.items():
            btn = QPushButton(mode_name)
            btn.setToolTip(cfg["label"])
            btn.setFixedHeight(34)
            btn.setFont(QFont("Sans", 10, QFont.Bold))
            btn.setStyleSheet(self._btn_style(mode_name, False))
            btn.clicked.connect(lambda checked, m=mode_name: self._activate(m))
            self.buttons[mode_name] = btn
            btn_row.addWidget(btn)

        layout.addLayout(btn_row)

        # 提示
        hint = QLabel("  Passive(趴下) | FixStand(站立) | Velocity(行走) | WBC(整机) | UFO(全身策略) | Dance(跳舞)")
        hint.setFont(QFont("Sans", 8))
        hint.setStyleSheet("color: #585b70;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        # ── 速度摇杆区 ──
        self.joystick_area = QWidget()
        joystick_area_layout = QVBoxLayout(self.joystick_area)
        joystick_area_layout.setContentsMargins(0, 4, 0, 0)
        joystick_area_layout.setSpacing(4)

        joy_label = QLabel("<b>🏃 速度控制 (Velocity 模式)</b>")
        joy_label.setFont(QFont("Sans", 10))
        joy_label.setStyleSheet("color: #4caf50;")
        joystick_area_layout.addWidget(joy_label)

        sticks_layout = QHBoxLayout()
        sticks_layout.setSpacing(16)

        # 左摇杆组: lx / ly
        left_group = QVBoxLayout()
        l_label = QLabel("平移  lx ⇔ / ly ⇕")
        l_label.setAlignment(Qt.AlignCenter)
        l_label.setFont(QFont("Sans", 9))
        l_label.setStyleSheet("color: #888;")
        self.joystick_2d = Joystick2D()
        self.lx_label = QLabel("lx:  0.00")
        self.ly_label = QLabel("ly:  0.00")
        for lbl in (self.lx_label, self.ly_label):
            lbl.setFont(QFont("Sans", 9))
            lbl.setStyleSheet("color: #aaa;")
        left_group.addWidget(l_label)
        left_group.addWidget(self.joystick_2d, alignment=Qt.AlignCenter)
        left_group.addWidget(self.lx_label)
        left_group.addWidget(self.ly_label)

        # 右滑块: rx 旋转（横向）
        right_group = QVBoxLayout()
        r_label = QLabel("旋转 rx  (← 逆时针 / 顺时针 →)")
        r_label.setAlignment(Qt.AlignCenter)
        r_label.setFont(QFont("Sans", 9))
        r_label.setStyleSheet("color: #888;")
        self.slider_1d = Slider1D()
        self.rx_label = QLabel("rx:  0.00")
        self.rx_label.setFont(QFont("Sans", 9))
        self.rx_label.setStyleSheet("color: #aaa;")
        right_group.addWidget(r_label)
        right_group.addWidget(self.slider_1d, alignment=Qt.AlignCenter)
        right_group.addWidget(self.rx_label)

        sticks_layout.addLayout(left_group)
        sticks_layout.addSpacing(20)
        sticks_layout.addLayout(right_group)
        sticks_layout.addStretch()
        joystick_area_layout.addLayout(sticks_layout)

        self.joystick_area.setVisible(False)  # 默认隐藏，Velocity 模式才显示
        layout.addWidget(self.joystick_area)

        # 25Hz 定时器：持续发布摇杆值
        self._joystick_timer = QTimer()
        self._joystick_timer.timeout.connect(self._tick_joystick)
        self._joystick_timer.start(40)

    def _btn_style(self, mode_name, active):
        colors = {
            "Passive":  ("#546e7a", "#607d8b"),
            "FixStand": ("#1565c0", "#1976d2"),
            "Velocity": ("#2e7d32", "#388e3c"),
            "WBC":      ("#e65100", "#ef6c00"),
            "UFO":      ("#00695c", "#00897b"),
            "Dance":    ("#6a1b9a", "#7b1fa2"),
        }
        base, hover = colors.get(mode_name, ("#444", "#555"))
        return (
            f"QPushButton {{ background: {base}; color: #e0e0e0; border: none; "
            f"border-radius: 4px; padding: 4px 8px; }}"
            f"QPushButton:hover {{ background: {hover}; }}"
        )

    def _activate(self, mode_name):
        """按钮点击：发送组合键，标记当前模式。"""
        cfg = self.MODES[mode_name]

        # Dance 按钮根据当前状态自动选择入口
        if mode_name == "Dance":
            if self.current_mode == "Velocity":
                sub = cfg["from_velocity"]
            else:
                sub = cfg["from_fixstand"]
            buttons = list(sub["buttons"])
            axes = dict(sub["axes"])
        else:
            buttons = list(cfg["buttons"])
            axes = dict(cfg["axes"])

        # 先更新显示
        self._set_display(mode_name)

        # 在后台线程发送组合键（避免阻塞 GUI）
        t = threading.Thread(
            target=lambda: self.writer.combo(buttons, axes if axes else None, 0.3),
            daemon=True,
        )
        t.start()

        # WBC / Dance 结束后自动切回 Velocity（只更新界面，不发指令。
        # C++ 代码已在动作结束时自动完成 FSM 切换，GUI 只需同步显示。）
        if mode_name == "WBC":
            WBC_DURATION = 4.0  # WBC 播放 ~3s + 切回开销
            QTimer.singleShot(
                int(WBC_DURATION * 1000),
                lambda: self._set_display("Velocity"),
            )
        elif mode_name == "UFO":
            UFO_DURATION = 10.0
            QTimer.singleShot(
                int(UFO_DURATION * 1000),
                lambda: self._set_display("Velocity"),
            )
        elif mode_name == "Dance":
            DANCE_DURATION = 28.0  # 与 config.yaml 中 time_end 一致
            QTimer.singleShot(
                int(DANCE_DURATION * 1000),
                lambda: self._set_display("Velocity"),
            )

    def _set_display(self, mode_name):
        """仅更新界面显示，不发送手柄指令（用于 C++ 已自动切换的场景）。"""
        self.current_mode = mode_name
        self.mode_label.setText(f"当前状态: {mode_name}")
        self.mode_label.setStyleSheet(
            f"color: {'#4caf50' if mode_name == 'Velocity' else '#ff9800'};"
        )
        self.joystick_area.setVisible(mode_name == "Velocity")

    def _tick_joystick(self):
        """定时读取摇杆值并写入 JSON"""
        if self.current_mode != "Velocity":
            return
        if self.writer.pulse_active:
            return  # velocity_pulse 正在发指令，不覆盖
        lx, ly = self.joystick_2d.value()
        rx = self.slider_1d.value()

        # 平滑更新
        alpha = 0.5
        self._vx = alpha * lx + (1 - alpha) * self._vx
        self._vy = alpha * ly + (1 - alpha) * self._vy
        self._vw = alpha * rx + (1 - alpha) * self._vw

        # 死区
        if abs(self._vx) < 0.03:
            self._vx = 0.0
        if abs(self._vy) < 0.03:
            self._vy = 0.0
        if abs(self._vw) < 0.03:
            self._vw = 0.0

        # 摇杆上(+y)→ly-，右(+x)→lx+；lx+=左移，ly+=前，rx+=逆时针
        self.writer.set_axis("lx", self._vx)
        self.writer.set_axis("ly", -self._vy)
        self.writer.set_axis("rx", self._vw)

        self.lx_label.setText(f"lx: {self._vx:+.2f}")
        self.ly_label.setText(f"ly: {self._vy:+.2f}")
        self.rx_label.setText(f"rx: {self._vw:+.2f}")


# ═══════════════════════════════════════════════════════════
#  动作按钮面板（右侧）
# ═══════════════════════════════════════════════════════════

class ActionPanel(QFrame):
    """八个动作按钮：前进/后退/左移/右移/左转弯/右转弯/挥手/跳舞。
    仅在 Velocity 模式下可用（除跳舞外）。
    """

    def __init__(self, writer: JoystickWriter, fsm_panel):
        super().__init__()
        self.writer = writer
        self.fsm_panel = fsm_panel  # 读取 current_mode

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setStyleSheet(
            "ActionPanel { background: #252535; border: 1px solid #3a3a50; "
            "border-radius: 6px; padding: 8px; }"
        )

        self._setup_ui()

        # 定时刷新按钮状态
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(200)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        title = QLabel("<b>🎯 动作指令</b>")
        title.setFont(QFont("Sans", 12))
        layout.addWidget(title)

        hint = QLabel("（需在 Velocity 模式下使用）")
        hint.setFont(QFont("Sans", 8))
        hint.setStyleSheet("color: #585b70;")
        layout.addWidget(hint)

        # 按钮定义: (文字, lx, ly, rx, 样式色)
        btns_def = [
            ("前 进",   0.0, -1.0,  0.0, "#2e7d32"),
            ("后 退",   0.0,  1.0,  0.0, "#2e7d32"),
            ("左 移",  -1.0,  0.0,  0.0, "#1565c0"),
            ("右 移",   1.0,  0.0,  0.0, "#1565c0"),
            ("左转弯",   0.0, -1.0, -2.0, "#e65100"),
            ("右转弯",   0.0, -1.0, 2.0, "#e65100"),
            # 挥手占位，跳舞复用 FSM
        ]

        self.vel_btns = []
        for text, lx, ly, rx, color in btns_def:
            btn = QPushButton(text)
            btn.setFixedHeight(34)
            btn.setFont(QFont("Sans", 11, QFont.Bold))
            btn.setStyleSheet(
                f"QPushButton {{ background: {color}; color: #e0e0e0; border: none; "
                f"border-radius: 4px; padding: 4px 8px; }}"
                f"QPushButton:hover {{ background: {color}dd; }}"
                f"QPushButton:disabled {{ background: #333; color: #666; }}"
            )
            btn.clicked.connect(
                lambda checked, lx=lx, ly=ly, rxx=rx: self._pulse(lx, ly, rxx)
            )
            self.vel_btns.append(btn)
            layout.addWidget(btn)

        # 分隔
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #3a3a50;")
        layout.addWidget(sep)

        # WBC（FSM 状态切换：Velocity → WBC）
        wbc_btn = QPushButton("W B C")
        wbc_btn.setFixedHeight(34)
        wbc_btn.setFont(QFont("Sans", 11, QFont.Bold))
        wbc_btn.setStyleSheet(
            "QPushButton { background: #e65100; color: #e0e0e0; border: none; "
            "border-radius: 4px; padding: 4px 8px; }"
            "QPushButton:hover { background: #ef6c00; }"
            "QPushButton:disabled { background: #333; color: #666; }"
        )
        wbc_btn.clicked.connect(self._wbc)
        self.wbc_btn = wbc_btn
        layout.addWidget(wbc_btn)

        # UFO-FB（发送 FSM 组合键）
        ufo_btn = QPushButton("U F O")
        ufo_btn.setFixedHeight(34)
        ufo_btn.setFont(QFont("Sans", 11, QFont.Bold))
        ufo_btn.setStyleSheet(
            "QPushButton { background: #00695c; color: #e0e0e0; border: none; "
            "border-radius: 4px; padding: 4px 8px; }"
            "QPushButton:hover { background: #00897b; }"
            "QPushButton:disabled { background: #333; color: #666; }"
        )
        ufo_btn.clicked.connect(self._ufo)
        self.ufo_btn = ufo_btn
        layout.addWidget(ufo_btn)

        # 跳舞（发送 FSM 组合键）
        dance_btn = QPushButton("跳 舞")
        dance_btn.setFixedHeight(34)
        dance_btn.setFont(QFont("Sans", 11, QFont.Bold))
        dance_btn.setStyleSheet(
            "QPushButton { background: #6a1b9a; color: #e0e0e0; border: none; "
            "border-radius: 4px; padding: 4px 8px; }"
            "QPushButton:hover { background: #7b1fa2; }"
            "QPushButton:disabled { background: #333; color: #666; }"
        )
        dance_btn.clicked.connect(self._dance)
        self.dance_btn = dance_btn
        layout.addWidget(dance_btn)

        layout.addStretch()

    def _refresh(self):
        """根据 FSM 状态启用/禁用按钮"""
        mode = self.fsm_panel.current_mode
        active = mode == "Velocity"
        for btn in self.vel_btns:
            btn.setEnabled(active)
        # WBC 和跳舞在 Velocity 下可用
        self.wbc_btn.setEnabled(mode == "Velocity")
        self.ufo_btn.setEnabled(mode == "Velocity")
        self.dance_btn.setEnabled(mode in ("FixStand", "Velocity"))

    def trigger(self, idx):
        """外部触发：1=前进 2=后退 3=左移 4=右移 5=左转弯 6=右转弯 7=WBC 8=跳舞 9=UFO"""
        if idx == 1:
            self._pulse(0.0, -1.0, 0.0)
        elif idx == 2:
            self._pulse(0.0, 1.0, 0.0)
        elif idx == 3:
            self._pulse(-1.0, 0.0, 0.0)
        elif idx == 4:
            self._pulse(1.0, 0.0, 0.0)
        elif idx == 5:
            self._pulse(0.0, -1.0, -2.0)
        elif idx == 6:
            self._pulse(0.0, -1.0, 2.0)
        elif idx == 7:
            self._wbc()
        elif idx == 8:
            self._dance()
        elif idx == 9:
            self._ufo()

    def _pulse(self, lx, ly, rx):
        """在后台线程发速度脉冲"""
        t = threading.Thread(
            target=lambda: self.writer.velocity_pulse(lx, ly, rx, 2.0),
            daemon=True,
        )
        t.start()

    def _wbc(self):
        """触发 WBC，结束后自动切回 Velocity"""
        self.fsm_panel._activate("WBC")

    def _dance(self):
        """触发跳舞，28s 后自动切回 Velocity"""
        self.fsm_panel._activate("Dance")

    def _ufo(self):
        """触发 UFO-FB，10s 后自动切回 Velocity"""
        self.fsm_panel._activate("UFO")


# ═══════════════════════════════════════════════════════════
#  UDP 监听器（外部触发）
# ═══════════════════════════════════════════════════════════

class UDPListener(QObject):
    """UDP 监听器，在后台线程接收触发指令，通过信号通知 GUI 主线程"""

    triggered = pyqtSignal(int)  # 1-9 对应动作按钮

    def __init__(self, port=9876):
        super().__init__()
        self.port = port
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _listen(self):
        import socket as sock_mod
        s = sock_mod.socket(sock_mod.AF_INET, sock_mod.SOCK_DGRAM)
        s.setsockopt(sock_mod.SOL_SOCKET, sock_mod.SO_REUSEADDR, 1)
        s.settimeout(0.5)
        try:
            s.bind(("0.0.0.0", self.port))
        except OSError as e:
            print(f"[UDP] 绑定端口 {self.port} 失败: {e}")
            return

        print(f"[UDP] 监听端口 {self.port} …")
        while self._running:
            try:
                data, addr = s.recvfrom(16)
                cmd = data.decode().strip()
                idx = int(cmd)
                if 1 <= idx <= 9:
                    print(f"[UDP] 收到 {addr} → 动作 {idx}")
                    self.triggered.emit(idx)
            except sock_mod.timeout:
                continue
            except (ValueError, UnicodeDecodeError):
                print(f"[UDP] 忽略无效数据: {data}")
            except OSError:
                break
        s.close()
        print("[UDP] 监听已停止")


# ═══════════════════════════════════════════════════════════
#  进程卡片
# ═══════════════════════════════════════════════════════════

class ProcessCard(QFrame):
    """单个进程卡片：名称、状态指示、启停按钮"""

    def __init__(self, name, description, cmd, cwd=None):
        super().__init__()
        self.name = name
        self.description = description
        self.cmd = cmd
        self.cwd = cwd
        self.process = None

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setStyleSheet(
            "ProcessCard { background: #252535; border: 1px solid #3a3a50; "
            "border-radius: 6px; padding: 8px; }"
        )
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._setup_ui()

        self._timer = QTimer()
        self._timer.timeout.connect(self._poll)
        self._timer.start(500)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        header = QHBoxLayout()
        title = QLabel(f"<b>{self.name}</b>")
        title.setFont(QFont("Sans", 12))
        header.addWidget(title)
        header.addStretch()

        self.status_dot = QLabel()
        self.status_dot.setFixedSize(12, 12)
        self._update_dot(False)
        header.addWidget(self.status_dot)

        self.status_label = QLabel("已停止")
        self.status_label.setFont(QFont("Sans", 9))
        self.status_label.setStyleSheet("color: #888;")
        header.addWidget(self.status_label)
        layout.addLayout(header)

        desc = QLabel(self.description)
        desc.setFont(QFont("Sans", 9))
        desc.setStyleSheet("color: #888;")
        desc.setWordWrap(True)
        layout.addWidget(desc)

        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("▶  启动")
        self.start_btn.clicked.connect(self.start)
        self.stop_btn = QPushButton("■  停止")
        self.stop_btn.clicked.connect(self.stop)
        self.stop_btn.setEnabled(False)

        for btn in (self.start_btn, self.stop_btn):
            btn.setFixedHeight(32)
            btn.setFont(QFont("Sans", 10))

        self.start_btn.setStyleSheet(
            "QPushButton { background: #2e7d32; color: #e0e0e0; border: none; "
            "border-radius: 4px; padding: 4px 16px; }"
            "QPushButton:hover { background: #388e3c; }"
            "QPushButton:disabled { background: #333; color: #666; }"
        )
        self.stop_btn.setStyleSheet(
            "QPushButton { background: #b71c1c; color: #e0e0e0; border: none; "
            "border-radius: 4px; padding: 4px 16px; }"
            "QPushButton:hover { background: #d32f2f; }"
            "QPushButton:disabled { background: #333; color: #666; }"
        )

        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def _update_dot(self, running):
        color = "#4caf50" if running else "#555"
        self.status_dot.setStyleSheet(
            f"QLabel {{ background: {color}; border-radius: 6px; }}"
        )

    def _poll(self):
        running = self.is_running
        if running:
            self._update_dot(True)
            self.status_label.setText("运行中")
            self.status_label.setStyleSheet("color: #4caf50;")
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
        else:
            self._update_dot(False)
            self.status_label.setText("已停止")
            self.status_label.setStyleSheet("color: #888;")
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            if self.process is not None:
                self.process = None

    @property
    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        if self.is_running:
            return
        try:
            self.process = subprocess.Popen(
                self.cmd,
                cwd=self.cwd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError:
            self.status_label.setText("文件未找到!")
            self.status_label.setStyleSheet("color: #f44336;")
        except Exception as e:
            self.status_label.setText(f"错误: {e}")
            self.status_label.setStyleSheet("color: #f44336;")

    def stop(self):
        if not self.is_running:
            return
        pid = self.process.pid
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        self.process = None


# ═══════════════════════════════════════════════════════════
#  主窗口
# ═══════════════════════════════════════════════════════════

class LauncherWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G1 控制台")
        self.setMinimumSize(860, 720)
        self.setStyleSheet("background-color: #1e1e2e; color: #cdd6f4;")

        # 手柄 JSON 写入器（全局单例）
        self.joystick = JoystickWriter()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        # ── 标题 ──
        title = QLabel("🦿 G1 机器人控制台")
        title.setFont(QFont("Sans", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        root.addWidget(self._sep())

        # ── 左右分栏 ──
        split = QHBoxLayout()
        split.setSpacing(12)

        # ═══ 左面板 ═══
        left = QVBoxLayout()
        left.setSpacing(6)

        # 进程管理
        proc_label = QLabel("<b>⚙️ 进程管理</b>")
        proc_label.setFont(QFont("Sans", 11))
        left.addWidget(proc_label)

        self.ctrl_card = ProcessCard(
            "G1 控制器 (g1_ctrl)",
            "23DOF FSM 策略控制器 · DDS 直连机器人",
            [CTRL_PATH],
            cwd=CTRL_CWD,
        )
        left.addWidget(self.ctrl_card)

        left.addWidget(self._sep())

        # FSM 控制
        self.fsm_panel = FsmControlPanel(self.joystick)
        left.addWidget(self.fsm_panel)

        left.addStretch()
        split.addLayout(left, stretch=3)

        # ═══ 右面板 ═══
        right = QVBoxLayout()
        right.setSpacing(6)

        self.action_panel = ActionPanel(self.joystick, self.fsm_panel)
        right.addWidget(self.action_panel)

        split.addLayout(right, stretch=1)

        root.addLayout(split)

        # ── UDP 监听器（外部触发） ──
        self.udp_listener = UDPListener(port=9876)
        self.udp_listener.triggered.connect(self.action_panel.trigger)
        self.udp_listener.start()

        # 底部
        footer = QLabel(
            "启动 g1_ctrl → FixStand → Velocity → WBC/Dance → 自动切回 Velocity | "
            "急停: Passive 按钮\n"
            "UDP 触发端口: 9876 | 1-6=速度 7=WBC 8=Dance | 远程: python3 udp_send.py <本机IP>"
        )
        footer.setFont(QFont("Sans", 8))
        footer.setStyleSheet("color: #585b70;")
        footer.setAlignment(Qt.AlignCenter)
        root.addWidget(footer)

    def _sep(self):
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #3a3a50;")
        return sep

    def _start_all(self):
        if not self.ctrl_card.is_running:
            self.ctrl_card.start()

    def _stop_all(self):
        self.ctrl_card.stop()

    def closeEvent(self, event):
        """关闭窗口：停止所有定时器、子进程，确保终端恢复正常"""
        self.udp_listener.stop()
        self.fsm_panel._joystick_timer.stop()
        self.action_panel._timer.stop()
        self.ctrl_card._timer.stop()
        self._stop_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = LauncherWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
