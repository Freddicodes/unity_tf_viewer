"""
viewer_window.py — Main application window.
Left panel: frame list + details. Right: 3D viewport. Bottom: status log.
"""

import math
import time
from typing import Optional

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFontDatabase
from PyQt6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from unity_tf_listener.gl_viewport import HAS_OPENGL, TFViewport
from unity_tf_listener.tcp_listener import TFListener
from unity_tf_listener.tf_tree import TFTree

DARK_BG = "#0f0f12"
PANEL_BG = "#16161c"
BORDER = "#2a2a35"
ACCENT = "#4a9eff"
TEXT_DIM = "#6a6a80"
TEXT_MAIN = "#d0d0e0"
SUCCESS = "#3ddc84"
WARNING = "#ffb74d"
DANGER = "#ff5c5c"


def _resolve_monospace_font() -> str:
    """
    Return the name of the first available monospace font from a preference
    list, falling back to Qt's built-in fixed-pitch family.
    Must be called after QApplication is constructed.
    """
    available = set(QFontDatabase.families())
    preferred = ["JetBrainsMono Nerd Font", "Menlo", "Monaco", "Courier New", "Courier"]

    for name in preferred:
        if name in available:
            return name
    # QFontDatabase.systemFont gives us the real fixed font Qt will use
    return QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont).family()


def _make_stylesheet(mono: str) -> str:
    return f"""
QMainWindow, QWidget {{
    background: {DARK_BG};
    color: {TEXT_MAIN};
    font-family: "{mono}";
    font-size: 12px;
}}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 8px;
    padding-top: 8px;
    color: {TEXT_DIM};
    font-size: 11px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    top: -1px;
    color: {ACCENT};
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1px;
    text-transform: uppercase;
}}
QListWidget {{
    background: {PANEL_BG};
    border: 1px solid {BORDER};
    border-radius: 4px;
    color: {TEXT_MAIN};
    padding: 4px;
    outline: none;
}}
QListWidget::item {{
    padding: 4px 8px;
    border-radius: 3px;
}}
QListWidget::item:selected {{
    background: {ACCENT};
    color: white;
}}
QListWidget::item:hover:!selected {{
    background: rgba(74, 158, 255, 0.1);
}}
QTextEdit {{
    background: {PANEL_BG};
    border: 1px solid {BORDER};
    border-radius: 4px;
    color: {TEXT_DIM};
    font-family: "{mono}";
    font-size: 10px;
    padding: 4px;
}}
QLabel {{
    color: {TEXT_MAIN};
}}
QSplitter::handle {{
    background: {BORDER};
    width: 1px;
    height: 1px;
}}
QPushButton {{
    background: transparent;
    border: 1px solid {BORDER};
    border-radius: 3px;
    color: {TEXT_DIM};
    font-family: "{mono}";
    font-size: 10px;
    padding: 2px 7px;
    min-width: 26px;
}}
QPushButton:hover {{
    border-color: {ACCENT};
    color: {TEXT_MAIN};
    background: rgba(74, 158, 255, 0.08);
}}
QPushButton:pressed {{
    background: rgba(74, 158, 255, 0.2);
}}
QPushButton[active="true"] {{
    border-color: {ACCENT};
    color: {ACCENT};
    background: rgba(74, 158, 255, 0.12);
}}
"""


class SignalBridge(QObject):
    """Thread-safe signal bridge for TF updates."""

    transforms_updated = pyqtSignal(object)
    status_message = pyqtSignal(str)


class ViewerWindow(QMainWindow):
    def __init__(self, listener: TFListener):
        super().__init__()
        self._listener = listener
        self._tree = TFTree()
        self._bridge = SignalBridge()
        self._bridge.transforms_updated.connect(self._on_transforms)
        self._bridge.status_message.connect(self._on_status)

        listener.on_transform_update(lambda t: self._bridge.transforms_updated.emit(t))
        listener.on_status_change(lambda s: self._bridge.status_message.emit(s))

        self._setup_ui()
        # self._setup_demo_data()

        # Stats timer
        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._update_stats)
        self._stats_timer.start(500)

    # ------------------------------------------------------------------ #
    #  UI setup                                                            #
    # ------------------------------------------------------------------ #

    def _setup_ui(self):
        self.setWindowTitle("ROS TF Viewer")
        self.resize(1280, 800)
        _resolve_monospace_font()
        self.setStyleSheet(_make_stylesheet(_resolve_monospace_font()))

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # Title bar
        title_bar = self._make_title_bar()
        root_layout.addWidget(title_bar)

        # Main splitter: left panel | viewport
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.setHandleWidth(1)
        root_layout.addWidget(main_splitter, 1)

        # Left panel
        left = self._make_left_panel()
        main_splitter.addWidget(left)

        # Right: viewport + log
        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.setHandleWidth(1)

        if HAS_OPENGL:
            self._viewport = TFViewport()
            self._viewport.view_changed.connect(self._on_view_changed)
        else:
            self._viewport = self._make_fallback_widget()
        right_splitter.addWidget(self._viewport)

        log_widget = self._make_log_panel()
        right_splitter.addWidget(log_widget)
        right_splitter.setStretchFactor(0, 4)
        right_splitter.setStretchFactor(1, 1)

        main_splitter.addWidget(right_splitter)
        main_splitter.setStretchFactor(0, 0)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([280, 1000])

    def _make_title_bar(self) -> QWidget:
        bar = QWidget()
        bar.setFixedHeight(44)
        bar.setStyleSheet(f"background: {PANEL_BG}; border-bottom: 1px solid {BORDER};")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 12, 0)
        layout.setSpacing(8)

        title = QLabel("◈  ROS TF Viewer")
        title.setStyleSheet(f"color: {ACCENT}; font-size: 13px; font-weight: bold; letter-spacing: 2px;")
        layout.addWidget(title)

        layout.addSpacing(16)

        # ---- Axis alignment buttons ----------------------------------------
        axis_label = QLabel("VIEW:")
        axis_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; letter-spacing: 1px;")
        layout.addWidget(axis_label)

        # (button_text, axis_key, tooltip)
        self._axis_buttons: dict[str, QPushButton] = {}
        axis_specs = [
            ("+X", "+X", "Look down +X axis — front  [X]"),
            ("−X", "-X", "Look down −X axis — back  [Shift+X]"),
            ("+Y", "+Y", "Look down +Y axis — left  [Y]"),
            ("−Y", "-Y", "Look down −Y axis — right  [Shift+Y]"),
            ("+Z", "+Z", "Look down +Z axis — top  [Z]"),
            ("−Z", "-Z", "Look down −Z axis — bottom  [Shift+Z]"),
        ]
        for label, key, tip in axis_specs:
            btn = QPushButton(label)
            btn.setToolTip(tip)
            btn.setProperty("active", "false")
            btn.clicked.connect(lambda checked, k=key: self._on_axis_btn(k))
            layout.addWidget(btn)
            self._axis_buttons[key] = btn

        layout.addSpacing(4)
        reset_btn = QPushButton("⟳ reset")
        reset_btn.setToolTip("Reset to perspective view  [R]")
        reset_btn.clicked.connect(self._on_reset_btn)
        layout.addWidget(reset_btn)
        self._reset_btn = reset_btn

        layout.addStretch()

        # ---- Status --------------------------------------------------------
        self._status_dot = QLabel("●")
        self._status_dot.setStyleSheet(f"color: {WARNING}; font-size: 16px;")
        layout.addWidget(self._status_dot)

        self._status_label = QLabel("Waiting for connection…")
        self._status_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")
        layout.addWidget(self._status_label)

        layout.addSpacing(16)

        hint = QLabel("Left-drag: orbit  ·  Scroll: zoom  ·  X/Y/Z: align axis")
        hint.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        layout.addWidget(hint)

        return bar

    def _on_axis_btn(self, key: str):
        if HAS_OPENGL:
            self._viewport.align_to_axis(key)
        self._highlight_axis_btn(key)

    def _on_reset_btn(self):
        if HAS_OPENGL:
            self._viewport.reset_view()
        self._highlight_axis_btn(None)

    def _on_view_changed(self, label: str):
        """Called by the viewport when the active view changes."""
        # Figure out which key matches this label (if any)
        from unity_tf_listener.gl_viewport import AXIS_VIEWS

        matched_key = None
        for key, (_, _, lbl) in AXIS_VIEWS.items():
            if lbl == label:
                matched_key = key
                break
        self._highlight_axis_btn(matched_key)

    def _highlight_axis_btn(self, active_key: Optional[str]):
        """Set the 'active' property on buttons so QSS can style them."""
        for key, btn in self._axis_buttons.items():
            is_active = key == active_key
            btn.setProperty("active", "true" if is_active else "false")
            # Force QSS repaint
            style = btn.style()
            if style is None:
                return
            style.unpolish(btn)
            style.polish(btn)

    def _make_left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(280)
        panel.setStyleSheet(f"background: {PANEL_BG}; border-right: 1px solid {BORDER};")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Frame list
        frame_group = QGroupBox("TF FRAMES")
        fg_layout = QVBoxLayout(frame_group)
        self._frame_list = QListWidget()
        self._frame_list.currentItemChanged.connect(self._on_frame_selected)
        fg_layout.addWidget(self._frame_list)
        layout.addWidget(frame_group, 2)

        # Frame details
        detail_group = QGroupBox("TRANSFORM DETAILS")
        dg_layout = QGridLayout(detail_group)
        dg_layout.setSpacing(4)

        def make_val_label(text="—"):
            lbl = QLabel(text)
            lbl.setStyleSheet(f"color: {ACCENT}; font-size: 11px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            return lbl

        def row(label, r):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            dg_layout.addWidget(lbl, r, 0)

        row("Parent frame", 0)
        self._lbl_parent = make_val_label()
        dg_layout.addWidget(self._lbl_parent, 0, 1)

        row("X (forward)", 1)
        self._lbl_x = make_val_label()
        dg_layout.addWidget(self._lbl_x, 1, 1)

        row("Y (left)", 2)
        self._lbl_y = make_val_label()
        dg_layout.addWidget(self._lbl_y, 2, 1)

        row("Z (up)", 3)
        self._lbl_z = make_val_label()
        dg_layout.addWidget(self._lbl_z, 3, 1)

        row("Roll", 4)
        self._lbl_roll = make_val_label()
        dg_layout.addWidget(self._lbl_roll, 4, 1)

        row("Pitch", 5)
        self._lbl_pitch = make_val_label()
        dg_layout.addWidget(self._lbl_pitch, 5, 1)

        row("Yaw", 6)
        self._lbl_yaw = make_val_label()
        dg_layout.addWidget(self._lbl_yaw, 6, 1)

        row("Quat X", 7)
        self._lbl_qx = make_val_label()
        dg_layout.addWidget(self._lbl_qx, 7, 1)

        row("Quat Y", 8)
        self._lbl_qy = make_val_label()
        dg_layout.addWidget(self._lbl_qy, 8, 1)

        row("Quat Z", 9)
        self._lbl_qz = make_val_label()
        dg_layout.addWidget(self._lbl_qz, 9, 1)

        row("Quat W", 10)
        self._lbl_qw = make_val_label()
        dg_layout.addWidget(self._lbl_qw, 10, 1)

        layout.addWidget(detail_group, 1)

        # Stats
        stats_group = QGroupBox("SESSION STATS")
        sg_layout = QGridLayout(stats_group)
        sg_layout.setSpacing(4)

        def stat_row(label, r):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
            sg_layout.addWidget(lbl, r, 0)
            val = QLabel("0")
            val.setStyleSheet(f"color: {SUCCESS}; font-size: 11px;")
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            sg_layout.addWidget(val, r, 1)
            return val

        self._stat_frames = stat_row("Known frames", 0)
        self._stat_edges = stat_row("Edges (TF links)", 1)
        self._stat_port = QLabel(f"TCP port: {self._listener.port}")
        self._stat_port.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")
        sg_layout.addWidget(self._stat_port, 2, 0, 1, 2)

        layout.addWidget(stats_group)
        return panel

    def _make_log_panel(self) -> QWidget:
        group = QGroupBox("CONNECTION LOG")
        layout = QVBoxLayout(group)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(120)
        layout.addWidget(self._log)
        return group

    @staticmethod
    def _make_fallback_widget() -> QLabel:
        lbl = QLabel(
            "⚠  PyOpenGL not installed.\n\n"
            "Install with:\n  pip install PyOpenGL PyOpenGL_accelerate\n\n"
            "TF data is still being received and shown in the frame list."
        )
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"color: {WARNING}; font-size: 13px; background: {DARK_BG};")
        return lbl

    # ------------------------------------------------------------------ #
    #  Slots                                                               #
    # ------------------------------------------------------------------ #

    def _on_transforms(self, transforms):
        self._tree.update(transforms)
        self._refresh_frame_list()
        if HAS_OPENGL:
            self._viewport.set_tree(self._tree)

        # Update dot
        self._status_dot.setStyleSheet(f"color: {SUCCESS}; font-size: 16px;")
        self._status_label.setText("Receiving TF data")

    def _on_status(self, msg: str):
        self._log.append(
            f"<span style='color:{TEXT_DIM}'>[{time.strftime('%H:%M:%S')}]</span> "
            f"<span style='color:{TEXT_MAIN}'>{msg}</span>"
        )
        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())
        if "🟢" in msg:
            self._status_dot.setStyleSheet(f"color: {WARNING}; font-size: 16px;")
            self._status_label.setText(f"Listening on port {self._listener.port}")
        elif "🔗" in msg:
            self._status_dot.setStyleSheet(f"color: {SUCCESS}; font-size: 16px;")
            self._status_label.setText("Client connected")
        elif "🔴" in msg:
            self._status_dot.setStyleSheet(f"color: {WARNING}; font-size: 16px;")
            self._status_label.setText("Client disconnected")

    def _on_frame_selected(self, current, _previous):
        if current is None:
            return
        frame = current.text().strip("★ ")
        if HAS_OPENGL:
            self._viewport.select_frame(frame)
        self._show_frame_details(frame)

    def _refresh_frame_list(self):
        current_text = ""
        if self._frame_list.currentItem():
            current_text = self._frame_list.currentItem().text()

        frames = self._tree.frames()
        roots = set(self._tree.roots())

        self._frame_list.clear()
        for f in frames:
            label = f"★ {f}" if f in roots else f"  {f}"
            item = QListWidgetItem(label)
            if f in roots:
                item.setForeground(QColor(ACCENT))
            self._frame_list.addItem(item)
            if label == current_text:
                self._frame_list.setCurrentItem(item)

    def _show_frame_details(self, frame: str):
        parent = self._tree.parent_of(frame)
        self._lbl_parent.setText(parent or "— (root)")

        local_tf = self._tree.edge_transform(frame)
        if local_tf is None:
            for lbl in [
                self._lbl_x,
                self._lbl_y,
                self._lbl_z,
                self._lbl_roll,
                self._lbl_pitch,
                self._lbl_yaw,
                self._lbl_qx,
                self._lbl_qy,
                self._lbl_qz,
                self._lbl_qw,
            ]:
                lbl.setText("—")
            return

        t = local_tf.translation
        q = local_tf.rotation

        self._lbl_x.setText(f"{t.x:.4f} m")
        self._lbl_y.setText(f"{t.y:.4f} m")
        self._lbl_z.setText(f"{t.z:.4f} m")

        self._lbl_qx.setText(f"{q.x:.4f}")
        self._lbl_qy.setText(f"{q.y:.4f}")
        self._lbl_qz.setText(f"{q.z:.4f}")
        self._lbl_qw.setText(f"{q.w:.4f}")

        # Euler RPY from quaternion
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self._lbl_roll.setText(f"{math.degrees(roll):.2f}°")
        self._lbl_pitch.setText(f"{math.degrees(pitch):.2f}°")
        self._lbl_yaw.setText(f"{math.degrees(yaw):.2f}°")

    def _update_stats(self):
        frames = self._tree.frames()
        self._stat_frames.setText(str(len(frames)))
        edges = sum(1 for f in frames if self._tree.parent_of(f) is not None)
        self._stat_edges.setText(str(edges))

    @staticmethod
    def _quat_to_rpy(x, y, z, w):
        """Convert quaternion to roll-pitch-yaw (ZYX intrinsic)."""
        sinr = 2 * (w * x + y * z)
        cosr = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)

        sinp = 2 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    # ------------------------------------------------------------------ #
    #  Demo data (shown before any real connection)                        #
    # ------------------------------------------------------------------ #

    def _setup_demo_data(self):
        """Populate a sample TF tree so the viewport isn't empty on launch."""
        import math

        from ros_message import Quaternion, Transform, TransformStamped, Vector3

        def make_ts(parent, child, tx, ty, tz, yaw=0.0):
            ts = TransformStamped()
            ts.frame_id = parent
            ts.child_frame_id = child
            ts.transform = Transform(
                translation=Vector3(tx, ty, tz), rotation=Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
            )
            return ts

        demo = {
            "world": {
                "base_link": make_ts("world", "base_link", 0, 0, 0.5),
            },
            "base_link": {
                "camera_link": make_ts("base_link", "camera_link", 0.3, 0, 0.2, 0),
                "lidar_link": make_ts("base_link", "lidar_link", 0.0, 0, 0.4, 0),
                "left_wheel": make_ts("base_link", "left_wheel", -0.1, 0.2, -0.1, 0),
                "right_wheel": make_ts("base_link", "right_wheel", -0.1, -0.2, -0.1, 0),
            },
            "camera_link": {
                "camera_optical": make_ts("camera_link", "camera_optical", 0, 0, 0.05, 0),
            },
        }
        self._tree.update(demo)
        self._refresh_frame_list()
        if HAS_OPENGL:
            self._viewport.set_tree(self._tree)
        self._on_status("ℹ️  Demo TF tree loaded. Connect ros_tcp_connector to replace.")
