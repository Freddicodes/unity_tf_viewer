"""
gl_viewport.py — PyQt6 + PyOpenGL 3D viewport that renders TF frames
in ROS2 convention: X=forward (red), Y=left (green), Z=up (blue).

Axis-aligned views (animated):
  X  — look down +X axis (from front)
  Y  — look down +Y axis (from left)
  Z  — look down +Z axis (from top)
  Shift+X/Y/Z — look from the opposite direction
  R  — reset to default perspective
"""

import math
from typing import Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QMouseEvent, QWheelEvent
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

try:
    from OpenGL.GL import *
    from OpenGL.GLU import gluPerspective
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False

from unity_tf_listener.tf_tree import TFTree
from unity_tf_listener.ros_message import Quaternion


AXIS_LENGTH = 0.3
WORLD_GRID  = 5.0
GRID_STEP   = 0.5

# Axis-aligned camera presets: (yaw_deg, pitch_deg, label)
# Camera sits at (pan_x, pan_y, -dist) in view space, rotated by pitch then yaw.
# With Z-up orbit: yaw rotates around Z, pitch tilts toward/away from Z.
#   +X view  : looking from +X toward origin → yaw=180, pitch=0
#   -X view  : looking from -X toward origin → yaw=0,   pitch=0
#   +Y view  : looking from +Y toward origin → yaw=270, pitch=0  (right-to-left)
#   -Y view  : looking from -Y toward origin → yaw=90,  pitch=0
#   +Z view  : looking down from +Z          → yaw=0,   pitch=90 (top-down)
#   -Z view  : looking up from -Z            → yaw=0,   pitch=-90
AXIS_VIEWS = {
    "+X": (180.0,   0.0, "+X  (front)"),
    "-X": (  0.0,   0.0, "−X  (back)"),
    "+Y": (270.0,   0.0, "+Y  (left)"),
    "-Y": ( 90.0,   0.0, "−Y  (right)"),
    "+Z": (  0.0,  90.0, "+Z  (top)"),
    "-Z": (  0.0, -90.0, "−Z  (bottom)"),
}

DEFAULT_YAW   = 45.0
DEFAULT_PITCH = 30.0
DEFAULT_DIST  = 5.0

ANIM_STEPS    = 20    # frames for snap animation
ANIM_INTERVAL = 16    # ms (~60 fps)


def quat_to_axis_angle(q: Quaternion):
    """Returns (ax, ay, az, deg) for glRotatef."""
    angle = 2.0 * math.acos(max(-1.0, min(1.0, q.w)))
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-8:
        return 0.0, 0.0, 1.0, 0.0
    return q.x / s, q.y / s, q.z / s, math.degrees(angle)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _ease(t: float) -> float:
    """Smooth-step easing (cubic)."""
    return t * t * (3.0 - 2.0 * t)


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference between two angles (degrees)."""
    d = (b - a) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


class TFViewport(QOpenGLWidget):
    """3D OpenGL widget showing TF frames."""

    frame_hovered = pyqtSignal(str)
    view_changed  = pyqtSignal(str)   # emits label like "+Z  (top)" or "Perspective"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tree = TFTree()
        self._selected_frame: Optional[str] = None

        # Current camera state
        self._yaw   = DEFAULT_YAW
        self._pitch = DEFAULT_PITCH
        self._dist  = DEFAULT_DIST
        self._pan_x = 0.0
        self._pan_y = 0.0

        # Animation state
        self._anim_from_yaw   = self._yaw
        self._anim_from_pitch = self._pitch
        self._anim_to_yaw     = self._yaw
        self._anim_to_pitch   = self._pitch
        self._anim_step       = 0
        self._anim_steps      = 0

        # Mouse
        self._last_mouse  = None
        self._mouse_button = None
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Render timer
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._tick)
        self._render_timer.start(ANIM_INTERVAL)

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def set_tree(self, tree: TFTree):
        self._tree = tree
        self.update()

    def select_frame(self, frame: Optional[str]):
        self._selected_frame = frame
        self.update()

    def align_to_axis(self, axis_key: str):
        """
        Smoothly animate camera to a named axis-aligned view.
        axis_key must be one of: '+X', '-X', '+Y', '-Y', '+Z', '-Z'
        """
        if axis_key not in AXIS_VIEWS:
            return
        target_yaw, target_pitch, label = AXIS_VIEWS[axis_key]

        self._anim_from_yaw   = self._yaw
        self._anim_from_pitch = self._pitch
        # Travel via shortest angular path
        self._anim_to_yaw   = self._yaw   + _angle_diff(self._yaw,   target_yaw)
        self._anim_to_pitch = self._pitch + _angle_diff(self._pitch, target_pitch)
        self._anim_step  = 0
        self._anim_steps = ANIM_STEPS

        self.view_changed.emit(label)

    def reset_view(self):
        self._anim_from_yaw   = self._yaw
        self._anim_from_pitch = self._pitch
        self._anim_to_yaw     = self._yaw   + _angle_diff(self._yaw,   DEFAULT_YAW)
        self._anim_to_pitch   = self._pitch + _angle_diff(self._pitch, DEFAULT_PITCH)
        self._anim_step  = 0
        self._anim_steps = ANIM_STEPS
        self._pan_x = self._pan_y = 0.0
        self.view_changed.emit("Perspective")

    # ------------------------------------------------------------------ #
    #  Animation tick                                                      #
    # ------------------------------------------------------------------ #

    def _tick(self):
        if self._anim_steps > 0 and self._anim_step < self._anim_steps:
            self._anim_step += 1
            t = _ease(self._anim_step / self._anim_steps)
            self._yaw   = _lerp(self._anim_from_yaw,   self._anim_to_yaw,   t)
            self._pitch = _lerp(self._anim_from_pitch, self._anim_to_pitch, t)
        self.update()

    # ------------------------------------------------------------------ #
    #  OpenGL                                                              #
    # ------------------------------------------------------------------ #

    def initializeGL(self):
        if not HAS_OPENGL:
            return
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glClearColor(0.08, 0.08, 0.10, 1.0)
        glLineWidth(1.5)

    def resizeGL(self, w, h):
        if not HAS_OPENGL:
            return
        glViewport(0, 0, w, max(h, 1))
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, w / max(h, 1), 0.01, 500.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        if not HAS_OPENGL:
            return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        glTranslatef(self._pan_x, self._pan_y, -self._dist)
        glRotatef(-self._pitch, 1, 0, 0)
        glRotatef(-self._yaw,   0, 0, 1)   # Z-up orbit

        self._draw_grid()
        self._draw_world_axes()
        self._draw_frames()
        self._draw_axis_indicator()

    # ------------------------------------------------------------------ #
    #  Drawing                                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _draw_grid():
        glLineWidth(0.5)
        glBegin(GL_LINES)
        n = int(WORLD_GRID / GRID_STEP)
        for i in range(-n, n + 1):
            x = i * GRID_STEP
            alpha = 0.15 if i != 0 else 0.35
            glColor4f(0.5, 0.5, 0.5, alpha)
            glVertex3f(x, -WORLD_GRID, 0)
            glVertex3f(x,  WORLD_GRID, 0)
            glVertex3f(-WORLD_GRID, x, 0)
            glVertex3f( WORLD_GRID, x, 0)
        glEnd()
        glLineWidth(1.5)

    @staticmethod
    def _draw_world_axes():
        l = 0.5
        glLineWidth(2.5)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.2, 0.2); glVertex3f(0,0,0); glVertex3f(l,0,0)   # X red
        glColor3f(0.2, 1.0, 0.2); glVertex3f(0,0,0); glVertex3f(0,l,0)   # Y green
        glColor3f(0.2, 0.4, 1.0); glVertex3f(0,0,0); glVertex3f(0,0,l)   # Z blue
        glEnd()
        glLineWidth(1.5)

    def _draw_frames(self):
        frames = self._tree.frames()
        if not frames:
            return

        drawn: set = set()

        for frame in frames:
            world_tf = self._tree.world_transform(frame)
            t = world_tf.translation
            q = world_tf.rotation

            glPushMatrix()
            glTranslatef(t.x, t.y, t.z)
            ax, ay, az, deg = quat_to_axis_angle(q)
            if abs(deg) > 1e-6:
                glRotatef(deg, ax, ay, az)

            is_selected = (frame == self._selected_frame)
            scale = 1.5 if is_selected else 1.0
            l = AXIS_LENGTH * scale

            glLineWidth(3.0 if is_selected else 2.0)
            glBegin(GL_LINES)
            glColor3f(1.0, 0.25, 0.25); glVertex3f(0,0,0); glVertex3f(l,0,0)
            glColor3f(0.25, 1.0, 0.25); glVertex3f(0,0,0); glVertex3f(0,l,0)
            glColor3f(0.25, 0.5,  1.0); glVertex3f(0,0,0); glVertex3f(0,0,l)
            glEnd()
            glLineWidth(1.5)

            if is_selected:
                glColor4f(1.0, 1.0, 0.3, 0.9)
            else:
                glColor4f(0.8, 0.8, 0.9, 0.7)
            self._draw_sphere(0.012 * scale)
            glPopMatrix()

            parent = self._tree.parent_of(frame)
            if parent and (parent, frame) not in drawn:
                drawn.add((parent, frame))
                pt = self._tree.world_transform(parent).translation
                glBegin(GL_LINES)
                glColor4f(0.6, 0.6, 0.7, 0.5)
                glVertex3f(pt.x, pt.y, pt.z)
                glVertex3f(t.x,  t.y,  t.z)
                glEnd()

    def _draw_axis_indicator(self):
        """
        Small orientation cube in the bottom-left corner showing which
        way each world axis currently points in screen space.
        Uses an inset viewport so it doesn't interfere with the main scene.
        """
        if not HAS_OPENGL:
            return

        w, h = self.width(), self.height()
        size = 80   # pixels

        # Save full viewport and projection
        glViewport(8, 8, size, size)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluPerspective(45.0, 1.0, 0.01, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glTranslatef(0, 0, -2.5)
        glRotatef(-self._pitch, 1, 0, 0)
        glRotatef(-self._yaw,   0, 0, 1)

        glClear(GL_DEPTH_BUFFER_BIT)   # don't wipe color — inset overlay
        glLineWidth(2.5)

        L = 0.8
        glBegin(GL_LINES)
        glColor3f(1.0, 0.3, 0.3); glVertex3f(0,0,0); glVertex3f(L,0,0)
        glColor3f(0.3, 1.0, 0.3); glVertex3f(0,0,0); glVertex3f(0,L,0)
        glColor3f(0.3, 0.5, 1.0); glVertex3f(0,0,0); glVertex3f(0,0,L)
        # negative halves, dimmer
        glColor3f(0.4, 0.1, 0.1); glVertex3f(0,0,0); glVertex3f(-L*0.4,0,0)
        glColor3f(0.1, 0.4, 0.1); glVertex3f(0,0,0); glVertex3f(0,-L*0.4,0)
        glColor3f(0.1, 0.2, 0.4); glVertex3f(0,0,0); glVertex3f(0,0,-L*0.4)
        glEnd()

        glLineWidth(1.5)
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

        # Restore full viewport
        glViewport(0, 0, w, h)

    @staticmethod
    def _draw_sphere(r: float, slices: int = 8):
        glBegin(GL_LINES)
        for i in range(slices):
            lat0 = math.pi * (-0.5 + i / slices)
            lat1 = math.pi * (-0.5 + (i + 1) / slices)
            for j in range(slices):
                lon0 = 2 * math.pi * j / slices
                x0 = r * math.cos(lat0) * math.cos(lon0)
                y0 = r * math.cos(lat0) * math.sin(lon0)
                z0 = r * math.sin(lat0)
                x1 = r * math.cos(lat1) * math.cos(lon0)
                y1 = r * math.cos(lat1) * math.sin(lon0)
                z1 = r * math.sin(lat1)
                glVertex3f(x0, y0, z0)
                glVertex3f(x1, y1, z1)
        glEnd()

    # ------------------------------------------------------------------ #
    #  Mouse / Keyboard                                                    #
    # ------------------------------------------------------------------ #

    def mousePressEvent(self, e: QMouseEvent):
        self._last_mouse   = e.position()
        self._mouse_button = e.button()
        # Stop any in-progress animation when user grabs the view
        self._anim_steps = 0

    def mouseReleaseEvent(self, e: QMouseEvent):
        self._last_mouse   = None
        self._mouse_button = None

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._last_mouse is None:
            return
        dx = e.position().x() - self._last_mouse.x()
        dy = e.position().y() - self._last_mouse.y()
        self._last_mouse = e.position()

        if self._mouse_button == Qt.MouseButton.LeftButton:
            self._yaw   += dx * 0.5
            self._pitch = max(-89, min(89, self._pitch + dy * 0.5))
            self.view_changed.emit("Perspective")
        elif self._mouse_button == Qt.MouseButton.MiddleButton:
            self._pan_x += dx * 0.005 * self._dist
            self._pan_y -= dy * 0.005 * self._dist

    def wheelEvent(self, e: QWheelEvent):
        # Stop animation so Zoom doesn't fight the tween
        self._anim_steps = 0
        self._dist *= 0.9 if e.angleDelta().y() > 0 else 1.1
        self._dist = max(0.1, min(100.0, self._dist))

    def keyPressEvent(self, e):
        # noinspection PyTypeChecker
        shift = bool(e.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        key   = e.key()

        if key == Qt.Key.Key_R:
            self.reset_view()
        elif key == Qt.Key.Key_X:
            self.align_to_axis("-X" if shift else "+X")
        elif key == Qt.Key.Key_Y:
            self.align_to_axis("-Y" if shift else "+Y")
        elif key == Qt.Key.Key_Z:
            self.align_to_axis("-Z" if shift else "+Z")
