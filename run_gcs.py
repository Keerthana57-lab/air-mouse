#!/usr/bin/env python3
"""
NIDAR AirMouse - Ground Control Station (GCS) dashboard
=======================================================
Install:  pip install PyQt5            (optional: pip install opencv-python for a real camera)

Run:      python gcs_dashboard.py                  -> follows DEMO_MODE below
          python gcs_dashboard.py --no-demo        -> force demo OFF  (waits for your drone software)
          python gcs_dashboard.py --demo           -> force demo ON
          python gcs_dashboard.py --camera 0       -> webcam index, or "rtsp://..." / "http://..." URL

Screen priority:  LIVE 2D MAP > LIVE CAMERA > SURVIVORS > MISSION STATUS > TELEMETRY

------------------------------------------------------------------------------
HOW TO CONNECT YOUR REAL DRONE SOFTWARE
Put your code inside  connect_real_drone(win)  (search for it below). Everything the
dashboard shows is driven by these MainWindow methods - call them from a QTimer, or
from another thread through Qt signals:
------------------------------------------------------------------------------
  win.set_connected(True/False)
  win.set_mission_state("DISARMED"|"ARMED"|"SEARCHING"|"RETURNING"|"COMPLETED"|"ABORTED")
  win.set_action("Exploring -> next target (3.5, 2.0)")      # what is the drone doing now
  win.set_timer(elapsed_s, limit_s)                          # mission clock
  win.set_telemetry({"battery":78, "signal":92, "altitude":1.5, "speed":0.8,
                     "heading":135, "gps":"NO FIX", "x":1.2, "y":3.4})

  # --- the live map (all coordinates in METRES, relative to the start point) ---
  win.map_add_cells([(i, j, 0|1), ...])   # occupancy grid updates: 0 = free/explored, 1 = wall
                                          # cell (i, j) is centred at (i*cell_size, j*cell_size)
  win.map_set_pose(x, y, yaw_deg)         # drone position; yaw 0 = +X axis, counter-clockwise
  win.map_set_target((x, y))              # the CURRENT autonomous exploration target (None to clear)
  win.set_survivor(1, x, y)               # tags survivor S1 on the map + survivor panel
  win.set_coverage(percent)               # optional "area explored" %
  win.set_camera_frame(QImage)            # optional - push frames yourself instead of --camera
  win.log("any text")                     # short event line in the mission panel

  The operator buttons emit:  win.command.connect(your_handler)
  your_handler receives one of "ARM", "DISARM", "START", "RTL", "ABORT".
"""
import sys
import math
import time
import random
import argparse
from collections import deque

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QObject, pyqtSignal
from PyQt5.QtGui import (QColor, QPainter, QPen, QBrush, QFont, QPolygonF,
                         QImage, QPixmap)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QLabel,
                             QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QSplitter, QProgressBar, QScrollArea, QMessageBox,
                             QSizePolicy)

try:
    import cv2  # optional, only needed for --camera
except ImportError:
    cv2 = None

# ============================================================================
#  SETTINGS  -  change these first
# ============================================================================
DEMO_MODE = True      # <-- True  = built-in fake drone + maze (for testing the screen)
                      #     False = demo OFF, dashboard waits for your real drone software
MISSION_LIMIT_MIN = 30
NUM_SURVIVORS = 3

# ----------------------------------------------------------------------------
# Look & feel
# ----------------------------------------------------------------------------
GREEN, BLUE, YELLOW, RED = "#3fb950", "#58a6ff", "#e3b341", "#f85149"
DIM, TEXT, BORDER = "#8b949e", "#e6edf3", "#30363d"

QSS = """
QMainWindow, QWidget#root { background:#0d1117; }
QWidget { color:#e6edf3; font-family:'Segoe UI','Inter','Arial'; font-size:13px; }
QLabel { background:transparent; }
QFrame#panel { background:#161b22; border:1px solid #30363d; border-radius:8px; }
QFrame#topbar { background:#161b22; border:1px solid #30363d; border-radius:8px; }
QFrame#tile { background:#0f141b; border:1px solid #30363d; border-radius:6px; }
QLabel#panelTitle { color:#58a6ff; font-weight:700; font-size:12px; }
QLabel#appTitle { color:#58a6ff; font-weight:700; font-size:19px; }
QLabel#cap { color:#8b949e; font-size:11px; }
QPushButton { background:#21262d; border:1px solid #30363d; border-radius:6px;
              padding:9px 16px; font-weight:700; }
QPushButton:hover { background:#30363d; }
QPushButton:disabled { color:#5b636d; background:#14181e; border-color:#21262d; }
QPushButton#start { background:#238636; border-color:#2ea043; color:white; }
QPushButton#start:hover { background:#2ea043; }
QPushButton#start:disabled { background:#14181e; border-color:#21262d; color:#5b636d; }
QPushButton#rtl { background:#9e6a03; border-color:#bb8009; color:white; }
QPushButton#rtl:hover { background:#bb8009; }
QPushButton#rtl:disabled { background:#14181e; border-color:#21262d; color:#5b636d; }
QPushButton#abort { background:#da3633; border-color:#f85149; color:white; }
QPushButton#abort:hover { background:#f85149; }
QPushButton#abort:disabled { background:#14181e; border-color:#21262d; color:#5b636d; }
QProgressBar { background:#21262d; border:none; border-radius:3px; max-height:6px; min-height:6px; }
QProgressBar::chunk { background:#58a6ff; border-radius:3px; }
QScrollArea { border:none; background:transparent; }
QSplitter::handle { background:#0d1117; }
QScrollBar:vertical { background:#0d1117; width:8px; }
QScrollBar::handle:vertical { background:#30363d; border-radius:4px; min-height:24px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
"""

STATE_COLORS = {            # state: (text colour, background)
    "DISCONNECTED": (RED, "#3d1418"),
    "DISARMED":     (DIM, "#21262d"),
    "ARMED":        (YELLOW, "#3a2e0c"),
    "SEARCHING":    (BLUE, "#0d2a4a"),
    "RETURNING":    ("#d2a8ff", "#2d1f45"),
    "COMPLETED":    (GREEN, "#0f2d1a"),
    "ABORTED":      (RED, "#3d1418"),
}
BUTTON_RULES = {            # state: (arm, start, rtl, abort) enabled?
    "DISCONNECTED": (0, 0, 0, 0),
    "DISARMED":     (1, 1, 0, 0),
    "ARMED":        (1, 1, 0, 1),
    "SEARCHING":    (0, 0, 1, 1),
    "RETURNING":    (0, 0, 0, 1),
    "COMPLETED":    (0, 1, 0, 0),
    "ABORTED":      (0, 1, 0, 0),
}


def fmt_time(sec):
    sec = max(0, int(sec))
    return f"{sec // 60:02d}:{sec % 60:02d}"


def grid_ref(x, y):
    """1 m grid reference relative to the start point, e.g. G(3,-2)."""
    return f"G({math.floor(x)},{math.floor(y)})"


def make_panel(title, right_widget=None):
    frame = QFrame()
    frame.setObjectName("panel")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(12, 10, 12, 12)
    lay.setSpacing(8)
    hdr = QHBoxLayout()
    t = QLabel(title)
    t.setObjectName("panelTitle")
    hdr.addWidget(t)
    hdr.addStretch()
    if right_widget is not None:
        hdr.addWidget(right_widget)
    lay.addLayout(hdr)
    return frame, lay


# ----------------------------------------------------------------------------
# LIVE 2D MAP
# ----------------------------------------------------------------------------
class MapView(QWidget):
    """Custom-painted live map. Auto-fits as the explored area grows."""

    LEGEND = [
        ("dot",  GREEN,     "Start point"),
        ("dot",  BLUE,      "Drone (current position)"),
        ("line", BLUE,      "Travelled path"),
        ("dot",  YELLOW,    "Next exploration target"),
        ("dot",  RED,       "Detected survivor"),
        ("sq",   "#e6edf3", "Mapped wall / obstacle"),
        ("pat",  "#4a5666", "Unexplored area"),
    ]

    def __init__(self, cell_size=0.25, parent=None):
        super().__init__(parent)
        self.cs = cell_size
        self._pulse = 0.0
        self.setMinimumSize(480, 340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.reset()
        t = QTimer(self)
        t.timeout.connect(self._tick)
        t.start(50)

    # ---- data API -----------------------------------------------------------
    def reset(self):
        self.cells = {}              # (i, j) -> 0 free / 1 wall
        self.path = []               # [(x, y), ...]
        self.start = (0.0, 0.0)
        self.drone = None            # (x, y, yaw_deg)
        self.target = None           # (x, y)
        self.survivors = {}          # id -> (x, y)
        self.coverage = None
        self.bounds = [-4.0, -3.0, 4.0, 3.0]   # only ever grows -> stable view
        self.update()

    def _grow(self, x, y, m=1.5):
        b = self.bounds
        if x < b[0]: b[0] = x - m
        if x > b[2]: b[2] = x + m
        if y < b[1]: b[1] = y - m
        if y > b[3]: b[3] = y + m

    def add_cells(self, updates):
        for i, j, v in updates:
            self.cells[(i, j)] = 1 if v else 0
            self._grow(i * self.cs, j * self.cs)
        self.update()

    def set_pose(self, x, y, yaw_deg=0.0):
        self.drone = (x, y, yaw_deg)
        if not self.path or math.hypot(x - self.path[-1][0], y - self.path[-1][1]) > self.cs * 0.5:
            self.path.append((x, y))
            if len(self.path) > 30000:
                self.path = self.path[-30000:]
        self._grow(x, y)
        self.update()

    def set_target(self, xy):
        self.target = xy
        if xy:
            self._grow(*xy)
        self.update()

    def set_start(self, x, y):
        self.start = (x, y)
        self._grow(x, y)
        self.update()

    def set_survivor(self, sid, x, y):
        self.survivors[sid] = (x, y)
        self._grow(x, y)
        self.update()

    def _tick(self):
        self._pulse = (self._pulse + 0.05) % 1.0
        if self.target or self.survivors:
            self.update()

    # ---- painting -----------------------------------------------------------
    @staticmethod
    def _text(p, text, x, y, color, size=9, bold=True):
        f = QFont("Segoe UI", size)
        f.setBold(bold)
        p.setFont(f)
        p.setPen(QColor(color))
        p.drawText(QPointF(x, y), text)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#0b0f14"))

        b = self.bounds
        vw, vh = b[2] - b[0], b[3] - b[1]
        pad = 26
        s = max(1.0, min((W - 2 * pad) / vw, (H - 2 * pad) / vh))   # px per metre
        ox = (W - vw * s) / 2 - b[0] * s
        oy = (H - vh * s) / 2 + b[3] * s
        X = lambda x: ox + x * s
        Y = lambda y: oy - y * s

        # unexplored area (hatched) --------------------------------------------
        world = QRectF(X(b[0]), Y(b[3]), vw * s, vh * s)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#10161d"))
        p.drawRect(world)
        p.setBrush(QBrush(QColor("#2b3542"), Qt.Dense5Pattern))
        p.drawRect(world)

        # explored free space + walls --------------------------------------------
        free_c, wall_c = QColor("#1b3147"), QColor("#e6edf3")
        cw = self.cs * s + 0.6
        for (i, j), v in self.cells.items():
            p.setBrush(wall_c if v else free_c)
            p.drawRect(QRectF(X((i - 0.5) * self.cs), Y((j + 0.5) * self.cs), cw, cw))

        # metre grid + axis labels ---------------------------------------------
        step = 1
        while step * s < 40:
            step *= 2
        p.setPen(QPen(QColor(255, 255, 255, 20), 1))
        x = math.ceil(b[0] / step) * step
        while x <= b[2]:
            p.drawLine(QPointF(X(x), world.top()), QPointF(X(x), world.bottom()))
            x += step
        y = math.ceil(b[1] / step) * step
        while y <= b[3]:
            p.drawLine(QPointF(world.left(), Y(y)), QPointF(world.right(), Y(y)))
            y += step
        x = math.ceil(b[0] / step) * step
        while x <= b[2]:
            self._text(p, f"{x:g}", X(x) + 2, world.bottom() - 3, "#6e7681", 7, False)
            x += step
        y = math.ceil(b[1] / step) * step
        while y <= b[3]:
            self._text(p, f"{y:g}", world.left() + 3, Y(y) - 2, "#6e7681", 7, False)
            y += step
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(world)

        # travelled path ---------------------------------------------------------
        if len(self.path) > 1:
            pen = QPen(QColor(BLUE), 2.2)
            pen.setJoinStyle(Qt.RoundJoin)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(QPolygonF([QPointF(X(a), Y(c)) for a, c in self.path]))

        # next exploration target --------------------------------------------------
        if self.target:
            tx, ty = X(self.target[0]), Y(self.target[1])
            if self.drone:
                pen = QPen(QColor(227, 179, 65, 170), 1.6, Qt.DashLine)
                p.setPen(pen)
                p.drawLine(QPointF(X(self.drone[0]), Y(self.drone[1])), QPointF(tx, ty))
            r = 9 + 7 * self._pulse
            p.setPen(QPen(QColor(227, 179, 65, int(200 * (1 - self._pulse))), 2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(tx, ty), r, r)
            p.setPen(QPen(QColor("#0b0f14"), 2))
            p.setBrush(QColor(YELLOW))
            p.drawEllipse(QPointF(tx, ty), 7, 7)
            self._text(p, "NEXT TARGET", tx + 12, ty - 8, YELLOW, 8)

        # start point --------------------------------------------------------------
        sx, sy = X(self.start[0]), Y(self.start[1])
        p.setPen(QPen(QColor("white"), 2))
        p.setBrush(QColor(GREEN))
        p.drawEllipse(QPointF(sx, sy), 7, 7)
        self._text(p, "START", sx + 11, sy + 4, GREEN, 8)

        # survivors ----------------------------------------------------------------
        for sid, (x, y) in self.survivors.items():
            cx, cy = X(x), Y(y)
            r = 10 + 8 * self._pulse
            p.setPen(QPen(QColor(248, 81, 73, int(180 * (1 - self._pulse))), 2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r, r)
            p.setPen(QPen(QColor("white"), 2))
            p.setBrush(QColor(RED))
            p.drawEllipse(QPointF(cx, cy), 8, 8)
            self._text(p, f"S{sid}", cx + 12, cy + 4, "#ffb3ae", 10)

        # drone --------------------------------------------------------------------
        if self.drone:
            dx, dy, yaw = self.drone
            cx, cy = X(dx), Y(dy)
            fx, fy = math.cos(math.radians(yaw)), -math.sin(math.radians(yaw))
            nx, ny = -fy, fx
            p.setPen(QPen(QColor("white"), 2))
            p.setBrush(QColor(BLUE))
            p.drawEllipse(QPointF(cx, cy), 9, 9)
            tri = QPolygonF([QPointF(cx + fx * 17, cy + fy * 17),
                             QPointF(cx + nx * 6 - fx * 4, cy + ny * 6 - fy * 4),
                             QPointF(cx - nx * 6 - fx * 4, cy - ny * 6 - fy * 4)])
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("white"))
            p.drawPolygon(tri)

        # HUD (top-left) -------------------------------------------------------------
        hud = [f"Mapped cells: {len(self.cells)}"]
        if self.coverage is not None:
            hud.append(f"Explored: {self.coverage:.0f} %")
        hud.append(f"Grid: {step} m")
        for k, line in enumerate(hud):
            self._text(p, line, 14, 22 + 16 * k, DIM, 9, False)

        # legend (top-right) ---------------------------------------------------------
        lw, rh = 172, 19
        lh = len(self.LEGEND) * rh + 12
        x0, y0 = W - lw - 10, 10
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(QColor(13, 17, 23, 215))
        p.drawRoundedRect(QRectF(x0, y0, lw, lh), 6, 6)
        for k, (kind, col, label) in enumerate(self.LEGEND):
            cy = y0 + 6 + rh * k + rh / 2
            cx = x0 + 16
            if kind == "dot":
                p.setPen(QPen(QColor("white"), 1))
                p.setBrush(QColor(col))
                p.drawEllipse(QPointF(cx, cy), 5, 5)
            elif kind == "line":
                p.setPen(QPen(QColor(col), 2.4))
                p.drawLine(QPointF(cx - 8, cy), QPointF(cx + 8, cy))
            elif kind == "sq":
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(col))
                p.drawRect(QRectF(cx - 5, cy - 5, 10, 10))
            else:
                p.setPen(QPen(QColor(BORDER), 1))
                p.setBrush(QBrush(QColor(col), Qt.Dense5Pattern))
                p.drawRect(QRectF(cx - 5, cy - 5, 10, 10))
            self._text(p, label, cx + 16, cy + 4, TEXT, 8, False)

        if not self.cells and not self.drone:
            self._text(p, "Waiting for map data ...", W / 2 - 80, H / 2, DIM, 11, False)


# ----------------------------------------------------------------------------
# LIVE CAMERA
# ----------------------------------------------------------------------------
class CameraWidget(QLabel):
    """Shows frames from OpenCV (--camera), from set_frame(), or a demo pattern."""

    def __init__(self, source=None, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumHeight(210)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setStyleSheet("background:#05080c; border:1px solid #1f6f3a; border-radius:6px;")
        self.external = None
        self.cap = None
        self.demo = True
        self.banner, self.banner_until = "", 0.0
        self._t = 0
        if source is not None and cv2 is not None:
            self.cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
            self.demo = False
        elif source is not None:
            print("[camera] opencv-python not installed -> showing demo feed")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._next)
        self.timer.start(40)

    def set_frame(self, img):
        self.external = img
        self.demo = False

    def flash(self, text, seconds=4.0):
        self.banner, self.banner_until = text, time.time() + seconds

    def _blank(self, text, color="#8b949e"):
        img = QImage(640, 360, QImage.Format_RGB32)
        img.fill(QColor("#05080c"))
        p = QPainter(img)
        p.setPen(QColor(color))
        p.setFont(QFont("Segoe UI", 20, QFont.Bold))
        p.drawText(img.rect(), Qt.AlignCenter, text)
        p.end()
        return img

    def _demo_frame(self):
        img = QImage(640, 360, QImage.Format_RGB32)
        p = QPainter(img)
        p.fillRect(img.rect(), QColor("#0c1218"))
        p.setPen(QPen(QColor(35, 60, 80), 1))
        for k in range(0, 360, 24):
            yy = (k + self._t * 2) % 360
            p.drawLine(0, yy, 640, yy)
        p.setPen(QPen(QColor(63, 185, 80, 120), 1))
        p.drawLine(320, 150, 320, 210)
        p.drawLine(290, 180, 350, 180)
        p.setPen(QColor("#6e7681"))
        p.setFont(QFont("Segoe UI", 12))
        p.drawText(img.rect().adjusted(0, 0, 0, -20), Qt.AlignHCenter | Qt.AlignVCenter,
                   "NO CAMERA CONNECTED\n(use --camera <index|url> or win.set_camera_frame())")
        if time.time() < self.banner_until:
            p.setPen(QPen(QColor(RED), 3))
            p.setBrush(Qt.NoBrush)
            p.drawRect(230, 90, 180, 180)
        p.end()
        return img

    def _next(self):
        self._t += 1
        if self.external is not None:
            img = self.external.convertToFormat(QImage.Format_RGB32)
        elif self.cap is not None:
            ok, fr = self.cap.read() if self.cap.isOpened() else (False, None)
            if ok:
                fr = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
                h, w, _ = fr.shape
                img = QImage(fr.data, w, h, 3 * w, QImage.Format_RGB888).convertToFormat(QImage.Format_RGB32)
            else:
                img = self._blank("NO SIGNAL", RED)
        else:
            img = self._demo_frame()

        # overlays: REC dot, clock, detection banner
        w, h = img.width(), img.height()
        fs = max(10, h // 30)
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing)
        p.setFont(QFont("Segoe UI", fs, QFont.Bold))
        if int(self._t / 12) % 2 == 0:
            p.setBrush(QColor(RED)); p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(h * 0.04 + 6, h * 0.05 + 6), fs * 0.45, fs * 0.45)
        p.setPen(QColor("white"))
        p.drawText(int(h * 0.04 + fs * 1.4), int(h * 0.05 + fs), "REC")
        p.drawText(QRectF(0, h * 0.03, w - 12, fs * 2), Qt.AlignRight, time.strftime("%H:%M:%S"))
        if time.time() < self.banner_until:
            p.fillRect(QRectF(0, h - fs * 2.6, w, fs * 2.6), QColor(218, 54, 51, 220))
            p.setPen(QColor("white"))
            p.drawText(QRectF(0, h - fs * 2.6, w, fs * 2.6), Qt.AlignCenter, self.banner)
        p.end()

        self.setPixmap(QPixmap.fromImage(img).scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


# ----------------------------------------------------------------------------
# Survivor card
# ----------------------------------------------------------------------------
class SurvivorCard(QFrame):
    def __init__(self, sid):
        super().__init__()
        self.setObjectName("tile")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        self.id_lbl = QLabel(f"S{sid}")
        self.id_lbl.setStyleSheet("font-size:18px; font-weight:700;")
        self.id_lbl.setFixedWidth(40)
        col = QVBoxLayout(); col.setSpacing(0)
        self.status = QLabel()
        self.loc = QLabel()
        self.loc.setObjectName("cap")
        col.addWidget(self.status); col.addWidget(self.loc)
        lay.addWidget(self.id_lbl); lay.addLayout(col, 1)
        self.set_searching()

    def set_searching(self):
        self.setStyleSheet("QFrame#tile{background:#0f141b;border:1px solid #30363d;border-radius:6px;}")
        self.id_lbl.setStyleSheet("font-size:18px;font-weight:700;color:#8b949e;")
        self.status.setText("SEARCHING ...")
        self.status.setStyleSheet("color:#8b949e;font-weight:600;")
        self.loc.setText("location unknown")

    def set_found(self, x, y, t):
        self.setStyleSheet("QFrame#tile{background:#2a1214;border:1px solid #f85149;border-radius:6px;}")
        self.id_lbl.setStyleSheet("font-size:18px;font-weight:700;color:#f85149;")
        self.status.setText(f"FOUND  (at {fmt_time(t)})")
        self.status.setStyleSheet("color:#f85149;font-weight:700;")
        self.loc.setText(f"Grid {grid_ref(x, y)}   X {x:.1f} m   Y {y:.1f} m")


# ----------------------------------------------------------------------------
# Main window
# ----------------------------------------------------------------------------
class MainWindow(QMainWindow):
    command = pyqtSignal(str)   # "ARM", "DISARM", "START", "RTL", "ABORT"

    def __init__(self, limit_s=1800, n_survivors=3, camera_src=None, cell_size=0.25):
        super().__init__()
        self.setWindowTitle("NIDAR AirMouse - GCS")
        self.resize(1600, 920)
        self.limit_s = limit_s
        self.n_survivors = n_survivors
        self.state = "DISCONNECTED"
        self.elapsed = 0.0
        self.found = {}
        self.events = deque(maxlen=3)
        self._time_level = -1

        root = QWidget(); root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(12, 10, 12, 12)
        outer.setSpacing(10)
        outer.addWidget(self._build_topbar())

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)

        # left: THE MAP
        self.map = MapView(cell_size)
        map_panel, ml = make_panel("LIVE 2D MAP")
        ml.addWidget(self.map, 1)
        split.addWidget(map_panel)

        # right: camera, survivors, mission, telemetry
        col = QWidget()
        cl = QVBoxLayout(col)
        cl.setContentsMargins(0, 0, 6, 0)
        cl.setSpacing(10)
        cl.addWidget(self._build_camera(camera_src))
        cl.addWidget(self._build_survivors())
        cl.addWidget(self._build_mission())
        cl.addWidget(self._build_telemetry())
        cl.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(col)
        scroll.setMinimumWidth(410)
        split.addWidget(scroll)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 2)
        split.setSizes([1100, 460])
        outer.addWidget(split, 1)

        self.set_mission_state("DISCONNECTED")
        self.set_timer(0, limit_s)
        self.set_telemetry({})

    # ---- UI builders ------------------------------------------------------------
    def _build_topbar(self):
        bar = QFrame(); bar.setObjectName("topbar")
        h = QHBoxLayout(bar)
        h.setContentsMargins(14, 8, 14, 8)
        h.setSpacing(12)
        title = QLabel("NIDAR AirMouse - GCS"); title.setObjectName("appTitle")
        self.badge = QLabel(); self.link = QLabel()
        h.addWidget(title); h.addSpacing(8)
        h.addWidget(self.badge); h.addWidget(self.link)
        h.addStretch(1)

        tbox = QVBoxLayout(); tbox.setSpacing(3)
        self.time_lbl = QLabel(); self.time_lbl.setAlignment(Qt.AlignRight)
        self.remain_lbl = QLabel(); self.remain_lbl.setObjectName("cap"); self.remain_lbl.setAlignment(Qt.AlignRight)
        self.time_bar = QProgressBar(); self.time_bar.setRange(0, 1000); self.time_bar.setTextVisible(False)
        self.time_bar.setFixedWidth(210)
        tbox.addWidget(self.time_lbl); tbox.addWidget(self.time_bar); tbox.addWidget(self.remain_lbl)
        h.addLayout(tbox)
        h.addSpacing(8)

        self.btn_arm = QPushButton("ARM")
        self.btn_start = QPushButton("START MISSION"); self.btn_start.setObjectName("start")
        self.btn_rtl = QPushButton("RTL"); self.btn_rtl.setObjectName("rtl")
        self.btn_abort = QPushButton("ABORT MISSION"); self.btn_abort.setObjectName("abort")
        for b in (self.btn_arm, self.btn_start, self.btn_rtl, self.btn_abort):
            b.setCursor(Qt.PointingHandCursor); h.addWidget(b)
        self.btn_arm.clicked.connect(lambda: self.command.emit("DISARM" if self.state == "ARMED" else "ARM"))
        self.btn_start.clicked.connect(lambda: self.command.emit("START"))
        self.btn_rtl.clicked.connect(lambda: self.command.emit("RTL"))
        self.btn_abort.clicked.connect(self._confirm_abort)
        return bar

    def _build_camera(self, src):
        self.camera = CameraWidget(src)
        f, l = make_panel("LIVE CAMERA STREAM")
        l.addWidget(self.camera)
        f.setMinimumHeight(290)
        return f

    def _build_survivors(self):
        self.surv_count = QLabel(f"0 / {self.n_survivors} FOUND")
        self.surv_count.setStyleSheet("color:#8b949e;font-weight:700;")
        f, l = make_panel("SURVIVORS TRACKING", self.surv_count)
        self.cards = {}
        for sid in range(1, self.n_survivors + 1):
            self.cards[sid] = SurvivorCard(sid)
            l.addWidget(self.cards[sid])
        return f

    def _build_mission(self):
        f, l = make_panel("MISSION STATUS")
        g = QGridLayout(); g.setHorizontalSpacing(12); g.setVerticalSpacing(6)
        self.m_state = QLabel("-"); self.m_state.setStyleSheet("font-size:16px;font-weight:700;")
        self.m_action = QLabel("-"); self.m_action.setWordWrap(True)
        self.m_armed = QLabel("-")
        self.cov_lbl = QLabel("0 %")
        self.cov_bar = QProgressBar(); self.cov_bar.setRange(0, 100); self.cov_bar.setTextVisible(False)
        self.m_events = QLabel("-"); self.m_events.setObjectName("cap"); self.m_events.setWordWrap(True)
        rows = [("Status", self.m_state), ("Action", self.m_action), ("Armed", self.m_armed),
                ("Area explored", self.cov_lbl)]
        for r, (k, w) in enumerate(rows):
            cap = QLabel(k); cap.setObjectName("cap"); cap.setAlignment(Qt.AlignTop)
            g.addWidget(cap, r, 0); g.addWidget(w, r, 1)
        g.addWidget(self.cov_bar, len(rows), 1)
        cap = QLabel("Events"); cap.setObjectName("cap"); cap.setAlignment(Qt.AlignTop)
        g.addWidget(cap, len(rows) + 1, 0); g.addWidget(self.m_events, len(rows) + 1, 1)
        g.setColumnStretch(1, 1)
        g.setColumnMinimumWidth(0, 88)
        l.addLayout(g)
        return f

    def _build_telemetry(self):
        f, l = make_panel("DRONE TELEMETRY")
        g = QGridLayout(); g.setSpacing(8)
        self.tel = {}
        spec = [("battery", "Battery"), ("signal", "Signal"), ("altitude", "Altitude"),
                ("speed", "Speed"), ("heading", "Heading"), ("gps", "GPS"), ("pos", "Position (X, Y)")]
        for k, (key, name) in enumerate(spec):
            tile = QFrame(); tile.setObjectName("tile")
            tl = QVBoxLayout(tile); tl.setContentsMargins(10, 6, 10, 6); tl.setSpacing(0)
            cap = QLabel(name); cap.setObjectName("cap")
            val = QLabel("--"); val.setStyleSheet("font-size:16px;font-weight:700;")
            tl.addWidget(cap); tl.addWidget(val)
            self.tel[key] = val
            if key == "pos":
                g.addWidget(tile, k // 2, 0, 1, 2)
            else:
                g.addWidget(tile, k // 2, k % 2)
        l.addLayout(g)
        return f

    # ---- operator actions ---------------------------------------------------------
    def _confirm_abort(self):
        r = QMessageBox.warning(self, "Abort mission",
                                "Abort the mission now?\nThe drone will stop searching and land.",
                                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r == QMessageBox.Yes:
            self.command.emit("ABORT")

    # ---- public API (drive the dashboard from your drone software) -------------------
    def set_connected(self, ok):
        if ok:
            self.link.setText("LINK OK")
            self.link.setStyleSheet(f"color:{GREEN};font-weight:700;")
            if self.state == "DISCONNECTED":
                self.set_mission_state("DISARMED")
        else:
            self.link.setText("LINK LOST")
            self.link.setStyleSheet(f"color:{RED};font-weight:700;")
            self.set_mission_state("DISCONNECTED")

    def set_mission_state(self, state):
        self.state = state
        fg, bg = STATE_COLORS.get(state, (DIM, "#21262d"))
        self.badge.setText(state)
        self.badge.setStyleSheet(f"color:{fg};background:{bg};border:1px solid {fg};"
                                 "border-radius:13px;padding:4px 16px;font-weight:700;")
        self.m_state.setText(state)
        self.m_state.setStyleSheet(f"color:{fg};font-size:16px;font-weight:700;")
        self.m_armed.setText("YES" if state in ("ARMED", "SEARCHING", "RETURNING") else "NO")
        if state == "DISCONNECTED":
            self.link.setText("LINK OFFLINE")
            self.link.setStyleSheet(f"color:{RED};font-weight:700;")
        arm, start, rtl, abort = BUTTON_RULES.get(state, (0, 0, 0, 0))
        self.btn_arm.setText("DISARM" if state == "ARMED" else "ARM")
        self.btn_arm.setEnabled(bool(arm)); self.btn_start.setEnabled(bool(start))
        self.btn_rtl.setEnabled(bool(rtl)); self.btn_abort.setEnabled(bool(abort))

    def set_action(self, text):
        self.m_action.setText(text)

    def set_timer(self, elapsed_s, limit_s):
        self.elapsed, self.limit_s = elapsed_s, limit_s
        left = max(0, limit_s - elapsed_s)
        self.time_lbl.setText(f"Time: {fmt_time(elapsed_s)} / {fmt_time(limit_s)}")
        self.remain_lbl.setText(f"{fmt_time(left)} remaining")
        self.time_bar.setValue(int(1000 * min(1.0, elapsed_s / max(1, limit_s))))
        level = 2 if left < 120 else 1 if left < 300 else 0
        if level != self._time_level:
            self._time_level = level
            col = (TEXT, YELLOW, RED)[level]
            chunk = (BLUE, YELLOW, RED)[level]
            self.time_lbl.setStyleSheet(f"color:{col};font-size:16px;font-weight:700;")
            self.time_bar.setStyleSheet(f"QProgressBar::chunk{{background:{chunk};border-radius:3px;}}")

    def set_telemetry(self, d):
        def put(key, text, color=TEXT):
            self.tel[key].setText(text)
            self.tel[key].setStyleSheet(f"color:{color};font-size:16px;font-weight:700;")
        b, s = d.get("battery"), d.get("signal")
        put("battery", "-- %" if b is None else f"{b:.0f} %", DIM if b is None else GREEN if b > 40 else YELLOW if b > 20 else RED)
        put("signal", "--" if s is None else f"{s:.0f} %", DIM if s is None else GREEN if s > 60 else YELLOW if s > 30 else RED)
        a, sp, hd = d.get("altitude"), d.get("speed"), d.get("heading")
        put("altitude", "-- m" if a is None else f"{a:.1f} m")
        put("speed", "-- m/s" if sp is None else f"{sp:.1f} m/s")
        put("heading", "-- deg" if hd is None else f"{hd:.0f} deg")
        gps = d.get("gps", "--")
        put("gps", gps, GREEN if gps.startswith("FIX") else RED if "NO" in gps else DIM)
        x, y = d.get("x"), d.get("y")
        put("pos", "--" if x is None else f"{x:.1f} m,  {y:.1f} m")

    def map_add_cells(self, updates):  self.map.add_cells(updates)
    def map_set_pose(self, x, y, yaw_deg=0.0):  self.map.set_pose(x, y, yaw_deg)
    def map_set_target(self, xy):  self.map.set_target(xy)
    def map_set_start(self, x, y):  self.map.set_start(x, y)
    def set_camera_frame(self, qimage):  self.camera.set_frame(qimage)

    def set_coverage(self, pct):
        self.map.coverage = pct
        self.cov_lbl.setText(f"{pct:.0f} %")
        self.cov_bar.setValue(int(pct))

    def set_survivor(self, sid, x, y):
        new = sid not in self.found
        self.found[sid] = (x, y)
        self.map.set_survivor(sid, x, y)
        if sid in self.cards:
            self.cards[sid].set_found(x, y, self.elapsed)
        self.surv_count.setText(f"{len(self.found)} / {self.n_survivors} FOUND")
        self.surv_count.setStyleSheet(f"color:{RED if self.found else DIM};font-weight:700;")
        if new:
            self.log(f"Survivor S{sid} located at {grid_ref(x, y)}")
            self.camera.flash(f"SURVIVOR S{sid} DETECTED")

    def log(self, msg):
        self.events.appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")
        self.m_events.setText("\n".join(self.events))

    def reset_dashboard(self):
        self.map.reset()
        self.found.clear()
        for c in self.cards.values():
            c.set_searching()
        self.surv_count.setText(f"0 / {self.n_survivors} FOUND")
        self.surv_count.setStyleSheet(f"color:{DIM};font-weight:700;")
        self.set_coverage(0)
        self.set_timer(0, self.limit_s)
        self.events.clear(); self.m_events.setText("-")


# ----------------------------------------------------------------------------
# DEMO SIMULATOR  (stand-in for your drone software - only runs when DEMO_MODE = True)
# Builds a hidden random maze, flies a frontier-exploration algorithm in it, and feeds the
# dashboard through the exact same public API a real drone would use.
# ----------------------------------------------------------------------------
class DemoSimulator(QObject):
    DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
    ALT = 1.5
    SENSE_RANGE = 3.5      # cells

    def __init__(self, win, speed=1.5, cell_m=0.5, rooms=10):
        super().__init__()
        self.win, self.cs, self.rooms = win, cell_m, rooms
        self.v = speed / cell_m            # cells per second
        self.state = "DISCONNECTED"
        self.new_world()
        self.last = time.monotonic()
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick)
        win.command.connect(self.handle_command)

    def start(self):
        QTimer.singleShot(900, self._connect)
        self.timer.start(40)

    def _connect(self):
        self.win.set_connected(True)
        self.set_state("DISARMED", "Waiting for operator - press ARM then START MISSION")

    # ---- world ---------------------------------------------------------------------
    def new_world(self):
        n = 2 * self.rooms + 1
        g = [[1] * n for _ in range(n)]
        g[1][1] = 0
        stack = [(1, 1)]
        while stack:
            x, y = stack[-1]
            nb = [(x + dx * 2, y + dy * 2, dx, dy) for dx, dy in self.DIRS
                  if 0 < x + dx * 2 < n - 1 and 0 < y + dy * 2 < n - 1 and g[y + dy * 2][x + dx * 2]]
            if not nb:
                stack.pop(); continue
            nx, ny, dx, dy = random.choice(nb)
            g[y + dy][x + dx] = 0; g[ny][nx] = 0
            stack.append((nx, ny))
        for _ in range(self.rooms * 2):        # a few loops so it isn't a perfect maze
            x, y = random.randint(2, n - 3), random.randint(2, n - 3)
            if g[y][x] and ((not g[y][x - 1] and not g[y][x + 1]) or (not g[y - 1][x] and not g[y + 1][x])):
                g[y][x] = 0
        self.grid, self.n = g, n
        self.total_free = sum(1 for row in g for c in row if c == 0)

        dist = {(1, 1): 0}; q = deque([(1, 1)])
        while q:
            c = q.popleft()
            for dx, dy in self.DIRS:
                m = (c[0] + dx, c[1] + dy)
                if m not in dist and g[m[1]][m[0]] == 0:
                    dist[m] = dist[c] + 1; q.append(m)
        far = [c for c, d in dist.items() if d > 10]
        random.shuffle(far)
        self.truth = {}
        for c in far:
            if all(abs(c[0] - o[0]) + abs(c[1] - o[1]) > 6 for o in self.truth.values()):
                self.truth[len(self.truth) + 1] = c
            if len(self.truth) >= self.win.n_survivors:
                break

        self.known, self.found = {}, set()
        self.pos, self.yaw = [1.5, 1.5], 0.0
        self.path, self.target = [], None
        self.alt, self.alt_goal = 0.0, 0.0
        self.elapsed, self.battery, self.speed_now = 0.0, 100.0, 0.0
        self.win.reset_dashboard()
        self.win.map_set_start(0.0, 0.0)
        self.win.map_set_pose(0.0, 0.0, 0.0)

    def xy(self, gx, gy):                       # sim cell coords -> metres relative to start
        return (gx - 1.5) * self.cs, (gy - 1.5) * self.cs

    # ---- state ----------------------------------------------------------------------
    def set_state(self, state, action=""):
        self.state = state
        self.win.set_mission_state(state)
        if action:
            self.win.set_action(action)
            self.win.log(action)

    def handle_command(self, cmd):
        s = self.state
        if cmd == "ARM" and s == "DISARMED":
            self.set_state("ARMED", "Motors armed")
        elif cmd == "DISARM" and s == "ARMED":
            self.set_state("DISARMED", "Motors disarmed")
        elif cmd == "START":
            if s in ("COMPLETED", "ABORTED"):
                self.new_world(); s = "DISARMED"
            if s in ("DISARMED", "ARMED"):
                self.alt_goal = self.ALT
                self.set_state("SEARCHING", "Taking off - autonomous exploration started")
        elif cmd == "RTL" and s == "SEARCHING":
            self.begin_return("RTL commanded by operator")
        elif cmd == "ABORT" and s in ("ARMED", "SEARCHING", "RETURNING"):
            self.path, self.target, self.alt_goal = [], None, 0.0
            self.win.map_set_target(None)
            self.set_state("ABORTED", "MISSION ABORTED - landing")

    # ---- sensing / planning ------------------------------------------------------------
    def sense(self):
        px, py = self.pos
        new = []
        for k in range(120):
            a = 2 * math.pi * k / 120
            dx, dy = math.cos(a), math.sin(a)
            d = 0.0
            while d <= self.SENSE_RANGE:
                i, j = int(px + dx * d), int(py + dy * d)
                v = self.grid[j][i]
                if self.known.get((i, j)) != v:
                    self.known[(i, j)] = v
                    new.append((i - 1, j - 1, v))
                if v:
                    break
                d += 0.2
        if new:
            self.win.map_add_cells(new)
            free = sum(1 for v in self.known.values() if v == 0)
            self.win.set_coverage(100.0 * free / self.total_free)
        for sid, (i, j) in self.truth.items():
            if sid not in self.found and (i, j) in self.known and \
               math.hypot(i + 0.5 - px, j + 0.5 - py) <= 3.0:
                self.found.add(sid)
                self.win.set_survivor(sid, *self.xy(i + 0.5, j + 0.5))

    def is_frontier(self, c):
        return any((c[0] + dx, c[1] + dy) not in self.known for dx, dy in self.DIRS)

    def bfs(self, src, goal):
        prev, q = {src: None}, deque([src])
        while q:
            c = q.popleft()
            if c != src and goal(c):
                out = []
                while c != src:
                    out.append(c); c = prev[c]
                return out[::-1]
            for dx, dy in self.DIRS:
                m = (c[0] + dx, c[1] + dy)
                if m not in prev and self.known.get(m) == 0:
                    prev[m] = c; q.append(m)
        return None

    def begin_return(self, why):
        cur = (int(self.pos[0]), int(self.pos[1]))
        self.path = self.bfs(cur, lambda c: c == (1, 1)) or []
        self.target = (1, 1)
        self.win.map_set_target(self.xy(1.5, 1.5))
        self.set_state("RETURNING", f"{why} - returning to start")

    def follow(self, dt):
        budget, moved = self.v * dt, 0.0
        while self.path and budget > 1e-6:
            tx, ty = self.path[0][0] + 0.5, self.path[0][1] + 0.5
            dx, dy = tx - self.pos[0], ty - self.pos[1]
            d = math.hypot(dx, dy)
            if d > 1e-6:
                self.yaw = math.degrees(math.atan2(dy, dx))
            if d <= budget:
                self.pos = [tx, ty]; budget -= d; moved += d; self.path.pop(0)
            else:
                self.pos[0] += dx / d * budget; self.pos[1] += dy / d * budget
                moved += budget; budget = 0
        self.speed_now = 0.8 * self.speed_now + 0.2 * (moved / dt * self.cs if dt else 0)

    # ---- main loop -----------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.2)
        self.last = now
        if self.state == "DISCONNECTED":
            return
        self.speed_now *= 0.9
        # altitude (take-off / landing)
        if abs(self.alt - self.alt_goal) > 0.01:
            self.alt += math.copysign(min(abs(self.alt_goal - self.alt), 0.8 * dt), self.alt_goal - self.alt)
        airborne = self.alt >= self.ALT - 0.1

        if self.state in ("SEARCHING", "RETURNING"):
            self.elapsed += dt
            self.battery = max(0.0, self.battery - dt * 0.03)
            self.win.set_timer(self.elapsed, self.win.limit_s)
            if airborne:
                self.sense()

        if self.state == "SEARCHING" and airborne:
            if self.elapsed >= self.win.limit_s:
                self.begin_return("Mission time limit reached")
            elif self.battery < 20:
                self.begin_return("Low battery")
            elif len(self.found) >= len(self.truth):
                self.begin_return("All survivors located")
            else:
                if self.target and not self.path:
                    self.target = None
                if self.target and not self.is_frontier(self.target):
                    self.target, self.path = None, []          # explored on the way -> re-plan
                if self.target is None:
                    cur = (int(self.pos[0]), int(self.pos[1]))
                    p = self.bfs(cur, self.is_frontier)
                    if p is None:
                        self.begin_return("Maze fully explored")
                    else:
                        self.path, self.target = p, p[-1]
                        tx, ty = self.xy(self.target[0] + 0.5, self.target[1] + 0.5)
                        self.win.map_set_target((tx, ty))
                        self.win.set_action(f"Exploring - heading to next target ({tx:.1f}, {ty:.1f}) m")
                self.follow(dt)
        elif self.state == "RETURNING" and airborne:
            if self.path:
                self.follow(dt)
            elif (int(self.pos[0]), int(self.pos[1])) == (1, 1):
                self.alt_goal = 0.0
                self.win.set_action("At start point - landing")
                if self.alt <= 0.05:
                    self.alt = 0.0
                    self.win.map_set_target(None)
                    self.set_state("COMPLETED", "Mission complete - landed safely")
            else:
                self.begin_return("Re-planning")

        # push state to the dashboard
        x, y = self.xy(*self.pos)
        self.win.map_set_pose(x, y, self.yaw)
        dist = math.hypot(x, y)
        self.win.set_telemetry({
            "battery": self.battery,
            "signal": max(35.0, 98 - dist * 2.5 + random.uniform(-2, 2)),
            "altitude": self.alt,
            "speed": self.speed_now,
            "heading": (90 - self.yaw) % 360,
            "gps": "NO FIX",           # indoor maze
            "x": x, "y": y,
        })


# ============================================================================
#  >>>>>>>>>>>>>>>>>>>>  YOUR REAL DRONE SOFTWARE GOES HERE  <<<<<<<<<<<<<<<<<<<<
#  This function runs once at start-up when DEMO_MODE = False (or --no-demo).
#  Replace the example lines with your own code (ROS, MAVLink, sockets, ...).
#  Keep it non-blocking: use a QTimer, or a thread that emits Qt signals.
# ============================================================================
def connect_real_drone(win):
    # 1) React to the operator buttons (ARM / DISARM / START / RTL / ABORT):
    def on_command(cmd):
        print("Operator command:", cmd)
        # e.g. send the command to your drone here
    win.command.connect(on_command)

    # 2) Feed the dashboard. Example (uncomment and adapt) - poll your drone 10x per second:
    #
    # def update():
    #     data = my_drone.get_state()                      # <- your code
    #     win.set_connected(data.link_ok)
    #     win.set_mission_state(data.mode)                 # "ARMED", "SEARCHING", "RETURNING", ...
    #     win.set_timer(data.elapsed_s, 30 * 60)
    #     win.set_telemetry({"battery": data.battery, "signal": data.rssi,
    #                        "altitude": data.alt, "speed": data.speed,
    #                        "heading": data.heading, "gps": "NO FIX",
    #                        "x": data.x, "y": data.y})
    #     win.map_set_pose(data.x, data.y, data.yaw_deg)
    #     win.map_set_target(data.next_target)             # (x, y) or None
    #     win.map_add_cells(data.new_map_cells)            # [(i, j, 0 or 1), ...]
    #     for sid, (sx, sy) in data.survivors.items():     # {1: (x, y), 2: (x, y)}
    #         win.set_survivor(sid, sx, sy)
    #
    # timer = QTimer(win); timer.timeout.connect(update); timer.start(100)
    # win._real_timer = timer        # keep a reference so it is not garbage-collected
    pass


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="NIDAR AirMouse GCS dashboard")
    ap.add_argument("--demo", action="store_true", help="force the built-in simulator ON")
    ap.add_argument("--no-demo", action="store_true", help="force the built-in simulator OFF")
    ap.add_argument("--camera", default=None, help="camera index (0) or stream URL (rtsp://, http://)")
    ap.add_argument("--limit-min", type=float, default=MISSION_LIMIT_MIN, help="mission time limit in minutes")
    ap.add_argument("--survivors", type=int, default=NUM_SURVIVORS, help="number of survivors to track")
    ap.add_argument("--speed", type=float, default=1.5, help="demo drone speed in m/s")
    args = ap.parse_args()

    use_demo = DEMO_MODE
    if args.demo:
        use_demo = True
    if args.no_demo:
        use_demo = False

    app = QApplication(sys.argv)
    app.setStyleSheet(QSS)
    win = MainWindow(limit_s=int(args.limit_min * 60), n_survivors=args.survivors, camera_src=args.camera)
    win.show()

    sim = None
    if use_demo:
        sim = DemoSimulator(win, speed=args.speed)
        sim.start()
    else:
        connect_real_drone(win)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()