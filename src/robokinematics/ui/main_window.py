"""Main window — RoboKinematics Studio.

Layout:
  ┌──────────────────────────────────────────────────────────────────────┐
  │ Toolbar: Open | Solve IK | Camera Presets | Inspector toggle | Status │
  ├─────────────────────┬────────────────────────────────┬───────────────┤
  │ Left: FK / IK Tabs  │  Centre: OpenGL 3-D Viewport   │ Right: DH /   │
  │  • Joint sliders    │   • Floating telemetry HUD     │  Analysis /   │
  │  • Live deg / rad   │   • Ghost-robot overlays       │  Calculations │
  │  • IK target entry  │   • Target reticle             │               │
  │  • Solution browser ├────────────────────────────────┤               │
  │                     │  Playback Timeline Controls    │               │
  └─────────────────────┴────────────────────────────────┴───────────────┘
"""

from __future__ import annotations

import math
import uuid
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QStatusBar,
    QStyledItemDelegate,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolBar,
    QVBoxLayout,
    QWidget,
)
from scipy.spatial.transform import Rotation

from robokinematics.app.application_controller import ApplicationController
from robokinematics.app.state import AppStatus
from robokinematics.domain.models import (
    DHParameters,
    JointType,
    Pose,
    RobotDefinition,
    RobotState,
    Target,
)
from robokinematics.persistence.serializers import load_model
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.visualization.scene import RoboKinematicsScene

# ── Palette ───────────────────────────────────────────────────────────────────
BG      = "#0a0e13"
PANEL   = "#111827"
PANEL2  = "#1a2332"
ACCENT  = "#2563eb"
ACCENT2 = "#3b82f6"
BORDER  = "#1e3a5f"
TEXT    = "#e2e8f0"
DIM     = "#7a9bbc"
OK      = "#22c55e"
WARN    = "#f59e0b"
ERR     = "#ef4444"
AMBER   = "#fbbf24"
CYAN    = "#22d3ee"

STYLE = f"""
QMainWindow, QWidget {{
    background-color: {BG};
    color: {TEXT};
    font-family: 'Segoe UI', 'Inter', -apple-system, sans-serif;
    font-size: 12px;
}}
QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 14px;
    padding: 10px 8px 8px 8px;
    font-weight: 600;
    background: {PANEL};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    color: {ACCENT2};
}}
QPushButton {{
    background: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 5px 12px;
    color: {TEXT};
    font-weight: 500;
}}
QPushButton:hover  {{ background: {PANEL2}; border-color: {ACCENT2}; }}
QPushButton:pressed {{ background: {ACCENT}; }}
QPushButton#accent {{
    background: {ACCENT}; border-color: {ACCENT2};
    font-weight: 700; color: white;
}}
QPushButton#accent:hover {{ background: {ACCENT2}; }}
QPushButton#ok {{
    background: #15803d; border-color: {OK};
    font-weight: 700; color: white;
}}
QPushButton#ok:hover {{ background: #16a34a; }}
QPushButton#small {{
    padding: 3px 7px; font-size: 11px;
}}
QTableWidget {{
    background: {PANEL}; border: 1px solid {BORDER};
    gridline-color: {BORDER}; border-radius: 5px;
    alternate-background-color: {PANEL2};
}}
QTableWidget::item {{ padding: 3px; }}
QTableWidget::item:selected {{ background: #1e3a8a; color: white; }}
QHeaderView::section {{
    background: {PANEL2}; border: 1px solid {BORDER};
    padding: 4px; font-weight: 600; color: {DIM};
}}
QScrollBar:vertical, QScrollBar:horizontal {{
    background: {BG}; width: 7px; height: 7px; border-radius: 3px;
}}
QScrollBar::handle:vertical, QScrollBar::handle:horizontal {{
    background: {BORDER}; border-radius: 3px; min-height: 20px;
}}
QTabWidget::pane {{
    border: 1px solid {BORDER}; border-radius: 6px; background: {PANEL};
}}
QTabBar::tab {{
    background: {BG}; border: 1px solid {BORDER};
    border-bottom: none; padding: 6px 14px;
    border-radius: 5px 5px 0 0; font-weight: 500;
}}
QTabBar::tab:selected {{ background: {PANEL}; color: {ACCENT2}; font-weight: 700; }}
QDoubleSpinBox, QLineEdit, QComboBox {{
    background: {BG}; border: 1px solid {BORDER};
    border-radius: 4px; padding: 3px 6px; color: {TEXT};
}}
QDoubleSpinBox:focus, QLineEdit:focus, QComboBox:focus {{ border-color: {ACCENT2}; }}
QSlider::groove:horizontal {{
    height: 6px; background: {BORDER}; border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT2}; border: 2px solid #93c5fd;
    width: 16px; height: 16px; margin: -5px 0; border-radius: 8px;
}}
QSlider::handle:horizontal:hover {{ background: #60a5fa; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 3px; }}
QCheckBox {{ spacing: 6px; }}
QCheckBox::indicator {{
    width: 13px; height: 13px;
    border: 1px solid {BORDER}; border-radius: 3px; background: {BG};
}}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT2}; }}
QToolBar {{
    background: {PANEL}; border-bottom: 1px solid {BORDER};
    spacing: 5px; padding: 4px 8px;
}}
QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDER}; font-size: 11px; }}
QTextEdit {{
    background: {BG}; border: 1px solid {BORDER}; border-radius: 5px;
    font-family: 'Consolas', 'Cascadia Code', monospace; font-size: 11px;
}}
"""


def _sec(text: str) -> QLabel:
    l = QLabel(text.upper())
    l.setStyleSheet(f"color:{DIM}; font-size:10px; font-weight:700; letter-spacing:0.8px; margin-top:6px;")
    return l


def _divider() -> QFrame:
    f = QFrame(); f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet(f"background:{BORDER}; max-height:1px; margin:3px 0;")
    return f


# ── DH column delegate ────────────────────────────────────────────────────────

class _JointTypeDelegate(QStyledItemDelegate):
    _OPT = ["revolute", "prismatic"]

    def createEditor(self, parent, option, index):
        c = QComboBox(parent); c.addItems(self._OPT); return c

    def setEditorData(self, editor, index):
        v = index.data() or "revolute"
        editor.setCurrentIndex(self._OPT.index(v) if v in self._OPT else 0)

    def setModelData(self, editor, model, index):
        model.setData(index, editor.currentText())

    def updateEditorGeometry(self, editor, option, index):
        editor.setGeometry(option.rect)


# ── Forward Kinematics Panel ──────────────────────────────────────────────────

class FKPanel(QWidget):
    joints_moved        = Signal(list)   # list[float]
    request_target_at_ee = Signal(list)  # current joint positions

    def __init__(self, parent=None):
        super().__init__(parent)
        self._robot: RobotDefinition | None = None
        self._sliders: list[QSlider] = []
        self._spins:   list[QDoubleSpinBox] = []
        self._vlabels: list[QLabel] = []
        self._lock = False
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        btn_row = QHBoxLayout()
        for label, tip, cb in [
            ("⟲ Home",    "Reset all joints to zero",                  self._home),
            ("🎲 Random", "Set random joint configuration",             self._random),
        ]:
            b = QPushButton(label); b.setToolTip(tip)
            b.setObjectName("small"); b.clicked.connect(cb)
            btn_row.addWidget(b)

        tgt_btn = QPushButton("🎯 Set IK Target at EE")
        tgt_btn.setObjectName("accent")
        tgt_btn.setToolTip("Copy current EE position to IK Target")
        tgt_btn.clicked.connect(lambda: self.request_target_at_ee.emit(self._get_q()))
        btn_row.addWidget(tgt_btn)
        lay.addLayout(btn_row)
        lay.addWidget(_divider())

        lay.addWidget(_sec("Joint Sliders — Real-Time Forward Kinematics"))

        self._container = QWidget()
        self._clay = QVBoxLayout(self._container)
        self._clay.setContentsMargins(0, 0, 0, 0)
        self._clay.setSpacing(5)

        scroll = QScrollArea()
        scroll.setWidget(self._container)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        lay.addWidget(scroll, stretch=1)

    def set_robot(self, robot: RobotDefinition):
        self._robot = robot
        # Wipe old cards
        while self._clay.count():
            item = self._clay.takeAt(0)
            if item is not None:
                w = item.widget()
                if w is not None:
                    w.deleteLater()
        self._sliders.clear()
        self._spins.clear()
        self._vlabels.clear()

        for idx, joint in enumerate(robot.joints):
            card = QFrame()
            card.setStyleSheet(
                f"background:{PANEL2}; border:1px solid {BORDER}; border-radius:6px; padding:5px;"
            )
            cl = QVBoxLayout(card)
            cl.setContentsMargins(6, 5, 6, 5)
            cl.setSpacing(3)

            # Header row
            hl = QHBoxLayout()
            name_l = QLabel(f"<b>{joint.name}</b>  <span style='color:{DIM};font-size:10px;'>({joint.type.value})</span>")
            name_l.setTextFormat(Qt.TextFormat.RichText)
            val_l  = QLabel("0.0°  (0.000 rad)")
            val_l.setStyleSheet(f"color:{ACCENT2}; font-weight:700; font-family:monospace;")
            hl.addWidget(name_l); hl.addStretch(); hl.addWidget(val_l)
            cl.addLayout(hl)

            # Slider + spinbox
            ctrl = QHBoxLayout()
            lo = int(joint.limits.position_min * 1000)
            hi = int(joint.limits.position_max * 1000)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(lo, hi); slider.setValue(0)

            spin = QDoubleSpinBox()
            spin.setRange(joint.limits.position_min, joint.limits.position_max)
            spin.setDecimals(3); spin.setSingleStep(0.05); spin.setValue(0.0)
            spin.setFixedWidth(70)

            def _mk(j=joint, s=slider, sp=spin, vl=val_l):
                def _fmt(q):
                    if j.type == JointType.REVOLUTE:
                        return f"{math.degrees(q):+.1f}°  ({q:+.4f} rad)"
                    return f"{q * 100:+.2f} cm  ({q:+.4f} m)"

                def on_s(v):
                    if self._lock: return
                    self._lock = True
                    q = v / 1000.0
                    sp.setValue(q); vl.setText(_fmt(q))
                    self._lock = False
                    self.joints_moved.emit(self._get_q())

                def on_sp(q):
                    if self._lock: return
                    self._lock = True
                    s.setValue(int(q * 1000)); vl.setText(_fmt(q))
                    self._lock = False
                    self.joints_moved.emit(self._get_q())

                s.valueChanged.connect(on_s)
                sp.valueChanged.connect(on_sp)

            _mk()

            ctrl.addWidget(slider, stretch=1); ctrl.addWidget(spin)
            cl.addLayout(ctrl)

            # Limit labels
            ll = QHBoxLayout()
            is_rev = joint.type == JointType.REVOLUTE
            lo_str = f"{math.degrees(joint.limits.position_min):.0f}°" if is_rev else f"{joint.limits.position_min:.2f} m"
            hi_str = f"{math.degrees(joint.limits.position_max):.0f}°" if is_rev else f"{joint.limits.position_max:.2f} m"
            lbl_min = QLabel(f"<span style='color:{DIM};font-size:10px;'>Min {lo_str}</span>")
            lbl_min.setTextFormat(Qt.TextFormat.RichText)
            lbl_max = QLabel(f"<span style='color:{DIM};font-size:10px;'>Max {hi_str}</span>")
            lbl_max.setTextFormat(Qt.TextFormat.RichText)
            ll.addWidget(lbl_min)
            ll.addStretch()
            ll.addWidget(lbl_max)

            cl.addLayout(ll)

            self._clay.addWidget(card)
            self._sliders.append(slider)
            self._spins.append(spin)
            self._vlabels.append(val_l)

        self._clay.addStretch()

    def _get_q(self) -> list[float]:
        return [sp.value() for sp in self._spins]

    def set_joint_positions(self, positions: list[float]):
        if not self._robot:
            return
        self._lock = True
        for q, s, sp, vl, j in zip(positions, self._sliders, self._spins, self._vlabels, self._robot.joints):
            sp.setValue(q); s.setValue(int(q * 1000))
            if j.type == JointType.REVOLUTE:
                vl.setText(f"{math.degrees(q):+.1f}°  ({q:+.4f} rad)")
            else:
                vl.setText(f"{q * 100:+.2f} cm  ({q:+.4f} m)")
        self._lock = False

    def _home(self):
        if self._robot:
            self.set_joint_positions([0.0] * len(self._robot.joints))
            self.joints_moved.emit(self._get_q())

    def _random(self):
        if not self._robot: return
        rng = np.random.default_rng()
        vals = [float(rng.uniform(j.limits.position_min * 0.8, j.limits.position_max * 0.8))
                for j in self._robot.joints]
        self.set_joint_positions(vals)
        self.joints_moved.emit(self._get_q())


# ── Inverse Kinematics Panel ──────────────────────────────────────────────────

class IKPanel(QWidget):
    target_changed           = Signal(Target)
    solve_requested          = Signal()
    solution_selected        = Signal(str)
    generate_traj_requested  = Signal(float)
    ghost_toggled            = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ids: list[str] = []
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)

        # ── Target ────────────────────────────────────────────────────────────
        gb = QGroupBox("Target Position")
        tl = QVBoxLayout(gb); tl.setSpacing(5)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self._mode = QComboBox()
        self._mode.addItems(["Point  (position only)", "Pose  (position + orientation)"])
        mode_row.addWidget(self._mode, stretch=1)
        tl.addLayout(mode_row)

        self._sx = QDoubleSpinBox(); self._sy = QDoubleSpinBox(); self._sz = QDoubleSpinBox()
        for spin in (self._sx, self._sy, self._sz):
            spin.setRange(-5.0, 5.0); spin.setDecimals(3); spin.setSingleStep(0.02)

        for axis, spin in [("X (m)", self._sx), ("Y (m)", self._sy), ("Z (m)", self._sz)]:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"<b>{axis}:</b>"))
            row.addWidget(spin, stretch=1)
            for sign, delta in [("−", -0.05), ("+", 0.05)]:
                b = QPushButton(sign); b.setFixedWidth(22); b.setObjectName("small")
                b.clicked.connect(lambda _, s=spin, d=delta: s.setValue(s.value() + d))
                row.addWidget(b)
            tl.addLayout(row)

        set_tgt = QPushButton("🎯 Apply Target to 3D Scene")
        set_tgt.clicked.connect(self._emit_target)
        tl.addWidget(set_tgt)
        lay.addWidget(gb)

        # ── Solve ─────────────────────────────────────────────────────────────
        self._solve_btn = QPushButton("🔍  Solve Inverse Kinematics")
        self._solve_btn.setObjectName("accent")
        self._solve_btn.setFixedHeight(34)
        font = self._solve_btn.font(); font.setPointSize(11); font.setBold(True)
        self._solve_btn.setFont(font)
        self._solve_btn.clicked.connect(self.solve_requested.emit)
        lay.addWidget(self._solve_btn)
        lay.addWidget(_divider())

        # ── Solutions ─────────────────────────────────────────────────────────
        sol_row = QHBoxLayout()
        self._sol_lbl = QLabel("No IK solutions computed.")
        self._sol_lbl.setStyleSheet(f"color:{DIM}; font-weight:600;")
        sol_row.addWidget(self._sol_lbl)
        sol_row.addStretch()
        self._ghost_cb = QCheckBox("Ghosts in 3D")
        self._ghost_cb.setChecked(True)
        self._ghost_cb.toggled.connect(self.ghost_toggled.emit)
        sol_row.addWidget(self._ghost_cb)
        lay.addLayout(sol_row)

        self._table = QTableWidget(0, 5)
        self._table.setHorizontalHeaderLabels(["ID", "Score", "Pos Err", "Margin", "Status"])
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setAlternatingRowColors(True)
        self._table.cellClicked.connect(lambda r, c: (
            self.solution_selected.emit(self._ids[r]) if r < len(self._ids) else None
        ))
        lay.addWidget(self._table, stretch=1)

        # ── Trajectory ────────────────────────────────────────────────────────
        traj_gb = QGroupBox("Trajectory Planning")
        tl2 = QHBoxLayout(traj_gb)
        tl2.addWidget(QLabel("Duration:"))
        self._dur = QDoubleSpinBox()
        self._dur.setRange(0.5, 30.0); self._dur.setValue(3.0); self._dur.setSuffix(" s")
        tl2.addWidget(self._dur)
        gen_btn = QPushButton("⚡ Plan Trajectory")
        gen_btn.setObjectName("ok")
        gen_btn.clicked.connect(lambda: self.generate_traj_requested.emit(self._dur.value()))
        tl2.addWidget(gen_btn)
        lay.addWidget(traj_gb)

    def set_target_pose(self, x: float, y: float, z: float):
        self._sx.setValue(x); self._sy.setValue(y); self._sz.setValue(z)
        self._emit_target()

    def _emit_target(self):
        mode = "point" if self._mode.currentIndex() == 0 else "pose"
        target = Target(
            id=str(uuid.uuid4()), mode=mode,
            pose=Pose(position=[self._sx.value(), self._sy.value(), self._sz.value()]),
            position_tolerance=0.002,
        )
        self.target_changed.emit(target)

    def update_solutions(self, candidates, analyses, selected_id):
        self._ids = [c.id for c in candidates]
        self._table.setRowCount(len(candidates))
        if not candidates:
            self._sol_lbl.setText("No feasible solutions found.")
            self._sol_lbl.setStyleSheet(f"color:{ERR}; font-weight:600;")
            return
        self._sol_lbl.setText(f"✓ {len(candidates)} Feasible Solutions")
        self._sol_lbl.setStyleSheet(f"color:{OK}; font-weight:600;")

        am = {a.solution_id: a for a in analyses}
        for row, c in enumerate(candidates):
            a = am.get(c.id)
            is_rec = c.id == selected_id
            items = [
                c.id[:8],
                f"{a.score:.3f}" if a and a.score is not None else "—",
                f"{c.position_error * 1000:.2f} mm" if c.position_error is not None else "—",
                f"{a.joint_limit_margin:.2f}" if a else "—",
                "⭐ Active" if is_rec else "Select",
            ]
            for col, txt in enumerate(items):
                item = QTableWidgetItem(txt)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if is_rec:
                    item.setForeground(QColor(AMBER))
                    f = item.font(); f.setBold(True); item.setFont(f)
                self._table.setItem(row, col, item)

        if selected_id in self._ids:
            self._table.selectRow(self._ids.index(selected_id))


# ── DH Parameters Panel ───────────────────────────────────────────────────────

class DHPanel(QWidget):
    dh_applied = Signal(RobotDefinition)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._robot: RobotDefinition | None = None
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)
        lay.addWidget(_sec("Standard DH Parameters"))

        self._tbl = QTableWidget(0, 8)
        self._tbl.setHorizontalHeaderLabels(
            ["Joint", "Type", "θ (rad)", "d (m)", "a (m)", "α (rad)", "q_min", "q_max"]
        )
        hdr = self._tbl.horizontalHeader()
        hdr.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for c, w in enumerate([90, 90, 75, 70, 70, 75, 68, 68]):
            self._tbl.setColumnWidth(c, w)
        self._del = _JointTypeDelegate(self._tbl)
        self._tbl.setItemDelegateForColumn(1, self._del)
        lay.addWidget(self._tbl)

        btn = QPushButton("↻  Apply DH Parameter Changes")
        btn.setObjectName("accent"); btn.clicked.connect(self._apply)
        lay.addWidget(btn)

    def set_robot(self, robot: RobotDefinition):
        self._robot = robot
        self._tbl.blockSignals(True)
        self._tbl.setRowCount(len(robot.joints))
        for row, j in enumerate(robot.joints):
            name = QTableWidgetItem(j.name)
            name.setFlags(name.flags() & ~Qt.ItemFlag.ItemIsEditable)
            name.setForeground(QColor(DIM))
            self._tbl.setItem(row, 0, name)
            self._tbl.setItem(row, 1, QTableWidgetItem(j.type.value))
            for col, v in enumerate([j.dh.theta, j.dh.d, j.dh.a, j.dh.alpha,
                                      j.limits.position_min, j.limits.position_max], start=2):
                self._tbl.setItem(row, col, QTableWidgetItem(f"{v:.4f}"))
        self._tbl.blockSignals(False)

    def _apply(self):
        if not self._robot:
            return
        try:
            new_joints = []
            for row, j in enumerate(self._robot.joints):
                raw = [self._tbl.item(row, c) for c in range(1, 8)]
                if any(x is None for x in raw):
                    continue
                # All entries are non-None — build a fully-typed list so the checker is happy
                cells: list[QTableWidgetItem] = [x for x in raw if x is not None]
                if len(cells) < 7:
                    continue
                jt = JointType.REVOLUTE if cells[0].text().strip().lower() == "revolute" else JointType.PRISMATIC
                vals = [float(cells[i].text()) for i in range(1, 7)]
                new_joints.append(j.model_copy(update={
                    "type": jt,
                    "dh": DHParameters(theta=vals[0], d=vals[1], a=vals[2], alpha=vals[3]),
                    "limits": j.limits.model_copy(update={
                        "position_min": vals[4], "position_max": vals[5]
                    }),
                }))
            self._robot = self._robot.model_copy(update={"joints": new_joints})
            self.dh_applied.emit(self._robot)
        except Exception as exc:
            QMessageBox.critical(self, "DH Parse Error", str(exc))


# ── Analysis Panel ────────────────────────────────────────────────────────────

class AnalysisPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        self._lbl = QLabel("Select an IK solution to view analysis.")
        self._lbl.setWordWrap(True)
        self._lbl.setStyleSheet(
            f"background:{PANEL2}; padding:10px; border-radius:6px;"
            f" border:1px solid {BORDER}; color:{TEXT};"
        )
        lay.addWidget(self._lbl)

        lay.addWidget(_sec("Joint Trajectory  (q vs Time)"))
        pg.setConfigOption("background", "#0a0e13")
        pg.setConfigOption("foreground", "#c9d1d9")
        self._plot = pg.PlotWidget()
        self._plot.setLabel("left", "q", units="rad / m")
        self._plot.setLabel("bottom", "t", units="s")
        self._plot.addLegend(); self._plot.showGrid(x=True, y=True, alpha=0.25)
        lay.addWidget(self._plot)

    def update_analysis(self, cand, analysis, robot):
        if not cand or not analysis:
            return
        self._lbl.setText(
            f"<b style='color:{CYAN}'>Solution {cand.id[:8]}</b><br>"
            f"<b>Pos Error:</b> {cand.position_error * 1000:.3f} mm &nbsp;"
            f"<b>Score:</b> {analysis.score:.4f}<br>"
            f"<b>Motion Cost:</b> {analysis.motion_cost:.3f} &nbsp;"
            f"<b>Limit Margin:</b> {analysis.joint_limit_margin:.3f}<br>"
            f"<b>Singularity:</b> {analysis.singularity_status} &nbsp;"
            f"<b>Manipulability:</b> {analysis.manipulability:.4f}"
        )

    def plot_trajectory(self, traj, robot):
        self._plot.clear(); self._plot.addLegend()
        if not traj or not robot:
            return
        times = [pt.t for pt in traj.points]
        palette = ["#38bdf8", "#fbbf24", "#34d399", "#f87171", "#c084fc", "#818cf8", "#fb923c"]
        for i, joint in enumerate(robot.joints):
            vals = [pt.q[i] for pt in traj.points]
            self._plot.plot(times, vals,
                            pen=pg.mkPen(palette[i % len(palette)], width=2.2),
                            name=joint.name)


# ── Calculation Explorer ──────────────────────────────────────────────────────

class CalcExplorer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.addWidget(_sec("Mathematical Inspection"))
        self._txt = QTextEdit(); self._txt.setReadOnly(True)
        self._txt.setPlaceholderText("Run IK to populate…")
        lay.addWidget(self._txt)

    def refresh(self, state, robot):
        from robokinematics.robotics.jacobian import analyse_jacobian
        if not robot or not state.robot_state:
            return
        q = state.robot_state.joint_positions
        tf = forward_kinematics(robot, q)
        ja = analyse_jacobian(robot, q)
        ee = tf[-1]; ep = ee[:3, 3]
        self._txt.setPlainText(
            f"=== {robot.name} ({len(robot.joints)} DOF) ===\n"
            f"q = [{', '.join(f'{x:+.4f}' for x in q)}]\n\n"
            f"EE Position : [{ep[0]:+.5f},  {ep[1]:+.5f},  {ep[2]:+.5f}]\n\n"
            f"T_0^EE:\n{np.array2string(ee, precision=4, suppress_small=True)}\n\n"
            f"Jacobian J(q) [6×{len(robot.joints)}]:\n"
            f"{np.array2string(ja.jacobian, precision=4, suppress_small=True)}\n\n"
            f"Singular Values: {np.array2string(ja.singular_values, precision=4)}\n"
            f"Rank: {ja.rank}   Condition Number: {ja.condition_number:.3f}\n"
            f"Manipulability : {ja.manipulability:.5f}"
        )


# ── Playback Controls ─────────────────────────────────────────────────────────

class PlaybackBar(QWidget):
    play_pressed   = Signal()
    pause_pressed  = Signal()
    stop_pressed   = Signal()
    speed_changed  = Signal(float)
    scrub_changed  = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build()

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 5, 10, 5)
        lay.setSpacing(6)

        for label, sig in [("▶ Play", self.play_pressed),
                            ("⏸ Pause", self.pause_pressed),
                            ("⏹ Stop",  self.stop_pressed)]:
            b = QPushButton(label); b.setFixedWidth(72)
            b.clicked.connect(sig.emit); lay.addWidget(b)

        self._tl = QSlider(Qt.Orientation.Horizontal)
        self._tl.setRange(0, 1000); self._tl.setValue(0)
        self._tl.valueChanged.connect(lambda v: self.scrub_changed.emit(v / 1000.0))
        lay.addWidget(self._tl, stretch=1)

        self._tlbl = QLabel("0.00 / 0.00 s")
        self._tlbl.setFixedWidth(105)
        self._tlbl.setStyleSheet("font-family:monospace; font-weight:600;")
        lay.addWidget(self._tlbl)

        lay.addWidget(QLabel("Speed:"))
        self._spd = QComboBox()
        self._spd.addItems(["0.25×", "0.5×", "1.0×", "2.0×", "4.0×"])
        self._spd.setCurrentIndex(2)
        self._spd.currentIndexChanged.connect(
            lambda i: self.speed_changed.emit([0.25, 0.5, 1.0, 2.0, 4.0][i])
        )
        lay.addWidget(self._spd)

    def set_time(self, t: float, total: float):
        self._tlbl.setText(f"{t:.2f} / {total:.2f} s")
        if total > 0:
            self._tl.blockSignals(True)
            self._tl.setValue(int(t / total * 1000))
            self._tl.blockSignals(False)


# ── Main Window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("RoboKinematics Studio  |  GPU Accelerated")
        self.resize(1560, 900)
        self.setMinimumSize(1100, 680)

        self._ctrl   = ApplicationController()
        self._ptimer = QTimer(self)       # playback tick
        self._playing = False
        self._ptime   = 0.0

        self._build_toolbar()
        self._build_central()
        self._build_statusbar()
        self._connect()
        self.setStyleSheet(STYLE)

        # Auto-load reference robot then set a nice initial pose & target
        ref = Path(__file__).parent.parent.parent.parent / "assets" / "robots" / "reference_4dof.json"
        if ref.exists():
            QTimer.singleShot(200, lambda: self._startup(ref))

    # ── Build ─────────────────────────────────────────────────────────────────

    def _build_toolbar(self):
        tb = self.addToolBar("Main"); tb.setMovable(False)

        open_btn = QPushButton("📂 Open Robot…")
        open_btn.clicked.connect(self._open_robot)
        tb.addWidget(open_btn)
        tb.addSeparator()

        self._solve_tb_btn = QPushButton("🔍  Solve IK")
        self._solve_tb_btn.setObjectName("accent")
        self._solve_tb_btn.clicked.connect(self._ctrl.solve_ik)
        tb.addWidget(self._solve_tb_btn)
        tb.addSeparator()

        tb.addWidget(QLabel(f"<span style='color:{DIM}'>View:</span>"))
        for preset, label in [("iso","Iso"), ("top","Top"), ("front","Front"), ("side","Side")]:
            b = QPushButton(label); b.setObjectName("small")
            b.clicked.connect(lambda _, p=preset: self._scene.set_camera_preset(p))
            tb.addWidget(b)

        fit_btn = QPushButton("⛶ Fit"); fit_btn.setObjectName("small")
        fit_btn.clicked.connect(self._fit)
        tb.addWidget(fit_btn)
        tb.addSeparator()

        self._insp_btn = QPushButton("📐 Inspector")
        self._insp_btn.setCheckable(True); self._insp_btn.setChecked(True)
        self._insp_btn.toggled.connect(lambda v: self._right.setVisible(v))
        tb.addWidget(self._insp_btn)
        tb.addSeparator()

        self._pill = QLabel("● READY")
        self._pill.setStyleSheet(
            f"color:{DIM}; background:{PANEL2}; padding:3px 12px; border-radius:10px; font-weight:600;"
        )
        tb.addWidget(self._pill)

    def _build_central(self):
        split = QSplitter(Qt.Orientation.Horizontal)
        self.setCentralWidget(split)

        # ── Left ──────────────────────────────────────────────────────────────
        self._left = QTabWidget()
        self._left.setMinimumWidth(300); self._left.setMaximumWidth(420)
        self._fk = FKPanel()
        self._ik = IKPanel()
        self._left.addTab(self._fk, "🎮 Forward K.")
        self._left.addTab(self._ik, "🎯 Inverse K.")
        split.addWidget(self._left)

        # ── Centre ────────────────────────────────────────────────────────────
        centre = QWidget()
        cl = QVBoxLayout(centre); cl.setContentsMargins(0,0,0,0); cl.setSpacing(0)

        vp_wrap = QWidget()
        vl = QVBoxLayout(vp_wrap); vl.setContentsMargins(0,0,0,0)
        self._scene = RoboKinematicsScene()
        vl.addWidget(self._scene)

        # Floating HUD
        self._hud = QFrame(self._scene)
        self._hud.setStyleSheet(
            f"background:rgba(10,14,19,0.88); border:1px solid {BORDER};"
            f" border-radius:8px; padding:8px;"
        )
        hud_l = QVBoxLayout(self._hud)
        hud_l.setContentsMargins(8,6,8,6); hud_l.setSpacing(2)

        self._hud_title = QLabel("RoboKinematics Studio")
        self._hud_title.setStyleSheet(f"color:{CYAN}; font-weight:700; font-size:11px;")
        self._hud_pos   = QLabel("EE:  X: +0.000 m   Y: +0.000 m   Z: +0.000 m")
        self._hud_pos.setStyleSheet(f"color:{TEXT}; font-family:monospace; font-size:11px;")
        self._hud_rot   = QLabel("RPY: R: +0.0°   P: +0.0°   Y: +0.0°")
        self._hud_rot.setStyleSheet(f"color:{DIM}; font-family:monospace; font-size:10px;")

        hud_l.addWidget(self._hud_title)
        hud_l.addWidget(self._hud_pos)
        hud_l.addWidget(self._hud_rot)
        self._hud.move(12, 12)
        self._hud.adjustSize()

        cl.addWidget(vp_wrap, stretch=1)

        self._pbar = PlaybackBar()
        cl.addWidget(self._pbar)
        split.addWidget(centre)

        # ── Right ─────────────────────────────────────────────────────────────
        self._right = QTabWidget()
        self._right.setMinimumWidth(300); self._right.setMaximumWidth(460)
        self._dh   = DHPanel()
        self._anal = AnalysisPanel()
        self._calc = CalcExplorer()
        self._right.addTab(self._dh,   "📐 DH Model")
        self._right.addTab(self._anal, "📊 Analysis")
        self._right.addTab(self._calc, "🧮 Calculations")
        split.addWidget(self._right)

        split.setSizes([360, 840, 380])

    def _build_statusbar(self):
        sb = QStatusBar(); self.setStatusBar(sb)
        self._stxt = QLabel("Ready"); sb.addPermanentWidget(self._stxt)

    # ── Signal wiring ─────────────────────────────────────────────────────────

    def _connect(self):
        c = self._ctrl

        # Controller → Window
        c.robot_changed.connect(self._on_robot)
        c.target_changed.connect(self._on_target)
        c.ik_started.connect(lambda: self._set_pill("SOLVING", WARN))
        c.ik_finished.connect(self._on_ik_done)
        c.solution_selected.connect(self._on_sol_selected)
        c.trajectory_updated.connect(self._on_traj)
        c.status_changed.connect(self._on_status)

        # FK panel
        self._fk.joints_moved.connect(self._on_fk)
        self._fk.request_target_at_ee.connect(self._copy_ee_to_target)

        # IK panel
        self._ik.target_changed.connect(c.set_target)
        self._ik.solve_requested.connect(c.solve_ik)
        self._ik.solution_selected.connect(c.select_solution)
        self._ik.generate_traj_requested.connect(c.generate_trajectory)
        self._ik.ghost_toggled.connect(
            lambda v: self._scene.set_group_visible("ghost_solutions", v)
        )

        # DH panel
        self._dh.dh_applied.connect(c.set_robot)

        # Playback
        self._pbar.play_pressed.connect(self._play)
        self._pbar.pause_pressed.connect(self._pause)
        self._pbar.stop_pressed.connect(self._stop)
        self._pbar.speed_changed.connect(lambda v: setattr(c.state, "playback_speed", v))
        self._pbar.scrub_changed.connect(self._scrub)
        self._ptimer.timeout.connect(self._tick)

    # ── Startup ───────────────────────────────────────────────────────────────

    def _startup(self, ref: Path):
        self._ctrl.load_robot(ref)
        robot = self._ctrl.state.robot
        if not robot:
            return
        # Nice initial pose so arm looks like an arm
        n = len(robot.joints)
        nice = [0.30, -0.40, 0.50, 0.20][:n] + [0.0] * max(0, n - 4)
        self._fk.set_joint_positions(nice)
        self._ctrl.set_joint_positions(nice)
        tf = self._ctrl.get_fk_transforms(nice)
        if tf:
            ep = tf[-1][:3, 3]
            self._ik.set_target_pose(float(ep[0]), float(ep[1]), float(ep[2]))
            self._scene.render_robot(robot, tf)
            self._update_hud(tf[-1])
            QTimer.singleShot(300, self._fit)

    # ── FK live update ────────────────────────────────────────────────────────

    def _on_fk(self, q: list[float]):
        """60 fps FK update driven by slider movement."""
        robot = self._ctrl.state.robot
        if not robot:
            return
        self._ctrl.set_joint_positions(q)
        tf = self._ctrl.get_fk_transforms(q)
        self._scene.render_robot(robot, tf)
        self._update_hud(tf[-1])

    def _update_hud(self, ee: np.ndarray):
        p = ee[:3, 3]
        self._hud_pos.setText(f"EE:  X:{p[0]:+.3f} m   Y:{p[1]:+.3f} m   Z:{p[2]:+.3f} m")
        try:
            rpy = Rotation.from_matrix(ee[:3, :3]).as_euler("xyz", degrees=True)
            self._hud_rot.setText(f"RPY: R:{rpy[0]:+.1f}°   P:{rpy[1]:+.1f}°   Y:{rpy[2]:+.1f}°")
        except Exception:
            pass
        self._hud.adjustSize()

    def _copy_ee_to_target(self, q: list[float]):
        tf = self._ctrl.get_fk_transforms(q)
        if tf:
            ep = tf[-1][:3, 3]
            self._ik.set_target_pose(float(ep[0]), float(ep[1]), float(ep[2]))

    def _update_scene(self):
        state = self._ctrl.state
        if state.robot and state.robot_state:
            q  = state.robot_state.joint_positions
            tf = self._ctrl.get_fk_transforms(q)
            self._scene.render_robot(state.robot, tf)
            if tf: self._update_hud(tf[-1])

    def _fit(self):
        tf = self._ctrl.get_fk_transforms()
        if tf: self._scene.fit_robot(tf)

    # ── Controller handlers ───────────────────────────────────────────────────

    def _on_robot(self):
        robot = self._ctrl.state.robot
        if not robot: return
        self._hud_title.setText(f"{robot.name}  ({len(robot.joints)} DOF)")
        self._fk.set_robot(robot)
        self._dh.set_robot(robot)
        self._update_scene()

    def _on_target(self):
        t = self._ctrl.state.target
        if t: self._scene.render_target(t)

    def _on_ik_done(self):
        state = self._ctrl.state
        self._ik.update_solutions(
            state.ik_candidates, state.solution_analyses, state.selected_candidate_id
        )
        if not state.ik_candidates or not state.robot:
            self._scene.clear_ghost_solutions(); return

        # Render selected (primary)
        sel = state.get_selected_candidate()
        if sel:
            tf = self._ctrl.get_fk_transforms(sel.joint_positions)
            self._scene.render_robot(state.robot, tf)
            self._fk.set_joint_positions(sel.joint_positions)
            self._update_hud(tf[-1])
            self._anal.update_analysis(sel, state.get_selected_analysis(), state.robot)

        # Render ghost alternatives
        ghosts = [
            self._ctrl.get_fk_transforms(c.joint_positions)
            for c in state.ik_candidates
            if (sel is None or c.id != sel.id)
        ]
        self._scene.render_ghost_solutions(state.robot, ghosts)
        self._calc.refresh(state, state.robot)

    def _on_sol_selected(self, cid: str):
        state = self._ctrl.state
        cand  = state.get_selected_candidate()
        if not cand or not state.robot: return

        tf = self._ctrl.get_fk_transforms(cand.joint_positions)
        self._scene.render_robot(state.robot, tf)
        self._fk.set_joint_positions(cand.joint_positions)
        self._update_hud(tf[-1])
        self._anal.update_analysis(cand, state.get_selected_analysis(), state.robot)

        ghosts = [
            self._ctrl.get_fk_transforms(c.joint_positions)
            for c in state.ik_candidates if c.id != cand.id
        ]
        self._scene.render_ghost_solutions(state.robot, ghosts)

    def _on_traj(self):
        state = self._ctrl.state
        if state.trajectory and state.robot:
            step = max(1, len(state.trajectory.points) // 80)
            tseq = [self._ctrl.get_fk_transforms(pt.q.tolist())
                    for pt in state.trajectory.points[::step]]
            self._scene.render_trajectory(tseq)
            self._anal.plot_trajectory(state.trajectory, state.robot)
            self._ptime = 0.0

    def _on_status(self, status: str, detail: str):
        colors = {
            "SOLVING": WARN, "SOLUTIONS FOUND": OK,
            "NO FEASIBLE SOLUTION": ERR, "TARGET UNREACHABLE": ERR,
            "TARGET REACHED": OK, "PLAYING": ACCENT2, "PAUSED": WARN,
        }
        col = colors.get(status, DIM)
        self._set_pill(status, col)
        self._stxt.setText(detail or status)

    def _set_pill(self, text: str, color: str):
        self._pill.setText(f"● {text}")
        self._pill.setStyleSheet(
            f"color:{color}; background:{PANEL2}; padding:3px 12px;"
            f" border-radius:10px; font-weight:600;"
        )

    # ── Dialogs ───────────────────────────────────────────────────────────────

    def _open_robot(self):
        p, _ = QFileDialog.getOpenFileName(self, "Load Robot JSON", "", "JSON (*.json)")
        if p:
            try:
                self._ctrl.load_robot(p)
                QTimer.singleShot(100, self._fit)
            except Exception as exc:
                QMessageBox.critical(self, "Load Error", str(exc))

    # ── Playback ──────────────────────────────────────────────────────────────

    def _play(self):
        if not self._ctrl.state.trajectory: return
        self._playing = True
        self._ctrl._set_status(AppStatus.PLAYING)
        self._ptimer.start(16)   # 60 fps

    def _pause(self):
        self._playing = False; self._ptimer.stop()
        self._ctrl._set_status(AppStatus.PAUSED)

    def _stop(self):
        self._playing = False; self._ptimer.stop(); self._ptime = 0.0
        self._ctrl._set_status(AppStatus.READY_FOR_PLAYBACK)

    def _scrub(self, norm: float):
        t = self._ctrl.state.trajectory
        if t:
            self._ptime = norm * t.duration
            self._render_frame()

    def _tick(self):
        t = self._ctrl.state.trajectory
        if not t or not self._playing: return
        self._ptime += 0.016 * self._ctrl.state.playback_speed
        if self._ptime >= t.duration:
            self._ptime = t.duration
            self._ptimer.stop(); self._playing = False
            self._ctrl._set_status(AppStatus.TARGET_REACHED, "Trajectory complete.")
        self._render_frame()

    def _render_frame(self):
        state = self._ctrl.state
        if not state.trajectory or not state.robot: return
        t = self._ptime
        traj = state.trajectory
        idx  = min(int(t / traj.duration * len(traj.points)), len(traj.points) - 1)
        q    = traj.points[idx].q.tolist()
        tf   = self._ctrl.get_fk_transforms(q)
        self._scene.render_robot(state.robot, tf)
        self._fk.set_joint_positions(q)
        self._update_hud(tf[-1])
        self._pbar.set_time(t, traj.duration)
