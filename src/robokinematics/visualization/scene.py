"""3D Scene renderer — GPU-accelerated via PyQtGraph OpenGL.

RTX 4070 optimised:
  - Closed-cap cylinder meshes for volumetric links
  - Smooth-shaded sphere joints and EE glyph
  - Translucent ghost-robot overlays for all IK alternatives
  - Reticle target marker with XYZ rings + triad
  - Coordinate frames per joint
  - Workspace point-cloud
  - Trajectory ribbon (line-strip)
  - Smooth-interpolated trajectory animation driven externally at 60 fps
"""

from __future__ import annotations

import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget, MeshData

from robokinematics.domain.models import RobotDefinition, Target

# ── Palette ───────────────────────────────────────────────────────────────────
_LINK_COLOR      = (0.18, 0.55, 0.92, 1.00)
_JOINT_COLOR     = (0.98, 0.62, 0.12, 1.00)
_BASE_COLOR      = (0.22, 0.28, 0.36, 1.00)
_EE_COLOR        = (0.12, 0.95, 0.60, 1.00)
_TARGET_COLOR    = (1.00, 0.22, 0.18, 1.00)
_TARGET_RING     = (1.00, 0.38, 0.28, 0.70)
_TRAJ_COLOR      = (0.12, 0.95, 0.60, 1.00)
_GHOST_LINK      = (0.22, 0.72, 1.00, 0.22)
_GHOST_JOINT     = (0.40, 0.80, 1.00, 0.30)
_GHOST_SKELETON  = (0.40, 0.88, 1.00, 0.55)
_FRAME_X         = (0.96, 0.22, 0.22, 1.00)
_FRAME_Y         = (0.22, 0.90, 0.28, 1.00)
_FRAME_Z         = (0.22, 0.50, 1.00, 1.00)
_WORKSPACE_COLOR = (0.22, 0.68, 0.95, 0.22)


# ── Mesh builders ─────────────────────────────────────────────────────────────

def _sphere_mesh(center, radius: float = 0.025, rows: int = 14, cols: int = 14):
    verts = []
    for i in range(rows + 1):
        phi = np.pi * i / rows
        for j in range(cols):
            theta = 2 * np.pi * j / cols
            verts.append([
                center[0] + radius * np.sin(phi) * np.cos(theta),
                center[1] + radius * np.sin(phi) * np.sin(theta),
                center[2] + radius * np.cos(phi),
            ])
    verts = np.array(verts, dtype=np.float32)
    faces = []
    for i in range(rows):
        for j in range(cols):
            a = i * cols + j
            b = i * cols + (j + 1) % cols
            c = (i + 1) * cols + (j + 1) % cols
            d = (i + 1) * cols + j
            faces += [[a, b, c], [a, c, d]]
    return verts, np.array(faces, dtype=np.int32)


def _cylinder_mesh(start, end, radius: float = 0.030, segs: int = 20):
    """Closed-cap cylinder between two 3-D points."""
    d = np.asarray(end, dtype=np.float64) - np.asarray(start, dtype=np.float64)
    length = float(np.linalg.norm(d))
    if length < 1e-7:
        return None, None
    axis = d / length

    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, ref))) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, ref)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    th = np.linspace(0, 2 * np.pi, segs, endpoint=False)
    circle = np.cos(th)[:, None] * u + np.sin(th)[:, None] * v
    bot = np.asarray(start)[None] + radius * circle          # segs rows
    top = bot + d                                             # segs rows
    c_bot = np.asarray(start, dtype=np.float64)[None]        # cap centre
    c_top = np.asarray(end,   dtype=np.float64)[None]

    verts = np.vstack([bot, top, c_bot, c_top]).astype(np.float32)
    ibot, itop = 2 * segs, 2 * segs + 1

    faces = []
    for i in range(segs):
        j = (i + 1) % segs
        faces += [
            [i, j, segs + j], [i, segs + j, segs + i],   # side
            [ibot, j, i],                                  # bottom cap
            [itop, segs + i, segs + j],                   # top cap
        ]
    return verts, np.array(faces, dtype=np.int32)


def _ring_pts(center, normal, radius: float, n: int = 40) -> np.ndarray:
    """Points for a circular ring."""
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    ref = np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(normal, ref); u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    th = np.linspace(0, 2 * np.pi, n)
    return (np.asarray(center)[None]
            + radius * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v)).astype(np.float32)


# ── Scene widget ──────────────────────────────────────────────────────────────

class RoboKinematicsScene(QWidget):
    """GPU-accelerated 3-D viewport (RTX 4070 ready)."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._items: dict[str, list] = {}
        self._ghost_count: int = 0
        self._build_ui()
        self._add_world_grid()
        self._add_world_frame()

    def _build_ui(self) -> None:
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        pg.setConfigOption("background", "#0a0e13")
        self._view = GLViewWidget()
        self._view.setBackgroundColor((10, 14, 19, 255))
        self._view.setCameraPosition(distance=1.8, elevation=28, azimuth=40)
        self._view.setMinimumSize(400, 300)
        lay.addWidget(self._view)

    # ── Item registry ─────────────────────────────────────────────────────────

    def _add_item(self, group: str, item) -> None:
        self._view.addItem(item)
        self._items.setdefault(group, []).append(item)

    def _remove_group(self, group: str) -> None:
        for item in self._items.pop(group, []):
            self._view.removeItem(item)

    # ── Primitive builders ────────────────────────────────────────────────────

    def _mesh_item(self, verts, faces, color, ghost: bool = False) -> gl.GLMeshItem:
        c = np.asarray(color, dtype=np.float32)
        opts = "translucent" if c[3] < 0.99 else "opaque"
        return gl.GLMeshItem(
            meshdata=MeshData(vertexes=verts, faces=faces),
            color=c,
            smooth=True,
            shader="shaded",
            drawEdges=ghost,
            edgeColor=(*c[:3].tolist(), 0.35) if ghost else (0, 0, 0, 0.18),
            glOptions=opts,
        )

    def _add_sphere(self, group: str, pos, radius: float, color, ghost: bool = False) -> None:
        v, f = _sphere_mesh(pos, radius)
        self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_cylinder(self, group: str, p0, p1, radius: float, color, ghost: bool = False) -> None:
        v, f = _cylinder_mesh(p0, p1, radius)
        if v is None:
            return
        self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_box(self, group: str, centre, hx, hy, hz, color) -> None:
        cx, cy, cz = centre
        verts = np.array([
            [cx-hx, cy-hy, cz-hz], [cx+hx, cy-hy, cz-hz],
            [cx+hx, cy+hy, cz-hz], [cx-hx, cy+hy, cz-hz],
            [cx-hx, cy-hy, cz+hz], [cx+hx, cy-hy, cz+hz],
            [cx+hx, cy+hy, cz+hz], [cx-hx, cy+hy, cz+hz],
        ], dtype=np.float32)
        faces = np.array([
            [0,1,2],[0,2,3],[4,5,6],[4,6,7],
            [0,1,5],[0,5,4],[2,3,7],[2,7,6],
            [0,3,7],[0,7,4],[1,2,6],[1,6,5],
        ], dtype=np.int32)
        item = gl.GLMeshItem(
            meshdata=MeshData(vertexes=verts, faces=faces),
            color=np.asarray(color, dtype=np.float32),
            smooth=False, shader="shaded",
            drawEdges=True, edgeColor=(0, 0, 0, 0.4),
            glOptions="opaque",
        )
        self._add_item(group, item)

    def _add_line(self, group: str, pts, color, width: float = 2.0, mode: str = "lines") -> None:
        item = gl.GLLinePlotItem(
            pos=np.asarray(pts, dtype=np.float32),
            color=color, width=width, antialias=True, mode=mode,
        )
        self._add_item(group, item)

    # ── Persistent elements ───────────────────────────────────────────────────

    def _add_world_grid(self) -> None:
        g = gl.GLGridItem()
        g.setSize(x=3.0, y=3.0)
        g.setSpacing(x=0.20, y=0.20)
        g.setColor((50, 65, 82, 130))
        self._view.addItem(g)

    def _add_world_frame(self) -> None:
        for d, c in [
            (np.array([1,0,0]), _FRAME_X),
            (np.array([0,1,0]), _FRAME_Y),
            (np.array([0,0,1]), _FRAME_Z),
        ]:
            pts = np.array([[0,0,0], d * 0.18], dtype=np.float32)
            item = gl.GLLinePlotItem(pos=pts, color=c, width=3.5, antialias=True)
            self._view.addItem(item)
            self._items.setdefault("world_frame", []).append(item)

    # ── Public API ────────────────────────────────────────────────────────────

    def initialize(self) -> None:
        pass

    def render_robot(
        self,
        robot: RobotDefinition,
        transforms: list[np.ndarray],
        opacity: float = 1.0,
        group: str = "robot",
        is_ghost: bool = False,
    ) -> None:
        self._remove_group(group)
        if not transforms:
            return

        a = opacity
        if is_ghost:
            link_c  = (*_GHOST_LINK[:3],  a * 0.85)
            joint_c = (*_GHOST_JOINT[:3], a * 1.10)
            ee_c    = (0.28, 1.00, 0.80,  a * 1.20)
            base_c  = (*_BASE_COLOR[:3],  a * 0.35)
        else:
            link_c  = (*_LINK_COLOR[:3],  a)
            joint_c = (*_JOINT_COLOR[:3], a)
            ee_c    = (*_EE_COLOR[:3],    a)
            base_c  = (*_BASE_COLOR[:3],  a)

        joint_positions = [np.array(t[:3, 3]) for t in transforms]

        # Base plate
        if not is_ghost:
            self._add_box(group, joint_positions[0] + np.array([0, 0, -0.022]),
                          0.11, 0.11, 0.022, base_c)

        # Link cylinders
        for i in range(len(joint_positions) - 1):
            p0, p1 = joint_positions[i], joint_positions[i + 1]
            seg = float(np.linalg.norm(p1 - p0))
            if seg < 1e-4:
                continue
            link_idx = min(i, len(robot.links) - 1)
            w = robot.links[link_idx].visual.width / 2 if robot.links else 0.025
            r = max(w, 0.015) * (0.75 if is_ghost else 1.0)
            self._add_cylinder(group, p0, p1, r, link_c, ghost=is_ghost)

        # Ghost skeleton centre-line
        if is_ghost and len(joint_positions) >= 2:
            pts = np.array(joint_positions, dtype=np.float32)
            self._add_line(group, pts, (*_GHOST_SKELETON[:3], min(1.0, a * 2.2)),
                           width=2.0, mode="line_strip")

        # Joint sphere housings
        for i, joint in enumerate(robot.joints):
            if i + 1 >= len(transforms):
                break
            pos = joint_positions[i + 1]
            r = max(joint.visual.radius * 0.65, 0.020) * (0.80 if is_ghost else 1.0)
            self._add_sphere(group, pos, r, joint_c, ghost=is_ghost)

        # End-effector TCP
        ee_pos = joint_positions[-1]
        ee_r = 0.026 * (0.80 if is_ghost else 1.0)
        self._add_sphere(group, ee_pos, ee_r, ee_c, ghost=is_ghost)

    # ── Ghost solutions ───────────────────────────────────────────────────────

    def render_ghost_solutions(
        self,
        robot: RobotDefinition,
        solutions_transforms: list[list[np.ndarray]],
        opacity: float = 0.30,
    ) -> None:
        self.clear_ghost_solutions()
        for i, transforms in enumerate(solutions_transforms):
            self.render_robot(robot, transforms, opacity=opacity,
                              group=f"ghost_sol_{i}", is_ghost=True)
        self._ghost_count = len(solutions_transforms)

    def clear_ghost_solutions(self) -> None:
        for g in [k for k in self._items if k.startswith("ghost_sol_")]:
            self._remove_group(g)
        self._ghost_count = 0

    def set_ghost_solutions_visible(self, visible: bool) -> None:
        for g, items in self._items.items():
            if g.startswith("ghost_sol_"):
                for item in items:
                    item.setVisible(visible)

    # ── Target ────────────────────────────────────────────────────────────────

    def render_target(self, target: Target) -> None:
        self._remove_group("target")
        pos = np.array(target.pose.position, dtype=np.float64)

        # Glowing core sphere
        self._add_sphere("target", pos, 0.034, _TARGET_COLOR)

        # Three reticle rings (one per axis)
        ring_r = 0.070
        for normal in [np.array([0,0,1]), np.array([1,0,0]), np.array([0,1,0])]:
            pts = _ring_pts(pos, normal, ring_r, n=48)
            # Close the ring
            pts_closed = np.vstack([pts, pts[:1]])
            self._add_line("target", pts_closed, _TARGET_RING, width=1.8, mode="line_strip")

        # Crosshair spikes
        cl = 0.090
        for ax, col in [([1,0,0], _FRAME_X), ([0,1,0], _FRAME_Y), ([0,0,1], _FRAME_Z)]:
            ax = np.array(ax, dtype=np.float64)
            self._add_line("target",
                           [pos - ax*cl, pos + ax*cl],
                           (*col[:3], 0.92), width=2.5)

        # Pose triad
        if target.mode == "pose":
            rot = np.array(target.pose.rotation_matrix)
            for ci, col in enumerate([_FRAME_X, _FRAME_Y, _FRAME_Z]):
                axis = rot[:, ci]
                self._add_line("target", [pos, pos + axis * 0.13], col, width=3.5)

    # ── Trajectory ────────────────────────────────────────────────────────────

    def render_trajectory(
        self,
        transforms_sequence: list[list[np.ndarray]],
        group: str = "trajectory",
    ) -> None:
        self._remove_group(group)
        if len(transforms_sequence) < 2:
            return
        pts = np.array([t[-1][:3, 3] for t in transforms_sequence], dtype=np.float32)
        color = _TRAJ_COLOR if group == "trajectory" else _GHOST_SKELETON
        width = 3.5 if group == "trajectory" else 1.8
        self._add_line(group, pts, color, width=width, mode="line_strip")

    # ── Joint frames ──────────────────────────────────────────────────────────

    def render_joint_frames(self, transforms: list[np.ndarray]) -> None:
        self._remove_group("joint_frames")
        length = 0.09
        for tf in transforms[1:]:
            pos = tf[:3, 3]
            rot = tf[:3, :3]
            for ci, col in enumerate([_FRAME_X, _FRAME_Y, _FRAME_Z]):
                axis = rot[:, ci]
                self._add_line("joint_frames",
                               [pos, pos + axis * length],
                               (*col[:3], 0.85), width=2.2)

    # ── Workspace cloud ───────────────────────────────────────────────────────

    def render_workspace(self, points: np.ndarray) -> None:
        self._remove_group("workspace")
        if len(points) == 0:
            return
        color = np.tile(np.array(_WORKSPACE_COLOR, dtype=np.float32), (len(points), 1))
        item = gl.GLScatterPlotItem(pos=points.astype(np.float32),
                                   color=color, size=2.5, pxMode=True)
        self._add_item("workspace", item)

    # ── Visibility ────────────────────────────────────────────────────────────

    def set_group_visible(self, group: str, visible: bool) -> None:
        if group == "ghost_solutions":
            self.set_ghost_solutions_visible(visible)
            return
        for item in self._items.get(group, []):
            item.setVisible(visible)

    # ── Camera ────────────────────────────────────────────────────────────────

    def reset_camera(self) -> None:
        self._view.setCameraPosition(
            pos=pg.Vector(0.25, 0.0, 0.20),
            distance=1.8, elevation=28, azimuth=40,
        )

    def set_camera_preset(self, preset: str) -> None:
        presets = {
            "iso":   dict(elevation=28,  azimuth=40),
            "top":   dict(elevation=89,  azimuth=0),
            "front": dict(elevation=5,   azimuth=0),
            "side":  dict(elevation=5,   azimuth=90),
        }
        if preset in presets:
            self._view.setCameraPosition(**presets[preset])

    def fit_robot(self, transforms: list[np.ndarray]) -> None:
        if not transforms:
            return
        positions = np.array([t[:3, 3] for t in transforms])
        center = positions.mean(axis=0)
        span   = float(np.linalg.norm(positions - center, axis=1).max())
        dist   = max(span * 2.2, 1.2)
        self._view.setCameraPosition(
            pos=pg.Vector(float(center[0]), float(center[1]), float(center[2]) + 0.05),
            distance=dist, elevation=28, azimuth=40,
        )
