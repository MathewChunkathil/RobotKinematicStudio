"""3D Scene renderer — GPU-accelerated via PyQtGraph OpenGL.

Ultra-realistic industrial robotic arm visualization:
  - Multi-tier cast-iron base mounting pedestal with turntable collar
  - Robust structural link castings with chamfered geometry & metallic cuffs
  - Cylindrical actuator housing pods (harmonic-drive servo pods) aligned with joint axes
  - Industrial 2-finger parallel servo gripper with Tool Center Point (TCP)
  - Workcell table: elevated, semi-translucent tempered acrylic/slate work surface with
    etched millimeter reference grid and corner support legs
  - Deep workcell volume: objects/targets can be placed ON the table OR UNDER the table
    (negative Z), with clear through-surface visibility & lower floor reference grid
  - Target workpiece billet: machined metallic stock object at the target position
  - Translucent holographic ghost-robot overlays for all IK branch alternatives
  - Trajectory ribbons, joint coordinate triads, and workspace point-clouds
"""

from __future__ import annotations

import numpy as np
from PySide6.QtWidgets import QWidget, QVBoxLayout
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget, MeshData

from robokinematics.domain.models import RobotDefinition, Target

# ── Palette (Industrial Robotics & High-Tech Workcell) ─────────────────────────
_LINK_BODY       = (0.18, 0.23, 0.30, 1.00)   # Deep titanium / slate-metallic
_LINK_ACCENT     = (0.98, 0.55, 0.10, 1.00)   # Industrial safety orange accent
_JOINT_POD       = (0.24, 0.29, 0.37, 1.00)   # Actuator housing drum
_JOINT_FACE      = (0.68, 0.73, 0.82, 1.00)   # Machined aluminum faceplates
_BASE_PEDESTAL   = (0.13, 0.17, 0.22, 1.00)   # Cast-iron pedestal plinth
_BASE_COLLAR     = (0.45, 0.52, 0.62, 1.00)   # Turntable bearing ring

_GRIPPER_BODY    = (0.12, 0.15, 0.20, 1.00)   # Dark graphite gripper chassis
_GRIPPER_JAW     = (0.75, 0.80, 0.88, 1.00)   # Polished steel gripper fingers
_GRIPPER_PAD     = (0.98, 0.72, 0.12, 1.00)   # High-friction tactile fingertip pads
_EE_TCP          = (0.10, 0.98, 0.70, 1.00)   # Glowing cyan/green TCP glyph

_TARGET_STOCK    = (0.95, 0.76, 0.18, 1.00)   # Machined brass workpiece stock
_TARGET_RING     = (1.00, 0.35, 0.22, 0.80)   # Target reticle alignment rings
_TARGET_SPIKE    = (1.00, 0.25, 0.18, 0.90)   # Target crosshair spikes
_TRAJ_COLOR      = (0.10, 0.96, 0.65, 1.00)   # Glowing trajectory path ribbon

_GHOST_LINK      = (0.15, 0.72, 1.00, 0.24)   # Holographic ghost link shell
_GHOST_JOINT     = (0.35, 0.82, 1.00, 0.32)   # Holographic actuator pod
_GHOST_GRIPPER   = (0.40, 0.90, 1.00, 0.35)   # Holographic gripper
_GHOST_SKELETON  = (0.45, 0.90, 1.00, 0.65)   # Luminescent centerline trace

_TABLE_SURFACE   = (0.07, 0.10, 0.15, 0.72)   # Semi-translucent dark tempered acrylic
_TABLE_RIM       = (0.30, 0.38, 0.48, 0.90)   # Brushed aluminum perimeter trim
_TABLE_LEG       = (0.14, 0.18, 0.24, 0.95)   # Structural tubular legs
_TABLE_GRID      = (45, 65, 85, 120)          # Etched millimeter grid lines
_FLOOR_GRID      = (30, 40, 52, 90)           # Lower floor reference grid (-600 mm)

_FRAME_X         = (0.96, 0.22, 0.22, 1.00)
_FRAME_Y         = (0.22, 0.90, 0.28, 1.00)
_FRAME_Z         = (0.22, 0.55, 1.00, 1.00)
_WORKSPACE_COLOR = (0.20, 0.70, 0.95, 0.20)


# ── Geometry & Mesh Builders ──────────────────────────────────────────────────

def _sphere_mesh(center, radius: float = 0.025, rows: int = 14, cols: int = 14):
    """Smooth-shaded UV sphere."""
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
    u = np.cross(axis, ref); u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    th = np.linspace(0, 2 * np.pi, segs, endpoint=False)
    circle = np.cos(th)[:, None] * u + np.sin(th)[:, None] * v
    bot = np.asarray(start)[None] + radius * circle
    top = bot + d
    c_bot = np.asarray(start, dtype=np.float64)[None]
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


def _oriented_cylinder_mesh(center, axis, radius: float, length: float, segs: int = 18):
    """Cylinder centered at `center` oriented along `axis`."""
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-6:
        axis = np.array([0.0, 0.0, 1.0])
    else:
        axis = axis / norm
    p0 = np.asarray(center, dtype=np.float64) - axis * (length / 2.0)
    p1 = np.asarray(center, dtype=np.float64) + axis * (length / 2.0)
    return _cylinder_mesh(p0, p1, radius, segs)


def _oriented_box_mesh(center, rot_matrix, size_xyz):
    """Oriented 3D box with dimensions `size_xyz` transformed by `rot_matrix`."""
    sx, sy, sz = [s / 2.0 for s in size_xyz]
    local_verts = np.array([
        [-sx, -sy, -sz], [ sx, -sy, -sz], [ sx,  sy, -sz], [-sx,  sy, -sz],
        [-sx, -sy,  sz], [ sx, -sy,  sz], [ sx,  sy,  sz], [-sx,  sy,  sz],
    ], dtype=np.float32)
    R = np.asarray(rot_matrix, dtype=np.float32)
    c = np.asarray(center, dtype=np.float32)
    world_verts = (local_verts @ R.T) + c
    faces = np.array([
        [0,1,2],[0,2,3],[4,5,6],[4,6,7],
        [0,1,5],[0,5,4],[2,3,7],[2,7,6],
        [0,3,7],[0,7,4],[1,2,6],[1,6,5],
    ], dtype=np.int32)
    return world_verts, faces


def _ring_pts(center, normal, radius: float, n: int = 40) -> np.ndarray:
    """Points for a circular ring in 3D."""
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    ref = np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(normal, ref); u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    th = np.linspace(0, 2 * np.pi, n)
    return (np.asarray(center)[None]
            + radius * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v)).astype(np.float32)


# ── Scene Widget ──────────────────────────────────────────────────────────────

class RoboKinematicsScene(QWidget):
    """GPU-accelerated 3D viewport (RTX 4070 ready) with realistic arm & workcell."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._items: dict[str, list] = {}
        self._ghost_count: int = 0
        self._build_ui()
        self._add_workcell_environment()
        self._add_world_frame()

    def _build_ui(self) -> None:
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        pg.setConfigOption("background", "#090d12")
        self._view = GLViewWidget()
        self._view.setBackgroundColor((9, 13, 18, 255))
        self._view.setCameraPosition(distance=2.1, elevation=26, azimuth=38)
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

    def _mesh_item(self, verts, faces, color, ghost: bool = False, shader: str = "shaded") -> gl.GLMeshItem:
        c = np.asarray(color, dtype=np.float32)
        opts = "translucent" if c[3] < 0.99 else "opaque"
        return gl.GLMeshItem(
            meshdata=MeshData(vertexes=verts, faces=faces),
            color=c,
            smooth=True,
            shader=shader,
            drawEdges=ghost,
            edgeColor=(*c[:3].tolist(), 0.40) if ghost else (0, 0, 0, 0.16),
            glOptions=opts,
        )

    def _add_sphere(self, group: str, pos, radius: float, color, ghost: bool = False) -> None:
        v, f = _sphere_mesh(pos, radius)
        self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_cylinder(self, group: str, p0, p1, radius: float, color, ghost: bool = False) -> None:
        v, f = _cylinder_mesh(p0, p1, radius)
        if v is not None:
            self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_oriented_cylinder(self, group: str, center, axis, radius: float, length: float, color, ghost: bool = False) -> None:
        v, f = _oriented_cylinder_mesh(center, axis, radius, length)
        if v is not None:
            self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_box(self, group: str, center, rot_matrix, size_xyz, color, ghost: bool = False) -> None:
        v, f = _oriented_box_mesh(center, rot_matrix, size_xyz)
        self._add_item(group, self._mesh_item(v, f, color, ghost))

    def _add_line(self, group: str, pts, color, width: float = 2.0, mode: str = "lines") -> None:
        item = gl.GLLinePlotItem(
            pos=np.asarray(pts, dtype=np.float32),
            color=color, width=width, antialias=True, mode=mode,
        )
        self._add_item(group, item)

    # ── Workcell Environment & Table Surface ──────────────────────────────────

    def _add_workcell_environment(self) -> None:
        """Create elevated workcell table at Z=0 and lower floor grid at Z=-600mm.

        Objects placed at negative Z sit in the under-table zone, visible through
        the semi-translucent dark tempered acrylic table surface.
        """
        group = "workcell"

        # 1. Main workcell table plate (Z = 0.0, 1.8m x 1.8m, thickness = 20mm)
        tw, td, th = 1.80, 1.80, 0.020
        table_top_center = [0.0, 0.0, -th / 2.0]
        self._add_box(group, table_top_center, np.eye(3), (tw, td, th), _TABLE_SURFACE)

        # 2. Brushed metallic perimeter rim / chamfer border
        rim_t = 0.025
        rim_h = 0.024
        # North & South trim
        self._add_box(group, [0.0,  td/2.0 + rim_t/2.0, -th/2.0], np.eye(3), (tw + 2*rim_t, rim_t, rim_h), _TABLE_RIM)
        self._add_box(group, [0.0, -td/2.0 - rim_t/2.0, -th/2.0], np.eye(3), (tw + 2*rim_t, rim_t, rim_h), _TABLE_RIM)
        # East & West trim
        self._add_box(group, [ tw/2.0 + rim_t/2.0, 0.0, -th/2.0], np.eye(3), (rim_t, td, rim_h), _TABLE_RIM)
        self._add_box(group, [-tw/2.0 - rim_t/2.0, 0.0, -th/2.0], np.eye(3), (rim_t, td, rim_h), _TABLE_RIM)

        # 3. Structural corner support legs (reaching down to floor at Z = -0.60m)
        floor_z = -0.60
        leg_len = abs(floor_z) - th
        leg_z   = -th - leg_len / 2.0
        leg_offset = 0.82
        for lx in (-leg_offset, leg_offset):
            for ly in (-leg_offset, leg_offset):
                self._add_box(group, [lx, ly, leg_z], np.eye(3), (0.055, 0.055, leg_len), _TABLE_LEG)

        # 4. Etched millimeter grid on table surface (at Z = +0.001m)
        table_grid = gl.GLGridItem()
        table_grid.setSize(x=1.8, y=1.8)
        table_grid.setSpacing(x=0.10, y=0.10)   # 100 mm grid
        table_grid.setColor(_TABLE_GRID)
        table_grid.translate(0, 0, 0.001)
        self._view.addItem(table_grid)
        self._items.setdefault(group, []).append(table_grid)

        # 5. Lower ground floor grid at Z = -0.60m (-600 mm)
        floor_grid = gl.GLGridItem()
        floor_grid.setSize(x=3.6, y=3.6)
        floor_grid.setSpacing(x=0.20, y=0.20)  # 200 mm grid
        floor_grid.setColor(_FLOOR_GRID)
        floor_grid.translate(0, 0, floor_z)
        self._view.addItem(floor_grid)
        self._items.setdefault(group, []).append(floor_grid)

    def _add_world_frame(self) -> None:
        for d, c in [
            (np.array([1,0,0]), _FRAME_X),
            (np.array([0,1,0]), _FRAME_Y),
            (np.array([0,0,1]), _FRAME_Z),
        ]:
            pts = np.array([[0,0,0.002], d * 0.16 + [0,0,0.002]], dtype=np.float32)
            item = gl.GLLinePlotItem(pos=pts, color=c, width=3.5, antialias=True)
            self._view.addItem(item)
            self._items.setdefault("world_frame", []).append(item)

    # ── Public Robot Rendering API ────────────────────────────────────────────

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
            body_c    = (*_GHOST_LINK[:3],     a * 0.70)
            accent_c  = (*_GHOST_JOINT[:3],    a * 0.90)
            pod_c     = (*_GHOST_JOINT[:3],    a * 0.85)
            face_c    = (*_GHOST_JOINT[:3],    a * 0.75)
            gripper_c = (*_GHOST_GRIPPER[:3],  a * 0.95)
            base_c    = (*_BASE_PEDESTAL[:3],  a * 0.30)
            tcp_c     = (0.28, 1.00, 0.80,     a * 1.10)
        else:
            body_c    = (*_LINK_BODY[:3],      a)
            accent_c  = (*_LINK_ACCENT[:3],    a)
            pod_c     = (*_JOINT_POD[:3],      a)
            face_c    = (*_JOINT_FACE[:3],     a)
            gripper_c = (*_GRIPPER_BODY[:3],   a)
            base_c    = (*_BASE_PEDESTAL[:3],  a)
            tcp_c     = (*_EE_TCP[:3],         a)

        joint_positions = [np.array(t[:3, 3]) for t in transforms]
        n_joints = len(robot.joints)

        # ── 1. Base Mounting Pedestal (Industrial turntable plinth) ───────────
        if not is_ghost:
            p_base = joint_positions[0]
            # Bottom mounting flange disk (radius 135 mm, height 18 mm)
            self._add_oriented_cylinder(group, p_base + [0, 0, 0.009], [0, 0, 1],
                                        radius=0.135, length=0.018, color=base_c)
            # Turntable bearing collar (radius 95 mm, height 22 mm)
            self._add_oriented_cylinder(group, p_base + [0, 0, 0.028], [0, 0, 1],
                                        radius=0.095, length=0.020, color=_BASE_COLLAR)
            # Rotating shoulder turret cylinder
            self._add_oriented_cylinder(group, p_base + [0, 0, 0.046], [0, 0, 1],
                                        radius=0.070, length=0.016, color=body_c)

        # ── 2. Structural Links (Realistic industrial castings with collars) ─
        for i in range(len(joint_positions) - 1):
            p0, p1 = joint_positions[i], joint_positions[i + 1]
            vec = p1 - p0
            seg_len = float(np.linalg.norm(vec))
            if seg_len < 1e-4:
                continue
            uvec = vec / seg_len

            # Radius tapers realistically down the kinematic chain
            if i == 0:
                base_r = 0.046
            elif i == 1:
                base_r = 0.040
            elif i == 2:
                base_r = 0.034
            else:
                base_r = max(0.022, 0.030 - (i - 3) * 0.004)
            r = base_r * (0.80 if is_ghost else 1.0)

            # Main structural link body
            self._add_cylinder(group, p0, p1, r, body_c, ghost=is_ghost)

            if not is_ghost and seg_len > 0.06:
                # Joint cuff rings (metallic/accent collars near joints)
                cuff_r = r * 1.15
                cuff_l = min(0.022, seg_len * 0.18)
                self._add_oriented_cylinder(group, p0 + uvec * (cuff_l / 2.0 + 0.005),
                                            uvec, radius=cuff_r, length=cuff_l, color=accent_c)
                self._add_oriented_cylinder(group, p1 - uvec * (cuff_l / 2.0 + 0.005),
                                            uvec, radius=cuff_r, length=cuff_l, color=accent_c)

        # Ghost centerline wireframe trace
        if is_ghost and len(joint_positions) >= 2:
            pts = np.array(joint_positions, dtype=np.float32)
            self._add_line(group, pts, (*_GHOST_SKELETON[:3], min(1.0, a * 2.2)),
                           width=2.2, mode="line_strip")

        # ── 3. Actuator Housing Pods (Cylindrical harmonic drive pods) ────────
        for i, joint in enumerate(robot.joints):
            if i + 1 >= len(transforms):
                break
            pos = joint_positions[i + 1]
            tf  = transforms[i + 1]

            # Rotation axis of the joint
            rot_axis = tf[:3, 2] if tf.shape == (4, 4) else np.array([0, 0, 1])
            pod_r = max(0.032, 0.045 - i * 0.003) * (0.80 if is_ghost else 1.0)
            pod_l = max(0.055, 0.076 - i * 0.004)

            # Actuator drum cylinder along rotation axis
            self._add_oriented_cylinder(group, pos, rot_axis, radius=pod_r, length=pod_l,
                                        color=pod_c, ghost=is_ghost)

            # Contrasting metallic end-caps on both faces of the actuator
            if not is_ghost:
                cap_offset = rot_axis * (pod_l / 2.0)
                cap_r = pod_r * 0.88
                self._add_sphere(group, pos + cap_offset, radius=cap_r * 0.45, color=face_c)
                self._add_sphere(group, pos - cap_offset, radius=cap_r * 0.45, color=face_c)

        # ── 4. Industrial Parallel 2-Finger Gripper at End-Effector ───────────
        ee_tf = transforms[-1]
        ee_pos = ee_tf[:3, 3]
        ee_rot = ee_tf[:3, :3]
        z_ee   = ee_rot[:, 2]   # Tool approach axis (forward)
        x_ee   = ee_rot[:, 0]   # Lateral jaw spread axis
        y_ee   = ee_rot[:, 1]   # Transverse jaw width axis

        # Tool mounting flange disk
        self._add_oriented_cylinder(group, ee_pos + z_ee * 0.006, z_ee,
                                    radius=0.026 * (0.8 if is_ghost else 1.0),
                                    length=0.012, color=_BASE_COLLAR if not is_ghost else body_c,
                                    ghost=is_ghost)

        # Gripper body housing block
        body_center = ee_pos + z_ee * 0.024
        self._add_box(group, body_center, ee_rot, (0.056, 0.034, 0.024), gripper_c, ghost=is_ghost)

        # Left and Right parallel gripper fingers
        jaw_len = 0.032
        jaw_center_z = ee_pos + z_ee * (0.024 + 0.012 + jaw_len / 2.0)
        spread = 0.018

        # Left jaw
        left_pos = jaw_center_z - x_ee * spread
        self._add_box(group, left_pos, ee_rot, (0.008, 0.014, jaw_len),
                      _GRIPPER_JAW if not is_ghost else body_c, ghost=is_ghost)

        # Right jaw
        right_pos = jaw_center_z + x_ee * spread
        self._add_box(group, right_pos, ee_rot, (0.008, 0.014, jaw_len),
                      _GRIPPER_JAW if not is_ghost else body_c, ghost=is_ghost)

        # High-visibility tactile fingertip pads
        if not is_ghost:
            pad_z = ee_pos + z_ee * (0.024 + 0.012 + jaw_len - 0.006)
            self._add_box(group, pad_z - x_ee * (spread - 0.004), ee_rot, (0.003, 0.012, 0.012), _GRIPPER_PAD)
            self._add_box(group, pad_z + x_ee * (spread - 0.004), ee_rot, (0.003, 0.012, 0.012), _GRIPPER_PAD)

        # Glowing Tool Center Point (TCP) glyph at center of grasp
        tcp_pos = ee_pos + z_ee * (0.024 + 0.012 + jaw_len - 0.004)
        self._add_sphere(group, tcp_pos, radius=0.012 * (0.8 if is_ghost else 1.0),
                         color=tcp_c, ghost=is_ghost)

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
        for g in [k for k in list(self._items.keys()) if k.startswith("ghost_sol_")]:
            self._remove_group(g)
        self._ghost_count = 0

    def set_ghost_solutions_visible(self, visible: bool) -> None:
        for g, items in self._items.items():
            if g.startswith("ghost_sol_"):
                for item in items:
                    item.setVisible(visible)

    # ── Target Object & Workpiece ─────────────────────────────────────────────

    def render_target(self, target: Target) -> None:
        self._remove_group("target")
        pos = np.array(target.pose.position, dtype=np.float64)

        # 1. Physical workpiece stock to reach/grasp (machined brass billet)
        workpiece_h = 0.038
        workpiece_r = 0.024
        self._add_oriented_cylinder("target", pos, [0, 0, 1],
                                    radius=workpiece_r, length=workpiece_h,
                                    color=_TARGET_STOCK)

        # 2. Glowing alignment reticle rings around the workpiece
        ring_r = 0.075
        for normal in [np.array([0,0,1]), np.array([1,0,0]), np.array([0,1,0])]:
            pts = _ring_pts(pos, normal, ring_r, n=48)
            pts_closed = np.vstack([pts, pts[:1]])
            self._add_line("target", pts_closed, _TARGET_RING, width=1.8, mode="line_strip")

        # 3. Crosshair spikes
        cl = 0.095
        for ax, col in [([1,0,0], _FRAME_X), ([0,1,0], _FRAME_Y), ([0,0,1], _FRAME_Z)]:
            ax = np.array(ax, dtype=np.float64)
            self._add_line("target", [pos - ax * cl, pos + ax * cl],
                           (*col[:3], 0.90), width=2.4)

        # 4. Pose orientation triad (if mode is pose)
        if target.mode == "pose":
            rot = np.array(target.pose.rotation_matrix)
            for ci, col in enumerate([_FRAME_X, _FRAME_Y, _FRAME_Z]):
                axis = rot[:, ci]
                self._add_line("target", [pos, pos + axis * 0.14], col, width=3.5)

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
            pos=pg.Vector(0.20, 0.0, 0.15),
            distance=2.1, elevation=26, azimuth=38,
        )

    def set_camera_preset(self, preset: str) -> None:
        presets = {
            "iso":   dict(elevation=26,  azimuth=38),
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
        dist   = max(span * 2.3, 1.4)
        self._view.setCameraPosition(
            pos=pg.Vector(float(center[0]), float(center[1]), float(center[2]) + 0.04),
            distance=dist, elevation=26, azimuth=38,
        )
