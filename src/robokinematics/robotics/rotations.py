"""Rotation utilities."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll, pitch, yaw (in radians) to a 3x3 rotation matrix.
    
    Uses intrinsic XYZ convention (often used in robotics for RPY,
    where R=X, P=Y, Y=Z).
    """
    # SciPy uses intrinsic rotations if we specify 'xyz' (lowercase for extrinsic, uppercase for intrinsic? No, lowercase is extrinsic, uppercase is intrinsic).
    # Wait, RPY is typically extrinsic xyz or intrinsic ZYX. Let's use intrinsic XYZ (roll about X, then pitch about new Y, then yaw about new Z).
    r = Rotation.from_euler('XYZ', [roll, pitch, yaw], degrees=False)
    return r.as_matrix()


def matrix_to_rpy(matrix: list[list[float]] | np.ndarray) -> tuple[float, float, float]:
    """Convert a 3x3 rotation matrix to roll, pitch, yaw (in radians)."""
    r = Rotation.from_matrix(matrix)
    euler = r.as_euler('XYZ', degrees=False)
    return tuple(euler)
