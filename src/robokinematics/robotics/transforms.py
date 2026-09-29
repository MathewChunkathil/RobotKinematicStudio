"""Homogeneous transforms and standard DH mathematics."""

from __future__ import annotations

import numpy as np


def standard_dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Return the standard DH homogeneous transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def compose(transforms: list[np.ndarray]) -> np.ndarray:
    """Multiply a sequence of homogeneous transforms."""
    result = np.eye(4, dtype=float)
    for transform in transforms:
        result = result @ transform
    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid homogeneous transform."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse
