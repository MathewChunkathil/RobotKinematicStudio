"""Workspace sampler.

Implements Phase 9 per 02_ROBOTICS_SPEC.md §13.

Samples valid joint configurations within limits, runs FK on each,
collects end-effector positions. Result is an approximation — not an exact boundary.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np

from robokinematics.domain.models import RobotDefinition
from robokinematics.robotics.forward_kinematics import forward_kinematics


@dataclass
class WorkspaceResult:
    """Sampled workspace result."""
    points: np.ndarray           # (N, 3) array of end-effector positions
    bounding_box_min: np.ndarray # (3,)
    bounding_box_max: np.ndarray # (3,)
    max_reach: float
    sample_count: int
    valid_count: int
    timestamp: str


def sample_workspace(
    robot: RobotDefinition,
    n_samples: int = 5_000,
    random_seed: int = 0,
) -> WorkspaceResult:
    """Sample the workspace by drawing random joint configurations.

    Respects joint limits. Discards FK results containing NaN/Inf.
    """
    rng = np.random.default_rng(random_seed)
    lbs = np.array([j.limits.position_min for j in robot.joints])
    ubs = np.array([j.limits.position_max for j in robot.joints])

    points: list[np.ndarray] = []
    valid = 0

    for _ in range(n_samples):
        q = rng.uniform(lbs, ubs)
        try:
            transforms = forward_kinematics(robot, q.tolist())
        except Exception:
            continue
        p = transforms[-1][:3, 3]
        if not np.all(np.isfinite(p)):
            continue
        points.append(p)
        valid += 1

    if not points:
        empty = np.zeros((0, 3))
        return WorkspaceResult(
            points=empty,
            bounding_box_min=np.zeros(3),
            bounding_box_max=np.zeros(3),
            max_reach=0.0,
            sample_count=n_samples,
            valid_count=0,
            timestamp=datetime.now(timezone.utc).isoformat(),
        )

    pts = np.array(points)
    bb_min = pts.min(axis=0)
    bb_max = pts.max(axis=0)
    norms = np.linalg.norm(pts, axis=1)
    max_reach = float(norms.max())

    return WorkspaceResult(
        points=pts,
        bounding_box_min=bb_min,
        bounding_box_max=bb_max,
        max_reach=max_reach,
        sample_count=n_samples,
        valid_count=valid,
        timestamp=datetime.now(timezone.utc).isoformat(),
    )
