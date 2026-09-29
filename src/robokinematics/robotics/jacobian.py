"""Jacobian, singularity analysis, and manipulability.

Implements Phase 7 per 02_ROBOTICS_SPEC.md §10-12.

Geometric Jacobian:
  - Revolute joint i: J_v = z_{i-1} × (p_end - p_{i-1}),  J_w = z_{i-1}
  - Prismatic joint i: J_v = z_{i-1},  J_w = 0
  
z_{i-1} is the third column of the (i-1)-th frame transform from FK.
p_{i-1} is the translation of the (i-1)-th frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from robokinematics.domain.models import RobotDefinition
from robokinematics.robotics.forward_kinematics import forward_kinematics


# ── Thresholds (named, not scattered magic numbers) ───────────────────────────

SINGULARITY_NEAR_THRESHOLD = 0.05   # min singular value below this → NEAR_SINGULAR
SINGULARITY_SINGULAR_THRESHOLD = 1e-6  # min singular value below this → SINGULAR


class SingularityStatus(str, Enum):
    SAFE = "SAFE"
    NEAR_SINGULAR = "NEAR_SINGULAR"
    SINGULAR = "SINGULAR"


@dataclass(frozen=True)
class JacobianAnalysis:
    jacobian: np.ndarray          # 6×N
    singular_values: np.ndarray   # min(6,N) values
    rank: int
    condition_number: float | None
    min_singular_value: float
    singularity_status: SingularityStatus
    manipulability: float | None   # sqrt(det(J J^T)) — None if not applicable


# ── Geometric Jacobian ────────────────────────────────────────────────────────

def geometric_jacobian(
    robot: RobotDefinition,
    joint_positions: list[float],
) -> np.ndarray:
    """Compute the 6×N geometric Jacobian.

    Frame index convention with our updated FK:
      transforms[0]  = base frame
      transforms[1]  = after joint 0
      ...
      transforms[N]  = after joint N-1
      transforms[N+1]= end-effector frame

    The z-axis for joint i comes from transforms[i] (the frame *before* joint i).
    The end-effector position comes from transforms[N+1].
    """
    transforms = forward_kinematics(robot, joint_positions)
    end_position = transforms[-1][:3, 3]  # end-effector position
    n = len(robot.joints)
    jacobian = np.zeros((6, n), dtype=float)

    for i, joint in enumerate(robot.joints):
        # Frame before joint i = transforms[i] (base=0, joint_0_frame=1, ...)
        frame = transforms[i]
        z_axis = frame[:3, 2]   # third column = z-axis of frame
        origin = frame[:3, 3]   # translation = frame origin

        if joint.type.value == "revolute":
            jacobian[:3, i] = np.cross(z_axis, end_position - origin)
            jacobian[3:, i] = z_axis
        else:
            jacobian[:3, i] = z_axis
            jacobian[3:, i] = 0.0

    return jacobian


# ── Singularity & manipulability ──────────────────────────────────────────────

def analyse_jacobian(
    robot: RobotDefinition,
    joint_positions: list[float],
) -> JacobianAnalysis:
    """Compute the Jacobian and derived singularity/manipulability metrics."""
    J = geometric_jacobian(robot, joint_positions)
    svd_values = np.linalg.svd(J, compute_uv=False)
    rank = int(np.linalg.matrix_rank(J))
    min_sv = float(svd_values[-1])
    max_sv = float(svd_values[0])

    condition_number: float | None = None
    if min_sv > 1e-12:
        condition_number = max_sv / min_sv

    # Singularity classification (application policy — see spec §11)
    if min_sv < SINGULARITY_SINGULAR_THRESHOLD:
        status = SingularityStatus.SINGULAR
    elif min_sv < SINGULARITY_NEAR_THRESHOLD:
        status = SingularityStatus.NEAR_SINGULAR
    else:
        status = SingularityStatus.SAFE

    # Manipulability: sqrt(det(J J^T)) — only meaningful when J is 6×N with N≥6
    # or for smaller tasks. We compute on J_pos (3×N) for position manipulability.
    J_pos = J[:3, :]  # positional sub-Jacobian
    manip: float | None = None
    try:
        JJT = J_pos @ J_pos.T
        det_val = float(np.linalg.det(JJT))
        # Clamp tiny negative values caused by floating-point noise at/near singularities.
        manip = float(np.sqrt(max(0.0, det_val)))
    except Exception:
        manip = None

    return JacobianAnalysis(
        jacobian=J,
        singular_values=svd_values,
        rank=rank,
        condition_number=condition_number,
        min_singular_value=min_sv,
        singularity_status=status,
        manipulability=manip,
    )
