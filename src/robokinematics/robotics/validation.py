"""Candidate validation service.

Validates every IK candidate via FK verification, joint limits, position error,
and orientation error. Per spec §5, §6, §9 — never trust a solver flag alone.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from robokinematics.domain.models import IKCandidate, Pose, RobotDefinition, Target
from robokinematics.robotics.forward_kinematics import forward_kinematics


def _rotation_error_rad(r_actual: np.ndarray, r_target: np.ndarray) -> float:
    """Compute orientation error as rotation-vector magnitude (radians)."""
    # R_err = R_actual^T @ R_target
    r_err = r_actual.T @ r_target
    rot = Rotation.from_matrix(r_err)
    return float(np.linalg.norm(rot.as_rotvec()))


def validate_candidate(
    robot: RobotDefinition,
    target: Target,
    candidate: IKCandidate,
) -> IKCandidate:
    """Run all validity checks and return an updated IKCandidate.

    Checks:
    1. All joint values must be finite.
    2. All joint values must be within limits.
    3. FK of the candidate must produce a finite pose.
    4. Position error must be <= target.position_tolerance.
    5. Orientation error (pose mode only) <= target.orientation_tolerance.
    """
    joint_positions = candidate.joint_positions

    # 1. Finiteness
    if not all(np.isfinite(q) for q in joint_positions):
        return candidate.model_copy(
            update={"valid": False, "solver_status": "INVALID_NON_FINITE"}
        )

    # 2. Joint limits
    for joint, q in zip(robot.joints, joint_positions, strict=True):
        if q < joint.limits.position_min or q > joint.limits.position_max:
            return candidate.model_copy(
                update={"valid": False, "solver_status": "INVALID_JOINT_LIMIT"}
            )

    # 3. FK
    try:
        transforms = forward_kinematics(robot, joint_positions)
    except Exception:
        return candidate.model_copy(
            update={"valid": False, "solver_status": "FK_ERROR"}
        )

    ee_transform = transforms[-1]
    actual_position = ee_transform[:3, 3]
    actual_rotation = ee_transform[:3, :3]

    if not np.all(np.isfinite(ee_transform)):
        return candidate.model_copy(
            update={"valid": False, "solver_status": "FK_NON_FINITE"}
        )

    achieved_pose = Pose(
        position=actual_position.tolist(),
        rotation_matrix=actual_rotation.tolist(),
    )

    # 4. Position error
    target_position = np.array(target.pose.position)
    pos_error = float(np.linalg.norm(actual_position - target_position))

    if pos_error > target.position_tolerance:
        return candidate.model_copy(
            update={
                "valid": False,
                "solver_status": "POSITION_TOLERANCE_VIOLATED",
                "achieved_pose": achieved_pose,
                "position_error": pos_error,
            }
        )

    # 5. Orientation error (pose mode only)
    orient_error: float | None = None
    if target.mode == "pose":
        target_rotation = np.array(target.pose.rotation_matrix)
        orient_error = _rotation_error_rad(actual_rotation, target_rotation)
        if orient_error > target.orientation_tolerance:
            return candidate.model_copy(
                update={
                    "valid": False,
                    "solver_status": "ORIENTATION_TOLERANCE_VIOLATED",
                    "achieved_pose": achieved_pose,
                    "position_error": pos_error,
                    "orientation_error": orient_error,
                }
            )

    return candidate.model_copy(
        update={
            "valid": True,
            "solver_status": "VALID",
            "achieved_pose": achieved_pose,
            "position_error": pos_error,
            "orientation_error": orient_error,
        }
    )
