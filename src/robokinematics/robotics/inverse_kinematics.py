"""Numerical multi-start IK solver.

Implements Phase 5 (IK), Phase 6 (validation), per 02_ROBOTICS_SPEC.md §7-9.

Strategy:
  1. Generate `settings.starts` random initial guesses within joint limits.
  2. Solve from each guess using scipy.optimize.least_squares (damped LS / LM).
  3. Normalize revolute angles to [-pi, pi].
  4. Reject non-finite / joint-limit-violating candidates.
  5. Deduplicate (joint-configuration distance < deduplication_tolerance).
  6. FK-verify every surviving candidate via validation.validate_candidate.
"""

from __future__ import annotations

import uuid

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from robokinematics.domain.models import IKCandidate, JointType, RobotDefinition, RobotState, Target
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.robotics.validation import validate_candidate


# ── Settings ────────────────────────────────────────────────────────────────

from dataclasses import dataclass


@dataclass(frozen=True)
class IKSettings:
    """Configurable IK solver settings. All tolerances in SI units."""

    max_iterations: int = 500
    position_tolerance: float = 1e-4       # metres
    orientation_tolerance: float = 1e-3    # radians
    deduplication_tolerance: float = 1e-3  # radians / metres
    random_seed: int = 42
    starts: int = 32


# ── Helpers ──────────────────────────────────────────────────────────────────

def _normalize_revolute(q: float) -> float:
    """Wrap a revolute angle into [-pi, pi]."""
    return float((q + np.pi) % (2 * np.pi) - np.pi)


def _residual(
    q: np.ndarray,
    robot: RobotDefinition,
    target: Target,
    mode: str,
) -> np.ndarray:
    """Residual vector for the optimizer. Returns 3 or 6 values."""
    try:
        transforms = forward_kinematics(robot, q.tolist())
    except Exception:
        # Return large residual so optimizer backs off
        return np.ones(6 if mode == "pose" else 3) * 1e6

    ee = transforms[-1]
    actual_pos = ee[:3, 3]
    target_pos = np.array(target.pose.position)
    pos_res = actual_pos - target_pos

    if mode == "pose":
        actual_rot = ee[:3, :3]
        target_rot = np.array(target.pose.rotation_matrix)
        r_err = actual_rot.T @ target_rot
        rotvec = Rotation.from_matrix(r_err).as_rotvec()
        return np.concatenate([pos_res, rotvec])

    return pos_res


def _within_limits(q: np.ndarray, robot: RobotDefinition) -> bool:
    for joint, qi in zip(robot.joints, q):
        if qi < joint.limits.position_min or qi > joint.limits.position_max:
            return False
    return True


def _random_q(robot: RobotDefinition, rng: np.random.Generator) -> np.ndarray:
    return np.array([
        rng.uniform(j.limits.position_min, j.limits.position_max)
        for j in robot.joints
    ])


def _deduplicate(
    candidates: list[np.ndarray],
    robot: RobotDefinition,
    tol: float,
) -> list[np.ndarray]:
    """Remove joint configurations that are within `tol` of an already accepted one."""
    accepted: list[np.ndarray] = []
    for q in candidates:
        for a in accepted:
            dists = []
            for joint, qi, ai in zip(robot.joints, q, a):
                if joint.type == JointType.REVOLUTE:
                    d = abs(_normalize_revolute(float(qi) - float(ai)))
                else:
                    d = abs(float(qi) - float(ai))
                dists.append(d)
            if max(dists) < tol:
                break
        else:
            accepted.append(q)
    return accepted


# ── Public solver ─────────────────────────────────────────────────────────────

def solve_inverse_kinematics(
    robot: RobotDefinition,
    target: Target,
    current_state: RobotState,
    settings: IKSettings | None = None,
) -> list[IKCandidate]:
    """Solve IK via numerical multi-start optimization.

    Returns only FK-verified feasible candidates. Never fabricates solutions.
    """
    if settings is None:
        settings = IKSettings()

    rng = np.random.default_rng(settings.random_seed)
    mode = target.mode

    # Build initial guesses: current state + (starts-1) random
    current_q = np.array(current_state.joint_positions, dtype=float)
    guesses: list[np.ndarray] = [current_q]
    for _ in range(settings.starts - 1):
        guesses.append(_random_q(robot, rng))

    # Lower / upper bounds for the optimizer
    lb = np.array([j.limits.position_min for j in robot.joints])
    ub = np.array([j.limits.position_max for j in robot.joints])

    raw_solutions: list[np.ndarray] = []

    for q0 in guesses:
        q0 = np.clip(q0, lb, ub)
        try:
            result = least_squares(
                _residual,
                q0,
                bounds=(lb, ub),
                method="trf",
                max_nfev=settings.max_iterations * len(robot.joints),
                args=(robot, target, mode),
            )
        except Exception:
            continue

        if not result.success and result.cost > 1e-3:
            # Only keep if residual is small enough to be promising
            if np.sqrt(result.cost * 2) > settings.position_tolerance * 100:
                continue

        q_sol = result.x.copy()

        # Normalize revolute joints
        for i, joint in enumerate(robot.joints):
            if joint.type == JointType.REVOLUTE:
                q_sol[i] = _normalize_revolute(q_sol[i])

        if not np.all(np.isfinite(q_sol)):
            continue

        if not _within_limits(q_sol, robot):
            continue

        raw_solutions.append(q_sol)

    # Deduplicate
    unique_solutions = _deduplicate(raw_solutions, robot, settings.deduplication_tolerance)

    # Build IKCandidate objects and FK-verify each one
    validated: list[IKCandidate] = []
    for q_sol in unique_solutions:
        residual_val = float(np.linalg.norm(_residual(q_sol, robot, target, mode)))
        candidate = IKCandidate(
            id=str(uuid.uuid4()),
            joint_positions=q_sol.tolist(),
            method="numerical_multi_start",
            solver_status="PENDING_VALIDATION",
            residual=residual_val,
        )
        validated_candidate = validate_candidate(robot, target, candidate)
        if validated_candidate.valid:
            validated.append(validated_candidate)

    return validated
