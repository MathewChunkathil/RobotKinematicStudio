"""Solution scoring and recommendation service.

Implements Phase 8 per 02_ROBOTICS_SPEC.md §14-17.

Metrics:
  - motion_cost: sum |q_i - q_current_i| (shortest angular for revolute)
  - limit_margin: min over joints of normalized distance to nearest limit
  - singularity_risk: derived from min singular value of Jacobian
  - manipulability: from JacobianAnalysis

Scoring is explicit, configurable, and inspectable.
"""

from __future__ import annotations

import numpy as np

from robokinematics.domain.models import (
    IKCandidate,
    JointType,
    RobotDefinition,
    ScoringPolicy,
    SolutionAnalysis,
)
from robokinematics.robotics.jacobian import analyse_jacobian


def _angular_distance(a: float, b: float) -> float:
    """Shortest angular distance between two angles (radians)."""
    diff = (a - b + np.pi) % (2 * np.pi) - np.pi
    return abs(diff)


def compute_motion_cost(
    robot: RobotDefinition,
    candidate_q: list[float],
    current_q: list[float],
) -> float:
    """Sum of shortest joint displacements from current state."""
    total = 0.0
    for joint, q, q0 in zip(robot.joints, candidate_q, current_q):
        if joint.type == JointType.REVOLUTE:
            total += _angular_distance(q, q0)
        else:
            total += abs(q - q0)
    return total


def compute_joint_limit_margin(
    robot: RobotDefinition,
    candidate_q: list[float],
) -> float:
    """Normalized minimum margin from any joint limit (0=at limit, 1=at center)."""
    margins = []
    for joint, q in zip(robot.joints, candidate_q):
        span = joint.limits.position_max - joint.limits.position_min
        if span <= 0.0:
            margins.append(0.0)
            continue
        d_min = q - joint.limits.position_min
        d_max = joint.limits.position_max - q
        normalized = min(d_min, d_max) / (span / 2.0)
        margins.append(float(np.clip(normalized, 0.0, 1.0)))
    return min(margins) if margins else 0.0


def score_candidates(
    robot: RobotDefinition,
    candidates: list[IKCandidate],
    current_q: list[float],
    policy: ScoringPolicy | None = None,
) -> list[SolutionAnalysis]:
    """Compute metrics and weighted scores for every valid candidate.

    Returns SolutionAnalysis list in the same order as candidates.
    Only valid candidates receive full analysis; invalid ones get sentinel values.
    """
    if policy is None:
        policy = ScoringPolicy()

    analyses: list[SolutionAnalysis] = []
    valid_motion_costs: list[float] = []

    # First pass: compute raw metrics for valid candidates
    raw: list[dict] = []
    for cand in candidates:
        if not cand.valid:
            raw.append({})
            continue

        q = cand.joint_positions
        motion = compute_motion_cost(robot, q, current_q)
        limit_margin = compute_joint_limit_margin(robot, q)
        jac_analysis = analyse_jacobian(robot, q)
        singularity_metric = jac_analysis.min_singular_value
        manip = jac_analysis.manipulability

        raw.append({
            "motion": motion,
            "limit_margin": limit_margin,
            "singularity_metric": singularity_metric,
            "singularity_status": jac_analysis.singularity_status.value,
            "manipulability": manip,
        })
        valid_motion_costs.append(motion)

    # Normalize motion cost across candidates (0–1 range)
    max_motion = max(valid_motion_costs) if valid_motion_costs else 1.0
    if max_motion < 1e-12:
        max_motion = 1.0

    for cand, r in zip(candidates, raw):
        if not cand.valid or not r:
            analyses.append(
                SolutionAnalysis(
                    solution_id=cand.id,
                    motion_cost=0.0,
                    normalized_motion_cost=0.0,
                    joint_limit_margin=0.0,
                    normalized_limit_risk=1.0,
                    singularity_status="N/A",
                    score=None,
                )
            )
            continue

        motion = r["motion"]
        limit_margin = r["limit_margin"]
        singularity_metric = r["singularity_metric"]
        manip = r["manipulability"]

        norm_motion = motion / max_motion
        # limit risk = 1 - margin (low margin = high risk)
        norm_limit_risk = 1.0 - limit_margin
        # singularity risk: normalize to [0,1] where 0=safe, 1=singular
        # using 1 / (1 + singularity_metric) is intuitive
        norm_singularity = 1.0 / (1.0 + singularity_metric) if singularity_metric is not None else 1.0

        # Weighted score (lower = better)
        score = (
            policy.motion_weight * norm_motion
            + policy.limit_weight * norm_limit_risk
            + policy.singularity_weight * norm_singularity
        )

        analyses.append(
            SolutionAnalysis(
                solution_id=cand.id,
                motion_cost=motion,
                normalized_motion_cost=norm_motion,
                joint_limit_margin=limit_margin,
                normalized_limit_risk=norm_limit_risk,
                singularity_status=r["singularity_status"],
                singularity_metric=singularity_metric,
                manipulability=manip,
                score=float(score),
            )
        )

    return analyses


def recommend_solution(
    candidates: list[IKCandidate],
    analyses: list[SolutionAnalysis],
) -> IKCandidate | None:
    """Return the valid candidate with the lowest score. None if no valid candidates."""
    best: IKCandidate | None = None
    best_score = float("inf")

    for cand, analysis in zip(candidates, analyses):
        if not cand.valid:
            continue
        if analysis.score is not None and analysis.score < best_score:
            best = cand
            best_score = analysis.score

    return best
