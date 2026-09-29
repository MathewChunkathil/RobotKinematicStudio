"""Numerical reference tests for the IK + Jacobian pipeline.

Uses the reference 4-DOF robot and independently verified reference cases.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robokinematics.domain.models import Pose, RobotDefinition, RobotState, ScoringPolicy, Target
from robokinematics.persistence.serializers import load_model
from robokinematics.robotics.inverse_kinematics import IKSettings, solve_inverse_kinematics
from robokinematics.robotics.jacobian import analyse_jacobian, geometric_jacobian
from robokinematics.robotics.scoring import recommend_solution, score_candidates
from robokinematics.robotics.validation import validate_candidate
from robokinematics.domain.models import IKCandidate


ROBOT_PATH = Path(__file__).parent.parent.parent / "assets" / "robots" / "reference_4dof.json"


@pytest.fixture(scope="module")
def robot() -> RobotDefinition:
    return load_model(RobotDefinition, ROBOT_PATH)


@pytest.fixture(scope="module")
def home_state(robot: RobotDefinition) -> RobotState:
    return RobotState(joint_positions=[0.0] * len(robot.joints))


# ── Jacobian tests ────────────────────────────────────────────────────────────

def test_jacobian_shape(robot, home_state):
    J = geometric_jacobian(robot, home_state.joint_positions)
    assert J.shape == (6, len(robot.joints))


def test_jacobian_finite(robot, home_state):
    J = geometric_jacobian(robot, home_state.joint_positions)
    assert np.all(np.isfinite(J))


def test_jacobian_finite_difference(robot, home_state):
    """Verify positional Jacobian rows via finite differences."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    q = np.array(home_state.joint_positions, dtype=float)
    J = geometric_jacobian(robot, q.tolist())
    J_pos = J[:3, :]  # positional sub-Jacobian

    h = 1e-7
    J_fd = np.zeros((3, len(robot.joints)))
    p0 = forward_kinematics(robot, q.tolist())[-1][:3, 3]

    for i in range(len(robot.joints)):
        q_plus = q.copy()
        q_plus[i] += h
        p_plus = forward_kinematics(robot, q_plus.tolist())[-1][:3, 3]
        J_fd[:, i] = (p_plus - p0) / h

    np.testing.assert_allclose(J_pos, J_fd, atol=1e-5,
                                err_msg="Geometric Jacobian does not match finite-difference approximation")


def test_jacobian_analysis_home(robot, home_state):
    analysis = analyse_jacobian(robot, home_state.joint_positions)
    assert analysis.rank > 0
    assert analysis.min_singular_value >= 0.0
    assert analysis.manipulability is not None and analysis.manipulability >= 0.0


# ── Validation tests ──────────────────────────────────────────────────────────

def test_validate_known_good_candidate(robot, home_state):
    """A candidate at home = forward kinematics of home should be valid for its own FK target."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    transforms = forward_kinematics(robot, home_state.joint_positions)
    ee = transforms[-1]
    target = Target(
        id="t1",
        mode="point",
        pose=Pose(
            position=ee[:3, 3].tolist(),
            rotation_matrix=ee[:3, :3].tolist(),
        ),
        position_tolerance=1e-3,
    )
    candidate = IKCandidate(
        id="c1",
        joint_positions=home_state.joint_positions,
        method="test",
        solver_status="PENDING",
    )
    result = validate_candidate(robot, target, candidate)
    assert result.valid, f"Expected valid but got: {result.solver_status}"


def test_validate_joint_limit_violation(robot):
    """A candidate that violates joint limits must be rejected."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    # Put joint 0 way outside limits
    bad_q = [1000.0] + [0.0] * (len(robot.joints) - 1)
    target = Target(
        id="t2",
        mode="point",
        pose=Pose(position=[0.0, 0.0, 0.0]),
        position_tolerance=1e-3,
    )
    candidate = IKCandidate(
        id="c2",
        joint_positions=bad_q,
        method="test",
        solver_status="PENDING",
    )
    result = validate_candidate(robot, target, candidate)
    assert not result.valid
    assert "JOINT_LIMIT" in result.solver_status


# ── IK solver tests ───────────────────────────────────────────────────────────

def test_ik_returns_verified_candidates(robot, home_state):
    """IK on the FK-computed target of home should return at least one verified candidate."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    transforms = forward_kinematics(robot, home_state.joint_positions)
    ee_pos = transforms[-1][:3, 3].tolist()

    target = Target(
        id="ik_t1",
        mode="point",
        pose=Pose(position=ee_pos),
        position_tolerance=1e-3,
    )
    settings = IKSettings(starts=16, random_seed=42)
    candidates = solve_inverse_kinematics(robot, target, home_state, settings)

    assert len(candidates) >= 1, "Expected at least one valid IK candidate"
    for c in candidates:
        assert c.valid
        assert c.position_error is not None
        assert c.position_error <= target.position_tolerance


def test_ik_unreachable_target(robot, home_state):
    """A target far outside workspace should return no valid candidates."""
    target = Target(
        id="ik_unreachable",
        mode="point",
        pose=Pose(position=[100.0, 100.0, 100.0]),
        position_tolerance=1e-3,
    )
    settings = IKSettings(starts=8, random_seed=0)
    candidates = solve_inverse_kinematics(robot, target, home_state, settings)
    assert len(candidates) == 0, "Expected no valid candidates for unreachable target"


# ── Scoring tests ─────────────────────────────────────────────────────────────

def test_scoring_deterministic(robot, home_state):
    """Score of a fixed IK solution must be deterministic."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    transforms = forward_kinematics(robot, home_state.joint_positions)
    ee_pos = transforms[-1][:3, 3].tolist()

    target = Target(id="st1", mode="point", pose=Pose(position=ee_pos), position_tolerance=1e-3)
    settings = IKSettings(starts=16, random_seed=42)
    candidates = solve_inverse_kinematics(robot, target, home_state, settings)

    analyses1 = score_candidates(robot, candidates, home_state.joint_positions)
    analyses2 = score_candidates(robot, candidates, home_state.joint_positions)

    for a1, a2 in zip(analyses1, analyses2):
        assert a1.score == a2.score


def test_recommend_picks_valid(robot, home_state):
    """recommend_solution must return only a valid candidate."""
    from robokinematics.robotics.forward_kinematics import forward_kinematics
    transforms = forward_kinematics(robot, home_state.joint_positions)
    ee_pos = transforms[-1][:3, 3].tolist()

    target = Target(id="rt1", mode="point", pose=Pose(position=ee_pos), position_tolerance=1e-3)
    settings = IKSettings(starts=16, random_seed=42)
    candidates = solve_inverse_kinematics(robot, target, home_state, settings)
    analyses = score_candidates(robot, candidates, home_state.joint_positions)
    recommendation = recommend_solution(candidates, analyses)

    assert recommendation is not None
    assert recommendation.valid
