"""Integration test: full pipeline from config → FK → target → IK → validate → score → trajectory."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robokinematics.domain.models import Pose, RobotState, Target
from robokinematics.persistence.serializers import load_model
from robokinematics.domain.models import RobotDefinition
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.robotics.inverse_kinematics import IKSettings, solve_inverse_kinematics
from robokinematics.robotics.scoring import recommend_solution, score_candidates
from robokinematics.robotics.trajectory import plan_trajectory


ROBOT_PATH = Path(__file__).parent.parent.parent / "assets" / "robots" / "reference_4dof.json"


def test_full_pipeline():
    """End-to-end: load robot → FK → IK → score → recommend → trajectory."""
    # 1. Load robot
    robot = load_model(RobotDefinition, ROBOT_PATH)
    assert robot is not None

    # 2. Home state
    home_q = [0.0] * len(robot.joints)
    state = RobotState(joint_positions=home_q)

    # 3. FK to get a reachable target
    transforms = forward_kinematics(robot, home_q)
    ee_tf = transforms[-1]
    ee_pos = ee_tf[:3, 3].tolist()

    target = Target(
        id="integration_target",
        mode="point",
        pose=Pose(position=ee_pos),
        position_tolerance=1e-3,
    )

    # 4. IK
    settings = IKSettings(starts=16, random_seed=0)
    candidates = solve_inverse_kinematics(robot, target, state, settings)
    assert len(candidates) >= 1, "Pipeline must find at least one valid IK solution"

    for c in candidates:
        assert c.valid
        assert c.position_error is not None
        assert c.position_error <= target.position_tolerance

    # 5. Score and recommend
    analyses = score_candidates(robot, candidates, home_q)
    recommendation = recommend_solution(candidates, analyses)
    assert recommendation is not None
    assert recommendation.valid

    # 6. Trajectory
    target_q = recommendation.joint_positions
    traj = plan_trajectory(robot, home_q, target_q, duration=2.0, n_samples=50)

    np.testing.assert_allclose(traj.points[0].q, home_q, atol=1e-12)
    np.testing.assert_allclose(traj.points[-1].q, target_q, atol=1e-6)
    assert all(np.all(np.isfinite(pt.q)) for pt in traj.points)
