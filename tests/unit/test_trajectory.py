"""Tests for trajectory planning."""

from __future__ import annotations

import numpy as np
import pytest

from robokinematics.domain.models import DHParameters, JointDefinition, JointLimits, LinkDefinition, RobotDefinition
from robokinematics.robotics.trajectory import plan_trajectory


def make_simple_robot() -> RobotDefinition:
    joints = [
        JointDefinition(
            id=f"j{i}", index=i, name=f"Joint {i}", type="revolute",
            dh=DHParameters(theta=0, d=0, a=0.3, alpha=0),
            limits=JointLimits(
                position_min=-np.pi, position_max=np.pi,
                velocity_max=2.0, acceleration_max=5.0,
            ),
        )
        for i in range(4)
    ]
    links = [
        LinkDefinition(id=f"l{i}", index=i, name=f"Link {i}")
        for i in range(4)
    ]
    return RobotDefinition(id="traj_test", name="Traj Robot", joints=joints, links=links)


def test_trajectory_starts_at_q_start():
    robot = make_simple_robot()
    q_start = [0.0] * 4
    q_end = [1.0, -0.5, 0.3, -1.0]
    result = plan_trajectory(robot, q_start, q_end, duration=2.0, n_samples=50)
    np.testing.assert_allclose(result.points[0].q, q_start, atol=1e-12)


def test_trajectory_ends_at_q_end():
    robot = make_simple_robot()
    q_start = [0.0] * 4
    q_end = [1.0, -0.5, 0.3, -1.0]
    result = plan_trajectory(robot, q_start, q_end, duration=2.0, n_samples=50)
    np.testing.assert_allclose(result.points[-1].q, q_end, atol=1e-6)


def test_trajectory_zero_velocity_boundaries():
    robot = make_simple_robot()
    q_start = [0.0] * 4
    q_end = [0.5] * 4
    result = plan_trajectory(robot, q_start, q_end, duration=2.0, n_samples=50)
    np.testing.assert_allclose(result.points[0].qdot, np.zeros(4), atol=1e-12)
    np.testing.assert_allclose(result.points[-1].qdot, np.zeros(4), atol=1e-12)


def test_trajectory_is_finite():
    robot = make_simple_robot()
    result = plan_trajectory(robot, [0.0]*4, [0.5]*4, duration=2.0)
    for pt in result.points:
        assert np.all(np.isfinite(pt.q))
        assert np.all(np.isfinite(pt.qdot))
        assert np.all(np.isfinite(pt.qddot))


def test_invalid_duration_raises():
    robot = make_simple_robot()
    with pytest.raises(ValueError):
        plan_trajectory(robot, [0.0]*4, [1.0]*4, duration=0.0)
