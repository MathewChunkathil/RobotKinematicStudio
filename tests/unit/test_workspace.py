"""Workspace sampler tests."""

import numpy as np

from robokinematics.domain.models import DHParameters, JointDefinition, JointLimits, LinkDefinition, RobotDefinition
from robokinematics.robotics.workspace import sample_workspace


def make_robot() -> RobotDefinition:
    joints = [
        JointDefinition(
            id=f"j{i}", index=i, name=f"J{i}", type="revolute",
            dh=DHParameters(theta=0, d=0, a=0.3, alpha=0),
            limits=JointLimits(position_min=-np.pi, position_max=np.pi),
        )
        for i in range(4)
    ]
    links = [LinkDefinition(id=f"l{i}", index=i, name=f"L{i}") for i in range(4)]
    return RobotDefinition(id="ws_test", name="WS Robot", joints=joints, links=links)


def test_workspace_returns_points():
    robot = make_robot()
    result = sample_workspace(robot, n_samples=500, random_seed=1)
    assert result.valid_count > 0
    assert result.points.shape[1] == 3


def test_workspace_bounding_box_consistent():
    robot = make_robot()
    result = sample_workspace(robot, n_samples=500)
    if result.valid_count > 0:
        assert np.all(result.bounding_box_min <= result.bounding_box_max)


def test_workspace_max_reach_positive():
    robot = make_robot()
    result = sample_workspace(robot, n_samples=200)
    assert result.max_reach > 0.0
