"""Integration test for experiment persistence (Phase 16)."""

from pathlib import Path

import pytest

from robokinematics.domain.models import (
    Pose, RobotDefinition, RobotState, ScoringPolicy, Target,
)
from robokinematics.persistence.experiment import load_experiment, save_experiment
from robokinematics.persistence.serializers import load_model
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.robotics.inverse_kinematics import IKSettings, solve_inverse_kinematics
from robokinematics.robotics.scoring import recommend_solution, score_candidates


ROBOT_PATH = Path(__file__).parent.parent.parent / "assets" / "robots" / "reference_4dof.json"


def test_experiment_save_load_roundtrip(tmp_path):
    robot = load_model(RobotDefinition, ROBOT_PATH)
    home_q = [0.0] * len(robot.joints)
    state = RobotState(joint_positions=home_q)

    transforms = forward_kinematics(robot, home_q)
    ee_pos = transforms[-1][:3, 3].tolist()
    target = Target(id="t1", mode="point", pose=Pose(position=ee_pos), position_tolerance=1e-3)

    settings = IKSettings(starts=8, random_seed=0)
    candidates = solve_inverse_kinematics(robot, target, state, settings)
    assert candidates

    analyses = score_candidates(robot, candidates, home_q)
    rec = recommend_solution(candidates, analyses)

    experiment_file = tmp_path / "test_experiment.json"
    save_experiment(
        path=experiment_file,
        robot=robot,
        target=target,
        candidates=candidates,
        selected_candidate_id=rec.id if rec else None,
        scoring_policy=ScoringPolicy(),
    )

    assert experiment_file.exists()

    loaded = load_experiment(experiment_file)
    assert loaded["robot"].id == robot.id
    assert loaded["target"].id == target.id
    assert len(loaded["candidates"]) == len(candidates)
    assert loaded["selected_candidate_id"] == (rec.id if rec else None)
