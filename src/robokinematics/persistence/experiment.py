"""Experiment persistence — full save/load for all experiment state.

Saves robot, target, selected solution ID, scoring policy, and IK settings
to a single JSON experiment file. Trajectory is regenerated on load.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from robokinematics.domain.models import (
    IKCandidate,
    RobotDefinition,
    ScoringPolicy,
    Target,
)
from robokinematics.persistence.serializers import load_model, save_model

SCHEMA_VERSION = "1.0.0"


def save_experiment(
    path: str | Path,
    robot: RobotDefinition,
    target: Target | None,
    candidates: list[IKCandidate],
    selected_candidate_id: str | None,
    scoring_policy: ScoringPolicy,
) -> None:
    """Save the full experiment state to a JSON file."""
    path = Path(path)
    experiment: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "robot": json.loads(robot.model_dump_json()),
    }
    if target is not None:
        experiment["target"] = json.loads(target.model_dump_json())
    if candidates:
        experiment["candidates"] = [json.loads(c.model_dump_json()) for c in candidates]
    experiment["selected_candidate_id"] = selected_candidate_id
    experiment["scoring_policy"] = json.loads(scoring_policy.model_dump_json())

    path.write_text(json.dumps(experiment, indent=2), encoding="utf-8")


def load_experiment(path: str | Path) -> dict[str, Any]:
    """Load an experiment file.

    Returns a dict with keys: robot, target, candidates, selected_candidate_id, scoring_policy.
    """
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))

    result: dict[str, Any] = {
        "schema_version": data.get("schema_version", "unknown"),
        "robot": RobotDefinition.model_validate(data["robot"]),
        "target": Target.model_validate(data["target"]) if "target" in data else None,
        "candidates": [IKCandidate.model_validate(c) for c in data.get("candidates", [])],
        "selected_candidate_id": data.get("selected_candidate_id"),
        "scoring_policy": ScoringPolicy.model_validate(data.get("scoring_policy", {})),
    }
    return result
