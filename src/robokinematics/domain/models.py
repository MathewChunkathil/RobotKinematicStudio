"""Core typed domain models.

Keep these models independent from PySide6, PyVista, and robotics algorithms.
"""

from __future__ import annotations

from enum import Enum
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator


class JointType(str, Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"


class Units(BaseModel):
    model_config = ConfigDict(frozen=True)
    length: Literal["m"] = "m"
    angle: Literal["rad"] = "rad"
    time: Literal["s"] = "s"


class DHParameters(BaseModel):
    model_config = ConfigDict(validate_assignment=True)

    theta: float = 0.0
    d: float = 0.0
    a: float = 0.0
    alpha: float = 0.0

    @field_validator("theta", "d", "a", "alpha")
    @classmethod
    def finite(cls, value: float) -> float:
        if not np.isfinite(value):
            raise ValueError("DH parameters must be finite.")
        return value


class JointLimits(BaseModel):
    model_config = ConfigDict(validate_assignment=True)

    position_min: float
    position_max: float
    velocity_max: float | None = None
    acceleration_max: float | None = None

    @field_validator("position_max")
    @classmethod
    def ordered_limits(cls, value: float, info):
        minimum = info.data.get("position_min")
        if minimum is not None and value < minimum:
            raise ValueError("position_max must be >= position_min.")
        return value


class LinkVisualDefinition(BaseModel):
    length: float = Field(default=0.1, gt=0.0)
    width: float = Field(default=0.04, gt=0.0)
    height: float = Field(default=0.04, gt=0.0)
    shape: Literal["box", "cylinder"] = "box"
    radius: float | None = Field(default=None, gt=0.0)


class JointVisualDefinition(BaseModel):
    radius: float = Field(default=0.04, gt=0.0)
    depth: float = Field(default=0.06, gt=0.0)


class JointDefinition(BaseModel):
    id: str
    index: int = Field(ge=0)
    name: str
    type: JointType
    dh: DHParameters
    limits: JointLimits
    visual: JointVisualDefinition = Field(default_factory=JointVisualDefinition)


class LinkDefinition(BaseModel):
    id: str
    index: int = Field(ge=0)
    name: str
    visual: LinkVisualDefinition = Field(default_factory=LinkVisualDefinition)


class Pose(BaseModel):
    position: list[float] = Field(min_length=3, max_length=3)
    rotation_matrix: list[list[float]] = Field(
        default_factory=lambda: np.eye(3).tolist()
    )

    @field_validator("position")
    @classmethod
    def position_finite(cls, value):
        if not all(np.isfinite(v) for v in value):
            raise ValueError("Position must be finite.")
        return value


class Target(BaseModel):
    id: str
    name: str = "Target"
    mode: Literal["point", "pose"] = "point"
    pose: Pose
    position_tolerance: float = Field(default=1e-4, gt=0.0)
    orientation_tolerance: float = Field(default=1e-3, gt=0.0)


class RobotDefinition(BaseModel):
    id: str
    name: str
    description: str = ""
    units: Units = Field(default_factory=Units)
    joints: list[JointDefinition]
    links: list[LinkDefinition]
    base_translation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_rotation: list[list[float]] = Field(default_factory=lambda: np.eye(3).tolist())
    end_effector_translation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    end_effector_rotation: list[list[float]] = Field(default_factory=lambda: np.eye(3).tolist())

    @field_validator("joints")
    @classmethod
    def contiguous_joint_indices(cls, value):
        indices = [j.index for j in value]
        if indices != list(range(len(value))):
            raise ValueError("Joint indices must be contiguous starting at 0.")
        return value


class RobotState(BaseModel):
    joint_positions: list[float]
    joint_velocities: list[float] = Field(default_factory=list)
    joint_accelerations: list[float] = Field(default_factory=list)
    timestamp: float = 0.0


class IKCandidate(BaseModel):
    id: str
    joint_positions: list[float]
    method: str
    solver_status: str
    residual: float | None = None
    achieved_pose: Pose | None = None
    position_error: float | None = None
    orientation_error: float | None = None
    valid: bool = False


class SolutionAnalysis(BaseModel):
    solution_id: str
    motion_cost: float
    normalized_motion_cost: float
    joint_limit_margin: float
    normalized_limit_risk: float
    singularity_status: str
    singularity_metric: float | None = None
    manipulability: float | None = None
    trajectory_cost: float | None = None
    score: float | None = None


class ScoringPolicy(BaseModel):
    motion_weight: float = 1.0
    singularity_weight: float = 1.0
    limit_weight: float = 1.0
    trajectory_weight: float = 1.0


class TargetPoseResult(BaseModel):
    position_error: float
    orientation_error: float | None = None
    within_tolerance: bool
