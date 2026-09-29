"""Application state container.

Holds all mutable application state in one place.
The controller mutates state; the UI reads state and renders.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from robokinematics.domain.models import (
    IKCandidate,
    RobotDefinition,
    RobotState,
    ScoringPolicy,
    SolutionAnalysis,
    Target,
)
from robokinematics.robotics.inverse_kinematics import IKSettings
from robokinematics.robotics.trajectory import TrajectoryResult


class AppStatus(str, Enum):
    READY = "READY"
    CONFIGURATION_INVALID = "CONFIGURATION INVALID"
    NO_TARGET = "NO TARGET"
    TARGET_UNREACHABLE = "TARGET UNREACHABLE"
    SOLVING = "SOLVING"
    SOLUTIONS_FOUND = "SOLUTIONS FOUND"
    NO_FEASIBLE_SOLUTION = "NO FEASIBLE SOLUTION"
    READY_FOR_PLAYBACK = "READY FOR PLAYBACK"
    PLAYING = "PLAYING"
    PAUSED = "PAUSED"
    TARGET_REACHED = "TARGET REACHED"
    TARGET_ERROR = "TARGET ERROR — TOLERANCE VIOLATED"


@dataclass
class VisualizationPreferences:
    show_robot: bool = True
    show_target: bool = True
    show_active_trajectory: bool = True
    show_ghost_trajectories: bool = True
    show_world_frame: bool = True
    show_joint_frames: bool = False
    show_joint_axes: bool = True
    show_workspace: bool = False
    show_singularity: bool = False
    show_dh_info: bool = False
    show_velocity_vectors: bool = False
    show_acceleration_vectors: bool = False


@dataclass
class AppState:
    # Core configuration
    robot: RobotDefinition | None = None
    robot_state: RobotState | None = None
    target: Target | None = None
    scoring_policy: ScoringPolicy = field(default_factory=ScoringPolicy)
    ik_settings: IKSettings = field(default_factory=IKSettings)

    # Results
    ik_candidates: list[IKCandidate] = field(default_factory=list)
    solution_analyses: list[SolutionAnalysis] = field(default_factory=list)
    selected_candidate_id: str | None = None
    trajectory: TrajectoryResult | None = None

    # Playback
    playback_time: float = 0.0
    playback_speed: float = 1.0

    # Status
    status: AppStatus = AppStatus.NO_TARGET
    status_detail: str = ""

    # Persistence
    last_saved_path: Path | None = None

    # Visualization
    vis_prefs: VisualizationPreferences = field(default_factory=VisualizationPreferences)

    def get_selected_candidate(self) -> IKCandidate | None:
        if self.selected_candidate_id is None:
            return None
        for c in self.ik_candidates:
            if c.id == self.selected_candidate_id:
                return c
        return None

    def get_selected_analysis(self) -> SolutionAnalysis | None:
        if self.selected_candidate_id is None:
            return None
        for a in self.solution_analyses:
            if a.solution_id == self.selected_candidate_id:
                return a
        return None

    def invalidate_ik(self) -> None:
        """Call whenever robot or target changes."""
        self.ik_candidates = []
        self.solution_analyses = []
        self.selected_candidate_id = None
        self.trajectory = None
        self.status = AppStatus.NO_TARGET if self.target is None else AppStatus.READY
