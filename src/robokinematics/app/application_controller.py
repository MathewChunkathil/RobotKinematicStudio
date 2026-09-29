"""Application controller — coordinates use cases.

This layer orchestrates use cases. It must not contain robotics equations.
It calls robotics services and updates AppState.
Signals (Qt) are used to notify the UI of state changes.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PySide6.QtCore import QObject, QThread, Signal

from robokinematics.app.state import AppState, AppStatus
from robokinematics.domain.models import (
    IKCandidate,
    RobotDefinition,
    RobotState,
    ScoringPolicy,
    Target,
)
from robokinematics.persistence.serializers import load_model, save_model
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.robotics.inverse_kinematics import IKSettings, solve_inverse_kinematics
from robokinematics.robotics.scoring import recommend_solution, score_candidates
from robokinematics.robotics.trajectory import plan_trajectory
from robokinematics.robotics.workspace import WorkspaceResult, sample_workspace


class _IKWorker(QThread):
    """Off-thread IK solver — keeps main thread and UI at 60fps."""

    finished = Signal(list)   # list[IKCandidate]
    error    = Signal(str)

    def __init__(
        self,
        robot: RobotDefinition,
        target: Target,
        state: RobotState,
        settings: IKSettings,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.robot    = robot
        self.target   = target
        self.state    = state
        self.settings = settings

    def run(self) -> None:
        try:
            candidates = solve_inverse_kinematics(
                self.robot, self.target, self.state, self.settings
            )
            self.finished.emit(candidates)
        except Exception as exc:
            self.error.emit(str(exc))


class _WorkspaceWorker(QThread):
    """Off-thread workspace sampler."""

    finished = Signal(object)  # WorkspaceResult

    def __init__(
        self,
        robot: RobotDefinition,
        n_samples: int,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.robot     = robot
        self.n_samples = n_samples

    def run(self) -> None:
        try:
            result = sample_workspace(self.robot, n_samples=self.n_samples)
            self.finished.emit(result)
        except Exception:
            pass


class ApplicationController(QObject):
    """Central application controller.

    Emits Qt signals so the UI can react to state changes without
    the controller knowing about specific widgets.
    """

    robot_changed      = Signal()
    target_changed     = Signal()
    ik_started         = Signal()
    ik_finished        = Signal()          # solutions ready
    solution_selected  = Signal(str)       # candidate id
    trajectory_updated = Signal()
    status_changed     = Signal(str, str)  # (status, detail)
    workspace_computed = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.state: AppState = AppState()
        self._ik_thread: _IKWorker | None = None
        self._ws_thread: _WorkspaceWorker | None = None
        self._workspace_result: WorkspaceResult | None = None

    # ── Robot ─────────────────────────────────────────────────────────────────

    def load_robot(self, path: str | Path) -> None:
        robot = load_model(RobotDefinition, path)
        self._set_robot(robot)

    def set_robot(self, robot: RobotDefinition) -> None:
        self._set_robot(robot)

    def _set_robot(self, robot: RobotDefinition) -> None:
        self.state.robot = robot
        self.state.robot_state = RobotState(
            joint_positions=[0.0] * len(robot.joints)
        )
        self.state.invalidate_ik()
        self._set_status(AppStatus.READY if self.state.target else AppStatus.NO_TARGET)
        self.robot_changed.emit()

    def set_joint_positions(self, positions: list[float]) -> None:
        if self.state.robot_state is None:
            return
        self.state.robot_state = RobotState(joint_positions=positions)

    # ── Target ────────────────────────────────────────────────────────────────

    def set_target(self, target: Target) -> None:
        self.state.target = target
        self.state.invalidate_ik()
        if self.state.robot is not None:
            self._set_status(AppStatus.READY)
        self.target_changed.emit()

    # ── IK ────────────────────────────────────────────────────────────────────

    def solve_ik(self) -> None:
        """Launch IK solver on a background QThread."""
        if self.state.robot is None or self.state.target is None:
            return
        if self.state.robot_state is None:
            return
        # Abort previous solve if still running
        if self._ik_thread is not None and self._ik_thread.isRunning():
            self._ik_thread.quit()
            self._ik_thread.wait(500)

        self._set_status(AppStatus.SOLVING)
        self.ik_started.emit()

        self._ik_thread = _IKWorker(
            robot    = self.state.robot,
            target   = self.state.target,
            state    = self.state.robot_state,
            settings = self.state.ik_settings,
            parent   = self,
        )
        self._ik_thread.finished.connect(self._on_ik_finished)
        self._ik_thread.error.connect(self._on_ik_error)
        self._ik_thread.start()

    def _on_ik_finished(self, candidates: list[IKCandidate]) -> None:
        self.state.ik_candidates = candidates

        if not candidates:
            self._set_status(AppStatus.NO_FEASIBLE_SOLUTION, "No feasible IK solution found.")
            self.ik_finished.emit()
            return

        current_q = (
            self.state.robot_state.joint_positions if self.state.robot_state else []
        )
        if self.state.robot is not None:
            self.state.solution_analyses = score_candidates(
                self.state.robot, candidates, current_q, self.state.scoring_policy
            )

        rec = recommend_solution(candidates, self.state.solution_analyses)
        if rec:
            self.state.selected_candidate_id = rec.id

        self._set_status(
            AppStatus.SOLUTIONS_FOUND,
            f"{len(candidates)} feasible candidate(s) found.",
        )
        self.ik_finished.emit()

    def _on_ik_error(self, msg: str) -> None:
        self._set_status(AppStatus.TARGET_UNREACHABLE, f"Solver error: {msg}")
        self.ik_finished.emit()

    # ── Solution selection ────────────────────────────────────────────────────

    def select_solution(self, candidate_id: str) -> None:
        self.state.selected_candidate_id = candidate_id
        self.state.trajectory = None
        self.solution_selected.emit(candidate_id)

    # ── Trajectory ────────────────────────────────────────────────────────────

    def generate_trajectory(self, duration: float = 3.0) -> None:
        if self.state.robot is None or self.state.robot_state is None:
            return
        candidate = self.state.get_selected_candidate()
        if candidate is None:
            return

        traj = plan_trajectory(
            robot     = self.state.robot,
            q_start   = self.state.robot_state.joint_positions,
            q_end     = candidate.joint_positions,
            duration  = duration,
            n_samples = 300,   # higher resolution for smooth playback
        )
        self.state.trajectory = traj
        self._set_status(AppStatus.READY_FOR_PLAYBACK, f"Trajectory ready ({duration:.1f}s, {len(traj.points)} pts)")
        self.trajectory_updated.emit()

    # ── FK helper ─────────────────────────────────────────────────────────────

    def get_fk_transforms(self, joint_positions: list[float] | None = None) -> list[np.ndarray]:
        if self.state.robot is None:
            return []
        q = joint_positions if joint_positions is not None else (
            self.state.robot_state.joint_positions if self.state.robot_state else []
        )
        return forward_kinematics(self.state.robot, q)

    # ── Workspace ─────────────────────────────────────────────────────────────

    def compute_workspace(self, n_samples: int = 8_000) -> None:
        if self.state.robot is None:
            return
        if self._ws_thread is not None and self._ws_thread.isRunning():
            return
        self._ws_thread = _WorkspaceWorker(self.state.robot, n_samples, parent=self)
        self._ws_thread.finished.connect(self._on_workspace_done)
        self._ws_thread.start()

    def _on_workspace_done(self, result: WorkspaceResult) -> None:
        self._workspace_result = result
        self.workspace_computed.emit()

    # ── Persistence ───────────────────────────────────────────────────────────

    def save_experiment(self, path: str | Path) -> None:
        if self.state.robot is None:
            return
        path = Path(path)
        save_model(self.state.robot, path.with_suffix(".robot.json"))
        if self.state.target:
            save_model(self.state.target, path.with_suffix(".target.json"))
        self.state.last_saved_path = path

    def load_experiment(self, robot_path: str | Path, target_path: str | Path | None = None) -> None:
        self.load_robot(robot_path)
        if target_path and Path(target_path).exists():
            target = load_model(Target, target_path)
            self.set_target(target)

    # ── Status helper ─────────────────────────────────────────────────────────

    def _set_status(self, status: AppStatus, detail: str = "") -> None:
        self.state.status = status
        self.state.status_detail = detail
        self.status_changed.emit(status.value, detail)
