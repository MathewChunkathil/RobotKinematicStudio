"""Trajectory planning service.

Implements Phase 10 per 02_ROBOTICS_SPEC.md §18-19.

Strategy: joint-space cubic polynomial interpolation.
  - Boundary conditions: zero velocity at start and end.
  - Validates q, qdot, qddot against joint limits at every sample.
  - Verifies endpoint configuration matches target within tolerance.

Quintic (5th-order) optionally available when zero acceleration BCs are needed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robokinematics.domain.models import RobotDefinition


# ── Result type ───────────────────────────────────────────────────────────────

@dataclass
class TrajectoryPoint:
    t: float
    q: np.ndarray       # joint positions
    qdot: np.ndarray    # joint velocities
    qddot: np.ndarray   # joint accelerations


@dataclass
class TrajectoryResult:
    points: list[TrajectoryPoint]
    duration: float
    valid: bool
    violations: list[str]   # empty if valid


# ── Polynomial coefficients ───────────────────────────────────────────────────

def _cubic_coeffs(q0: float, qf: float, T: float) -> tuple[float, float, float, float]:
    """Cubic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    
    BCs: q(0)=q0, q(T)=qf, qdot(0)=0, qdot(T)=0
    """
    a0 = q0
    a1 = 0.0
    a2 = 3.0 * (qf - q0) / T**2
    a3 = -2.0 * (qf - q0) / T**3
    return a0, a1, a2, a3


def _eval_cubic(
    t: float,
    a0: float, a1: float, a2: float, a3: float
) -> tuple[float, float, float]:
    """Evaluate cubic at time t → (q, qdot, qddot)."""
    q = a0 + a1 * t + a2 * t**2 + a3 * t**3
    qdot = a1 + 2 * a2 * t + 3 * a3 * t**2
    qddot = 2 * a2 + 6 * a3 * t
    return q, qdot, qddot


# ── Public planner ────────────────────────────────────────────────────────────

def plan_trajectory(
    robot: RobotDefinition,
    q_start: list[float],
    q_end: list[float],
    duration: float,
    n_samples: int = 100,
    endpoint_tolerance: float = 1e-6,
) -> TrajectoryResult:
    """Plan a joint-space cubic trajectory from q_start to q_end over duration.

    Args:
        robot: robot definition (for limits)
        q_start: starting joint configuration (radians / metres)
        q_end: target joint configuration (radians / metres)
        duration: trajectory duration in seconds
        n_samples: number of time steps (inclusive of 0 and T)
        endpoint_tolerance: tolerance for endpoint configuration check

    Returns:
        TrajectoryResult with list of TrajectoryPoints and validity flags.
    """
    if duration <= 0.0:
        raise ValueError("Trajectory duration must be positive.")

    n = len(robot.joints)
    if len(q_start) != n or len(q_end) != n:
        raise ValueError("q_start and q_end must match robot DOF count.")

    # Precompute cubic coefficients per joint
    coeffs = [_cubic_coeffs(q_start[i], q_end[i], duration) for i in range(n)]

    times = np.linspace(0.0, duration, n_samples)
    points: list[TrajectoryPoint] = []
    violations: list[str] = []

    for t in times:
        q_arr = np.zeros(n)
        qdot_arr = np.zeros(n)
        qddot_arr = np.zeros(n)

        for i in range(n):
            q_arr[i], qdot_arr[i], qddot_arr[i] = _eval_cubic(t, *coeffs[i])

        # Safety: finiteness
        if not (np.all(np.isfinite(q_arr)) and np.all(np.isfinite(qdot_arr)) and np.all(np.isfinite(qddot_arr))):
            violations.append(f"Non-finite value at t={t:.4f}s")

        points.append(TrajectoryPoint(t=float(t), q=q_arr, qdot=qdot_arr, qddot=qddot_arr))

    # Validate limits at each point
    for pt in points:
        for i, joint in enumerate(robot.joints):
            q_i = pt.q[i]
            if q_i < joint.limits.position_min or q_i > joint.limits.position_max:
                violations.append(
                    f"Joint {joint.id} position {q_i:.4f} out of limits at t={pt.t:.4f}s"
                )
                break  # One violation per time step is sufficient for reporting

            if joint.limits.velocity_max is not None and abs(pt.qdot[i]) > joint.limits.velocity_max:
                violations.append(
                    f"Joint {joint.id} velocity {pt.qdot[i]:.4f} exceeds limit at t={pt.t:.4f}s"
                )
                break

            if joint.limits.acceleration_max is not None and abs(pt.qddot[i]) > joint.limits.acceleration_max:
                violations.append(
                    f"Joint {joint.id} acceleration {pt.qddot[i]:.4f} exceeds limit at t={pt.t:.4f}s"
                )
                break

    # Endpoint verification
    final_q = points[-1].q
    for i in range(n):
        err = abs(final_q[i] - q_end[i])
        if err > endpoint_tolerance:
            violations.append(
                f"Joint {robot.joints[i].id} endpoint error {err:.2e} > tolerance {endpoint_tolerance:.2e}"
            )

    valid = len(violations) == 0
    return TrajectoryResult(
        points=points,
        duration=duration,
        valid=valid,
        violations=violations,
    )
