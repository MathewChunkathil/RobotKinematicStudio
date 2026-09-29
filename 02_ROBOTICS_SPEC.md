# Robotics and Mathematical Specification

## 1. Coordinate convention

Choose and freeze one DH convention before implementation.

Recommended initial convention: Standard Denavit-Hartenberg.

For each joint/link: - theta_i - d_i - a_i - alpha_i

The implementation must use the same convention in: - configuration; -
transformation generation; - FK; - IK derivations; - frame
visualization; - documentation.

## 2. Homogeneous transform

For standard DH:

T\_(i-1)\^i = RotZ(theta_i) TransZ(d_i) TransX(a_i) RotX(alpha_i)

The exact matrix must be implemented once in a transformation service
and reused.

## 3. Joint state

For N joints:

q = \[q_1, ..., q_N\]\^T

qdot = \[qdot_1, ..., qdot_N\]\^T

qddot = \[qddot_1, ..., qddot_N\]\^T

The application state may additionally contain: - timestamp; - active
solution ID; - target; - playback state.

## 4. FK

For a robot with N joints:

T_0\^N = T_0\^1 T_1\^2 ... T\_(N-1)\^N

The system must retain intermediate transforms.

Output: - all frame transforms; - end-effector transform; - position; -
orientation.

## 5. Position-only target

Target:

p_target = \[x, y, z\]\^T

A valid candidate must satisfy:

\|\|p_actual - p_target\|\| \<= position_tolerance.

The tolerance must be configurable.

## 6. Pose target

Target:

T_target = \[R_target p_target; 0 1\]

Candidate validity requires both: - position tolerance; - orientation
tolerance.

Orientation error representation must be selected and documented. Prefer
a numerically stable rotation-vector or equivalent error measure
internally.

## 7. IK architecture

Use an IK solver interface.

Conceptually:

solve(target, robot, initial_conditions, settings) -\> IKResult

IKResult contains: - candidates; - solver status; - method; - iteration
information; - residuals; - diagnostics.

### Analytical IK

Use only when a mathematically verified derivation exists for a
supported structure.

### Numerical IK

Provide the general fallback.

Possible numerical approaches: - least-squares; - damped least
squares; - SciPy least_squares or an equivalent robust method.

Multiple solutions can be found by multi-start: 1. generate multiple
initial guesses; 2. solve from each; 3. normalize equivalent angles; 4.
reject invalid candidates; 5. reject duplicates within configuration
tolerance; 6. FK-verify remaining candidates.

## 8. IK uniqueness/deduplication

Two candidates should be considered the same if their normalized joint
configuration distance is below a configured threshold, accounting for
revolute wraparound.

For prismatic joints use direct absolute difference.

Do not report numerical duplicates as separate solutions.

## 9. Joint limits

For every joint:

min_i \<= q_i \<= max_i

Revolute angles must use a documented normalization policy.

Prismatic values use SI distance.

## 10. Jacobian

Use a geometric Jacobian for the initial system.

For a revolute joint: - linear component = z\_(i-1) × (p_end -
p\_(i-1)) - angular component = z\_(i-1)

For a prismatic joint: - linear component = z\_(i-1) - angular component
= zero.

The implementation must derive these from the actual frame transforms
rather than hard-coded robot geometry.

## 11. Singularity

For square Jacobians, determinant may be informative but must not be the
sole general test.

Prefer: - matrix rank; - singular values; - condition number where
meaningful; - minimum singular value; - manipulability where dimensions
permit.

Classify: - SAFE; - NEAR_SINGULAR; - SINGULAR; using configurable
thresholds.

Do not present a numerical threshold as a universal physical law. Label
it as an application policy.

## 12. Manipulability

For an appropriate Jacobian:

w = sqrt(det(J J\^T))

If the Jacobian is non-square or the selected measure is inappropriate
for the robot/task dimension, use a documented alternative or mark the
metric unavailable.

## 13. Workspace

Generate workspace by sampling valid joint configurations.

Sampling must respect joint limits.

For each sample: q -\> FK -\> end-effector position.

Store: - sampled points; - bounding box; - approximate maximum reach; -
approximate minimum distance from base where meaningful; - sampling
settings; - timestamp/version.

Workspace is an approximation, not an exact mathematical boundary.

## 14. Solution motion cost

Initial metric:

C_motion = sum_i \|q_i - q_current_i\|

For revolute joints, use shortest angular displacement.

Optionally normalize by each joint's allowed range so heterogeneous
joints are comparable.

## 15. Joint-limit margin

For each joint: margin_i = min(q_i - q_min_i, q_max_i - q_i)

A normalized margin is preferred for scoring.

## 16. Scoring

Use explicit weighted metrics.

Example:

score = w_motion \* normalized_motion + w_singularity \*
normalized_singularity_risk + w_limits \* normalized_limit_risk +
w_trajectory \* normalized_trajectory_cost

Lower cost can represent better suitability.

The UI must expose weights and explain them.

Do not call this AI unless an actual learning system is later
introduced.

## 17. Recommendation

Select the feasible candidate with the lowest configured cost after
applying hard constraints.

Hard constraints must be separated from soft preferences.

Example: - joint limit violation = reject; - position tolerance
violation = reject; - singularity risk = soft or hard depending on
policy; - motion cost = soft; - trajectory time = soft.

## 18. Trajectory

Initial method: - joint-space interpolation; - preferably cubic or
quintic polynomial interpolation.

Input: - start q; - target q; - total duration or derived duration; -
velocity/acceleration limits.

Output: - t; - q(t); - qdot(t); - qddot(t).

## 19. Trajectory validation

Verify: - q remains within limits; - qdot remains within limits; - qddot
remains within limits; - no NaN/Inf; - endpoint position matches
intended configuration within tolerance.

## 20. Playback

Playback samples trajectory at time t and sends the joint state to: -
robot renderer; - numerical state panel; - optional charts.

The renderer must not recompute IK during ordinary playback.

## 21. Target reached verification

At final time: q_final -\> FK -\> T_final.

Compute: - position error; - orientation error if applicable.

Report: - PASS if within target tolerances; - FAIL otherwise.

## 22. Units

All internal calculations use SI: - m; - rad; - s.

UI conversion functions must be centralized.

## 23. Numerical tolerance policy

Define separate tolerances: - position tolerance; - orientation
tolerance; - joint deduplication tolerance; - solver residual
tolerance; - singularity threshold.

Do not scatter magic numbers across the codebase.

## 24. Numerical safety

Every solver result must be checked for: - finite values; - convergence
status; - residual; - joint limits; - FK verification.

Never trust a solver's success flag alone.
