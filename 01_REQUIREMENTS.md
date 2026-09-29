# Functional and Non-Functional Requirements

## 1. Functional requirements

### FR-001 Custom robot creation

The system shall allow creation of a serial manipulator with
configurable joint count.

### FR-002 Joint types

The system shall support revolute and prismatic joints.

### FR-003 DH parameters

Each joint shall expose the four DH parameters: - theta - d - a - alpha

The application shall document whether standard DH or modified DH is
used. The initial implementation shall use one convention consistently
throughout all modules.

### FR-004 Joint constraints

Each joint may define: - position minimum; - position maximum; -
velocity maximum; - acceleration maximum.

### FR-005 Visual geometry

Each link shall have configurable visual dimensions. Each joint shall
have configurable visual dimensions. The renderer shall generate volume
geometry automatically.

### FR-006 Base

The robot shall support configurable base transform and base visual
geometry.

### FR-007 End effector

The robot shall support a configurable tool/end-effector transform and
visual geometry.

### FR-008 Save configuration

A robot configuration shall be serializable to JSON.

### FR-009 Load configuration

A saved configuration shall recreate the mathematical and visual robot.

### FR-010 Forward kinematics

Given a valid robot and joint configuration, the system shall compute
all frame transforms and the end-effector pose.

### FR-011 Intermediate transforms

The system shall retain T_0_1, T_0_2, ..., T_0_n and equivalent local
transforms required for inspection.

### FR-012 Target object

The user shall be able to add one active target to the scene.

### FR-013 Point target

The system shall support position-only targets.

### FR-014 Pose target

The system shall support position plus orientation targets when the
solver/configuration permits.

### FR-015 Target editing

The user shall be able to edit target coordinates numerically.

### FR-016 Interactive target manipulation

The application should provide a 3D manipulation mechanism where
technically practical. A numerical alternative must always exist.

### FR-017 Reachability

The system shall perform a preliminary reachability check and
distinguish: - clearly outside reachable region; - potentially
reachable; - solver failed; - feasible.

### FR-018 Inverse kinematics

The system shall calculate IK candidates for supported configurations.

### FR-019 Multiple candidates

The solver shall attempt to find multiple distinct candidate
configurations.

### FR-020 No fabricated candidates

Every candidate shall be verified by FK.

### FR-021 Validation

Candidates shall be checked against: - position tolerance; - orientation
tolerance where applicable; - joint limits; - solver validity; -
numerical tolerance.

### FR-022 IK method reporting

Results shall identify: - analytical; - numerical; - solver name; -
solver settings; as applicable.

### FR-023 Jacobian

The system shall calculate a Jacobian appropriate to the robot
configuration.

### FR-024 Singularity analysis

The system shall report singular/near-singular status using numerical
metrics.

### FR-025 Workspace

The system shall support sampled workspace generation.

### FR-026 Manipulability

The system should calculate a manipulability measure where
mathematically appropriate.

### FR-027 Solution analysis

Each valid candidate shall have: - joint motion cost; - joint-limit
margin; - singularity metric; - manipulability where available; -
trajectory metrics after trajectory generation.

### FR-028 Solution recommendation

The system shall rank candidates according to a transparent configurable
scoring policy.

### FR-029 Alternative solutions

All feasible non-selected solutions shall remain inspectable.

### FR-030 Ghost rendering

Alternative configurations and/or trajectories shall be rendered as
ghost objects.

### FR-031 Solution switching

Clicking an alternative solution shall make it active.

### FR-032 Trajectory

The system shall generate a time-parameterized joint trajectory.

### FR-033 Velocity and acceleration

The trajectory system shall calculate joint velocity and acceleration.

### FR-034 Constraint checking

The trajectory system shall report whether configured velocity and
acceleration limits are respected.

### FR-035 Playback

The system shall provide play/pause/stop/restart/step/timeline controls.

### FR-036 Target verification

At the end of playback, the system shall compute actual FK and report
final error.

### FR-037 Visualization overlays

The system shall provide toggles for major visualization layers.

### FR-038 Selection inspection

Clicking a joint, target, frame, solution, or trajectory shall expose
relevant information.

### FR-039 Calculation viewer

The system shall expose calculation steps and intermediate results.

### FR-040 Experiment save/load

The system shall save and load: - robot configuration; - target; -
selected solution; - candidate solutions; - analysis results where
reproducible; - trajectory; - visualization preferences where
appropriate.

## 2. Non-functional requirements

### NFR-001 Correctness

Core robotics calculations must be covered by automated tests.

### NFR-002 Determinism

Given identical configuration, target, solver settings, and initial
conditions, results should be reproducible within numerical tolerance.

### NFR-003 Modularity

Robotics mathematics, application state, visualization, UI, persistence,
and testing must remain separate.

### NFR-004 Maintainability

Functions should be small, typed, documented where non-obvious, and
testable.

### NFR-005 Usability

A beginner should be able to configure and simulate the reference robot
without editing source code.

### NFR-006 Visual quality

The 3D scene must use volumetric geometry and deliberate
lighting/material/camera design.

### NFR-007 Robustness

Invalid user inputs must generate clear validation messages instead of
crashes.

### NFR-008 Performance

Interactive target movement should not freeze the application for
ordinary reference configurations.

### NFR-009 Accessibility

Numerical input alternatives must exist for drag-based interactions.

### NFR-010 Traceability

Major calculated results should be traceable to the underlying input
configuration.

## 3. Acceptance workflow

The following end-to-end workflow is mandatory:

1.  Load/create a custom robot.
2.  Edit at least one DH parameter.
3.  Confirm 3D model updates.
4.  Set joint limits.
5.  Place a target.
6.  Run reachability/IK.
7.  Generate candidates.
8.  Verify candidate FK errors.
9.  Display feasible alternatives.
10. Calculate analysis metrics.
11. Select recommended solution.
12. Display ghost alternatives.
13. Click another solution.
14. Generate/select its trajectory.
15. Play motion.
16. Verify target reached.
17. Open calculation viewer.
18. Save experiment.
19. Reload experiment.
20. Confirm equivalent state.
