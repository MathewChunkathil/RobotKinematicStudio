# RoboKinematics Studio --- Master Project Specification

## 1. Project identity

**Working product name:** RoboKinematics Studio

**Full project title:**\
**Configurable Robotic Manipulator with DH-Based Kinematic Analysis,
Multi-Solution Inverse Kinematics, and Interactive 3D Motion
Simulation**

## 2. Product vision

Build a polished desktop engineering application for configuring,
analysing, visualising, and simulating serial robotic manipulators.

The product must not be a collection of disconnected robotics
demonstrations. It must behave like one coherent engineering tool:

1.  Create/configure a serial manipulator.
2.  Generate its mathematical and visual robot model.
3.  Define or edit DH parameters.
4.  Inspect coordinate frames and transformations.
5.  Calculate forward kinematics.
6.  Place a target point or target pose in 3D.
7.  Determine reachability.
8.  Calculate multiple feasible inverse-kinematic configurations where
    mathematically possible.
9.  Validate every candidate using forward-kinematic verification and
    constraints.
10. Analyse candidates using joint motion, limits, singularity risk,
    manipulability, and trajectory-related metrics.
11. Recommend one solution according to a transparent configurable
    scoring policy.
12. Display other feasible solutions as ghost
    configurations/trajectories.
13. Allow the user to select a ghost solution and make it active.
14. Generate a smooth trajectory.
15. Play back the robot motion until it reaches the target.
16. Verify the final error.
17. Allow the user to inspect all important mathematical and engineering
    details through visualization layers and calculation panels.
18. Save and reload robot configurations and experiments.

## 3. Core philosophy

### Mathematical model vs visual model

The mathematical robot and visual robot are separate representations.

The mathematical model is responsible for: - DH parameters -
transformations - FK - IK - Jacobian - workspace - singularity
analysis - trajectory mathematics - validation

The visual model is responsible for: - link thickness - joint housings -
base geometry - end-effector appearance - materials - lighting -
camera - rendering - animation

Visual thickness must never silently alter the kinematic mathematics.

### Product quality principle

Every important visual result must correspond to a real mathematical
result.

Examples: - A ghost robot must represent a real validated IK solution. -
A ghost path must represent a real generated trajectory. - "Target
reached" must be based on measured FK error. - A recommendation must
come from explicit scoring criteria. - A workspace visualization must
come from actual robot configurations. - Displayed joint values must
come from application state.

## 4. Reference robot

The initial reference/test robot is a 4-DOF serial revolute manipulator.

This is NOT the hard-coded identity of the product.

The product must support custom serial manipulator configurations
through data-driven definitions.

The reference robot exists to: - provide a known validation case; - make
development manageable; - provide demonstration scenarios; - allow
analytical IK if a suitable geometry is selected.

## 5. Configuration philosophy

Custom configuration is a mandatory core requirement.

Users must be able to define: - number of joints; - joint type; - DH
parameters; - joint position limits; - velocity limits; - acceleration
limits; - link dimensions; - joint visual dimensions; - base
transform; - end-effector/tool transform; - tool geometry; - solver
settings; - scoring weights.

At minimum support: - revolute joints; - prismatic joints.

The architecture must be N-DOF rather than hard-coded for four joints.

## 6. Target philosophy

Two target modes are required:

### Point target

Only end-effector position is constrained:

P_target = \[x, y, z\]

The system attempts to satisfy positional tolerance.

### Pose target

End-effector position and orientation are constrained:

T_target = \[R p; 0 1\]

Orientation can initially be represented through rotation matrices and a
user-friendly RPY interface. Internal orientation handling must be
numerically robust.

The UI must clearly distinguish point and pose targets.

## 7. Target workflow

The user can: - add a target object; - move it numerically; - move it
interactively in 3D; - optionally rotate it for pose mode.

When the target changes:

Target → reachability pre-check → IK → candidate validation → solution
analysis → recommendation → trajectory generation → visualization
update.

The system must never silently use stale solutions after the target or
robot configuration changes.

## 8. Multiple IK solution philosophy

The application must not fabricate alternatives.

Every candidate must be verified by FK.

A candidate is only a feasible solution if: - numerical
solver/analytical solver reports convergence; - joint limits are
satisfied; - position error is within configured tolerance; -
orientation error is within configured tolerance when orientation is
constrained; - required solver validity checks pass.

For arbitrary custom serial manipulators, numerical IK is the general
fallback.

Analytical IK can be provided for supported robot structures where a
correct derivation exists.

The application must explicitly indicate which IK method was used.

## 9. Recommendation philosophy

The system does not claim a universal mathematical "best" solution
unless the optimization objective and constraints establish one.

It provides a **recommended solution according to a configured selection
policy**.

Candidate metrics can include: - joint motion cost; - joint-limit
margin; - singularity risk; - manipulability; - trajectory duration; -
maximum velocity; - maximum acceleration; - optional future collision
cost.

The scoring policy must be transparent and inspectable.

## 10. Ghost solution concept

The recommended solution is visually active.

Other feasible solutions appear as: - translucent robot
configurations; - translucent trajectory paths; - selectable solution
cards.

Clicking a ghost solution: - changes the selected solution; - changes
visual emphasis; - regenerates or retrieves its trajectory; - updates
numerical panels; - makes the selected solution active for playback.

This is one of the product's signature interactions.

## 11. Playback

Playback must show the selected trajectory from the current robot
configuration to the target configuration.

Controls: - play; - pause; - stop; - restart; - step; - timeline
scrub; - playback speed.

During playback: - the active robot is animated; - active trajectory
remains visible; - ghost alternatives can remain visible but visually
subdued; - current joint values update; - elapsed time updates; -
optional velocity/acceleration displays update.

At completion, the application verifies the final pose error.

## 12. Visualization philosophy

The 3D scene is the visual hero of the product.

The robot should have visible volume: - links are solids, not lines; -
joints have visible housings; - base has geometry; - end effector has
geometry; - target has a clear 3D marker.

The scene should look like a deliberately designed engineering product,
not a raw plotting window.

However, the application must avoid visual clutter.

Information is divided into: - 3D overlays for spatial information; -
panels/tables for numerical information; - charts for time-dependent
information.

## 13. Visualization layers

The user must be able to toggle:

### Core

-   robot geometry;
-   target;
-   active trajectory;
-   ghost trajectories.

### Kinematic

-   world frame;
-   base frame;
-   joint frames;
-   end-effector frame;
-   joint axes;
-   DH parameters;
-   joint values.

### Workspace/analysis

-   workspace point cloud;
-   workspace boundary;
-   reachability status;
-   singularity indicators;
-   manipulability indicators.

### Motion

-   velocity vectors;
-   acceleration vectors;
-   current path position;
-   trajectory timing.

The application must not force every layer to be visible simultaneously.

## 14. Interaction model

Clicking a visual object should reveal relevant information.

Click joint: - joint ID; - joint type; - DH values; - current value; -
limits; - velocity limit; - acceleration limit; - axis; - frame; -
connected links.

Click target: - target type; - position; - orientation if applicable; -
reachability; - selected solution; - final error when available.

Click solution/path: - solution ID; - joint values; - errors; -
singularity metrics; - motion cost; - score; - trajectory information.

## 15. Units

Internal calculations use SI conventions: - distance: metres; - angle:
radians; - linear velocity: m/s; - angular velocity: rad/s; - linear
acceleration: m/s²; - angular acceleration: rad/s².

The UI may display: - mm/cm/m; - degrees; - mm/s or m/s; - degrees/s; -
degrees/s².

Conversions must occur at the boundary between UI/data entry and
internal calculations.

## 16. Scope boundary

The first production-quality release does NOT require: - full rigid-body
dynamics; - torque control; - motor electrical models; - ROS/ROS 2; -
physical hardware; - computer vision; - reinforcement learning; -
AI-based IK; - arbitrary closed-chain mechanisms; - humanoid robots; -
mobile robots; - industrial PLC integration; - full research-grade
collision planning.

These may be future extensions.

## 17. Core technology stack

-   Python 3.12+ (prefer Python 3.13 if dependency compatibility is
    confirmed)
-   NumPy
-   SciPy
-   PySide6 / Qt
-   PyVista + VTK
-   PyQtGraph
-   Pydantic
-   JSON
-   pytest
-   Ruff
-   Pyright or mypy
-   Git
-   Google Antigravity for agent-assisted development

## 18. Architecture rule

GUI code must not contain robotics mathematics.

Preferred flow:

GUI → application/controller layer → domain/robotics services → typed
result objects → GUI rendering.

Visualization must consume state/results rather than calculate robotics
itself.

## 19. Engineering standard

The project is complete only when: - mathematical tests pass; - known
reference cases pass; - configuration changes regenerate the model
correctly; - invalid configurations are rejected clearly; - target
changes invalidate/recalculate dependent results; - multiple IK
candidates are independently verified; - trajectory constraints are
checked; - playback and final target verification work; - visualization
layers reflect actual state; - save/load reproduces the configuration
and experiment; - the application can recover gracefully from invalid
user input.

## 20. Definition of success

A reviewer should be able to: 1. create a custom robot; 2. see it
generated in 3D; 3. edit its DH parameters; 4. see the robot update; 5.
place a target; 6. obtain multiple feasible configurations when
available; 7. see the recommended configuration and ghost alternatives;
8. click an alternative; 9. play the resulting motion; 10. watch the end
effector reach the target; 11. inspect joint values, frames,
Jacobian/singularity data, trajectory data, and calculations; 12. save
and reload the experiment.

That complete workflow is the project's primary acceptance
demonstration.
