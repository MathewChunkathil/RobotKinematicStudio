# Development Plan

## Phase 0 --- Repository and specification

Deliver: - project repository; - pyproject.toml; - src layout; - test
structure; - all specification files; - AGENTS.md; - development
environment.

No complex UI.

Exit criteria: - project installs; - test runner works; - lint/type
checks work.

## Phase 1 --- Domain data model

Implement: - RobotDefinition; - JointDefinition; -
LinkVisualDefinition; - DHParameters; - JointLimits; - Pose; - Target; -
RobotState; - experiment models.

Exit: - models validate; - JSON round-trip tests pass.

## Phase 2 --- Mathematical foundation

Implement: - units; - rotations; - transforms; - standard DH; - frame
utilities.

Exit: - reference mathematical tests pass.

## Phase 3 --- Robot engine

Implement: - robot chain; - joint state; - FK; - intermediate
transforms.

Exit: - known FK reference cases pass.

## Phase 4 --- Reference robot

Create reference 4-DOF robot as JSON/configuration data.

Do not hard-code it into the engine.

Exit: - reference robot loads; - 3D-free FK test works.

## Phase 5 --- General numerical IK

Implement solver interface.

Implement robust numerical IK.

Implement multi-start.

Implement candidate normalization/deduplication.

Exit: - known reachable targets produce verified candidates; -
unreachable targets fail cleanly.

## Phase 6 --- Validation

Implement: - joint limits; - pose error; - solver residual; - candidate
validation.

Exit: - invalid candidates rejected.

## Phase 7 --- Jacobian and analysis

Implement: - geometric Jacobian; - finite-difference verification; -
singularity metrics; - manipulability; - joint-limit margins; - motion
cost.

Exit: - numerical tests pass.

## Phase 8 --- Solution scoring and selection

Implement: - metric normalization; - configurable weights; - hard
constraints; - recommendation; - alternative list.

Exit: - deterministic recommendation for fixed input.

## Phase 9 --- Workspace

Implement sampled workspace.

Exit: - reference workspace renders from actual samples; - configuration
changes affect workspace.

## Phase 10 --- Trajectory

Implement: - joint-space cubic/quintic trajectory; -
velocity/acceleration; - constraint validation.

Exit: - trajectory starts/ends exactly at intended configurations within
tolerance.

## Phase 11 --- 3D renderer prototype

Implement: - scene; - base; - links; - joints; - end effector; -
frames; - target.

Exit: - reference robot renders correctly.

## Phase 12 --- Dynamic visualization

Implement: - target; - active path; - ghost paths; - ghost
configurations; - selectable objects; - overlays.

Exit: - selected solution is visibly distinct; - ghost alternatives are
clickable.

## Phase 13 --- Playback

Implement: - timeline; - play/pause/stop; - speed; - trajectory
animation; - final verification.

Exit: - robot reaches target and reports measured error.

## Phase 14 --- Main GUI

Implement: - robot configuration panel; - target panel; - solution
panel; - analysis panel; - playback panel; - inspector; - visualization
controls.

Exit: - complete workflow can be performed without source-code edits.

## Phase 15 --- Calculation explorer

Implement: - target; - DH; - transforms; - FK; - IK; - validation; -
Jacobian; - scoring; - trajectory steps.

Exit: - user can inspect important intermediate calculations.

## Phase 16 --- Persistence

Implement: - save robot; - load robot; - save experiment; - load
experiment.

Exit: - round-trip integration test passes.

## Phase 17 --- Polish

Improve: - geometry; - lighting; - materials; - camera; - transitions; -
spacing; - typography; - error states; - tooltips; - performance.

Do not add major new functionality in this phase.

## Phase 18 --- Final verification

Run: - all unit tests; - integration tests; - numerical reference
tests; - manual visual tests; - end-to-end acceptance workflow.

Freeze feature scope.

## Development rule

Never implement more than one major subsystem without tests.

Never let UI development become a substitute for validating mathematics.

Never accept an agent claim of correctness without running the relevant
tests.
