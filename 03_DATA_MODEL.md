# Data Model and Contracts

## 1. Design principles

Use typed immutable/value-oriented structures where practical.

Pydantic models are recommended for configuration/persistence
boundaries.

NumPy arrays may be used internally for numerical computation.

Do not expose raw mutable arrays throughout the entire application.

## 2. RobotDefinition

Conceptual fields:

-   id
-   name
-   description
-   units
-   joints\[\]
-   links\[\]
-   base
-   end_effector
-   solver_settings
-   visual_settings

## 3. JointDefinition

Fields: - id - index - name - type: revolute \| prismatic - dh:
DHParameters - limits: JointLimits - dynamics_limits: optional - visual:
JointVisualDefinition

## 4. DHParameters

Fields: - theta - d - a - alpha - variable_parameter: theta or d
depending on joint type

Important: For a revolute joint, theta is variable and d is fixed. For a
prismatic joint, d is variable and theta is fixed.

The representation must make this explicit.

## 5. JointLimits

Fields: - position_min - position_max - velocity_max - acceleration_max

All internal values use SI units.

## 6. LinkVisualDefinition

Fields: - length - width - height - shape - radius where applicable -
material/appearance identifier

Visual fields must not leak into kinematic calculations.

## 7. RobotState

Fields: - joint_positions - joint_velocities - joint_accelerations -
timestamp - source

## 8. Pose

Fields: - position: 3-vector - orientation: rotation representation

Store a canonical internal orientation representation.

## 9. Target

Fields: - id - name - mode: point \| pose - pose - position_tolerance -
orientation_tolerance - visual_definition

## 10. TransformResult

Fields: - local_transforms\[\] - global_transforms\[\] -
end_effector_pose - metadata

## 11. IKCandidate

Fields: - id - joint_positions - method - solver_status - residual -
achieved_pose - position_error - orientation_error - valid

## 12. SolutionAnalysis

Fields: - solution_id - motion_cost - normalized_motion_cost -
joint_limit_margin - normalized_limit_risk - singularity_status -
singularity_metric - manipulability - trajectory_cost - score -
scoring_policy_id

## 13. SolutionSet

Fields: - target_id - candidates\[\] - feasible_candidates\[\] -
recommended_solution_id - selected_solution_id - generated_at -
solver_metadata

## 14. Trajectory

Fields: - trajectory_id - start_configuration - target_configuration -
duration - timestamps - joint_positions - joint_velocities -
joint_accelerations - validation_report

## 15. Experiment

Fields: - experiment_id - robot_definition - initial_state - target -
solution_set - selected_solution - trajectory -
visualization_preferences - application_version

## 16. CalculationStep

Fields: - id - title - category - explanation - formula -
substituted_values - result - related_entity_id - visualization_hint

## 17. VisualizationState

Fields: - show_robot - show_target - show_active_path -
show_ghost_paths - show_world_frame - show_joint_frames -
show_joint_axes - show_dh_values - show_workspace - show_singularity -
show_velocity_vectors - show_acceleration_vectors

## 18. Contracts

### FK contract

Input: - RobotDefinition - JointConfiguration

Output: - TransformResult

### IK contract

Input: - RobotDefinition - Target - IKSettings - optional initial state

Output: - SolutionSet or raw IKResult before validation.

### Validation contract

Input: - RobotDefinition - Target - IKCandidate\[\]

Output: - validated IKCandidate\[\]

### Analysis contract

Input: - RobotDefinition - RobotState - Target - validated candidates -
ScoringPolicy

Output: - SolutionAnalysis\[\]

### Selection contract

Input: - validated candidates - analyses - SelectionPolicy

Output: - selected solution ID - ordered feasible solutions

### Trajectory contract

Input: - RobotDefinition - RobotState - selected IK solution -
TrajectorySettings

Output: - Trajectory

### Rendering contract

Input: - RobotDefinition - RobotState - Target - SolutionSet -
Trajectory - VisualizationState

Output: - rendered scene only; no robotics decisions.

## 19. State invalidation

Changes must invalidate dependent results.

Examples:

Robot configuration changed: → invalidate FK, IK, workspace,
trajectories, analyses.

Joint limits changed: → invalidate validation, analysis, recommendation,
trajectory.

Target changed: → invalidate IK, validation, analysis, recommendation,
trajectory.

Scoring weights changed: → invalidate recommendation only, not IK.

Visualization preference changed: → invalidate render state only.

Playback time changed: → update displayed state only.

This dependency model is mandatory.
