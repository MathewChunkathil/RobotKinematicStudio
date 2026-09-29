# Software Architecture

## 1. Architectural goal

Create a modular desktop application where robotics mathematics is
independent from GUI and visualization.

## 2. Recommended layers

### Domain layer

Contains: - RobotDefinition - JointDefinition - LinkDefinition - Pose -
Target - constraints - domain validation

### Robotics engine

Contains: - DH - transforms - FK - IK - Jacobian - workspace -
singularity - manipulability - scoring - trajectory

### Application layer

Coordinates use cases: - create robot; - update configuration; - solve
target; - select solution; - generate trajectory; - playback; -
save/load.

### Visualization layer

Contains: - scene; - robot geometry; - target; - frames; - workspace; -
trajectories; - overlays.

### UI layer

Contains: - windows; - panels; - forms; - tables; - controls; - charts.

### Persistence layer

Contains: - JSON serialization; - versioning; - experiment storage.

## 3. Dependency rule

Preferred dependency direction:

UI → Application → Robotics/Domain

Visualization may consume domain/application state but must not decide
robotics results.

Persistence may serialize domain models but should not own domain logic.

Domain must not import PySide6 or PyVista.

Robotics engine must not import UI modules.

## 4. Suggested folder structure

robokinematics/ app/ main.py application_controller.py state/ domain/
robot.py joint.py link.py pose.py target.py constraints.py robotics/
dh.py transforms.py forward_kinematics.py inverse_kinematics.py
jacobian.py validation.py workspace.py singularity.py manipulability.py
scoring.py trajectory.py visualization/ scene.py robot_renderer.py
target_renderer.py frame_renderer.py trajectory_renderer.py
workspace_renderer.py overlays.py ui/ main_window.py
robot_configuration_panel.py target_panel.py solution_panel.py
analysis_panel.py playback_panel.py calculation_panel.py
settings_panel.py persistence/ serializers.py repositories.py config/
defaults.py tests/ unit/ integration/ numerical/ visual/ assets/ docs/

## 5. Application controller

The controller coordinates use cases but should not contain formulas.

Example: on_target_changed(target): - update application state; - run
reachability check; - invoke IK service; - validate candidates; -
analyse candidates; - select recommendation; - request trajectory; -
update scene state.

## 6. Event/state model

Recommended events: - robot_changed - joint_changed - target_changed -
solution_set_updated - solution_selected - trajectory_updated -
playback_started - playback_paused - playback_stopped -
visualization_changed - experiment_loaded - experiment_saved

## 7. Caching

Caching is optional but useful for: - repeated FK; - workspace; - IK
results; - trajectory.

Cache keys must include every dependency that affects the result.

Do not cache stale results.

## 8. Error model

Distinguish: - invalid configuration; - invalid target; - unreachable
target; - IK solver failure; - no feasible candidate; - trajectory
constraint violation; - persistence failure; - rendering failure.

User-facing errors must be understandable.

Developer diagnostics should contain detailed technical context.

## 9. Configuration versioning

Saved JSON must include: - schema_version; - application_version.

Future migrations should be possible.

## 10. Threading/performance

Long operations such as: - workspace sampling; - multi-start IK; - large
trajectory calculations

should not block the UI.

Use Qt-compatible background workers where necessary.

Do not introduce concurrency before correctness is established.

## 11. Visualization architecture

The scene should have independent renderers: - robot; - target; -
frames; - paths; - workspace; - analysis overlays.

The scene receives a complete visualization state and updates only
necessary components.

## 12. Renderer geometry

Visual geometry should be generated from primitives: - boxes; -
cylinders; - rounded/capped shapes where practical; - spheres; - custom
end-effector primitives.

Do not require Blender for the base product.

## 13. Camera

Provide: - orbit; - pan; - zoom; - fit robot; - fit target; - reset
view; - standard views where practical.

## 14. Styling

Use a restrained engineering aesthetic: - neutral scene background; -
one primary accent; - clear active/ghost distinction; - consistent
typography; - minimal decorative elements.

Avoid neon-heavy "sci-fi dashboard" styling.

## 15. Performance target

For the reference robot: - target movement should feel interactive; -
ordinary playback should be smooth; - scene updates should not rebuild
all geometry unnecessarily; - ghost paths should use lightweight
geometry.

Exact FPS target should be established after a prototype benchmark
rather than guessed.

## 16. Testing boundaries

Each robotics module should be unit-testable without the GUI.

A full integration test should exercise: configuration → FK → target →
IK → validation → scoring → trajectory.
