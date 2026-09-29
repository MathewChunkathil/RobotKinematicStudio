# Technology Stack and Development Standards

## 1. Runtime

### Python

Use Python 3.12 or newer.

Prefer Python 3.13 if all selected dependencies are compatible in the
actual development environment.

Do not upgrade Python solely because a newer release exists.

## 2. Numerical stack

### NumPy

Use for: - vectors; - matrices; - transforms; - numerical array
operations.

### SciPy

Use for: - nonlinear least squares; - numerical optimization; -
interpolation; - numerical linear algebra where appropriate.

Do not reimplement mature numerical routines unnecessarily.

## 3. GUI

### PySide6

Use Qt for: - application window; - layouts; - forms; - tables; -
tabs; - dialogs; - dock panels; - buttons; - menus; - status messages; -
threading integration.

Keep widgets thin.

## 4. 3D visualization

### PyVista + VTK

Use for: - robot geometry; - coordinate frames; - target; - workspace; -
trajectories; - camera; - animation; - selection.

Avoid Blender as a runtime dependency for the core application.

## 5. Plotting

### PyQtGraph

Preferred for interactive time-series charts embedded in the Qt
application.

Matplotlib may be used for offline/export plots if necessary, but avoid
two plotting stacks unless there is a clear reason.

## 6. Data validation

### Pydantic

Use for: - configuration models; - JSON boundaries; - schema
validation; - persistence.

## 7. Persistence

Start with JSON.

No database is required initially.

Schema must include a version.

## 8. Testing

### pytest

Test: - DH transforms; - FK; - IK; - Jacobian; - singularity metrics; -
workspace sampling; - scoring; - trajectory; - serialization.

## 9. Code quality

### Ruff

Use for formatting/linting.

### Pyright

Preferred type checker unless project setup establishes mypy instead.

Use type hints throughout public APIs.

## 10. Packaging

Use a modern Python project layout with pyproject.toml.

Recommended: - src layout; - project dependencies; - development
dependencies; - test configuration; - lint configuration.

## 11. Git

Use: - small commits; - meaningful commit messages; - feature
branches; - no generated build artifacts committed; - no secrets.

## 12. Antigravity

Use Google Antigravity as an agent-assisted IDE.

The agent should: - inspect the repository; - read project
instructions; - produce plans before large changes; - implement one
bounded feature at a time; - run tests; - inspect failures; - avoid
unrelated refactors; - provide verification artifacts when useful.

Do not ask the agent to generate the whole project in one prompt.

## 13. Recommended project files

At repository root: - AGENTS.md - README.md - REQUIREMENTS.md -
ARCHITECTURE.md - ROBOTICS_SPEC.md - DATA_MODEL.md - VISUAL_UX_SPEC.md -
TECH_STACK.md - TEST_PLAN.md - DEVELOPMENT_PLAN.md -
ACCEPTANCE_TESTS.md - OUT_OF_SCOPE.md

## 14. Dependency policy

Avoid adding libraries merely because an agent suggests them.

Every dependency should have: - purpose; - license compatibility; -
maintenance justification; - integration cost; - test coverage.

Prefer the smallest reliable dependency set.

## 15. Security

Do not: - execute arbitrary downloaded code; - accept executable robot
configuration files; - evaluate Python from JSON; - deserialize unsafe
object formats.

JSON must be data-only.

## 16. Performance

Profile before optimizing.

Do not introduce complex caching or multiprocessing prematurely.

Prioritize correctness first.
