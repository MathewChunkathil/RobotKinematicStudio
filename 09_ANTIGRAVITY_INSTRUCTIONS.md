# Antigravity Development Instructions

## 1. Mission

You are an engineering agent working on RoboKinematics Studio.

The repository specification files are the source of truth.

Do not invent a different product direction.

Read: 1. 00_MASTER_SPEC.md 2. 01_REQUIREMENTS.md 3. 02_ROBOTICS_SPEC.md
4. 03_DATA_MODEL.md 5. 04_ARCHITECTURE.md 6. 05_VISUAL_UX_SPEC.md 7.
06_TECH_STACK.md 8. 07_TEST_PLAN.md 9. 08_DEVELOPMENT_PLAN.md 10.
10_ACCEPTANCE_TESTS.md 11. 11_OUT_OF_SCOPE.md

before substantial implementation work.

## 2. General behavior

Before implementing a large feature: - inspect existing code; - identify
affected modules; - explain the implementation plan; - identify tests; -
implement the smallest coherent change; - run tests; - report failures
honestly.

Do not make unrelated refactors.

## 3. Mathematics rule

Never guess robotics equations.

For: - DH; - FK; - IK; - Jacobian; - singularity; - trajectory;

consult the robotics specification and verify against
independent/reference calculations.

If a mathematical ambiguity is found, stop and flag it rather than
silently choosing.

## 4. IK rule

Never fabricate multiple solutions.

Every candidate must be FK-verified.

Candidates that fail constraints are not feasible solutions.

If only one valid solution exists, report one solution honestly.

## 5. Configuration rule

Never hard-code the reference robot into generic robotics functions.

The reference robot must be represented as configuration data.

Generic code must operate on RobotDefinition.

## 6. Visual rule

The renderer consumes calculated state.

It must not: - calculate IK; - choose solutions; - modify robot
mathematics; - alter DH parameters.

The renderer translates domain state into geometry.

## 7. Geometry rule

Visual link thickness, joint radius, base size, and material must be
separate from kinematic parameters.

Do not modify a DH parameter merely to make a mesh look thicker.

## 8. UI rule

Do not put mathematical calculations directly into Qt widgets.

Widgets call application services/controllers.

## 9. State invalidation rule

When dependencies change: - invalidate stale results; - never display
results that belong to an older configuration.

## 10. Units rule

Internal calculations: - metres; - radians; - seconds.

Centralize conversions.

Do not mix degrees and radians in internal robotics code.

## 11. Numerical rule

Do not scatter tolerances.

Use named configuration values.

Check finite values before rendering or storing solver outputs.

## 12. Testing rule

Every robotics function added should have tests.

For numerical functions, include at least one independently verified
reference case where practical.

## 13. Visual verification rule

After major visualization changes: - launch the application; - exercise
the relevant workflow; - inspect the scene; - verify labels/overlays; -
verify ghost/active distinction; - verify playback.

If a screenshot/artifact can help document the result, produce it.

## 14. Dependency rule

Do not add a dependency without documenting: - why it is needed; - why
existing stack is insufficient; - maintenance implications.

## 15. Code quality

Use: - type hints; - clear names; - small functions; - explicit error
handling; - no dead code; - no debug prints in production paths.

## 16. Scope discipline

Do not implement: - ROS; - dynamics; - AI IK; - computer vision; -
advanced collision planning; - closed-chain mechanisms; unless the
project owner explicitly changes scope.

## 17. Completion report

For each completed task report: - files changed; - what was
implemented; - tests run; - test results; - known limitations; - next
recommended task.

Never report "done" when tests are failing unless explicitly explaining
the failure.

## 18. Prompting pattern

Good request:

"Implement Phase 3 FK according to 02_ROBOTICS_SPEC.md. Do not modify
GUI or visualization. Add unit tests and run them."

Bad request:

"Build the robot simulator."

The agent must work in bounded increments.
