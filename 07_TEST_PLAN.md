# Testing and Verification Plan

## 1. Testing philosophy

Robotics software must be verified mathematically, not only visually.

A robot that looks correct but has an incorrect transformation matrix is
a failed implementation.

## 2. Unit tests

### DH tests

Verify: - identity-like cases; - pure translations; - pure rotations; -
known standard DH examples; - degree/radian conversion boundaries.

### Transform tests

Verify: - matrix multiplication; - inverse transforms; - frame
composition; - orthogonality of rotation matrices within tolerance.

### FK tests

For known robot configurations: - compare intermediate transforms; -
compare end-effector position; - compare orientation.

### IK tests

For known reachable targets: - candidate exists; - FK(candidate) reaches
target; - joint limits are respected; - duplicate candidates are
removed.

### Jacobian tests

Use finite differences to compare analytical/geometric Jacobian columns
against numerical derivatives where appropriate.

### Singularity tests

Construct known or near-singular configurations and verify
classification behavior.

### Workspace tests

Verify: - limits are respected; - samples produce finite positions; -
configuration changes alter workspace.

### Scoring tests

Verify: - hard-invalid solutions are excluded; - weights affect ranking
as expected; - normalization behaves sensibly.

### Trajectory tests

Verify: - endpoints; - finite values; - velocity limits; - acceleration
limits; - monotonic time; - smoothness.

### Persistence tests

Save and load: - reference robot; - custom robot; - target; -
experiment.

Verify semantic equivalence.

## 3. Property-style tests

Where practical: - FK of finite valid joint configurations should
produce finite transforms. - Rotation matrices should remain orthonormal
within tolerance. - Forward-transform inverse should recover original
frame within tolerance. - Valid IK candidates should satisfy target
tolerance.

## 4. Integration tests

### Test A --- Reference robot

Configure reference robot → FK → target → IK → validation → scoring →
trajectory.

### Test B --- Custom robot

Create a different serial robot with similar kinematic structure →
verify FK/IK pipeline still works.

### Test C --- Target movement

Change target → verify old IK/trajectory is invalidated and new results
are generated.

### Test D --- Joint limit change

Change limits → verify feasible solution set changes when appropriate.

### Test E --- Solution switching

Select ghost solution → verify active solution ID changes and trajectory
updates.

### Test F --- Playback

Run playback → verify final FK reaches target within tolerance.

## 5. Numerical reference tests

Maintain a small set of hand-verified or independently calculated
reference cases.

Do not rely solely on AI-generated expected values.

## 6. Visual tests

Visual tests should verify: - robot appears; - link dimensions respond
to configuration; - joints appear; - target appears; - frames appear
when enabled; - ghost paths appear; - active path is visually
distinct; - playback updates robot pose.

Visual correctness may initially be verified by screenshots/manual
review.

## 7. Regression testing

Every bug fixed in: - FK; - IK; - Jacobian; - trajectory; - state
invalidation

must receive a regression test where practical.

## 8. Tolerance policy

Tolerances must be centralized.

Never scatter: 1e-6, 1e-8, etc. throughout source files without
explanation.

Use named configuration constants.

## 9. Failure handling tests

Test: - empty robot; - invalid DH values; - reversed joint limits; -
NaN/Inf; - impossible target; - solver failure; - no feasible
solution; - invalid JSON; - old schema version; - trajectory constraint
violation.

The application should remain usable and explain the failure.

## 10. Performance tests

Measure: - target-to-result latency; - workspace generation time; - IK
multi-start time; - scene update time; - playback smoothness.

Do not establish arbitrary performance claims until measurements exist.

## 11. Definition of test completion

A feature is not complete until: - implementation exists; - unit tests
exist where applicable; - integration behavior is checked; - failure
behavior is checked; - documentation is updated.
