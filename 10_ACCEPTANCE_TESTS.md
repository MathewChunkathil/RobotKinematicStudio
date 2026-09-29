# End-to-End Acceptance Tests

## AT-001 Create reference robot

Expected: - application opens; - reference robot loads; - robot appears
in 3D; - joints and links have visible volume.

## AT-002 Create custom robot

Action: - create a new serial robot; - choose a different joint count; -
assign joint types; - enter DH parameters; - set limits.

Expected: - configuration validates; - 3D robot regenerates.

## AT-003 Edit DH parameter

Action: - change one link length.

Expected: - mathematical model changes; - rendered link changes; - FK
result changes.

## AT-004 Edit visual thickness

Action: - increase link width/height.

Expected: - visual appearance changes; - FK result remains unchanged.

This explicitly proves mathematical/visual separation.

## AT-005 FK

Action: - set known joint configuration.

Expected: - intermediate frames display correctly; - end-effector pose
matches reference values.

## AT-006 Point target

Action: - add a point target inside the workspace.

Expected: - target appears; - system reports potential reachability; -
IK can be requested.

## AT-007 Pose target

Action: - switch to pose target; - specify orientation.

Expected: - orientation is included in IK validation where supported.

## AT-008 Multiple IK

Action: - use a target known to admit multiple configurations.

Expected: - multiple candidates are found; - each candidate is
FK-verified; - duplicates are removed.

## AT-009 Invalid IK

Action: - use target outside workspace.

Expected: - no false valid solution; - clear message.

## AT-010 Joint-limit rejection

Action: - create candidate requiring an invalid joint value.

Expected: - candidate is rejected.

## AT-011 Solution analysis

Expected: - each feasible candidate has metrics.

## AT-012 Recommendation

Expected: - one feasible candidate is selected according to configured
policy; - explanation/metrics are visible.

## AT-013 Ghost solutions

Expected: - non-selected feasible solutions appear as ghost
configurations and/or paths; - active solution is visually distinct.

## AT-014 Select ghost

Action: - click a ghost solution.

Expected: - ghost becomes active; - previous active becomes
alternative; - numerical panels update; - trajectory updates.

## AT-015 Playback

Action: - press play.

Expected: - robot animates smoothly; - joint values update; - target
remains visible; - trajectory is displayed.

## AT-016 Final verification

Expected: - final FK is calculated; - position error is reported; -
orientation error is reported where applicable; - target reached status
is accurate.

## AT-017 Visualization layers

Toggle: - frames; - axes; - DH values; - workspace; - velocity; -
acceleration; - trajectories.

Expected: - each layer changes only visualization; - calculations remain
unchanged.

## AT-018 Joint inspection

Action: - click a joint.

Expected: - joint type; - DH values; - limits; - current state; -
frame/axis data appear.

## AT-019 Target inspection

Action: - click target.

Expected: - target coordinates; - mode; - orientation; - reachability; -
active solution appear.

## AT-020 Calculation explorer

Expected: - user can step through: target → DH → transforms → FK → IK →
validation → Jacobian → scoring → trajectory.

## AT-021 Save/load robot

Expected: - custom robot can be saved; - application can restart/load
it; - configuration matches original.

## AT-022 Save/load experiment

Expected: - robot; - target; - solution set; - selected solution; -
trajectory; are restored or deterministically regenerated.

## AT-023 State invalidation

Action: - modify target after generating solutions.

Expected: - old solution/trajectory is not presented as current.

## AT-024 Similar custom robot

Action: - create another robot with similar serial kinematic structure
but different lengths.

Expected: - generic FK/IK architecture works without source-code
changes; - numerical results differ appropriately.

## AT-025 Invalid input

Action: - enter malformed or physically invalid configuration.

Expected: - clear validation error; - application remains usable.
