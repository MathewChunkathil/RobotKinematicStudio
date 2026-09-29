# Visual Design and UX Specification

## 1. Product personality

The application should feel: - technical; - calm; - precise; - modern; -
premium; - engineering-focused.

It should NOT feel: - like a raw scientific notebook; - like a gaming
HUD; - like a generic CRUD admin dashboard.

## 2. Primary layout

Recommended: - left sidebar for robot/configuration/analysis controls; -
large central 3D viewport; - right contextual inspector or collapsible
details panel; - bottom playback/timeline area.

Panels should be resizable or collapsible.

## 3. 3D viewport

The 3D viewport is the dominant visual element.

It should support: - orbit; - pan; - zoom; - reset camera; - fit
robot; - fit target; - selectable objects; - target manipulation where
available.

## 4. Robot visual geometry

Links: - solid volumetric beams; - consistent proportions; -
rounded/capped appearance where practical.

Joints: - visible cylindrical or mechanical housings; - visually
distinguish joint centers/axes.

Base: - stable pedestal/base plate.

End effector: - clear tool body; - visually distinct TCP.

The geometry must be automatically generated from configuration.

## 5. Active vs ghost

Active: - opaque; - visually prominent; - active trajectory emphasized.

Ghost: - translucent; - visually subordinate; - still selectable.

Do not make ghost paths so faint that they cannot be understood.

## 6. Target visual

Point target: - sphere/crosshair/marker.

Pose target: - marker plus orientation triad.

Target should remain visible during playback.

## 7. Frames

Frame display: - X/Y/Z axes; - labels; - optional frame ID.

Frames should be toggled independently.

## 8. Joint axes

Joint axes should be represented in 3D and optionally labeled.

For revolute joints: - rotation axis.

For prismatic joints: - translation axis.

## 9. Visualization layer controls

Required toggles: - robot; - target; - active trajectory; - ghost
trajectories; - world frame; - joint frames; - joint axes; - DH
information; - workspace; - singularity; - velocity vectors; -
acceleration vectors.

## 10. Context inspector

Clicking a joint should show: - ID; - type; - DH parameters; - current
value; - limits; - velocity; - acceleration; - frame; - connected links.

Clicking target should show: - target mode; - position; - orientation; -
reachability; - active solution; - final error.

Clicking a solution/path should show: - ID; - joint values; - pose
errors; - motion cost; - joint margin; - singularity; -
manipulability; - score.

## 11. Robot configuration UX

Provide a table editor.

Columns: - joint; - type; - theta; - d; - a; - alpha; - position min; -
position max; - velocity max; - acceleration max.

A separate visual geometry section should contain: - link length; -
width; - height; - joint radius; - base dimensions; - end-effector
dimensions.

The UI must clearly distinguish: - mathematical DH values; -
physical/visual dimensions.

## 12. Target placement UX

Support: - numerical position; - numerical orientation in pose mode; -
3D manipulation if practical.

Always provide a numerical fallback.

## 13. Solution panel

Each solution card: - solution ID; - validity; - joint values; -
position error; - orientation error; - score; - recommendation status.

Selected solution is highlighted.

Ghost solutions are clearly labeled "Alternative".

## 14. Calculation explorer

Navigation: 1. Target definition 2. Robot configuration 3. DH table 4.
Frame assignment 5. Individual transforms 6. Final FK 7. IK method 8. IK
candidates 9. Candidate validation 10. Jacobian 11. Singularity analysis
12. Solution metrics 13. Scoring 14. Selected solution 15. Trajectory
16. Playback/final verification

Each step should show: - explanation; - formula; - substituted values; -
result; - optional visualization link/highlight.

## 15. Charts

Use PyQtGraph for: - joint position vs time; - velocity vs time; -
acceleration vs time; - tracking error vs time.

Charts must support selecting a joint.

## 16. Status communication

The UI should always communicate system state: - READY - CONFIGURATION
INVALID - TARGET UNREACHABLE - SOLVING - SOLUTIONS FOUND - NO FEASIBLE
SOLUTION - READY FOR PLAYBACK - PLAYING - TARGET REACHED - TARGET ERROR

Do not hide important state inside logs.

## 17. Empty/error states

Examples:

No target: "Add a target to begin IK analysis."

Target unreachable: "No feasible configuration was found. The target may
be outside the reachable workspace or violate joint constraints."

No multiple solutions: "One feasible configuration was found for the
current target and solver settings."

This is better than pretending every target must have several solutions.

## 18. Accessibility

-   keyboard-accessible controls;
-   numerical alternatives to drag operations;
-   readable contrast;
-   clear focus states;
-   no information available only through hover;
-   tooltips only as supplementary information.

## 19. Visual quality acceptance

A reviewer should be able to say: - robot has believable volume; -
joints are visually distinct; - target is obvious; - active path is
obvious; - ghost paths are understandable; - frames/axes are legible; -
UI does not overwhelm the 3D scene; - numerical panels are readable; -
playback looks smooth.
