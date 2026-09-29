# RoboKinematics Studio — Agent Rules

Read the specification files in the repository root before substantial work.

## Non-negotiable rules

1. Custom robot configuration is a core requirement.
2. The reference 4-DOF robot is configuration data, not hard-coded architecture.
3. Internal units are SI: metres, radians, seconds.
4. Mathematical and visual models are separate.
5. Never fabricate IK solutions.
6. Every feasible IK candidate must be FK-verified.
7. Never put robotics mathematics inside UI widgets.
8. Never let visual link thickness alter DH mathematics.
9. Never scatter numerical tolerances through the code.
10. Do not add ROS, dynamics, hardware, computer vision, AI IK, or closed-chain mechanisms without an explicit scope change.
11. Add tests with robotics features.
12. Run relevant tests before reporting completion.
13. Do not hide failures.
14. Preserve existing public contracts unless the specification is intentionally revised.

## Working style

Before a major change:
- inspect existing code;
- identify dependencies;
- propose a bounded implementation;
- implement;
- test;
- report files changed and verification results.

Prefer small, reviewable changes.
