# Explicitly Out of Scope for Initial Release

This document prevents scope creep.

## Not required

### 1. Full rigid-body dynamics

Do not implement: - complete mass matrix derivation; - Coriolis
matrix; - friction models; - motor torque; - actuator electrical models.

### 2. Real hardware

No Arduino/ESP32/servo integration is required for the initial release.

### 3. ROS/ROS 2

No ROS dependency.

### 4. Computer vision

No camera-based target detection.

### 5. AI-based robotics

No machine-learning IK or reinforcement learning.

### 6. Closed-chain robots

No Stewart platform, parallel mechanism, delta robot, or arbitrary
closed kinematic chain.

### 7. Humanoids/mobile robots

The system is for serial manipulators.

### 8. Advanced collision planning

Basic geometry awareness may be considered later, but a research-grade
collision planner is out of scope.

### 9. Arbitrary analytical IK

Do not promise analytical IK for every custom robot.

General custom robots use numerical IK.

### 10. Infinite configurability

Support configurable serial DH-based chains, not every possible robotics
topology.

### 11. Cloud backend

No cloud server is required.

### 12. Multi-user collaboration

Not required.

### 13. Mobile application

Not required.

### 14. Web application

Not required for the initial product.

### 15. Game-engine migration

Do not move the application to Unity/Unreal solely for graphics.

### 16. Perfect physical realism

The 3D model is a visualization representation, not a CAD-grade
manufacturing model.

## Why these are excluded

The central research/engineering value is:

custom configuration → DH model → FK → target → multi-solution IK →
validation → analysis → recommendation → trajectory → interactive 3D
simulation → transparent calculations.

Adding unrelated subsystems would increase development risk without
strengthening the central demonstration proportionally.
