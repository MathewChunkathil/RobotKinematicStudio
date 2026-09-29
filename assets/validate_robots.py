# Quick validation script for all example robot configurations.
import sys; sys.stdout.reconfigure(encoding='utf-8')
from robokinematics.persistence.serializers import load_model
from robokinematics.domain.models import RobotDefinition
from robokinematics.robotics.forward_kinematics import forward_kinematics
from pathlib import Path
import numpy as np

robots_dir = Path("assets/robots")
print("=" * 60)
print("Example Robot Configurations — FK Validation")
print("=" * 60)

for f in sorted(robots_dir.glob("*.json")):
    robot = load_model(RobotDefinition, f)
    q0 = [0.0] * len(robot.joints)
    tfs = forward_kinematics(robot, q0)
    ee = tfs[-1][:3, 3]
    reach = float(np.linalg.norm(ee))
    types = " / ".join(j.type.value for j in robot.joints)
    print(f"\n  {robot.name}")
    print(f"  {'-'*40}")
    print(f"    File   : {f.name}")
    print(f"    DOF    : {len(robot.joints)}")
    print(f"    Types  : {types}")
    print(f"    EE @q=0: [{ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}]  reach={reach:.3f}m")
    print(f"    Status : OK")

print()
print("=" * 60)
print("All robots validated successfully.")
