"""Forward kinematics service."""

from __future__ import annotations

import numpy as np

from robokinematics.domain.models import RobotDefinition
from robokinematics.robotics.transforms import compose, standard_dh_transform


def forward_kinematics(
    robot: RobotDefinition,
    joint_positions: list[float],
) -> list[np.ndarray]:
    """Return global transforms for base, every robot joint/frame, and end-effector.

    Returns:
        A list of homogenous transform matrices in order:
        [base_transform, joint_1, ..., joint_N, end_effector]
    """
    if len(joint_positions) != len(robot.joints):
        raise ValueError("Joint position count must match robot joint count.")

    # Construct base transform
    base_transform = np.eye(4, dtype=float)
    base_transform[:3, :3] = np.array(robot.base_rotation)
    base_transform[:3, 3] = np.array(robot.base_translation)

    transforms = [base_transform.copy()]
    current = base_transform.copy()

    for joint, q in zip(robot.joints, joint_positions, strict=True):
        theta = joint.dh.theta
        d = joint.dh.d

        if joint.type.value == "revolute":
            theta += q
        else:
            d += q

        local = standard_dh_transform(theta, d, joint.dh.a, joint.dh.alpha)
        current = compose([current, local])
        transforms.append(current.copy())

    # Construct end-effector transform
    ee_local = np.eye(4, dtype=float)
    ee_local[:3, :3] = np.array(robot.end_effector_rotation)
    ee_local[:3, 3] = np.array(robot.end_effector_translation)
    
    current = compose([current, ee_local])
    transforms.append(current.copy())

    return transforms
