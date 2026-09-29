import numpy as np
import pytest

from robokinematics.domain.models import (
    DHParameters,
    JointDefinition,
    JointLimits,
    LinkDefinition,
    LinkVisualDefinition,
    RobotDefinition,
)
from robokinematics.robotics.forward_kinematics import forward_kinematics
from robokinematics.robotics.transforms import standard_dh_transform


def create_simple_robot() -> RobotDefinition:
    # 2-DOF planar robot
    return RobotDefinition(
        id="test_robot",
        name="Test Robot",
        joints=[
            JointDefinition(
                id="j0",
                index=0,
                name="Joint 0",
                type="revolute",
                dh=DHParameters(theta=0, d=0, a=1.0, alpha=0),
                limits=JointLimits(position_min=-np.pi, position_max=np.pi),
            ),
            JointDefinition(
                id="j1",
                index=1,
                name="Joint 1",
                type="revolute",
                dh=DHParameters(theta=0, d=0, a=1.0, alpha=0),
                limits=JointLimits(position_min=-np.pi, position_max=np.pi),
            ),
        ],
        links=[
            LinkDefinition(id="l0", index=0, name="Link 0", visual=LinkVisualDefinition()),
            LinkDefinition(id="l1", index=1, name="Link 1", visual=LinkVisualDefinition()),
        ],
        base_translation=[1.0, 0.0, 0.0],
        base_rotation=np.eye(3).tolist(),
        end_effector_translation=[0.5, 0.0, 0.0],
        end_effector_rotation=np.eye(3).tolist(),
    )


def test_forward_kinematics_simple():
    robot = create_simple_robot()
    
    # FK for [0, 0]
    transforms = forward_kinematics(robot, [0.0, 0.0])
    
    # We expect:
    # Base transform
    # Joint 0 transform (relative to base)
    # Joint 1 transform (relative to joint 0)
    # End-effector transform (relative to joint 1)
    
    # Check number of transforms
    assert len(transforms) == 4, "Expected base + 2 joints + end effector"
    
    # Base transform
    base_tf = transforms[0]
    assert np.allclose(base_tf[:3, 3], [1.0, 0.0, 0.0])
    
    # Joint 0
    # Base is at [1,0,0], link 0 length is 1, so joint 1 should be at [2,0,0]
    j1_tf = transforms[1]
    assert np.allclose(j1_tf[:3, 3], [2.0, 0.0, 0.0])
    
    # Joint 1
    # link 1 length is 1, so end of link 1 is at [3,0,0]
    j2_tf = transforms[2]
    assert np.allclose(j2_tf[:3, 3], [3.0, 0.0, 0.0])
    
    # End effector
    # added 0.5 in translation
    ee_tf = transforms[3]
    assert np.allclose(ee_tf[:3, 3], [3.5, 0.0, 0.0])
