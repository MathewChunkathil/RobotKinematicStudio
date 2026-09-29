from pathlib import Path

from robokinematics.domain.models import RobotDefinition
from robokinematics.persistence.serializers import load_model, save_model


def test_reference_robot_loads():
    # Load the reference 4dof robot
    robot_path = Path(__file__).parent.parent.parent / "assets" / "robots" / "reference_4dof.json"
    robot = load_model(RobotDefinition, robot_path)
    
    assert robot.id == "reference-4dof"
    assert len(robot.joints) == 4
    assert len(robot.links) == 4


def test_save_and_load_roundtrip(tmp_path):
    robot_path = Path(__file__).parent.parent.parent / "assets" / "robots" / "reference_4dof.json"
    robot = load_model(RobotDefinition, robot_path)
    
    # Save it to a temporary file
    temp_file = tmp_path / "temp_robot.json"
    save_model(robot, temp_file)
    
    # Reload it
    reloaded_robot = load_model(RobotDefinition, temp_file)
    
    assert robot.id == reloaded_robot.id
    assert len(robot.joints) == len(reloaded_robot.joints)
