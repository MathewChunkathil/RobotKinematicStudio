import numpy as np

from robokinematics.robotics.rotations import matrix_to_rpy, rpy_to_matrix


def test_rpy_to_matrix_identity():
    mat = rpy_to_matrix(0.0, 0.0, 0.0)
    np.testing.assert_allclose(mat, np.eye(3), atol=1e-12)


def test_matrix_to_rpy_identity():
    roll, pitch, yaw = matrix_to_rpy(np.eye(3))
    assert np.isclose(roll, 0.0)
    assert np.isclose(pitch, 0.0)
    assert np.isclose(yaw, 0.0)


def test_rpy_round_trip():
    r, p, y = 0.1, -0.2, 0.3
    mat = rpy_to_matrix(r, p, y)
    r2, p2, y2 = matrix_to_rpy(mat)
    assert np.isclose(r, r2)
    assert np.isclose(p, p2)
    assert np.isclose(y, y2)
