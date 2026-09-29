import numpy as np

from robokinematics.robotics.transforms import standard_dh_transform


def test_zero_dh_is_identity():
    result = standard_dh_transform(0.0, 0.0, 0.0, 0.0)
    np.testing.assert_allclose(result, np.eye(4), atol=1e-12)
