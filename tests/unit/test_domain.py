import pytest

from robokinematics.domain.models import DHParameters, JointLimits


def test_dh_parameters_accept_finite_values():
    params = DHParameters(theta=0.1, d=0.2, a=0.3, alpha=0.4)
    assert params.a == 0.3


def test_joint_limits_reject_reversed_range():
    with pytest.raises(ValueError):
        JointLimits(position_min=2.0, position_max=1.0)
