"""Unit tests for kinematics.py. They need pytest only, not ROS."""

import pytest

from rov_motor_sim.kinematics import twist_to_rpm

# same values as config/params.yaml
RPM_GAIN = 3000.0   # RPM per (m/s)
MAX_RPM = 3000.0
SEPARATION = 0.3    # metres


def convert(linear_x, angular_z, linear_z):
    return twist_to_rpm(
        linear_x, angular_z, linear_z, SEPARATION, RPM_GAIN, MAX_RPM)


def test_zero_input_gives_zero_rpm():
    assert convert(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)


def test_pure_forward_drives_both_horizontal_motors_equally():
    left, right, vertical = convert(0.5, 0.0, 0.0)

    # 0.5 m/s * 3000 = 1500 RPM
    assert left == pytest.approx(1500.0)
    assert right == pytest.approx(1500.0)
    assert vertical == 0.0


def test_pure_yaw_spins_horizontal_motors_in_opposite_directions():
    # +ve yaw is a left turn, so right goes forward and left goes backward
    left, right, vertical = convert(0.0, 1.0, 0.0)

    # 1 rad/s * (0.3 m / 2) = 0.15 m/s -> 450 RPM
    assert left == pytest.approx(-450.0)
    assert right == pytest.approx(450.0)
    assert vertical == 0.0


def test_heave_uses_only_the_vertical_motor():
    assert convert(0.0, 0.0, 0.4) == pytest.approx((0.0, 0.0, 1200.0))
    assert convert(0.0, 0.0, -0.4) == pytest.approx((0.0, 0.0, -1200.0))


def test_rpm_is_clamped_to_max_rpm_in_both_directions():
    # 5 m/s would be 15000 RPM without the clamp
    assert convert(5.0, 0.0, 5.0) == (MAX_RPM, MAX_RPM, MAX_RPM)
    assert convert(-5.0, 0.0, -5.0) == (-MAX_RPM, -MAX_RPM, -MAX_RPM)
