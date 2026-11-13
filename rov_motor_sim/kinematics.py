"""
Thruster mixing and velocity -> RPM conversion for the 3-thruster ROV.

No ROS imports in here, so the maths can be tested with plain pytest.
"""

from typing import Tuple


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit]."""
    return max(-limit, min(limit, value))


def mix_thrusters(
    linear_x: float,
    angular_z: float,
    linear_z: float,
    thruster_separation: float,
) -> Tuple[float, float, float]:
    """
    Split a body velocity command into (left, right, vertical) in m/s.

    linear_x is forward speed, angular_z is yaw rate (+ve = turn left) and
    linear_z is vertical speed (+ve = up). thruster_separation is the
    distance L between the two horizontal thrusters in metres.
    """
    # same as a differential drive: yaw rate w gives +/- w*L/2 at each side
    yaw_part = angular_z * thruster_separation / 2.0

    left = linear_x - yaw_part
    right = linear_x + yaw_part    # right pushes harder for a left turn
    vertical = linear_z

    return left, right, vertical


def velocity_to_rpm(velocity: float, rpm_gain: float, max_rpm: float) -> float:
    """rpm = k * v, limited to +/- max_rpm. Gain is in RPM per (m/s)."""
    return clamp(rpm_gain * velocity, max_rpm)


def twist_to_rpm(
    linear_x: float,
    angular_z: float,
    linear_z: float,
    thruster_separation: float,
    rpm_gain: float,
    max_rpm: float,
) -> Tuple[float, float, float]:
    """Mix the command, then scale and clamp. Returns (left, right, vertical) RPM."""
    left, right, vertical = mix_thrusters(
        linear_x, angular_z, linear_z, thruster_separation)

    return (
        velocity_to_rpm(left, rpm_gain, max_rpm),
        velocity_to_rpm(right, rpm_gain, max_rpm),
        velocity_to_rpm(vertical, rpm_gain, max_rpm),
    )
