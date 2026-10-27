"""Pure differential-drive kinematics and geometry helpers.

Keeping this module free of hardware and OpenCV imports makes the core math
easy to unit-test and reuse in notebooks or offline simulations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Pose:
    """Robot pose in the arena frame (cm, cm, rad)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.theta)


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to the interval (-pi, pi]."""

    angle = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
    if angle < -math.pi:
        angle += 2.0 * math.pi
    elif abs(angle + math.pi) < 1e-12:
        # atan2 can return -pi for inputs mathematically equivalent to pi.
        angle = math.pi
    return angle


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def heading_to(pose: Pose, x_goal: float, y_goal: float) -> float:
    """Absolute heading that points from ``pose`` towards ``(x_goal, y_goal)``."""

    return math.atan2(y_goal - pose.y, x_goal - pose.x)


def angular_error(current_rad: float, target_rad: float) -> float:
    """Signed shortest rotation from ``current_rad`` to ``target_rad``."""

    return normalize_angle(target_rad - current_rad)


def wheel_speeds_to_dps(
    v_cm_s: float,
    w_rad_s: float,
    wheel_radius_cm: float,
    wheel_base_cm: float,
) -> tuple[float, float]:
    """Convert body twist ``(v, w)`` to left/right motor speeds in deg/s.

    Forward kinematics of a differential drive:

    .. math::

        \\omega_l = \\frac{v - \\omega L / 2}{R}, \\qquad
        \\omega_r = \\frac{v + \\omega L / 2}{R}
    """

    left_rad_s = (v_cm_s - w_rad_s * wheel_base_cm / 2.0) / wheel_radius_cm
    right_rad_s = (v_cm_s + w_rad_s * wheel_base_cm / 2.0) / wheel_radius_cm
    return math.degrees(left_rad_s), math.degrees(right_rad_s)


def dps_to_wheel_speeds(
    left_dps: float,
    right_dps: float,
    wheel_radius_cm: float,
    wheel_base_cm: float,
) -> tuple[float, float]:
    """Inverse kinematics: motor speeds in deg/s to body twist ``(v, w)``."""

    left_rad_s = math.radians(left_dps)
    right_rad_s = math.radians(right_dps)
    v_cm_s = wheel_radius_cm * (right_rad_s + left_rad_s) / 2.0
    w_rad_s = wheel_radius_cm * (right_rad_s - left_rad_s) / wheel_base_cm
    return v_cm_s, w_rad_s


def encoder_delta_to_twist(
    delta_left_deg: float,
    delta_right_deg: float,
    wheel_radius_cm: float,
    wheel_base_cm: float,
) -> tuple[float, float]:
    """Convert wheel encoder increments to body displacement and rotation.

    Returns ``(ds_cm, dtheta_rad)``. Wheel increments are *not* wrapped: a
    single sampling interval should never rotate a wheel by more than 180
    degrees, and wrapping each wheel separately would corrupt the heading.
    """

    ds_left = wheel_radius_cm * math.radians(delta_left_deg)
    ds_right = wheel_radius_cm * math.radians(delta_right_deg)
    ds_cm = (ds_left + ds_right) / 2.0
    dtheta_rad = (ds_right - ds_left) / wheel_base_cm
    return ds_cm, dtheta_rad


def integrate_pose(pose: Pose, ds_cm: float, dtheta_rad: float) -> Pose:
    """Midpoint integration of a pose increment.

    Using the heading halfway through the interval is a second-order accurate
    approximation and noticeably reduces drift on curved trajectories compared
    with integrating at the previous heading.
    """

    mid_theta = pose.theta + dtheta_rad / 2.0
    return Pose(
        x=pose.x + ds_cm * math.cos(mid_theta),
        y=pose.y + ds_cm * math.sin(mid_theta),
        theta=normalize_angle(pose.theta + dtheta_rad),
    )


def path_length(points: Iterable[tuple[float, float]]) -> float:
    """Total Euclidean length of a polyline."""

    pts = list(points)
    return sum(distance(x1, y1, x2, y2) for (x1, y1), (x2, y2) in zip(pts, pts[1:]))
