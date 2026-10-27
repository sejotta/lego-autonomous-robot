import math
import unittest

from robot.kinematics import (
    Pose,
    dps_to_wheel_speeds,
    encoder_delta_to_twist,
    integrate_pose,
    normalize_angle,
    wheel_speeds_to_dps,
)

RADIUS = 2.6
BASE = 15.0


class KinematicsTests(unittest.TestCase):
    def test_velocity_round_trip(self):
        for v, w in [(0.0, 0.0), (10.0, 0.0), (0.0, 0.5), (12.0, -0.4), (-5.0, 0.2)]:
            with self.subTest(v=v, w=w):
                left, right = wheel_speeds_to_dps(v, w, RADIUS, BASE)
                v2, w2 = dps_to_wheel_speeds(left, right, RADIUS, BASE)
                self.assertAlmostEqual(v2, v, places=9)
                self.assertAlmostEqual(w2, w, places=9)

    def test_straight_line_integration(self):
        pose = integrate_pose(Pose(), ds_cm=10.0, dtheta_rad=0.0)
        self.assertAlmostEqual(pose.x, 10.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 0.0)

    def test_quarter_turn_integration_uses_midpoint_heading(self):
        pose = integrate_pose(Pose(), ds_cm=10.0, dtheta_rad=math.pi / 2)
        # Chord of a 90-degree arc: 10 / sqrt(2) in both axes.
        self.assertAlmostEqual(pose.x, 10.0 / math.sqrt(2.0))
        self.assertAlmostEqual(pose.y, 10.0 / math.sqrt(2.0))
        self.assertAlmostEqual(pose.theta, math.pi / 2)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-3 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(math.pi / 2 + 2 * math.pi), math.pi / 2)

    def test_encoder_delta_straight(self):
        ds, dtheta = encoder_delta_to_twist(360.0, 360.0, RADIUS, BASE)
        self.assertAlmostEqual(ds, 2 * math.pi * RADIUS)
        self.assertAlmostEqual(dtheta, 0.0)

    def test_encoder_delta_spin_in_place(self):
        ds, dtheta = encoder_delta_to_twist(-90.0, 90.0, RADIUS, BASE)
        self.assertAlmostEqual(ds, 0.0)
        self.assertAlmostEqual(dtheta, math.pi * RADIUS / BASE)

    def test_positive_angular_speed_turns_left_in_wheel_convention(self):
        left, right = wheel_speeds_to_dps(10.0, 0.4, RADIUS, BASE)
        self.assertGreater(right, left)


if __name__ == "__main__":
    unittest.main()
