import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from robot.config import VisionConfig
from robot.vision import (
    BallDetector,
    BallTracker,
    MarkerDetector,
    MarkerFollower,
)

ROOT = Path(__file__).resolve().parents[1]


class BallVisionTests(unittest.TestCase):
    def test_red_ball_detection_on_synthetic_frame(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(frame, (200, 120), 40, (0, 0, 255), -1)

        detection = BallDetector().detect(frame)
        self.assertIsNotNone(detection)
        self.assertAlmostEqual(detection.center_x, 200, delta=3)
        self.assertAlmostEqual(detection.center_y, 120, delta=3)
        self.assertGreater(detection.area_px, 3000)

    def test_no_ball_in_blank_frame(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        self.assertIsNone(BallDetector().detect(frame))

    def test_ball_tracker_commands(self):
        config = VisionConfig()
        tracker = BallTracker(config)

        centered = SimpleNamespace(center_x=160.0, center_y=120.0, area_px=12000.0)
        command = tracker.command(centered, 320)
        self.assertTrue(command.aligned)
        self.assertEqual(command.v_cm_s, 0.0)
        self.assertEqual(command.w_rad_s, 0.0)

        left = SimpleNamespace(center_x=80.0, center_y=120.0, area_px=5000.0)
        command = tracker.command(left, 320)
        self.assertLess(command.w_rad_s, 0)
        self.assertGreater(command.v_cm_s, 0)

        searching = tracker.command(None, 320)
        self.assertTrue(searching.searching)
        self.assertNotEqual(searching.w_rad_s, 0)


class MarkerVisionTests(unittest.TestCase):
    def test_marker_detection_on_recorded_frame(self):
        frame = cv2.imread(str(ROOT / "assets" / "samples" / "markers_test.jpg"))
        self.assertIsNotNone(frame)

        detector = MarkerDetector()
        detections = detector.detect(frame)

        self.assertIn("R2D2", detections)
        self.assertIn("BB8", detections)
        self.assertLess(detections["R2D2"].center[0], detections["BB8"].center[0])
        self.assertGreaterEqual(detections["R2D2"].inliers, 15)
        self.assertGreaterEqual(detections["BB8"].inliers, 15)
        self.assertEqual(detections["R2D2"].polygon.shape, (4, 1, 2))
        self.assertEqual(detector.sides(detections), {"R2D2": "left", "BB8": "right"})

    def test_marker_detector_returns_empty_for_blank_frame(self):
        detector = MarkerDetector()
        blank = np.zeros((240, 320, 3), dtype=np.uint8)
        self.assertEqual(detector.detect(blank), {})

    def test_marker_follower_command_is_aligned_at_target(self):
        config = VisionConfig()
        follower = MarkerFollower(config)
        detection = SimpleNamespace(
            center=(160.0, 120.0),
            area_px=config.marker_target_area_px,
        )
        command = follower.command(detection, 320)
        self.assertTrue(command.aligned)
        self.assertEqual(command.v_cm_s, 0)
        self.assertEqual(command.w_rad_s, 0)


if __name__ == "__main__":
    unittest.main()
