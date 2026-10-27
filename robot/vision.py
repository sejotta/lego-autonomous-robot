"""Perception: HSV red-ball tracking and ORB droid-marker recognition.

Both pipelines are deliberately conventional and explainable:

* Ball: colour threshold -> morphological opening -> largest contour ->
  centroid and area -> proportional visual servo.
* Marker: ORB keypoints -> brute-force Hamming matching -> RANSAC homography
  -> projected template corners -> centre and apparent area.

These are the algorithms used on the real robot. The original coursework also
experimented with BRISK; ORB was kept because it is fast on a Raspberry Pi and
its binary descriptors match the Hamming-distance matcher.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional

import cv2
import numpy as np

from .config import MARKER_REFERENCES, VisionConfig
from .kinematics import clamp

logger = logging.getLogger(__name__)


class CameraError(RuntimeError):
    """Raised when the camera cannot be opened or read."""


# ----------------------------------------------------------------------
# Camera helpers
# ----------------------------------------------------------------------
def open_camera(config: VisionConfig, index: Optional[int] = None) -> cv2.VideoCapture:
    camera = cv2.VideoCapture(config.camera_index if index is None else index)
    if not camera.isOpened():
        raise CameraError(f"could not open camera index {index if index is not None else config.camera_index}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
    camera.set(cv2.CAP_PROP_FPS, config.camera_fps)
    try:
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:  # not supported by every backend
        pass
    return camera


def read_frame(camera: cv2.VideoCapture) -> np.ndarray:
    ok, frame = camera.read()
    if not ok or frame is None:
        raise CameraError("camera returned an empty frame")
    return frame


# ----------------------------------------------------------------------
# Red ball
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class BallDetection:
    center_x: float
    center_y: float
    area_px: float
    radius_px: float


class BallDetector:
    """Find the largest red blob in a BGR frame."""

    def __init__(self, config: Optional[VisionConfig] = None) -> None:
        self.config = config or VisionConfig()
        self.kernel = np.ones((5, 5), np.uint8)

    def mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower1 = np.array(self.config.ball_lower_red_1, dtype=np.uint8)
        upper1 = np.array(self.config.ball_upper_red_1, dtype=np.uint8)
        lower2 = np.array(self.config.ball_lower_red_2, dtype=np.uint8)
        upper2 = np.array(self.config.ball_upper_red_2, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.dilate(mask, self.kernel, iterations=1)

    def detect(self, frame_bgr: np.ndarray) -> Optional[BallDetection]:
        mask = self.mask(frame_bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.config.ball_min_area_px:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None

        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        radius = math.sqrt(area / math.pi)
        return BallDetection(center_x, center_y, area, radius)

    def annotate(self, frame_bgr: np.ndarray, detection: Optional[BallDetection]) -> np.ndarray:
        output = frame_bgr.copy()
        if detection is None:
            return output

        center = (int(detection.center_x), int(detection.center_y))
        cv2.circle(output, center, int(detection.radius_px), (0, 255, 0), 2)
        cv2.drawMarker(output, center, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(
            output,
            f"ball area={detection.area_px:.0f}px",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        return output


@dataclass(frozen=True)
class BallCommand:
    v_cm_s: float
    w_rad_s: float
    aligned: bool
    searching: bool = False


class BallTracker:
    """Proportional visual servo that drives the ball into the gripper."""

    def __init__(self, config: Optional[VisionConfig] = None) -> None:
        self.config = config or VisionConfig()
        self.detector = BallDetector(self.config)
        self.last_direction = 1.0

    def command(self, detection: Optional[BallDetection], frame_width: int) -> BallCommand:
        if detection is None:
            return BallCommand(
                v_cm_s=0.0,
                w_rad_s=self.config.ball_search_angular_speed_rad_s * self.last_direction,
                aligned=False,
                searching=True,
            )

        pixel_error = detection.center_x - frame_width / 2.0
        if pixel_error < 0:
            self.last_direction = -1.0
        elif pixel_error > 0:
            self.last_direction = 1.0

        area_error = (self.config.ball_target_area_px - detection.area_px) / max(
            1.0, self.config.ball_target_area_px
        )
        w = self.config.ball_max_angular_speed_rad_s * clamp(
            pixel_error / max(1.0, frame_width / 2.0), -1.0, 1.0
        )
        v = self.config.ball_max_linear_speed_cm_s * clamp(area_error, -1.0, 1.0)

        aligned = (
            abs(self.config.ball_target_area_px - detection.area_px)
            <= self.config.ball_area_tolerance_px
            and abs(pixel_error) <= self.config.ball_center_tolerance_px
        )
        if aligned:
            v = 0.0
            w = 0.0

        return BallCommand(v_cm_s=v, w_rad_s=w, aligned=aligned)

    def track(
        self,
        robot,
        camera: Optional[cv2.VideoCapture] = None,
        timeout: Optional[float] = None,
        show: bool = False,
    ) -> bool:
        """Drive towards the ball until the gripper can close.

        Returns ``True`` when the ball is centred and at the target apparent
        size; ``False`` on timeout or camera failure.
        """

        timeout = self.config.ball_timeout_s if timeout is None else timeout
        owns_camera = camera is None
        camera = camera or open_camera(self.config)
        deadline = time.monotonic() + timeout
        aligned_frames = 0
        last_searching: Optional[bool] = None

        try:
            while time.monotonic() < deadline:
                frame = read_frame(camera)
                detection = self.detector.detect(frame)
                command = self.command(detection, frame.shape[1])

                if last_searching != command.searching:
                    if command.searching:
                        logger.info("Ball not visible; searching")
                    else:
                        logger.info("Ball acquired")
                    last_searching = command.searching

                if command.searching:
                    logger.debug("Ball not visible; searching")
                else:
                    logger.debug(
                        "Ball area=%.0f px, v=%.1f cm/s, w=%.2f rad/s",
                        detection.area_px if detection else -1,
                        command.v_cm_s,
                        command.w_rad_s,
                    )

                if command.aligned:
                    aligned_frames += 1
                    robot.stop()
                    if aligned_frames >= 3:
                        logger.info("Ball aligned; closing gripper")
                        robot.close_gripper()
                        return True
                else:
                    aligned_frames = 0
                    robot.set_speed(command.v_cm_s, command.w_rad_s)

                if show:
                    _show("ball tracking", frame, detection, self.detector)
                time.sleep(0.05)

            robot.stop()
            logger.warning("Ball tracking timed out")
            return False
        finally:
            robot.stop()
            if owns_camera:
                camera.release()
            if show:
                cv2.destroyAllWindows()


def _show(window: str, frame: np.ndarray, detection, detector: BallDetector) -> None:
    try:
        cv2.imshow(window, detector.annotate(frame, detection))
        cv2.waitKey(1)
    except cv2.error:
        # No display available (headless Raspberry Pi / CI); ignore.
        pass


# ----------------------------------------------------------------------
# Droid markers
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class MarkerDetection:
    label: str
    center: tuple[float, float]
    polygon: np.ndarray
    area_px: float
    matches: int
    inliers: int


class MarkerDetector:
    """Recognise printed R2-D2 / BB-8 markers with ORB + RANSAC."""

    def __init__(
        self,
        references: Optional[Mapping[str, str | Path]] = None,
        config: Optional[VisionConfig] = None,
    ) -> None:
        self.config = config or VisionConfig()
        self.references = dict(references or MARKER_REFERENCES)
        self.orb = cv2.ORB_create(nfeatures=self.config.marker_orb_features)
        self._templates: dict[str, tuple[np.ndarray, tuple[cv2.KeyPoint, ...], np.ndarray, tuple[int, int]]] = {}

        for label, path in self.references.items():
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise FileNotFoundError(f"marker reference not found or unreadable: {path}")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
            if descriptors is None or len(descriptors) == 0:
                raise ValueError(f"no ORB features in marker reference {path}")
            self._templates[label] = (gray, keypoints, descriptors, gray.shape)

    def detect(self, frame_bgr: np.ndarray) -> dict[str, MarkerDetection]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        frame_keypoints, frame_descriptors = self.orb.detectAndCompute(gray, None)
        if frame_descriptors is None or len(frame_descriptors) < self.config.marker_min_matches:
            return {}

        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        detections: dict[str, MarkerDetection] = {}

        for label, (_, ref_keypoints, ref_descriptors, (height, width)) in self._templates.items():
            matches = matcher.match(ref_descriptors, frame_descriptors)
            if len(matches) <= self.config.marker_min_matches:
                continue

            matches = sorted(matches, key=lambda match: match.distance)
            src_pts = np.float32([ref_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([frame_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            try:
                homography, mask = cv2.findHomography(
                    src_pts, dst_pts, cv2.RANSAC, self.config.marker_ransac_reproj_px
                )
            except cv2.error:
                logger.debug("Homography failed for marker %s", label)
                continue
            if homography is None or mask is None:
                continue

            inliers = int(np.sum(mask))
            if inliers < self.config.marker_min_inliers:
                continue

            corners = np.float32(
                [[0, 0], [0, height - 1], [width - 1, height - 1], [width - 1, 0]]
            ).reshape(-1, 1, 2)
            try:
                projected = cv2.perspectiveTransform(corners, homography)
            except cv2.error:
                logger.debug("Could not project marker %s corners", label)
                continue
            center = projected.mean(axis=0).ravel()
            area = abs(float(cv2.contourArea(np.int32(projected).reshape(-1, 2))))

            detections[label] = MarkerDetection(
                label=label,
                center=(float(center[0]), float(center[1])),
                polygon=projected,
                area_px=area,
                matches=len(matches),
                inliers=inliers,
            )

        return detections

    def annotate(
        self,
        frame_bgr: np.ndarray,
        detections: Mapping[str, MarkerDetection],
    ) -> np.ndarray:
        output = frame_bgr.copy()
        colors = {"R2D2": (0, 0, 255), "BB8": (0, 200, 0)}
        for label, detection in detections.items():
            color = colors.get(label, (255, 128, 0))
            polygon = np.int32(detection.polygon).reshape(-1, 2)
            cv2.polylines(output, [polygon], True, color, 2, cv2.LINE_AA)
            center = (int(detection.center[0]), int(detection.center[1]))
            cv2.circle(output, center, 4, color, -1)

            text = f"{label}  {detection.inliers} inliers"
            (text_width, text_height), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1
            )
            x = int(polygon[:, 0].min())
            y = max(text_height + 4, int(polygon[:, 1].min()) - 6)
            x = max(2, min(x, output.shape[1] - text_width - 2))
            cv2.rectangle(
                output,
                (x - 2, y - text_height - 2),
                (x + text_width + 2, y + 2),
                (255, 255, 255),
                -1,
            )
            cv2.putText(
                output,
                text,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        return output

    @staticmethod
    def sides(detections: Mapping[str, MarkerDetection]) -> dict[str, str]:
        """Return ``{"R2D2": "left", "BB8": "right"}`` style labels."""

        if len(detections) < 2:
            return {}
        ordered = sorted(detections.values(), key=lambda detection: detection.center[0])
        return {ordered[0].label: "left", ordered[1].label: "right"}


class MarkerFollower:
    """Visual servo that approaches a chosen droid marker."""

    def __init__(self, config: Optional[VisionConfig] = None) -> None:
        self.config = config or VisionConfig()
        self.detector = MarkerDetector(config=self.config)
        self.last_direction = 1.0

    def command(self, detection: Optional[MarkerDetection], frame_width: int) -> BallCommand:
        if detection is None:
            return BallCommand(
                v_cm_s=0.0,
                w_rad_s=self.config.marker_search_angular_speed_rad_s * self.last_direction,
                aligned=False,
                searching=True,
            )

        pixel_error = detection.center[0] - frame_width / 2.0
        if pixel_error < 0:
            self.last_direction = -1.0
        elif pixel_error > 0:
            self.last_direction = 1.0

        area_error = (self.config.marker_target_area_px - detection.area_px) / max(
            1.0, self.config.marker_target_area_px
        )
        v = self.config.marker_max_linear_speed_cm_s * clamp(area_error, -1.0, 1.0)
        w = self.config.marker_max_angular_speed_rad_s * clamp(
            pixel_error / max(1.0, frame_width / 2.0), -1.0, 1.0
        )

        aligned = (
            abs(self.config.marker_target_area_px - detection.area_px)
            <= self.config.marker_area_tolerance_px
            and abs(pixel_error) <= self.config.marker_center_tolerance_px
        )
        return BallCommand(v_cm_s=0.0 if aligned else v, w_rad_s=0.0 if aligned else w, aligned=aligned)

    def follow(
        self,
        robot,
        target_label: str,
        camera: Optional[cv2.VideoCapture] = None,
        timeout: Optional[float] = None,
        show: bool = False,
    ) -> bool:
        timeout = self.config.marker_timeout_s if timeout is None else timeout
        owns_camera = camera is None
        camera = camera or open_camera(self.config)
        deadline = time.monotonic() + timeout
        aligned_frames = 0
        last_sides: Optional[dict[str, str]] = None

        try:
            while time.monotonic() < deadline:
                frame = read_frame(camera)
                detections = self.detector.detect(frame)
                target = detections.get(target_label)
                sides = self.detector.sides(detections)

                if sides and sides != last_sides:
                    logger.info("Marker sides: %s", sides)
                    last_sides = sides

                command = self.command(target, frame.shape[1])
                if command.aligned:
                    aligned_frames += 1
                    robot.stop()
                    if aligned_frames >= 3:
                        logger.info("%s marker reached", target_label)
                        return True
                else:
                    aligned_frames = 0
                    robot.set_speed(command.v_cm_s, command.w_rad_s)

                if show:
                    try:
                        cv2.imshow("marker following", self.detector.annotate(frame, detections))
                        cv2.waitKey(1)
                    except cv2.error:
                        pass
                time.sleep(0.05)

            robot.stop()
            logger.warning("Could not reach marker %s before timeout", target_label)
            return False
        finally:
            robot.stop()
            if owns_camera:
                camera.release()
            if show:
                cv2.destroyAllWindows()
