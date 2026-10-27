"""Background odometry estimation from wheel encoders and the EV3 gyro."""

from __future__ import annotations

import csv
import logging
import math
import threading
import time
from pathlib import Path
from typing import Optional

from .config import RobotConfig
from .hardware import Hardware, HardwareError, SensorReadError
from .kinematics import Pose, encoder_delta_to_twist, integrate_pose, normalize_angle

logger = logging.getLogger(__name__)


class Odometry:
    """Continuously integrate wheel motion in a background thread.

    A thread is used instead of a separate process because the workload is
    I2C-bound rather than CPU-bound, and a thread shares the pose without
    pickle/serialisation overhead. All reads and writes of ``_pose`` are
    protected by ``_lock`` so the control loop always sees a consistent pose.
    """

    def __init__(
        self,
        hardware: Hardware,
        config: RobotConfig,
        initial_pose: Optional[Pose] = None,
    ) -> None:
        self.hardware = hardware
        self.config = config
        self._pose = initial_pose.copy() if initial_pose else Pose()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_start = 0.0
        self._gyro_offset = 0.0
        self._heading_source = config.heading_source

        self.last_error: Optional[str] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._csv_file, self._csv_writer = self._open_log()
        self._csv_start = time.perf_counter()

        # Establish the encoder baseline before the loop starts.
        self._prev_left = self._safe_encoder(self.config.left_motor_port)
        self._prev_right = self._safe_encoder(self.config.right_motor_port)

        if self._heading_source == "gyro":
            try:
                gyro_angle, _ = self.hardware.read_gyro(self.config.gyro_port)
                with self._lock:
                    initial_theta = self._pose.theta
                self._gyro_offset = initial_theta - math.radians(
                    self.config.gyro_sign * gyro_angle
                )
            except SensorReadError as exc:
                logger.warning("Gyro unavailable (%s); falling back to wheel heading", exc)
                self._heading_source = "encoders"

        self._thread = threading.Thread(
            target=self._run, name="odometry", daemon=True
        )
        self._thread.start()
        logger.info("Odometry started (heading source: %s)", self._heading_source)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        logger.info("Odometry stopped")

    def reset(self, pose: Pose) -> None:
        """Reset the pose and the encoder/gyro baselines atomically."""

        with self._lock:
            self._pose = pose.copy()

        self._prev_left = self._safe_encoder(self.config.left_motor_port)
        self._prev_right = self._safe_encoder(self.config.right_motor_port)

        if self._heading_source == "gyro":
            try:
                gyro_angle, _ = self.hardware.read_gyro(self.config.gyro_port)
                self._gyro_offset = pose.theta - math.radians(
                    self.config.gyro_sign * gyro_angle
                )
            except SensorReadError as exc:
                logger.warning("Could not reset gyro heading: %s", exc)

    # ------------------------------------------------------------------
    # Read API
    # ------------------------------------------------------------------
    def read(self) -> Pose:
        with self._lock:
            return self._pose.copy()

    @property
    def pose(self) -> Pose:
        return self.read()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _safe_encoder(self, port: str) -> float:
        try:
            return self.hardware.get_encoder(port)
        except HardwareError as exc:
            logger.warning("Initial encoder read failed: %s", exc)
            return 0.0

    def _open_log(self):
        if self.config.odometry_log is None:
            return None, None
        path = Path(self.config.odometry_log)
        path.parent.mkdir(parents=True, exist_ok=True)
        handle = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(handle)
        writer.writerow(["t_s", "x_cm", "y_cm", "theta_deg"])
        return handle, writer

    def _read_gyro_heading(self) -> Optional[float]:
        try:
            angle_deg, _ = self.hardware.read_gyro(self.config.gyro_port)
        except SensorReadError as exc:
            self.last_error = str(exc)
            return None
        return normalize_angle(
            math.radians(self.config.gyro_sign * angle_deg) + self._gyro_offset
        )

    def _run(self) -> None:
        while not self._stop_event.is_set():
            loop_start = time.perf_counter()
            try:
                left = self.hardware.get_encoder(self.config.left_motor_port)
                right = self.hardware.get_encoder(self.config.right_motor_port)
            except HardwareError as exc:
                self.last_error = str(exc)
                logger.warning("Encoder read failed: %s", exc)
                self._stop_event.wait(self.config.odometry_period_s)
                continue

            ds_cm, dtheta_rad = encoder_delta_to_twist(
                left - self._prev_left,
                right - self._prev_right,
                self.config.wheel_radius_cm,
                self.config.wheel_base_cm,
            )
            self._prev_left = left
            self._prev_right = right

            with self._lock:
                merged = integrate_pose(self._pose, ds_cm, dtheta_rad)

            if self._heading_source == "gyro":
                gyro_heading = self._read_gyro_heading()
                if gyro_heading is not None:
                    merged = Pose(merged.x, merged.y, gyro_heading)

            with self._lock:
                self._pose = merged

            if self._csv_writer is not None:
                t = time.perf_counter() - self._csv_start
                self._csv_writer.writerow(
                    [f"{t:.3f}", f"{merged.x:.3f}", f"{merged.y:.3f}",
                     f"{math.degrees(merged.theta):.2f}"]
                )

            elapsed = time.perf_counter() - loop_start
            self._stop_event.wait(max(0.0, self.config.odometry_period_s - elapsed))
