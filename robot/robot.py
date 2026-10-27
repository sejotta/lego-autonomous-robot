"""Hardware-facing robot API: differential drive, odometry, and waypoints."""

from __future__ import annotations

import logging
import math
import statistics
import time
from enum import Enum, auto
from typing import Optional

from .config import RobotConfig
from .hardware import Hardware, HardwareError, SensorReadError, settle
from .kinematics import (
    Pose,
    angular_error,
    clamp,
    distance,
    heading_to,
    normalize_angle,
    wheel_speeds_to_dps,
)
from .odometry import Odometry

logger = logging.getLogger(__name__)


class GoResult(Enum):
    """Outcome of a waypoint attempt."""

    REACHED = auto()
    BLOCKED = auto()
    STUCK = auto()
    TIMEOUT = auto()


class Robot:
    """Differential-drive robot with closed-loop waypoint navigation."""

    def __init__(
        self,
        hardware: Hardware,
        config: Optional[RobotConfig] = None,
        initial_pose: Optional[Pose] = None,
    ) -> None:
        self.hardware = hardware
        self.config = config or RobotConfig()
        self.odometry = Odometry(hardware, self.config, initial_pose)

        self._configure_hardware()
        self.stop()

    # ------------------------------------------------------------------
    # Setup and teardown
    # ------------------------------------------------------------------
    def _configure_hardware(self) -> None:
        self.hardware.set_sensor_type(self.config.ultrasonic_port, "EV3_ULTRASONIC_CM")
        self.hardware.set_sensor_type(self.config.gyro_port, "EV3_GYRO_ABS_DPS")
        self.hardware.set_sensor_type(self.config.light_port, "NXT_LIGHT_ON")
        settle(2.0)

        for port in (
            self.config.left_motor_port,
            self.config.right_motor_port,
            self.config.gripper_motor_port,
        ):
            try:
                self.hardware.reset_encoder(port)
            except HardwareError as exc:
                logger.warning("Could not reset encoder on port %s: %s", port, exc)

    def start_odometry(self) -> None:
        self.odometry.start()

    def stop_odometry(self) -> None:
        self.odometry.stop()

    def reset_odometry(self, pose: Pose) -> None:
        self.odometry.reset(pose)

    def shutdown(self) -> None:
        try:
            self.stop()
            self.stop_odometry()
        finally:
            self.hardware.reset_all()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.shutdown()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------
    @property
    def pose(self) -> Pose:
        return self.odometry.read()

    def read_pose(self) -> Pose:
        return self.odometry.read()

    def read_light(self) -> float:
        return self.hardware.read_light(self.config.light_port)

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------
    def set_speed(self, v_cm_s: float, w_rad_s: float) -> None:
        """Command body linear/angular speed (cm/s, rad/s)."""

        v_cm_s = clamp(v_cm_s, -self.config.max_linear_speed_cm_s, self.config.max_linear_speed_cm_s)
        w_rad_s = clamp(
            w_rad_s,
            -self.config.max_body_angular_speed_rad_s,
            self.config.max_body_angular_speed_rad_s,
        )
        left_dps, right_dps = wheel_speeds_to_dps(
            v_cm_s,
            w_rad_s,
            self.config.wheel_radius_cm,
            self.config.wheel_base_cm,
        )
        self.hardware.set_motor_dps(self.config.left_motor_port, left_dps)
        self.hardware.set_motor_dps(self.config.right_motor_port, right_dps)

    def stop(self) -> None:
        try:
            self.hardware.set_motor_dps(self.config.left_motor_port, 0.0)
            self.hardware.set_motor_dps(self.config.right_motor_port, 0.0)
        except HardwareError:
            logger.exception("Could not stop the drive motors")

    def timed_drive(self, v_cm_s: float, w_rad_s: float, seconds: float) -> None:
        """Drive open-loop for a fixed duration (used by the slalom phase)."""

        self.set_speed(v_cm_s, w_rad_s)
        try:
            time.sleep(seconds)
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------
    def read_distance_cm(self, samples: Optional[int] = None) -> Optional[float]:
        """Median ultrasonic distance, or ``None`` if every sample failed."""

        count = samples or self.config.obstacle_samples
        values: list[float] = []
        for _ in range(max(1, count)):
            try:
                value = self.hardware.read_ultrasonic(self.config.ultrasonic_port)
                if math.isfinite(value) and value > 0:
                    values.append(value)
            except SensorReadError as exc:
                logger.debug("Ultrasonic read failed: %s", exc)
            time.sleep(0.01)

        if not values:
            return None
        return float(statistics.median(values))

    def obstacle_ahead(self, threshold_cm: Optional[float] = None) -> bool:
        threshold = self.config.obstacle_threshold_cm if threshold_cm is None else threshold_cm
        distance_cm = self.read_distance_cm()
        if distance_cm is None:
            return False
        logger.debug("Front distance: %.1f cm (threshold %.1f)", distance_cm, threshold)
        return distance_cm < threshold

    # ------------------------------------------------------------------
    # Rotation and waypoint control
    # ------------------------------------------------------------------
    def rotate_to(
        self,
        theta_target: float,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> bool:
        """Rotate in place to an absolute heading with a P controller."""

        tolerance = self.config.heading_tolerance_rad if tolerance is None else tolerance
        timeout = self.config.waypoint_timeout_s if timeout is None else timeout
        target = normalize_angle(theta_target)
        deadline = time.monotonic() + timeout

        while True:
            error = angular_error(self.pose.theta, target)
            if abs(error) <= tolerance:
                self.stop()
                return True

            if time.monotonic() >= deadline:
                self.stop()
                logger.warning("rotate_to timed out with %.3f rad error", error)
                return False

            w = clamp(
                self.config.heading_gain * error,
                -self.config.max_angular_speed_rad_s,
                self.config.max_angular_speed_rad_s,
            )
            # A minimum duty cycle avoids stalling just outside the tolerance
            # band. The loop still checks every period, so the overshoot is at
            # most min_speed * period (about 0.006 rad here).
            if abs(w) < self.config.min_angular_speed_rad_s:
                w = math.copysign(self.config.min_angular_speed_rad_s, error)

            self.set_speed(0.0, w)
            time.sleep(self.config.odometry_period_s)

    def go_to(
        self,
        x_goal: float,
        y_goal: float,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> GoResult:
        """Drive to a point with rotate-then-drive plus heading correction.

        Obstacles are checked before starting and continuously while moving.
        A small "distance getting worse" counter detects the robot pushing
        against an obstacle or slipping instead of reaching the target.
        """

        tolerance = self.config.position_tolerance_cm if tolerance is None else tolerance
        timeout = self.config.waypoint_timeout_s if timeout is None else timeout
        deadline = time.monotonic() + timeout

        if distance(self.pose.x, self.pose.y, x_goal, y_goal) <= tolerance:
            return GoResult.REACHED

        desired_heading = heading_to(self.pose, x_goal, y_goal)
        if not self.rotate_to(
            desired_heading,
            timeout=min(10.0, max(2.0, deadline - time.monotonic())),
        ):
            return GoResult.TIMEOUT

        if self.obstacle_ahead():
            logger.info("Obstacle detected before moving towards (%.1f, %.1f)", x_goal, y_goal)
            self.stop()
            return GoResult.BLOCKED

        previous_distance = distance(self.pose.x, self.pose.y, x_goal, y_goal)
        worsening = 0

        while True:
            pose = self.pose
            remaining = distance(pose.x, pose.y, x_goal, y_goal)
            if remaining <= tolerance:
                self.stop()
                return GoResult.REACHED

            if time.monotonic() >= deadline:
                self.stop()
                logger.warning("go_to timed out %.1f cm from target", remaining)
                return GoResult.TIMEOUT

            if self.obstacle_ahead():
                self.stop()
                logger.info("Obstacle detected %.1f cm from target", remaining)
                return GoResult.BLOCKED

            heading_error = angular_error(pose.theta, heading_to(pose, x_goal, y_goal))

            if abs(heading_error) > 0.7:
                # Too far off course to drive forward safely: turn first.
                self.set_speed(0.0, clamp(heading_error, -0.3, 0.3))
            else:
                w = clamp(
                    self.config.heading_correction_gain * heading_error,
                    -self.config.max_angular_speed_rad_s,
                    self.config.max_angular_speed_rad_s,
                )
                speed_scale = min(1.0, remaining / max(1.0, self.config.slow_radius_cm))
                v = max(self.config.min_linear_speed_cm_s, self.config.max_linear_speed_cm_s * speed_scale)
                self.set_speed(v, w)

                if remaining > previous_distance + 1.0:
                    worsening += 1
                else:
                    worsening = 0

                if worsening >= self.config.stuck_cycles:
                    self.stop()
                    logger.warning("Robot is not making progress towards (%.1f, %.1f)", x_goal, y_goal)
                    return GoResult.STUCK

            previous_distance = remaining
            time.sleep(self.config.odometry_period_s)

    def approach_wall_distance(
        self,
        target_cm: float,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> bool:
        """Drive forward/back until the front ultrasonic reads ``target_cm``.

        Used after the slalom to square the robot up against a side wall.
        """

        tolerance = self.config.wall_reorient_tolerance_cm if tolerance is None else tolerance
        timeout = self.config.wall_reorient_timeout_s if timeout is None else timeout
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            measured = self.read_distance_cm()
            if measured is None:
                self.stop()
                return False

            error = measured - target_cm
            if abs(error) <= tolerance:
                self.stop()
                logger.info("Wall distance aligned at %.1f cm", measured)
                return True

            v = clamp(error * 0.8, -self.config.wall_reorient_speed_cm_s, self.config.wall_reorient_speed_cm_s)
            if abs(v) < 0.5:
                v = math.copysign(0.5, error)
            self.set_speed(v, 0.0)
            time.sleep(self.config.sensor_period_s)

        self.stop()
        logger.warning("Wall alignment timed out (target %.1f cm)", target_cm)
        return False

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------
    def _run_gripper(self, direction: float) -> None:
        self.hardware.set_motor_dps(
            self.config.gripper_motor_port, direction * self.config.gripper_dps
        )
        try:
            time.sleep(self.config.gripper_time_s)
        finally:
            self.hardware.set_motor_dps(self.config.gripper_motor_port, 0.0)

    def close_gripper(self) -> None:
        logger.info("Closing gripper")
        self._run_gripper(+1.0)

    def open_gripper(self) -> None:
        logger.info("Opening gripper")
        self._run_gripper(-1.0)
