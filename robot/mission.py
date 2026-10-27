"""Four-phase autonomous mission orchestration.

The mission deliberately keeps each phase independent and observable:

1. ``slalom``  timed open-loop arcs with ultrasonic wall alignment
2. ``maze``    NF1 planning + waypoint navigation + obstacle replanning
3. ``ball``    HSV visual servoing and gripper closure
4. ``door``    ORB marker recognition, approach, and arena exit

A failure in one phase raises :class:`MissionError` instead of silently
continuing with a corrupted pose, which makes the logs much easier to debug.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Optional, Sequence

from .config import ARENAS, ArenaConfig, VisionConfig
from .kinematics import Pose, distance
from .mapping import Cell, GridMap
from .robot import GoResult, Robot
from .vision import BallTracker, CameraError, MarkerFollower

logger = logging.getLogger(__name__)

PHASES = ("slalom", "maze", "ball", "door")


class MissionError(RuntimeError):
    """Raised when a mission phase cannot be completed safely."""


class Mission:
    def __init__(
        self,
        robot: Robot,
        arena: ArenaConfig,
        vision_config: Optional[VisionConfig] = None,
        artifacts_dir: Optional[str | Path] = None,
    ) -> None:
        self.robot = robot
        self.arena = arena
        self.vision_config = vision_config or VisionConfig()
        self.artifacts_dir = Path(artifacts_dir) if artifacts_dir else None
        self.map = GridMap(arena.map_path)
        self.path: Optional[list[Cell]] = None
        self.trajectory: list[Pose] = []

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------
    def run(self, phases: Sequence[str] = PHASES) -> None:
        invalid = set(phases) - set(PHASES)
        if invalid:
            raise ValueError(f"unknown mission phases: {sorted(invalid)}")

        logger.info("Arena %s: %s", self.arena.key.upper(), self.arena.description)
        logger.info("Assigned marker: %s", self.arena.assigned_marker)
        self.robot.start_odometry()

        for phase in phases:
            logger.info("=" * 60)
            logger.info("PHASE: %s", phase.upper())
            logger.info("=" * 60)
            try:
                getattr(self, f"run_{phase}")()
            except MissionError:
                raise
            except Exception as exc:
                raise MissionError(f"phase '{phase}' failed: {exc}") from exc

        logger.info("Mission complete")
        self._save_map(self.trajectory[-1] if self.trajectory else None)

    # ------------------------------------------------------------------
    # Phase 1: slalom
    # ------------------------------------------------------------------
    def run_slalom(self) -> None:
        """Follow a timed slalom and square up against the arena walls.

        This phase is intentionally open-loop: the track is fixed, so timed
        arcs are faster and more repeatable than trying to localise during the
        slalom. The ultrasonic wall measurements remove the accumulated
        heading/distance error at the two checkpoints.
        """

        if self.arena.slalom_profile == "white":
            segments = (
                (0.0, -math.pi / 4.0, 2.3),
                (3.5 * math.pi, math.pi / 10.0, 10.5),
                (4.0 * math.pi, -math.pi / 10.0, 10.4),
            )
            final_rotation = math.pi / 4.0
        elif self.arena.slalom_profile == "black":
            segments = (
                (0.0, math.pi / 4.0, 2.3),
                (3.5 * math.pi, -math.pi / 10.0, 11.0),
                (4.0 * math.pi, math.pi / 10.0, 10.4),
            )
            final_rotation = -math.pi / 4.0
        else:
            raise MissionError(f"unknown slalom profile {self.arena.slalom_profile!r}")

        for index, (v, w, seconds) in enumerate(segments):
            logger.info("Slalom segment %d: v=%.1f cm/s w=%.3f rad/s t=%.1fs", index, v, w, seconds)
            self.robot.timed_drive(v, w, seconds)

        self.robot.approach_wall_distance(60.0)
        self.robot.timed_drive(0.0, final_rotation, 2.1)
        self.robot.approach_wall_distance(100.0)
        self.robot.stop()
        logger.info("Slalom complete")

    # ------------------------------------------------------------------
    # Phase 2: maze
    # ------------------------------------------------------------------
    def run_maze(self) -> None:
        """Plan with NF1 and navigate cell-by-cell, replanning when blocked."""

        self.robot.reset_odometry(self.arena.start_pose)
        self.robot.start_odometry()

        path = self.map.plan(self.arena.start_cell, self.arena.goal_cell)
        if path is None:
            raise MissionError(
                f"no path from {self.arena.start_cell} to {self.arena.goal_cell} "
                f"in {self.arena.map_name}"
            )
        self.path = path
        logger.info("NF1 path (%d cells): %s", len(path), path)

        index = 1
        while index < len(path):
            previous_cell = path[index - 1]
            next_cell = path[index]
            x_goal, y_goal = self.map.cell_center_cm(next_cell)

            logger.info("Navigating to cell %s (%.0f, %.0f cm)", next_cell, x_goal, y_goal)
            result = self.robot.go_to(x_goal, y_goal)

            if result is GoResult.REACHED:
                self.trajectory.append(self.robot.pose)
                index += 1
                continue

            if result is GoResult.BLOCKED:
                logger.warning("Blocked between %s and %s", previous_cell, next_cell)
                # Back away from the obstacle before re-planning so the new
                # heading can actually be executed.
                self.robot.timed_drive(-5.0, 0.0, 0.8)

                direction = (next_cell[0] - previous_cell[0], next_cell[1] - previous_cell[1])
                new_path = self.map.replan(previous_cell, self.arena.goal_cell, direction)
                if new_path is None:
                    raise MissionError(
                        f"no alternative path after blocking edge {previous_cell}->{next_cell}"
                    )
                logger.info("Replanned path (%d cells): %s", len(new_path), new_path)
                path = new_path
                self.path = new_path
                index = 1
                continue

            raise MissionError(f"navigation to {next_cell} failed: {result.name}")

        self.robot.stop()
        final_pose = self.robot.pose
        goal_x, goal_y = self.map.cell_center_cm(self.arena.goal_cell)
        error = distance(final_pose.x, final_pose.y, goal_x, goal_y)
        logger.info("Maze complete; final pose error %.1f cm", error)
        self._save_map(final_pose)

    # ------------------------------------------------------------------
    # Phase 3: ball
    # ------------------------------------------------------------------
    def run_ball(self) -> None:
        """Locate the red ball with the camera, approach it, and grasp it."""

        tracker = BallTracker(self.vision_config)
        try:
            success = tracker.track(self.robot, timeout=self.vision_config.ball_timeout_s)
        except CameraError as exc:
            raise MissionError(f"ball phase camera failure: {exc}") from exc

        if not success:
            raise MissionError("ball not found or not aligned before timeout")
        logger.info("Ball grasped")

    # ------------------------------------------------------------------
    # Phase 4: door
    # ------------------------------------------------------------------
    def run_door(self) -> None:
        """Recognise the assigned droid marker, approach it, and exit."""

        goal_x, goal_y = self.map.cell_center_cm(self.arena.goal_cell)
        result = self.robot.go_to(goal_x, goal_y)
        if result is not GoResult.REACHED:
            raise MissionError(f"could not reach the observation point: {result.name}")

        follower = MarkerFollower(self.vision_config)
        try:
            reached = follower.follow(
                self.robot,
                self.arena.assigned_marker,
                timeout=self.vision_config.marker_timeout_s,
            )
        except CameraError as exc:
            raise MissionError(f"door phase camera failure: {exc}") from exc

        if not reached:
            raise MissionError(
                f"could not approach the {self.arena.assigned_marker} marker"
            )

        # Cross the gate: the marker is approximately at the target apparent
        # area when the robot is just in front of it.
        logger.info("Crossing the exit gate")
        self.robot.timed_drive(10.0, 0.0, 3.0)
        logger.info("Arena exit complete")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _save_map(self, final_pose: Optional[Pose] = None) -> None:
        if self.artifacts_dir is None or self.path is None:
            return
        output = self.artifacts_dir / f"mission_{self.arena.key}.png"
        try:
            self.map.draw(
                path=self.path,
                robot_pose=final_pose.as_tuple() if final_pose else None,
                save_path=output,
                title=f"Arena {self.arena.key.upper()} — NF1 mission",
            )
            logger.info("Saved mission map to %s", output)
        except Exception:
            logger.exception("Could not render the mission map")


def choose_arena(robot: Robot, requested: str = "auto") -> ArenaConfig:
    """Select the arena from the CLI choice or the floor light sensor."""

    requested = requested.lower()
    if requested in ARENAS:
        return ARENAS[requested]
    if requested != "auto":
        raise MissionError(f"unknown arena {requested!r}; choose auto, a, or b")

    try:
        value = robot.read_light()
    except Exception as exc:
        raise MissionError(f"could not read the floor light sensor: {exc}") from exc

    white_low, white_high = robot.config.white_floor_range
    black_low, black_high = robot.config.black_floor_range

    if white_low <= value <= white_high:
        logger.info("Light sensor %.0f -> white floor (arena A)", value)
        return ARENAS["a"]
    if black_low <= value <= black_high:
        logger.info("Light sensor %.0f -> black floor (arena B)", value)
        return ARENAS["b"]

    raise MissionError(
        f"light sensor value {value:.0f} does not match a known floor; "
        "pass --arena a or --arena b to override"
    )
