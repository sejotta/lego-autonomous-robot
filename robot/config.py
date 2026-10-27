"""Configuration objects for the robot, vision stack, and arenas.

Values are expressed in the units used by the controllers:

- distances in centimetres (cm)
- angles in radians (rad)
- speeds in cm/s and rad/s
- map cell size in millimetres (mm) as stored by the map files
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .kinematics import Pose

REPO_ROOT = Path(__file__).resolve().parents[1]
MAPS_DIR = REPO_ROOT / "maps"
ASSETS_DIR = REPO_ROOT / "assets"


@dataclass(frozen=True)
class RobotConfig:
    """Physical and control parameters for the differential-drive robot."""

    # --- Physical geometry -------------------------------------------------
    wheel_radius_cm: float = 2.6
    wheel_base_cm: float = 15.0

    # --- BrickPi3 ports (real hardware) -----------------------------------
    left_motor_port: str = "C"
    right_motor_port: str = "D"
    gripper_motor_port: str = "B"
    ultrasonic_port: str = "1"
    gyro_port: str = "3"
    light_port: str = "4"

    # --- Odometry ----------------------------------------------------------
    odometry_period_s: float = 0.05
    # "gyro" fuses the absolute EV3 gyro heading; "encoders" is wheel-only.
    heading_source: str = "gyro"
    gyro_sign: float = -1.0
    odometry_log: Optional[Path] = None

    # --- Waypoint navigation ----------------------------------------------
    max_linear_speed_cm_s: float = 15.0
    min_linear_speed_cm_s: float = 3.0
    max_angular_speed_rad_s: float = 0.4
    min_angular_speed_rad_s: float = 0.12
    # Generous actuator saturation used by the open-loop slalom. Closed-loop
    # navigation still uses the tighter 0.4 rad/s limit above.
    max_body_angular_speed_rad_s: float = 1.2
    heading_tolerance_rad: float = 0.05
    position_tolerance_cm: float = 3.0
    heading_gain: float = 1.5
    heading_correction_gain: float = 0.8
    slow_radius_cm: float = 15.0
    waypoint_timeout_s: float = 30.0
    stuck_cycles: int = 3

    # --- Obstacle handling -------------------------------------------------
    obstacle_threshold_cm: float = 25.0
    obstacle_samples: int = 5
    sensor_period_s: float = 0.05

    # --- Slalom / wall alignment ------------------------------------------
    wall_reorient_speed_cm_s: float = 2.0
    wall_reorient_tolerance_cm: float = 1.5
    wall_reorient_timeout_s: float = 12.0

    # --- Gripper -----------------------------------------------------------
    gripper_dps: float = 20.0
    gripper_time_s: float = 2.0

    # --- Light sensor calibration -----------------------------------------
    # Raw NXT light-sensor values observed on the two arena floors.
    white_floor_range: tuple[int, int] = (1200, 1900)
    black_floor_range: tuple[int, int] = (2200, 2800)


@dataclass(frozen=True)
class VisionConfig:
    """Camera and perception parameters."""

    camera_index: int = 0
    frame_width: int = 320
    frame_height: int = 240
    camera_fps: int = 20

    # --- Red ball (HSV, OpenCV ranges: H 0-179, S/V 0-255) -----------------
    ball_lower_red_1: tuple[int, int, int] = (0, 150, 50)
    ball_upper_red_1: tuple[int, int, int] = (10, 255, 255)
    ball_lower_red_2: tuple[int, int, int] = (170, 150, 50)
    ball_upper_red_2: tuple[int, int, int] = (180, 255, 255)
    ball_min_area_px: float = 300.0
    ball_target_area_px: float = 12000.0
    ball_area_tolerance_px: float = 900.0
    ball_center_tolerance_px: float = 12.0
    ball_max_linear_speed_cm_s: float = 20.0
    ball_max_angular_speed_rad_s: float = 1.0
    ball_search_angular_speed_rad_s: float = 0.25
    ball_timeout_s: float = 20.0

    # --- Droid markers -----------------------------------------------------
    # ORB needs enough features to survive the low resolution of the 320x240
    # camera stream; 300 features gave only 7 inliers on the recorded test
    # frame, while 1000 gives ~30.
    marker_orb_features: int = 1000
    marker_min_matches: int = 20
    marker_min_inliers: int = 15
    marker_ransac_reproj_px: float = 3.0
    marker_target_area_px: float = 14000.0
    marker_area_tolerance_px: float = 1000.0
    marker_center_tolerance_px: float = 15.0
    marker_max_linear_speed_cm_s: float = 15.0
    marker_max_angular_speed_rad_s: float = 0.4
    marker_search_angular_speed_rad_s: float = 0.2
    marker_timeout_s: float = 25.0


@dataclass(frozen=True)
class ArenaConfig:
    """Everything that changes between the two competition arenas."""

    key: str
    map_path: Path
    start_pose: Pose
    start_cell: tuple[int, int]
    goal_cell: tuple[int, int]
    assigned_marker: str
    slalom_profile: str
    description: str

    @property
    def map_name(self) -> str:
        return self.map_path.name


ARENAS: dict[str, ArenaConfig] = {
    "a": ArenaConfig(
        key="a",
        map_path=MAPS_DIR / "arena_a.txt",
        start_pose=Pose(x=60.0, y=100.0, theta=-3.141592653589793 / 2.0),
        start_cell=(1, 2),
        goal_cell=(3, 3),
        assigned_marker="R2D2",
        slalom_profile="white",
        description="White floor arena; the assigned exit marker is R2-D2.",
    ),
    "b": ArenaConfig(
        key="b",
        map_path=MAPS_DIR / "arena_b.txt",
        start_pose=Pose(x=220.0, y=100.0, theta=-3.141592653589793 / 2.0),
        start_cell=(5, 2),
        goal_cell=(3, 3),
        assigned_marker="BB8",
        slalom_profile="black",
        description="Black floor arena; the assigned exit marker is BB-8.",
    ),
}

MARKER_REFERENCES: dict[str, Path] = {
    "R2D2": ASSETS_DIR / "markers" / "r2d2.png",
    "BB8": ASSETS_DIR / "markers" / "bb8.png",
}

DEFAULT_VISION_CONFIG = VisionConfig()
