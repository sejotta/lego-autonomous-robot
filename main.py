#!/usr/bin/env python3
"""Run the four-phase mission on the physical BrickPi3 robot.

The hardware is imported lazily so ``--help`` and the offline demo work on any
machine. Run the full mission with::

    python main.py --arena auto

Partial phases are useful for bench testing, but they assume the robot starts
in a known pose; the supported end-to-end path is ``--phase all``.
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path
from typing import Optional

from robot.config import DEFAULT_VISION_CONFIG, REPO_ROOT
from robot.mission import Mission, MissionError, PHASES, choose_arena
from robot.robot import Robot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arena",
        choices=("auto", "a", "b"),
        default="auto",
        help="arena selection; auto reads the floor light sensor",
    )
    parser.add_argument(
        "--phase",
        choices=("all",) + PHASES,
        default="all",
        help="run the full mission or a single phase",
    )
    parser.add_argument("--camera", type=int, default=None, help="override camera index")
    parser.add_argument(
        "--artifacts",
        type=Path,
        default=REPO_ROOT / "artifacts",
        help="directory for mission maps and generated figures",
    )
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING"))
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("main")

    try:
        from robot.hardware import BrickPiHardware, HardwareUnavailable
    except ImportError as exc:  # pragma: no cover - packaging issue
        logger.error("Could not import the hardware layer: %s", exc)
        return 2

    try:
        hardware = BrickPiHardware()
    except HardwareUnavailable as exc:
        logger.error("%s", exc)
        logger.error("Use 'python demo.py' for the hardware-free planning demo.")
        return 2

    vision_config = DEFAULT_VISION_CONFIG
    if args.camera is not None:
        vision_config = replace(vision_config, camera_index=args.camera)

    robot = None
    try:
        robot = Robot(hardware)
        arena = choose_arena(robot, args.arena)
        mission = Mission(
            robot,
            arena,
            vision_config=vision_config,
            artifacts_dir=args.artifacts,
        )
        phases = PHASES if args.phase == "all" else (args.phase,)
        mission.run(phases)
        return 0
    except MissionError as exc:
        logger.error("Mission failed: %s", exc)
        return 1
    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping the robot")
        return 130
    except Exception as exc:
        logger.exception("Unexpected failure: %s", exc)
        return 1
    finally:
        if robot is not None:
            robot.shutdown()


if __name__ == "__main__":
    sys.exit(main())
