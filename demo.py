#!/usr/bin/env python3
"""Offline NF1 planning demo -- no robot hardware required.

Examples
--------
    python demo.py
    python demo.py --map maps/arena_b.txt --start 5 2 --goal 3 3 \
        --output docs/images/planned_path_arena_b.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless-safe; the demo always writes a PNG

from robot.config import REPO_ROOT
from robot.kinematics import path_length
from robot.mapping import GridMap


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map",
        type=Path,
        default=REPO_ROOT / "maps" / "arena_a.txt",
        help="map file to plan on",
    )
    parser.add_argument("--start", nargs=2, type=int, metavar=("X", "Y"), default=(1, 2))
    parser.add_argument("--goal", nargs=2, type=int, metavar=("X", "Y"), default=(3, 3))
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "docs" / "images" / "planned_path_arena_a.png",
        help="PNG output path",
    )
    parser.add_argument("--title", default=None)
    return parser


def run(args: argparse.Namespace) -> int:
    start = tuple(args.start)
    goal = tuple(args.goal)

    grid = GridMap(args.map)
    print(f"Map: {args.map.name}")
    print(f"  size: {grid.size_x} x {grid.size_y} cells")
    print(f"  cell size: {grid.cell_size_cm:.1f} cm")
    print(f"  start: {start}  goal: {goal}")

    path = grid.plan(start, goal)
    if path is None:
        print("No path found.")
        return 1

    centers = [grid.cell_center_cm(cell) for cell in path]
    print(f"  path: {path}")
    print(f"  cells: {len(path)}")
    print(f"  geometric length: {path_length(centers):.1f} cm")
    print(f"  NF1 cost at start: {grid.cost_matrix[start]}")

    grid.draw(
        path=path,
        save_path=args.output,
        title=args.title or f"NF1 plan — {args.map.stem.replace('_', ' ').title()}",
    )
    print(f"Saved figure: {args.output}")
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
