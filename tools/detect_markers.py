#!/usr/bin/env python3
"""Annotate an image with R2-D2 / BB-8 marker detections.

Example
-------
    python tools/detect_markers.py \
        --image assets/samples/markers_test.jpg \
        --output docs/images/marker_detection.jpg
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402

from robot.vision import MarkerDetector  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--show", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None:
        print(f"Could not read image: {args.image}")
        return 2

    detector = MarkerDetector()
    detections = detector.detect(frame)

    if not detections:
        print("No markers detected.")
        return 1

    for label, detection in sorted(detections.items()):
        print(
            f"{label}: center=({detection.center[0]:.1f}, {detection.center[1]:.1f}) "
            f"area={detection.area_px:.0f}px matches={detection.matches} "
            f"inliers={detection.inliers}"
        )
    print("sides:", detector.sides(detections) or "only one marker visible")

    annotated = detector.annotate(frame, detections)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output), annotated)
        print(f"Saved: {args.output}")
    if args.show:
        cv2.imshow("marker detections", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
