# Validation

This page records what was actually run for this repository, what the results were, and what remains hardware-dependent. It is deliberately separate from the README so the claims can be checked.

## Environment

| Item | Value |
|------|-------|
| Python | 3.11.7 |
| NumPy | 1.23.3 |
| OpenCV | 4.13.0 |
| Matplotlib | 3.10.8 |
| OS used for the public checks | Windows 11 |
| Robot runtime | Raspberry Pi + BrickPi3 (not used for the public checks) |

The code targets Python 3.10+ and has no OS-specific dependency in the hardware-independent modules.

## Unit tests

```bash
python -m unittest discover -s tests -v
```

Result:

```text
Ran 20 tests

OK
```

| Test group | Count | Covers |
|------------|-------|--------|
| Kinematics | 7 | Forward/inverse kinematics, encoder deltas, midpoint integration, angle wrapping |
| Mapping | 7 | Map parsing, NF1 paths on both arenas, cost gradient, unreachable goal, replanning, malformed header |
| Vision | 6 | Synthetic red ball, blank frame, ball commands, recorded marker frame, blank marker frame, marker follower |

The marker test uses `assets/samples/markers_test.jpg`, a real 320 × 240 camera frame of the printed markers. It asserts that both markers are found, that R2-D2 is left of BB-8, and that each has at least 15 RANSAC inliers.

## NF1 planning

```bash
python demo.py --map maps/arena_a.txt --start 1 2 --goal 3 3
python demo.py --map maps/arena_b.txt --start 5 2 --goal 3 3
```

| Arena | Start | Goal | Path | Cells | Geometric length | Cost at start |
|-------|-------|------|------|-------|------------------|---------------|
| A (white) | `(1, 2)` | `(3, 3)` | `(1,2) → (2,2) → (3,2) → (3,3)` | 4 | 120.0 cm | 3 |
| B (black) | `(5, 2)` | `(3, 3)` | `(5,2) → (4,2) → (3,2) → (3,3)` | 4 | 120.0 cm | 3 |

Both paths are four-connected, every step is an open connection, and the cost decreases by exactly one per step. The generated figures are:

- `docs/images/planned_path_arena_a.png`
- `docs/images/planned_path_arena_b.png`

The replanning test closes the first edge of the arena-A path and verifies that the new path does not use it. The unreachable-goal test isolates the top-right cell and verifies that planning returns `None` rather than a partial route.

## Marker recognition

```bash
python tools/detect_markers.py \
  --image assets/samples/markers_test.jpg \
  --output docs/images/marker_detection.jpg
```

Result:

```text
BB8: center=(216.7, 139.6) area=20427px matches=206 inliers=86
R2D2: center=(46.6, 135.7) area=25044px matches=229 inliers=97
sides: {'R2D2': 'left', 'BB8': 'right'}
```

The annotated output is `docs/images/marker_detection.jpg`. Both projected polygons land on the printed markers and the side classification is correct.

### Feature-count sensitivity

The original coursework detector used 300 ORB features. On a harder frame from the same recorded set, 300 features produced only **7** RANSAC inliers for R2-D2, below the 15-inlier acceptance threshold. Raising the budget to 1000 produced **31** inliers for R2-D2 and **27** for BB-8 on the same frame. The default in `VisionConfig` is therefore 1000.

## Ball detection

The ball test is synthetic: a 320 × 240 black frame with a red circle at `(200, 120)` and radius 40 px. The detector returns a centre within 3 px and an area above 3000 px². A blank frame returns `None`.

A recorded ball frame was not included because the available snapshots contain people and are not suitable for a public repository. The HSV ranges and target area are configuration values and must be retuned for the final camera, lighting, and ball size.

## What is not validated here

| Item | Status |
|------|--------|
| Full mission on the physical robot | Developed during the course; not reproducible in this environment |
| Motor direction / wheel-radius calibration on the final build | Hardware-specific; use `--phase` bench tests |
| Gripper force and ball retention | Hardware-specific; the gripper is time-controlled |
| CoppeliaSim scenes | Not included; process documented in `SIMULATION.md` |
| Long-run odometry drift | Depends on the floor, wheels, and gyro; resets at known cells |

## Issues found during the refactor

The original coursework folder contained several iterations and a few integration bugs. The public repository is a cleaned refactor, not a copy of the final scripts. The main fixes:

1. **Broken indentation.** `mainBuenoGuillen.py` and `mainTrabajo1612.py` both contain an over-indented block after `if obstaculo:`, which raises `IndentationError`. The logic was reorganised into `Mission.run_maze()`.
2. **Swapped marker results.** `detect_robots_from_cam()` returned `{"R2D2": cx2, "BB8": cx1}` even though `cx1` belonged to R2-D2. The detector now keys detections by label.
3. **Wrong template shape.** `match_images_pose()` used `self.ref1_gray.shape` for both markers, so the BB-8 polygon was projected with the R2-D2 dimensions. Each template now carries its own shape.
4. **Constant ball speed.** The original gain `v_max / (target_area - area)` cancelled the area error, making `v` constant. The controller now normalises by the target area.
5. **Unsafe target condition.** `followRobot_Img()` used `if centroid_xCercana is not None or AreaImg is None`, which enters the block when the marker is missing, and could use an uninitialised `v`. The follower uses a typed command and an explicit search state.
6. **Simulator/real port mismatch.** The simulation used `B`/`C`/`A` for right/left/claw while the real robot used `D`/`C`/`B`. Ports are now configuration values.
7. **Out-of-map exit cells.** The original door routine used cells such as `[7, 8]` on a 7 × 7 map. The mission now approaches the recognised marker directly and does not plan outside the arena.
8. **Mixed units.** Millimetres, centimetres, and cell indices were interleaved in the navigation code. The refactor keeps one convention and centralises the cell-centre conversion.
9. **Odometry lifecycle.** The original `Process` was not joined and could outlive the main program. Odometry is now a joinable thread with a stop event and an optional CSV log.
10. **Camera failure handling.** `if not ret and not frame` is the wrong condition for a failed read. The camera helpers raise `CameraError` and the mission reports it.

These are behavioural changes only where the original behaviour was incorrect; the mission phases, sensor choices, and control structure follow the coursework design.

## Demo media

The GIFs and stills in `docs/gifs/` and `docs/images/` are the authors' own captures, optimized for web:

| Original capture | Included as | Size |
|---|---|---|
| `gifvision.gif` (800 × 955, 120 frames) | `docs/gifs/orb-ransac.gif` | 1.96 MB |
| `followrobb.gif` (478 × 850, 200 frames) | `docs/gifs/marker-following.gif` | 3.27 MB |
| WhatsApp run (478 × 850, 240 frames) | `docs/gifs/ball-tracking.gif` | 3.26 MB |
| `robot_autonomous map.png` | `docs/images/trajectory.png` | 173 KB |
| `vandwcurves.png` | `docs/images/velocity-profile.png` | 142 KB |
| `detect_blob.png` | `docs/images/blob-detection.png` | 273 KB |

The raw recordings (33 MB, 76 MB and 82 MB) and the 4 MB poster are not committed. The GIFs were resized, frame-sampled, and quantized with a shared palette; the motion and the detected overlays remain visible. The ORB GIF is cropped to the detection window so the terminal log does not dominate the frame.

## Contrast with the explanatory poster

The poster `Cartel_Robot.pdf` and this repository describe the same project and agree on the main pipeline:

| Topic | Poster | Repository |
|---|---|---|
| Pose estimation | Encoder odometry + differential kinematic model | Same, plus EV3 gyro fusion |
| Global planning | NF1 cost field from the goal + gradient descent | Same |
| Ball perception | HSV red mask → blob detector → area as distance | Same; area filter, largest contour |
| Visual servoing | Centroid error → closed-loop `(v, ω)` | Same |
| Simulation | Validate and tune before final implementation | Same, documented in `SIMULATION.md` |

Three implementation details are more specific in the code than in the poster:

1. **Replanning.** The poster says the robot marks occupied cells and increases their cost. The implemented `GridMap.replan()` closes the blocked graph edge between the current cell and the neighbour and rebuilds the BFS cost field. The operational result is the same — avoid the blocked move and find a new route — but the bookkeeping is edge-based rather than cell-cost-based.
2. **Ball filter.** The poster mentions filtering by area/circularity. `BallDetector` currently filters by area only and keeps the largest red contour; circularity is not used. This is a deliberate simplification because the ball is the dominant red region in the arena.
3. **Marker recognition.** The poster describes “detección de imágenes por comparación”. The implementation uses ORB keypoints, a brute-force Hamming matcher, and a RANSAC homography, which is a feature-based comparison rather than raw template correlation. The result is a projected polygon, centre, and apparent area.

The poster credits **Guillén Sainctavit** and **Sergio Toro**; this repository uses **Sergio Toro Codo** and **Guillén Sainctavit**. The poster’s “4º GIEA” corresponds to the 4th-year GIEA group of the **Escuela de Ingeniería y Arquitectura (EINA), Universidad de Zaragoza**.

The poster is intentionally not vendored: it duplicates the technical documentation, is 4 MB, and is not needed to build or understand the robot. The maintained, code-accurate explanation is this repository’s `README.md` and `docs/`.
