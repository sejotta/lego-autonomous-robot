# Algorithms

This document explains the math behind the implementation. The code follows the same order; each section points to the relevant module.

## Notation

| Symbol | Meaning | Unit |
|--------|---------|------|
| `R` | wheel radius | cm |
| `L` | distance between wheel contact points | cm |
| `v` | body linear speed | cm/s |
| `w` (or `ω`) | body angular speed | rad/s |
| `ω_l`, `ω_r` | left/right wheel angular speeds | rad/s |
| `d` | distance to a waypoint | cm |
| `θ` | robot heading | rad |
| `A` | apparent blob area in the image | px² |
| `A*` | target apparent area | px² |

The robot frame is `+x` forward, `+y` left, and positive `ω` is counter-clockwise.

## 1. Differential drive

### Forward kinematics

For a differential drive with no wheel slip:

$$
v = \frac{R(\omega_r + \omega_l)}{2}, \qquad
\omega = \frac{R(\omega_r - \omega_l)}{L}.
$$

### Inverse kinematics

Given a desired body twist `(v, ω)`:

$$
\omega_l = \frac{v - \omega L/2}{R}, \qquad
\omega_r = \frac{v + \omega L/2}{R}.
$$

`wheel_speeds_to_dps()` converts these to degrees per second for the BrickPi motor API; `dps_to_wheel_speeds()` is the inverse, used in tests and for speed estimation.

A positive `ω` makes the right wheel faster than the left, which turns the robot counter-clockwise (to its left). The convention is covered by `test_positive_angular_speed_turns_left_in_wheel_convention`.

## 2. Odometry

### Encoder integration

Every 50 ms the odometry thread reads the two encoders and computes the distance travelled by each wheel:

$$
\Delta s_l = R \, \Delta\theta_l, \qquad
\Delta s_r = R \, \Delta\theta_r.
$$

The body displacement and rotation are

$$
\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \qquad
\Delta\theta = \frac{\Delta s_r - \Delta s_l}{L}.
$$

The pose is integrated at the **midpoint heading**:

$$
x_{k+1} = x_k + \Delta s \cos\!\left(\theta_k + \frac{\Delta\theta}{2}\right),
$$

$$
y_{k+1} = y_k + \Delta s \sin\!\left(\theta_k + \frac{\Delta\theta}{2}\right),
$$

$$
\theta_{k+1} = \operatorname{normalize}(\theta_k + \Delta\theta).
$$

This is a second-order approximation of the true arc. It reduces the heading-dependent drift of the simpler "integrate at the previous heading" update, especially on the slalom arcs.

### Gyro fusion

The EV3 gyro reports an absolute angle in degrees. When `heading_source="gyro"`, the odometry thread replaces the integrated heading with

$$
\theta = \operatorname{normalize}\!\left(s_g \cdot \operatorname{rad}(\theta_{\text{gyro}}) + b\right),
$$

where `s_g = -1` matches the sensor mounting orientation and `b` is captured at start/reset so the gyro agrees with the initial pose. Encoder integration still provides `x` and `y`; the gyro only stabilises `θ`. If the gyro cannot be read, the thread logs a warning and falls back to wheel heading.

### Why the original wheel wrapping was removed

The coursework code applied `normalize_rad()` to each wheel's encoder delta. That is only valid while a wheel turns less than half a revolution between samples; at 20 Hz and 15 cm/s this is normally true, but a fast spin or a scheduling hiccup can wrap a delta and corrupt the heading. The refactor integrates raw deltas and leaves wrapping to the final pose angle.

## 3. NF1 wavefront planning

The map is a grid of 7 × 7 cells (40 cm each). `GridMap` stores the connection matrix from the course format and exposes `is_connected(cell, neighbour)`.

### Cost field

`fill_cost_matrix(goal)` runs a breadth-first search from the goal over the four-connected free cells:

1. Set `cost[goal] = 0`.
2. Push the goal into a FIFO queue.
3. For each cell, visit north/east/south/west. If the edge is open and the neighbour is unvisited, set `cost[neighbour] = cost[cell] + 1` and enqueue it.

The result is the shortest number of cell moves from every reachable cell to the goal. Unreachable cells stay at `-1`.

### Path extraction

Starting at the start cell, repeatedly move to the connected neighbour with the smallest cost. Because the field is a shortest-path distance function, every step decreases the cost by exactly one and the descent cannot get stuck in a local minimum:

$$
\text{cost}(c_{k+1}) = \text{cost}(c_k) - 1.
$$

The implementation also caps the number of steps at `size_x * size_y` as a guard against a malformed connection matrix.

### Replanning

When the robot is blocked between `current` and `next`, `replan()`:

1. Converts the cell-space direction `(dx, dy)` to the map neighbour index.
2. Sets that connection to `0`.
3. Rebuilds the wavefront and extracts a new path.

The edge stays closed for the rest of the mission, so the planner will not send the robot back into the same obstacle.

### Complexity

For `N = size_x * size_y` cells and four neighbours, the BFS is `O(N)` time and `O(N)` memory. Replanning repeats the BFS, which is negligible for a 49-cell arena.

## 4. Waypoint navigation

`Robot.go_to(x, y)` uses a rotate-then-drive primitive.

### Rotate to heading

Let

$$
e_\theta = \operatorname{normalize}(\theta^* - \theta).
$$

The angular command is a saturated proportional law:

$$
\omega = \operatorname{clip}(k_\theta e_\theta, -\omega_{\max}, \omega_{\max}).
$$

The robot stops when `|eθ| <= 0.05 rad` (about 3°). A minimum duty cycle of 0.12 rad/s is applied whenever the error is outside that band, to avoid stalling just outside it; the 50 ms loop bounds the resulting overshoot to about 0.006 rad. Every rotation has a deadline; a timeout returns `False` and the waypoint aborts.

### Drive to the point

After alignment:

$$
e_\psi = \operatorname{normalize}\!\left(\operatorname{atan2}(y_g - y, x_g - x) - \theta\right),
$$

$$
\omega = \operatorname{clip}(k_\psi e_\psi, -\omega_{\max}, \omega_{\max}),
$$

$$
v = \max\!\left(v_{\min}, v_{\max} \min\!\left(1, \frac{d}{d_{\text{slow}}}\right)\right).
$$

The `d / d_slow` factor slows the robot as it approaches the waypoint. If the heading error exceeds 0.7 rad, the robot turns in place instead of driving forward, which prevents cutting corners into walls.

The loop terminates on:

- `d <= 3 cm` → `REACHED`;
- front distance below 25 cm → `BLOCKED`;
- distance to the goal growing on three consecutive cycles → `STUCK`;
- 30 s elapsed → `TIMEOUT`.

The "distance getting worse" counter is a cheap slip/jam detector. It is not a substitute for wheel encoders or a current sensor, but it catches the common case of the robot pushing against a wall while the controller thinks it is still approaching.

## 5. Slalom and wall alignment

The slalom is a sequence of `(v, ω, duration)` segments calibrated on the track. The two profiles mirror each other for the white and black arenas:

| Segment | White arena | Black arena |
|---------|-------------|-------------|
| 0 | `ω = -π/4`, 2.3 s | `ω = +π/4`, 2.3 s |
| 1 | `v = 3.5π`, `ω = +π/10`, 10.5 s | `v = 3.5π`, `ω = -π/10`, 11.0 s |
| 2 | `v = 4π`, `ω = -π/10`, 10.4 s | `v = 4π`, `ω = +π/10`, 10.4 s |
| 3 | `ω = +π/4`, 2.1 s | `ω = -π/4`, 2.1 s |

At the two checkpoints, `approach_wall_distance(target)` closes the loop on the ultrasonic sensor:

$$
v = \operatorname{clip}(k_w (d_{\text{measured}} - d_{\text{target}}), -v_w, v_w),
$$

with a minimum magnitude of 0.5 cm/s and a 1.5 cm tolerance. This removes the accumulated heading/distance error before the maze phase, where odometry is re-initialised at the known start cell.

## 6. Ball detection and visual servoing

### Detection

The frame is converted to HSV and thresholded with two red ranges (the hue circle wraps around 0):

```text
H ∈ [0, 10]   ∪ [170, 180]
S ∈ [150, 255]
V ∈ [50, 255]
```

The two masks are OR-ed, opened with a 5 × 5 kernel to remove speckle, and dilated once. `cv2.findContours` returns external contours; the largest by area is the ball candidate. Its centroid comes from image moments:

$$
c_x = \frac{M_{10}}{M_{00}}, \qquad c_y = \frac{M_{01}}{M_{00}}.
$$

The apparent area is `cv2.contourArea`; a minimum area rejects noise.

### Servo

Let `e_A = (A* - A) / A*` and `e_x = (c_x - W/2) / (W/2)`, where `W` is the frame width.

$$
v = v_{\max} \operatorname{clip}(e_A, -1, 1), \qquad
\omega = \omega_{\max} \operatorname{clip}(e_x, -1, 1).
$$

The robot is aligned when both errors are inside their deadbands:

$$
|A^* - A| \le 900\ \text{px}^2, \qquad |c_x - W/2| \le 12\ \text{px}.
$$

If the ball is not visible, the robot rotates at 0.25 rad/s in the direction where it was last seen. This is a simple memory-based search; it avoids spinning the wrong way after the ball leaves the frame.

### A fixed gain bug from the original

The coursework controller computed

```python
kA = v_max / (AreaCoger - AreaGrande1)
v  = kA * (AreaCoger - AreaGrande1)
```

which algebraically cancels the error and always yields `v = v_max`. The refactored controller normalises by the target area instead:

```python
area_error = (target_area - area) / target_area
v = v_max * clip(area_error, -1, 1)
```

This is what makes the approach slow down as the ball gets closer.

## 7. Marker recognition

### Feature matching

The detector uses ORB with 1000 features per image (reference and frame). ORB is a rotated BRIEF variant with a FAST detector and an orientation estimate; it is fast enough on a Raspberry Pi and produces binary descriptors.

For each reference marker:

1. Detect keypoints and descriptors in the grayscale reference and frame.
2. Match with `BFMatcher(NORM_HAMMING, crossCheck=True)`. Cross-check keeps only mutual nearest neighbours, which removes many ambiguous matches.
3. Sort by Hamming distance.
4. Require more than 20 matches.

### Homography and inliers

The reference and the printed marker lie on a plane, so a projective transform maps one to the other:

$$
\begin{bmatrix} x' \\ y' \\ 1 \end{bmatrix}
\sim H
\begin{bmatrix} x \\ y \\ 1 \end{bmatrix}.
$$

`cv2.findHomography(..., cv2.RANSAC, 3.0)` estimates `H` and labels each correspondence as an inlier or outlier. The detector requires at least 15 inliers. On the recorded frame, the two markers produce 97 and 86 inliers respectively.

### Pose in the image

The four reference corners are projected with `cv2.perspectiveTransform`. Their mean is the marker centre; the polygon area is the apparent size used by the follower. The side of a marker is simply its centre x-coordinate relative to the other marker.

### Why 1000 features

At 300 features (the original setting) the 320 × 240 test frame produced only 7 RANSAC inliers for R2-D2, below the 15-inlier acceptance threshold. At 1000 features the same frame produces 31 and 27 inliers for the two markers, and on the cleaner test image used in the README, 97 and 86. The cost is a few milliseconds per frame on a desktop and still acceptable on a Pi 4.

### Failure modes

- **Low texture / motion blur** — too few keypoints; the detector returns no detection.
- **Repeated patterns** — cross-check matching usually rejects these, but a wrong homography can still pass if it has enough inliers. The inlier threshold and the RANSAC reprojection error are the main guards.
- **Marker partly outside the frame** — the projected polygon can extend beyond the image; the centre and area are still usable, but the approach should stop before that happens.
- **Similar markers** — R2-D2 and BB-8 share few features, so cross-matching is usually clean; if a scene contains near-duplicates, add a uniqueness check on the inlier counts.

## 8. Parameter summary

| Parameter | Default | Effect |
|-----------|---------|--------|
| `wheel_radius_cm` | 2.6 | Scales encoder distance and motor speed |
| `wheel_base_cm` | 15.0 | Scales differential rotation |
| `odometry_period_s` | 0.05 | Encoder sampling period |
| `max_linear_speed_cm_s` | 15.0 | Waypoint cruise speed |
| `position_tolerance_cm` | 3.0 | Waypoint acceptance radius |
| `heading_tolerance_rad` | 0.05 | Rotation deadband |
| `obstacle_threshold_cm` | 25.0 | Ultrasonic block distance |
| `ball_target_area_px` | 12000 | Apparent area at grasping distance |
| `marker_orb_features` | 1000 | ORB feature budget |
| `marker_min_inliers` | 15 | Homography acceptance threshold |
| `marker_target_area_px` | 14000 | Apparent marker area at the gate |

All values live in `robot/config.py`; tuning them does not require touching the controllers.
