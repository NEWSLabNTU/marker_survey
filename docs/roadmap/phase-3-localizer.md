# Phase 3: one localizer, 3D and planar

## Goal

A localizer node that reads `marker_map.yaml` and a camera, and publishes
the robot's pose from the markers in view, in one of two modes:

| Mode | Output | For |
|---|---|---|
| `3d` | 6-DoF pose | wall boards; a vehicle whose roll and pitch matter |
| `planar` | x, y, yaw (z, roll, pitch fixed) | a robot on a flat floor, ceiling or wall markers |

## Shared part

- Detection from phase 2 (same settings as the survey).
- The camera's mounting from TF (`base_frame -> camera optical frame`).
- Output `geometry_msgs/PoseWithCovarianceStamped` of `base_frame` in the
  map's `frame_id`, stamped with the image's capture time (fusing it at the
  wrong moment shifts the pose while driving).
- Covariance from the fit: grows with distance, shrinks with the number of
  markers and corners; a floor value so the estimator averages instead of
  jumping.
- Optional gate against a prior pose (`prior_topic`): drop a fix farther
  than `gate_distance` or `gate_angle_deg` from the state estimator. This
  rejects a misread ID, a moved marker, and a far marker with a poor angle.
- `enabled` parameter, settable at run time.
- `~/debug/image` with the detections and the fit residuals, published only
  while someone watches.

## Mode `3d`

Start from each marker's IPPE solution, then refine all corners of all
markers in view together (Levenberg-Marquardt on reprojection, robust
loss). One marker alone has the planar-target ambiguity; keep the IPPE
solution closer to the prior pose when there is one.

## Mode `planar`

Known: the camera's height and tilt on the robot (TF), each marker's plane
(from the map, so uneven ceilings and tilted boards are handled). Unknown:
x, y, yaw.

1. For each corner, the ray from the camera, rotated by the robot's roll and
   pitch (IMU, or zero on a flat floor).
2. Cut the ray with its marker's plane: a point on the plane in robot
   coordinates.
3. Fit x, y, yaw by 2D least squares between those points and the corners'
   map positions; Huber loss; residual per corner kept for the debug image.

This is the ceiling implementation's method with one change: each marker
uses its own plane instead of one ceiling height.

## Performance

Python, measured on the target computers before any port:

| Image | Detection | Fit | Budget |
|---|---|---|---|
| 640 x 480 grey (ceiling webcam) | to be measured | to be measured | 15 Hz, under 15 % of a core |
| 1920 x 1200 colour (wide forward camera) | to be measured | to be measured | 10 Hz; downscale for detection, refine corners at full size if needed |

A large image across processes costs bandwidth; the node can also run in
the camera driver's process (rclpy cannot be loaded as a C++ component, so
the option is a downscaled or region-of-interest image topic from the
driver).

## Tests

| Case | Target |
|---|---|
| ceiling, simulated, 9 markers, robot driving routes | planar: mean 1 cm, max 3 cm (source: 0.5 / 1.6 cm) |
| ceiling, two markers in view | planar within 2 cm where 3d is worse (source measured 36 cm for full 6-DoF) |
| wall course, rendered, 11 boards | 3d: no worse than the source localizer on the same frames |
| wall course on a flat floor | planar against 3d: report which is better per distance band |
| misread ID, moved marker | rejected by the gate |
| timing | stamps equal capture times |

## Tasks

- [ ] `core/localize.py`: both solvers, covariance, gate
- [ ] `localizer_node.py`: parameters, TF, debug image, `enabled`
- [ ] Simulated and rendered test cases above
- [ ] Timing on a laptop CPU and on an ARM board (Jetson class)
- [ ] Example launch files for both setups

## Done when

Each mode matches or beats its source implementation in the tests above,
and the timing budget holds on an ARM board.
