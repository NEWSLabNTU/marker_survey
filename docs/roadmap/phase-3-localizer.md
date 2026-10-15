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

- [x] `core/localize.py`: both solvers, covariance, gate
- [x] `localizer_node.py`: parameters, TF, debug image, `enabled`
- [x] Simulated and rendered test cases above
- [x] Timing on an x86 CPU and on an arm64 core (CI runner); a Jetson-class
      board is still open (see Result)
- [x] Example launch files for both setups

## Done when

Each mode matches or beats its source implementation in the tests above,
and the timing budget holds on an ARM board.

## Result

Both modes pass every test of the table, as corner-level simulations
(`marker_localizer/test/test_localize.py`, 0.5 px corner noise), as rendered
frames compared with the source implementations' methods
(`ci/localizer_comparison.sh`, blur 0.8 px and sensor noise of 3 grey
levels) and as rendered recordings played through the node
(`ci/synthetic_localize.sh`); the numbers are the same on Humble (OpenCV
4.5.4) and Jazzy (4.6.0). The timing budget holds on a desktop x86 core and
on an arm64 server core (GitHub's `ubuntu-24.04-arm` runner); a
Jetson-class board was not available, so that part of "done when" is open.
CI run 38673801192 is green: unit tests, the rendered recordings through
both nodes and the comparisons, on Humble and Jazzy, and the timing on
both runners.

Errors are against the true base pose, mean / max, in cm (x, y for planar
and the ceiling source; x, y, z for 3d and the wall source):

| Case | Target | Corner-level | Rendered | Through the node |
|---|---|---|---|---|
| ceiling, office, 9 markers, two laps, planar | mean 1, max 3 (source in its simulator: 0.5 / 1.6) | 0.38 / 2.44 | 0.01 / 0.09 | 0.02 / 0.13 |
| the ceiling source's method, same frames | | 0.38 / 2.45 | 0.09 / 0.54 | |
| uneven ceiling, tilted markers, planar | | 0.43 / 2.59 | 0.01 / 0.09 | |
| the ceiling source's method, same frames | | 0.78 / 4.60 | 0.55 / 3.86 | |
| ceiling, two markers in view: planar | within 2 | 0.14 / 0.37 | 0.01 / 0.03 | |
| the same frames: 3d | worse (source: 36 cm) | 2.50 / 7.25 | 0.15 / 0.46 | |
| ceiling, one marker in view: planar / 3d | | 0.42 / 2.44, 10.99 / 65.59 | 0.02 / 0.09, 0.35 / 2.10 | |
| wall course, 11 boards, one lap, 3d | no worse than the source | 5.36 / 38.6 (413 fixes of 426 frames) | 0.82 / 6.77 (393 of 423) | 0.90 / 7.80 (393 of 426) |
| the wall source's method, same frames | | 33.5 / 102 (321) | 5.52 / 97.3 (353) | |
| moved marker (1.5 m), misread id (two ids swapped) | rejected by the gate | every single-marker fix rejected, both modes | | 92 fixes gated, published max 0.13 |
| timing | stamps equal capture times | | | every fix's stamp is its image's |

Planar against 3d on the wall course (flat floor), x, y error by the mean
distance to the markers in the fit, rendered, mean / max cm (fixes):

| Distance | 3d | planar |
|---|---|---|
| 0 to 3 m | 1.04 / 4.80 (46) | 0.97 / 3.58 (45) |
| 3 to 5 m | 0.46 / 3.22 (235) | 0.39 / 3.17 (225) |
| 5 to 8 m | 1.07 / 6.71 (109) | 1.00 / 6.47 (104) |
| over 8 m | 1.82 / 3.18 (3) | 0.76 / 0.82 (2) |

Planar is slightly better in every band; for wall boards the gain is small,
because the angle planar fixes (yaw) trades with a sideways shift just as a
6-DoF solve does, while on a ceiling it is the rotation in the image plane,
which four corners give well. Corner-level, over the whole lap, planar was
3.43 / 28.2 cm against 5.36 / 38.6 cm for 3d. For ceilings the difference
is the one the source measured: with one marker in view (86 % of the
office frames) 3d is 26 times worse.

The covariance is consistent with the errors: 99 to 100 % of the fixes lie
inside the 99 % ellipse of their own covariance in every case above.

### How the comparisons were done

- **Ceiling source.** Its pure-Python fit and detector were run, unchanged,
  from a read-only checkout on the same rendered office and uneven-office
  frames (a scratch script, not in this repository); the re-implementation
  in `core/source_methods.py` (horizontal plane at each marker's height,
  plain 2D rigid fit, 5 cm RMS limit, its detector settings) reproduces it
  within 0.008 cm on every frame and is what the tests and CI use. On a flat
  ceiling with the same corners, the planar solver gives the same pose as
  the source (corner-level, 0.38 / 2.44 against 0.38 / 2.45 cm): the
  geometry is the same. The gain on rendered frames comes from the
  detection (the edge fit of phase 0, 0.09 against 0.01 cm mean); on the
  uneven ceiling from each marker's own plane (4.60 against 2.59 cm maximum
  corner-level). The source's 0.5 / 1.6 cm come from another simulator and
  renderer; our renderer has no lens model and its corners are easier, so
  the corner-level column with 0.5 px noise is the fairer reading of the
  target.
- **Wall source.** It is a C++ node on a large autonomous-driving stack; it
  was not built. `core/source_methods.py` implements its method from its
  source and parameters: each marker's single-view IPPE pose (the lower
  reprojection error of the two), markers farther than 5 m ignored, each
  detection placed in the map with the estimator's pose and matched to the
  nearest map marker whatever its id, the base pose from the detection
  closest to its match, dropped beyond 1 m from the estimator's pose, a
  constant covariance scaled by max(1, (d/5)^3). It gets the same detections
  as ours (it uses another library's detector), and where OpenCV 4.5's IPPE
  fails (phase 0) it gets the general solver's pose instead, so that neither
  the detector nor an OpenCV defect counts against it. Its large errors come
  from single-marker orientations at 3 to 8 m (one fix 97 cm off), which the
  joint fit over all corners avoids.
- The prior pose (the estimator's) is the truth with 5 cm per axis and
  1 degree of noise in every case; in the recordings it is `/prior_pose`.

### Timing

`ci/timing.sh`: one core, one OpenCV thread, rendered frames with noise
(rendering not timed), mean per frame over 150 to 200 frames:

| Image | Mode | Desktop x86 core | x86 server core (CI runner) | arm64 server core (CI runner) | Budget |
|---|---|---|---|---|---|
| 640 x 480 grey, tag36h11 | planar | 2.0 + 2.0 ms | 1.7 + 1.8 ms | 1.8 + 2.0 ms | 15 Hz, under 15 % of a core (10 ms per frame) |
| 1920 x 1200 colour, DICT_5X5_1000 | 3d | 19.2 + 4.8 ms | 30.0 + 7.4 ms | 39.2 + 8.2 ms | 10 Hz (100 ms per frame) |
| the same, `detect_scale` 0.5 | 3d | 3.9 + 4.8 ms | 5.6 + 7.3 ms | 8.1 + 8.1 ms | |

(detection + fit; CI run 38673801192, OpenCV 4.6 on both runners; the
desktop with OpenCV 4.5.4, on a shared machine under load, where repeated
runs varied by up to a factor of two.) On the arm64 core the ceiling webcam
at 15 Hz takes 6 % of the core, the wide camera at 10 Hz 47 %, or 16 % with
`detect_scale` 0.5 (whose corners are refined on the full image; the
largest error on the lap grew from 1.2 to 4.0 cm). So the budget holds on
arm64. These are server cores, though: a Jetson-class board (Cortex-A78AE
at a lower clock) is likely two to three times slower per core, which would
still fit the ceiling budget and, with `detect_scale` 0.5, the wide
camera's. Measuring on such a board is open.

Two detector changes made this possible (shared with the survey,
`marker_survey/core/detection.py`), each measured on the same frames:

- On a frame with sensor noise, `CORNER_REFINE_APRILTAG` spent 110 to
  130 ms on 640 x 480 in AprilTag's own quad search over the whole image.
  A contrast floor (`aprilTagMinWhiteBlackDiff` 30) and, from OpenCV 4.6 on,
  a quad decimation of 2 bring it to 2 to 5 ms with the same markers found
  and corners within 0.05 px after the refinement. OpenCV 4.5 hangs in that
  search with any decimation on some clean frames, so it does not decimate.
- The edge fit sampled the whole image as float and fitted lines with a full
  SVD: 25 ms per 1920 x 1200 frame. It now works on the marker's
  surroundings, vectorised (corners within 0.006 px of before, the
  interpolation's fixed-point rounding).

### Pieces

- `marker_localizer/core/localize.py` (no ROS): `Localizer.locate`
  (detections, prior, roll, pitch) -> `Result` with a `Fix` (pose,
  covariance in ROS order, markers used, per-corner residuals in metres
  (planar) or px (3d), projected map corners for the debug image) or the reason there
  is none; the gate; markers not in the map, cut by the border, too small,
  too far (`max_distance`) or outliers (more than `outlier_px` after the fit,
  then refitted without them) are listed with the reason.
- `marker_localizer/localizer_node.py` and `params.py`: parameters checked
  without ROS (`test_params.py`); camera mounting from TF; roll and pitch
  (`roll_pitch`: `zero`, `tf` from a level frame, `imu`); prior from
  `prior_topic` (PoseWithCovarianceStamped or Odometry), interpolated at the
  image's stamp; `enabled` settable at run time; `~/debug/image` only while
  subscribed; `process_all` for recordings (every frame, in order, each once
  its prior has arrived); a status line every 10 s with fixes, rejections
  and the detection and fit times.
- `core/source_methods.py`, `tools/evaluate.py` (`marker_localizer_evaluate`:
  any scene, rendered or corner-level, every method, by marker count and
  distance band, timing), `tools/score.py` (`marker_localizer_score`: a
  recording of the fixes against the truth, with the stamp check).
- `marker_survey_synthetic` also writes `/prior_pose` and
  `truth_poses.csv`.
- `marker_localizer/launch/localizer.launch.xml`;
  `examples/launch/ceiling_markers_localizer.launch.xml` (planar, an
  Odometry prior) and `wall_boards_localizer.launch.xml` (3d,
  `detect_scale` 0.5).
- CI: `synthetic-localize` (ceiling, ceiling_gate, wall through the node),
  `localizer-comparison` (ceiling, wall), both on Humble and Jazzy, and
  `timing` on `ubuntu-24.04` and `ubuntu-24.04-arm`.

### Deviations, and why

- **A fix is also dropped when its position sd is over 0.25 m**
  (`max_position_sd`; not in the plan). One wall board seen alone from 5 to
  8 m gave fixes up to 77 cm off, correctly with an sd of 45 to 80 cm, and
  inside a 1 m gate. A state estimator would weight them low, but not every
  one gates by Mahalanobis distance; dropping them cost 30 of 423 wall
  frames and took the maximum of the rendered lap from 77 cm to under 8 cm. Set it to 0 to keep all.
- **Robust loss `soft_l1`, not Huber**, as in the survey (phase 0): scipy's
  `huber` zeroes the Jacobian rows of residuals in its linear zone.
- **Planar residuals are weighted to pixels** (metres on the plane divided
  by the metres one pixel spans there, foreshortening included), so near and
  far markers count by how well they are seen and the covariance comes out
  in the same units as the 3d one; the residuals kept for the debug image
  are in metres.
- **Covariance of planar fixes**: x, y, yaw from the fit, plus the effect of
  a roll or pitch error of `tilt_sd_deg` (0.2 degree) at the markers'
  distance; z, roll and pitch carry a variance of 1000 (not measured), as in
  the ceiling source. Floors: 1 cm and 0.3 degree.
- **3d with one marker and a prior**: of the IPPE solutions that reproject
  within three times the best (or under 1 px), the one closest to the prior
  is kept; without a prior, the best. On 200 views of one board from 3 to
  6 m this lowered the mean and 95th-percentile error
  (`test_single_marker_ambiguity_resolved_towards_the_prior`).
- **The planar start** is the ceiling source's own method (horizontal planes
  at the corners' heights, 2D rigid fit) when every marker in view is near
  horizontal, else the 3d solution; the fit then uses each marker's plane.
- **The gate test through the node** uses a moved marker; the misread id
  (two ids swapped) is tested corner-level. Through the node, `process_all`
  waits for the prior of each frame: on a loaded machine the prior messages
  arrived up to 3 s late and 20 % of the wall frames were first processed
  without one.
- **`roll_pitch` tf and imu** are tested in the core (a robot tilted 1.5 and
  2 degrees: under 0.2 cm with its roll and pitch, over 5 cm without), not through
  the node; the recordings drive a flat floor.
- **Running the node in the camera driver's process** is not done (rclpy
  nodes cannot be composed into a C++ container); `detect_scale` is the way
  to keep a large image cheap.
