# Phase 0: the repository

## Goal

A ROS 2 workspace layout that builds on Humble and Jazzy, with the survey
code moved in from the wall-board implementation and the ceiling helpers
from the ceiling implementation, renamed, cleaned of project-specific names,
and tested in CI.

## Packages

| Package | Contents | Depends on |
|---|---|---|
| `marker_survey` | `marker_survey.core`: detection, SE(3) helpers, the survey and its bundle adjustment, the map file, the report (plain Python, no ROS imports); the survey node; command-line tools | numpy, scipy, OpenCV (`python3-opencv`), PyYAML, rclpy |
| `marker_localizer` | the localizer core (no ROS) and node (phase 3) | `marker_survey` |
| `marker_survey_web` | the browser interface (phase 5) | `marker_survey`, `marker_localizer` |

The `core` modules never import rclpy, so they run in tests, notebooks and
offline tools without a ROS installation.

## What comes in

From the wall-board implementation, as a fresh import (its history speaks of
one project's exercises and stays there):

| Source file | Becomes | Change |
|---|---|---|
| `survey.py` | `core/survey.py`, `core/se3.py`, `core/detection.py` | split; drop the course-specific comments |
| `finish.py` | `core/solve.py` + the `marker_survey_solve` tool | writes `marker_map.yaml` (phase 1) instead of a course |
| `output.py` | `core/report.py` | course and lanelet2 writing removed |
| `simulate.py`, `synthetic_bag.py` | `core/simulate.py`, `marker_survey_synthetic` and `marker_survey_compare` tools | read a marker map and a path CSV of their own, not a course |
| `marker_survey_node.py` | `marker_survey/survey_node.py` | parameters renamed (below); no vehicle launch inside |
| `config/zed_survey.yaml`, launch file | `examples/` | camera driver settings are an example, not a dependency |

From the ceiling implementation: the tag36h11 detector settings
(`CORNER_REFINE_APRILTAG`, `cornerSubPix`, border rejection), the ray to
plane geometry (`marker_pose.py`), the marker size and spacing planner
(`marker_plan.py`), the camera tilt fit (`mount_calibration.py`), each with
its unit tests.

## Renames

| Before | After |
|---|---|
| `course_dir` | `output_dir` |
| the source's solve tool | `marker_survey_solve` |
| the source's synthetic recording tool | `marker_survey_synthetic` |
| the source's compare tool | `marker_survey_compare` |
| `origin_id` | `frame.origin_marker` (phase 2) |
| `floor_offset` | unchanged |

## CI

GitHub Actions on `ros:humble` and `ros:jazzy`:

1. `colcon build`, `colcon test` (unit tests of the core and the nodes'
   parameter handling);
2. a synthetic survey: render a recording of a wall course, play it through
   the node, solve, compare with the truth; fail above 4 cm median;
3. a lint step for project names and personal data (a word list), so the
   repository stays neutral.

OpenCV differs between the two (4.5.4 on Humble, 4.6 on Jazzy); the
detector already handles both APIs.

## Tasks

- [x] Workspace layout, `package.xml` and `setup.py` for the three packages
- [x] Import and split the survey code; rename as above
- [x] Import the ceiling helpers with tests
- [x] Synthetic recording without course files (own marker map + path CSV)
- [x] CI on Humble and Jazzy, with the synthetic survey check
- [x] Neutral-wording check in CI

## Done when

Both distros build and pass, and the synthetic survey reaches the source's
documented accuracy (rendered recording: median 2.9 cm, max 4.5 cm).

## Result

Done. Measured with `ci/synthetic_survey.sh wall` (the source's rendered
test: the 11 boards of the test course, 0.38 m, forward camera 960 x 600 at
102 degrees, one lap at 3 Hz with four turns on the spot, odometry drift
1 % and 0.3 degrees per metre, frame of marker 10), played through the
survey node:

| | median | max |
|---|---|---|
| source (its README) | 2.9 cm | 4.5 cm |
| Humble (OpenCV 4.5.4) | 0.78 cm | 1.36 cm |
| Jazzy (OpenCV 4.6.0) | 0.74 cm | 1.36 cm |

Corner-level simulation of the same course with the source's settings
(0.48 m boards, 1920 x 1200, two laps): median 0.35 cm, max 0.54 cm
(source: 1.0 cm, 4.0 cm). The CI gate is the phase's target, median
3 cm and max 4.5 cm.

### What changed on the way, and why

The import kept the method; five defects of the source showed up while
matching its accuracy and were fixed, each measured:

1. **Robust loss.** scipy's `huber` loss zeroes the Jacobian rows of
   residuals in its linear zone (the scale `rho' + 2 rho'' f^2` is 0 there).
   The start keyframe's observations, which fix the frame, started as
   outliers, dropped out of the solve, and the map drifted from them: the
   normal matrix had a near-zero direction (a global translation) and the
   uncertainties were meaningless. `soft_l1` (Huber-like, never zero) took
   the corner-level two-lap survey from median 1.13 cm to 0.35 cm.
2. **Single-view pose.** On OpenCV 4.5, `SOLVEPNP_IPPE_SQUARE` returns a
   wrong rotation (20 px reprojection error) or NaN for a marker whose
   centre is at the camera's height, the normal case for wall boards seen
   from a vehicle; 75 % of the rendered detections were thrown away. The
   pose now takes the best of IPPE_SQUARE, IPPE, the iterative solver and
   SQPnP, refined by Levenberg-Marquardt.
3. **Corner bias.** `cornerSubPix` leaves corners 0.15 to 0.2 px inside the
   black square; markers then look about 1 % smaller and farther away,
   which put the origin marker 0.3 degrees off and every marker 4 cm off at
   5 m. Detection now fits a line to each edge of the square and intersects
   them (bias under 0.01 px down to 25 px markers). OpenCV's own
   `CORNER_REFINE_SUBPIX` was worse on small markers: its 5 px window
   reaches the bits inside and pulls corners up to 3 px off.
4. **Rendering.** The synthetic images aliased small markers (edges jumping
   by up to a pixel); markers are now drawn at three times the resolution
   and averaged down.
5. **Uncertainty.** The marker covariances came from a dense
   pseudo-inverse of the whole normal matrix: 130 s for two laps, and
   unreliable. They now come from a Schur complement over the keyframes
   (under a second), and are expressed in the result frame (relative to
   the origin marker in `origin_marker` mode).

Other differences from the plan:

- `core/marker_map.py` landed now, because the synthetic recording reads a
  marker map; phase 1 adds the schema, the check tool, the importers and
  the tests.
- The node's parameters already follow phase 2 (`frame.mode`,
  `frame.origin_marker`, `pose_source: odometry | tf`; `odom` is accepted).
  `pose_frame` replaces the source's `world_frame`.
- `examples/` holds generic launch files (a wall-board vehicle with
  visual-inertial odometry, a ceiling robot with a LiDAR map) instead of one
  camera driver's settings, so the repository names no product.
- The word list of the neutral-wording check is stored as SHA-256 hashes
  (`ci/banned_words.sha256`), so the list itself names nobody; it checks the
  files, their names and every commit message, and e-mail addresses.
- The packages' maintainer field is a neutral placeholder.
