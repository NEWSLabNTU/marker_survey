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

- [ ] Workspace layout, `package.xml` and `setup.py` for the three packages
- [ ] Import and split the survey code; rename as above
- [ ] Import the ceiling helpers with tests
- [ ] Synthetic recording without course files (own marker map + path CSV)
- [ ] CI on Humble and Jazzy, with the synthetic survey check
- [ ] Neutral-wording check in CI

## Done when

Both distros build and pass, and the synthetic survey reaches the source's
documented accuracy (rendered recording: median 2.9 cm, max 4.5 cm).
