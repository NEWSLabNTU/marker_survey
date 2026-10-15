# Phase 4: adoption by robot projects

## Goal

A robot project uses the packages as a git submodule (or from a release)
with nothing but its own launch file, its camera remappings and its state
estimator's configuration. This phase writes the guide and the examples for
that, and moves the two source implementations onto the packages.

## What a robot project provides

| Item | Example |
|---|---|
| Camera topics | rectified image and `camera_info` |
| TF | `base_frame -> camera optical frame` (URDF); a pose source for the survey (odometry topic, or the map localizer's TF) |
| Map file location | `sites/<site>/marker_map.yaml` |
| Estimator wiring | remap the localizer's output into robot_localization or another EKF, with the covariance it publishes |

## What the repository provides

- `examples/wall_course/`: survey and localizer launch for a forward camera
  with visual-inertial tracking, a 6-DoF estimator;
- `examples/ceiling/`: survey (`align_to_track` against a LiDAR localizer)
  and planar localizer launch for an upward camera, robot_localization;
- `docs/integration.md`: the table above, frame conventions, parameters,
  how to check the setup (debug image, report, a drive with the check
  screen).

## Moving the source implementations

Each source project, in its own repository:

1. adds the submodule, pinned to a tag;
2. imports its existing site with the phase 1 importer, once;
3. switches its launch files to `survey_node` and `localizer_node`;
4. runs its own checks (simulation goals, kidnap recovery, course runs);
5. deletes its copies.

## Releases

Semantic version tags; the map file has its own `version` and the loader
keeps reading older versions. A changelog entry for every change of a
parameter or a file format.

## Tasks

- [x] `docs/integration.md`
- [x] The two examples, each with a simulated recording that runs in CI
- [ ] Tag 0.1.0 once phases 1 to 3 are done
- [ ] Support both source projects' migration (issues and fixes here)

## Done when

Both source projects run on a tagged release with their own checks green,
and an example runs from a clean checkout following only the guide.

## Result

The in-repository part is done: the guide, the two examples and their CI
job, and the changelog. Tagging 0.1.0 and the two source projects'
migration are open; the migration happens in their repositories.

The examples run through their own launch files, on the rendered recordings
of phases 2 and 3, in the `example` job of CI run 38688987866, which
installs the dependencies with rosdep as the guide does (survey: 3D marker
position error against the truth; localizer: base position error against
the true poses):

| Example | Survey, median / max | Localizer, mean / max | Fixes |
|---|---|---|---|
| wall course, `origin_marker` (board 10), then `3d` on the course map | 0.89 / 1.45 cm | 0.90 / 7.8 cm | 393 of 426 frames |
| ceiling, `align_to_track`, then `planar` on the map just surveyed | 0.30 / 0.45 cm (Humble), 0.33 / 0.54 cm (Jazzy) | 0.29 / 0.57 cm (Humble), 0.33 / 0.65 cm (Jazzy) | 753 of 920 frames |

The wall numbers are identical on Humble and Jazzy and equal those of
`ci/synthetic_survey.sh wall` and `ci/synthetic_localize.sh wall`: the
example launch files with the recording's topics passed in give the same
result as the bare nodes. The limits in `ci/example.sh` are the phase 2 and
3 targets: wall survey median 1 cm, max 4 cm, localizer mean 2 cm, max
10 cm; ceiling survey median 2 cm, localizer mean 1 cm, max 3 cm. The
ceiling localizer works on the surveyed map, so its error includes the
survey's: on the true map the same recording gave 0.02 / 0.13 cm in
phase 3, and the 0.3 cm here is the surveyed markers' own error. Each
example job takes 6 to 9 minutes.

### Pieces

- `docs/integration.md`: what a robot project provides and a project
  layout; the frames; topics and every parameter of both nodes, from
  `params.py`, with the launch arguments; the covariance in each mode and a
  robot_localization configuration (`pose0` for the fixes, its
  `/odometry/filtered` back as the gate's prior); eight checks of a setup,
  from the map file to an offline replay; both examples step by step from
  a clean checkout.
- `examples/wall_course/` (`launch/survey.launch.xml`,
  `launch/localizer.launch.xml`, `sites/test_course/`) and
  `examples/ceiling/` (the same, `sites/office/` and
  `sites/office_uneven/`), laid out as a robot project; they replace
  `examples/launch/` and `examples/sites/`. The launch files keep a real
  robot's topics as defaults and take `use_sim_time`, `process_all` and
  `base_frame`; the guide's commands pass the recording's topics.
- `ci/example.sh`: renders the recording, surveys it through the example's
  survey launch file, compares, localizes through the localizer launch
  file, records and scores; CI job `example` on Humble and Jazzy.
- `CHANGELOG.md` with the 0.1.0 entry, including the moved example files.
- The packages now declare `ros2launch`, `launch_xml`, `ros2bag` and
  `rosbag2_storage_default_plugins`, which the examples need and rosdep
  did not install before.
- `survey.launch.xml` takes `process_all`; the survey node's status line
  counts frames dropped for want of a camera pose.

### Deviations, and why

- **The wall localizer uses the course map, not the surveyed one.** The
  survey's frame is board 10's, the recording's true poses are in the
  course map's frame; the ceiling example, whose survey is in the map
  frame, localizes on its own survey.
- **The ceiling localizer's prior is a `PoseWithCovarianceStamped`** in the
  recording, so the guide passes `prior_type:=pose`; the launch file's
  default stays robot_localization's `Odometry`.
- **A hang in CI.** `synthetic-survey (jazzy, wall)` hung once for almost
  two hours (run 38676467725): the map was saved and the service call
  answered, then the node did not exit on SIGTERM. Thirty SIGTERMs to an
  idle node on Jazzy did not reproduce it. The CI scripts now stop every
  node with a bounded wait (`ci/lib.sh`: the signal, 30 s, then SIGKILL,
  with a line in the log when it was needed), the example script stops
  `ros2 launch` with SIGINT as Ctrl-C would, and every CI job has a
  30 minute limit. No job has needed the SIGKILL since. Later the nodes
  were moved off `rclpy.spin` to a loop that waits at most 0.5 s at a time
  and handles SIGTERM itself (`marker_survey/spin.py`); the hang never
  reproduced locally, so its cause stays unconfirmed.
- **Dropped frames under load.** A first local run of the wall example,
  with a second example running beside it, gave a median of 1.85 cm: about
  25 frames had been dropped because odometry messages were lost on the
  best-effort subscription (a gap over 0.2 s leaves the frames in it
  without a pose). With `process_all` the subscription now keeps 1000
  messages; four recordings run side by side with both distros' unit
  tests then all gave the CI numbers, with at most one frame dropped.
