# Changelog

Versions follow semantic versioning. Every change of a parameter, a launch
argument, a topic or a file format gets an entry. The marker map file has
its own `version` (now 1); the loader keeps reading older versions.

## Unreleased

The web interface on the robot: survey, check and use the map from a
browser (phase 5).

### Packages

- `marker_survey_web`: the ROS 2 node `marker_survey_web_node` and
  `launch/web.launch.xml`: the survey, check and map screens, the JSON API
  (`/api/state`, `/api/survey`, `/api/report`, `/api/map`, `/api/check`) and
  the WebSocket `/api/live` (the state twice a second, camera frames as
  JPEG only while shown). Parameters `host`, `port`, `survey_node`,
  `localizer_node`, `map_path`, `camera_rate`, `camera_width`, `check.*`.
  `marker_survey_web` without ROS still serves printing and identifying.
  The package now depends on `marker_localizer`, `rclpy` and the message
  packages.
- `marker_survey`: `core/live.py`, the survey's status (lights, hints,
  detections) and the kept earlier solves; a reference track recorded in a
  mode other than `align_to_track` no longer anchors the uncertainty.
- `marker_localizer`: `core/check.py`, fix records and per-marker agreement
  with the estimator's pose.

### Nodes and launch files

- Survey node: topic `~/status` (`std_msgs/String`, JSON); services
  `~/start`, `~/pause`, `~/reset`; parameters `start_paused` (false) and
  `keep_history` (true: the previous `marker_map.yaml`,
  `survey_report.yaml` and `survey_data.npz` are copied to
  `output_dir/history/<date-time>/` before a save); `frame.mode`,
  `frame.origin_marker` and `frame.frame_id` can be set at run time;
  `frame.reference_frame` is looked up whenever it is set, not only in
  `align_to_track`. `survey.launch.xml` and the ceiling example's survey
  launch file take `start_paused`.
- Localizer node: topics `~/fixes` and `~/status` (`std_msgs/String`,
  JSON, only while subscribed); service `~/reload`; `marker_map` can be
  set at run time (refused if the file does not load); a map file missing
  at start is waited for instead of stopping the node.
- `examples/ceiling/launch/web.launch.xml`: survey, localizer and web
  interface in one launch.

### Files

- Beside the map in use, `marker_map_versions/` keeps every map the web
  interface replaced (`marker_map.<date-time>.yaml`, and
  `.undone.yaml` for the ones undo replaced).

### Tools and tests

- `marker_survey_synthetic --split-at S` also writes the recording in two
  parts (`bag_1`, `bag_2`), and `--start-time` sets its first stamp.
- `ci/web_browser.sh` and `ci/web_browser_test.py`: the survey, check and
  map steps in Chromium (Playwright) on simulated recordings; CI job
  `web-browser`.

### Fixes from the first project moved onto 0.1.0

- Survey verdicts: the 1-sigma values were far too large (a 36 m hall:
  14 to 55 cm where the real errors were 1 to 13 cm, every marker `bad` or
  `pass again`; the wall course: 7 of 11 `pass again` at 4 to 6 cm, real
  errors at most 1.5 cm). Three causes: the covariance assumed the solve's
  0.5 px corner noise where rendered and real corners are several times
  better; in `origin_marker` mode the floor was taken as exact while the
  markers' heights carried the odometry's drift; and the origin marker's
  facing, which turns every far marker about it, counted against each
  marker. Now the corner noise is measured from the solve's residuals
  (robust spread, at least `min_pixel_sd`, 0.05 px; reported as
  `solve.pixel_noise_px`), the floor's uncertainty enters with the
  markers', and the per-marker `sd_xy_cm`, `sd_z_cm` and `sd_angle_deg`,
  which the verdicts use, are relative to the other markers (a common
  shift and turn about the vertical taken out). New in the report:
  `frame_sd_xy_cm` and `frame_sd_z_cm` per marker (the 1-sigma in the
  result frame, frame definition included) and `frame.frame_sd_xy_max_cm`;
  the log says when the origin marker's facing dominates. Both now follow
  the real errors on the CI recordings (error / 1-sigma about 1).
- `marker_map_import` copies only the comments next to markers; the
  source's header and footer comments (about its own use: routes, tape
  marks) are replaced by one line naming the source. `--keep-header`
  copies them as before.

## 0.1.0 (2026-10-15)

The first release: the survey, the marker map file and the localizer for
both setups, wall boards and ceiling markers, with the tools, the printing
of marker sheets, the integration guide and two examples. ROS 2 Humble and
Jazzy, Python.

### Packages

- `marker_survey`: detection of ArUco and AprilTag families on OpenCV 4.5
  to 4.10 (`CORNER_REFINE_APRILTAG` for AprilTag families, `cornerSubPix`
  and an edge-line fit for all); the survey, a bundle adjustment of every
  keyframe and marker with a robust loss; frame modes `origin_marker`,
  `pose_frame` and `align_to_track`; pose sources `odometry` and `tf`;
  `survey_report.yaml` with a verdict per marker; the survey node.
- `marker_localizer`: modes `3d` (6-DoF) and `planar` (x, y, yaw, with
  roll and pitch from `zero`, `tf` or `imu`); covariance from the fit with
  floors; a gate against the state estimator's pose; the localizer node.
- `marker_survey_web`: the browser interface, so far printing marker
  sheets and identifying markers from a photo.

### Files

- `marker_map.yaml`, version 1: `frame_id`, `dictionary`, `marker_size`,
  `mount` (`wall`, `ceiling`, `floor`) and per marker `id`, `x`, `y`, `z`,
  the mount's angles in degrees and an optional `size`; comments survive a
  survey. JSON Schema in `marker_survey/schema/marker_map.schema.json`.
- `survey_report.yaml`: per marker views, reprojection RMS, 1-sigma
  position, height and angle, flags and a verdict (`good`, `pass again`,
  `bad`); in `align_to_track` the fit's residual.
- `survey_data.npz`: the survey's recording, solved again by
  `marker_survey_solve`.

### Nodes and launch files

- `marker_survey` node and `marker_survey/launch/survey.launch.xml`; the
  parameters are listed in `docs/integration.md`. The launch file takes
  `process_all` (every frame of a recording, in order).
- `marker_localizer` node and `marker_localizer/launch/localizer.launch.xml`;
  output `~/pose` (`geometry_msgs/PoseWithCovarianceStamped` of
  `base_frame` in the map's `frame_id`, stamped with the image's capture
  time) and `~/debug/image`.
- The survey node's status line counts the frames dropped for want of a
  camera pose; with `process_all` its odometry subscription keeps 1000
  messages instead of 50.
- The packages declare `ros2launch`, `launch_xml`, `ros2bag` and
  `rosbag2_storage_default_plugins`, so that rosdep installs everything
  the examples use.

### Tools

`marker_survey_solve`, `marker_survey_synthetic`, `marker_survey_compare`,
`marker_map_check`, `marker_map_import` (the two older site formats),
`marker_plan`, `marker_sheets`, `marker_identify`,
`marker_localizer_evaluate`, `marker_localizer_score`, and
`marker_survey_web`.

### Examples and documentation

- `docs/integration.md`: what a robot project provides, the frames, every
  parameter, wiring into robot_localization, how to check a setup, and the
  examples run from a clean checkout.
- `examples/wall_course/` and `examples/ceiling/`: survey and localizer
  launch files and a site each (`sites/<site>/marker_map.yaml` and the
  route of its simulated drive). They replace `examples/launch/` and
  `examples/sites/` of the development versions:

  | Before | Now |
  |---|---|
  | `examples/launch/wall_boards_odometry.launch.xml` | `examples/wall_course/launch/survey.launch.xml` |
  | `examples/launch/wall_boards_localizer.launch.xml` | `examples/wall_course/launch/localizer.launch.xml` |
  | `examples/launch/ceiling_markers_lidar_map.launch.xml` | `examples/ceiling/launch/survey.launch.xml` |
  | `examples/launch/ceiling_markers_localizer.launch.xml` | `examples/ceiling/launch/localizer.launch.xml` |
  | `examples/sites/test_course/` | `examples/wall_course/sites/test_course/` |
  | `examples/sites/office/`, `office_uneven/` | `examples/ceiling/sites/office/`, `office_uneven/` |

  The example launch files gained `use_sim_time`, `process_all` and
  `base_frame`; the ceiling localizer also `prior_type`, the ceiling survey
  `reference_frame`.

### Accuracy, in simulation

Rendered recordings through the nodes, in CI on Humble and Jazzy: wall
survey median 0.9 cm, ceiling surveys median 0.6 to 0.7 cm, planar
localizer mean 0.02 cm, 3d localizer mean 0.9 cm. Not yet measured on a
real site.
