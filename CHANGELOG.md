# Changelog

Versions follow semantic versioning. Every change of a parameter, a launch
argument, a topic or a file format gets an entry. The marker map file has
its own `version` (now 1); the loader keeps reading older versions.

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
