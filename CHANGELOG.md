# Changelog

Versions follow semantic versioning. Every change of a parameter, a launch
argument, a topic or a file format gets an entry. The marker map file has
its own `version` (now 1); the loader keeps reading older versions.

## Unreleased

### A survey with the camera alone (handheld, no robot)

- Survey parameter `pose_source: markers`, next to `odometry` and `tf`: no
  odometry and no TF pose. The first frame with a marker starts the map
  (its camera pose from the origin marker in `origin_marker` mode when it
  is in view, else the identity); every later frame is placed by PnP on
  all the mapped markers it sees (robust: a marker that does not fit is
  left out; with one marker, the previous pose picks between its two
  solutions). A frame with no mapped marker in view is skipped; new
  markers are mapped once seen in a placed frame. The bundle adjustment
  has no motion factor then. `core/tracking.py`.
- `base_frame` empty (or the image's frame) with `pose_source: markers`:
  the camera is its own base; `floor_offset` is then its height above the
  floor.
- Survey parameter and launch argument `vertical` (`base`, the default, as
  before; or `ceiling`): up from the plane through the ceiling markers, for
  a camera with no level of its own. Stored in `survey_data.npz`.
- The survey's `~/status` gains `pose_source`, `tracking` (`ok` or `lost`:
  no mapped marker in the last frame), `lost_since` (seconds since the last
  placed frame, `null` while ok), `last_known_marker` (id or `null`) and
  `unplaced` (frames not placed); the existing fields are unchanged.
- With no motion factor and no pose prior, the relative 1-sigma (the
  verdicts) also takes out a common tilt of the map: nothing but the
  markers levels such a survey.
- `examples/handheld_ceiling/`: twenty tag36h11 boards (0.40 m) on a 1.4 m
  grid under a 2.9 m ceiling, a webcam held 1.0 m up and walked along the
  rows and columns, with a hand's shake, a stretch past the end with no
  marker in view, and 3 s with the camera tilted away; its survey and
  localizer launch files, and `ci/example.sh handheld_ceiling` in CI.
- `marker_survey_synthetic --camera handheld` (with `--hand-height`,
  `--shake-deg`, `--tilt-at`, `--tilt-for`): such a recording, only the
  camera's topics in it, the truth poses the camera's; `scene.json` gives
  the stamps of the tilted stretch (`tilt_window`). `core/simulate.py`:
  `Hand`, `walk`, `handheld_camera`, `simulate_markers`.

### Survey uncertainty

- Fixed: a marker whose solved rotation vector had passed pi (a ceiling
  marker, upside down, turned past 180 degrees) got its covariance mapped
  through the other rotation vector of the same pose, which could put its
  angle 1-sigma at 5 degrees where it was 0.5. The solve now keeps its own
  parameters (`Survey.joint_params`) and the covariance uses them.
- The corner noise behind the covariance is at least the plain estimate
  (sum of squared residuals over the degrees of freedom, outliers clipped
  at 5 px) as well as the robust median one: keyframes that see one marker
  only fit their corners almost exactly and pulled the median far down.

### Localizer

- `~/map_markers` (`visualization_msgs/MarkerArray`, latched, transient
  local): each marker of the loaded map as a thin cube of its size and a
  text label with its id, in the map's `frame_id`; published again when
  the map is loaded again.
- `~/status` gains `last_fix_age`: seconds since the stamp of the last
  published fix, by the node's clock (`null` before the first).
- `base_frame` empty (or the image's frame): the camera is the base, no
  TF needed (a handheld camera).
- Parameter and launch argument `debug_image_topic` (default
  `~/debug/image`, as before).
- `marker_localizer_score --gap T0 T1 [--max-first-cm X]`: no fix may
  carry a stamp in the gap, and the first fix after it is reported with
  its error and delay.

### Launch files

- `survey.launch.xml`: arguments `marker_size_sd` and `vertical`;
  `pose_source` takes `markers`.
- `localizer.launch.xml`: argument `debug_image_topic`.

### Tools

- `marker_plan`: `route_spacing_no_odom` = 2R, the spacing along a route
  with no odometry (one marker always fully in view, two at the midpoint),
  printed as its own line; `--handheld` (camera height 1.0 m, speed
  0.5 m/s; explicit flags win), which also prints the zone along the
  image's long side; `--route-length L` (and `--spacing D`) counts the
  markers for a route, ceil(L / D) + 1 (`core/marker_plan.route_markers`).
- New `marker_map_to_sdf`: a marker map as Gazebo models (`plane` visuals
  of the printed sheet with its white margin, textured from the
  dictionary's bits, PNGs written next to the SDF) at any pose (ceiling,
  wall, floor), facing the marker's z axis; `--world` for a whole Gazebo
  Harmonic world (ground, a ceiling plane at `--ceiling` or behind the
  highest ceiling marker, lights from below and above, the physics,
  sensors, scene broadcaster and user commands systems);
  `--replace-between FILE` rewrites the lines between
  `<!-- BEGIN ceiling markers` and `<!-- END ceiling markers -->` of an
  existing world (`--begin`, `--end` for other lines). `core/sdf.py`.
- `marker_survey_compare`: with the survey's report (beside the result, or
  `--report`), each marker's verdict and its error in units of the frame's
  1-sigma (horizontal / `frame_sd_xy_cm`, height / `frame_sd_z_cm`), with
  their median and maximum; `--require-good` fails unless every marker is
  good.

### CI

- `ci/synthetic_survey.sh wall`: the median limit is 1.2 cm (was 1.0): the
  same recording gives 0.89 to 1.04 cm from run to run, since the odometry
  arrives best effort and the poses interpolated at the frames vary a
  little; 1.0 failed once at 1.04.

### Docs

- `docs/integration.md`: "A handheld survey, no robot"; with a measured
  marker size, `marker_size_sd` can be set lower.

### Web interface: no robot, a handheld webcam

- Plan tab (`#plan`, also without ROS): ceiling and camera heights with
  presets (a robot; a person holding the camera, 1.0 m, 0.5 m/s), field
  of view (typed, or read from the live `camera_info`), image size, speed,
  exposure and a route length give, from `core/marker_plan.py`, the
  readable size range, the recommended size and sheet, the zone R, the
  spacing for a marker always in view with no odometry and the marker
  count; a side view drawn from the same numbers; a link to the print tab
  with that size and the IDs. New `POST /api/plan`; `/api/info` lists the
  presets; the print tab takes a job in the address
  (`#print?size=40&count=23&ids=0-22`).
- Survey screen with `pose_source: markers` (a camera-only survey): only
  "Start from a marker" is offered, and a banner says when no mapped marker
  is in view and which one to go back to.
- Test localization tab (`#localize`): the localizer's camera, its pose
  (x, y, yaw, z), the time since the last fix, the trajectory with
  kidnaps (the pose froze, then jumped) dashed, and test points measured
  with a tape with their errors. New `GET|POST /api/localize`,
  `GET /api/localize.csv`; node parameters `localize.gap_s` (1.0) and
  `localize.jump_m` (0.5).
- Node parameter and launch argument `camera_info_topic` (empty: not read).
- `launch/handheld.launch.xml`: `usb_cam` (optional: `launch_camera`, a
  params file, `config/handheld_usb_cam.yaml`), the survey with the camera
  alone (`vertical: ceiling`, the floor `camera_height`, 1.0 m, below
  the camera), the localizer in 3d with no prior, the web node; both
  nodes with `base_frame` empty: the camera is the base, no TF.
- Tests: `test_handheld_api.py` on a simulated handheld walk; the launch
  file without the camera; `ci/web_handheld.sh`, the handheld case in
  Chromium on the `examples/handheld_ceiling` recording through
  `handheld.launch.xml`, in the `web-browser` job.

## 0.3.0 (2026-10-15)

### Survey uncertainty

- `align_to_track`: the 1-sigma in the result frame (`frame_sd_xy_cm`,
  `frame_sd_z_cm`) now holds how well the fit placed the survey on the
  reference track. The fit's residuals are taken as correlated along the
  track (a localizer's error holds over metres): their autocorrelation
  gives the number of independent samples (`frame.fit_samples` in the
  report), so an error that holds over the whole drive does not shrink
  with more keyframes. The markers' own part is now relative to the
  solved trajectory as a whole, no longer to its first keyframe (a marker
  seen from the start had a 1-sigma near zero). New report fields
  `frame.fit_samples`, `frame.fit_sd_xy_cm`, `frame.fit_sd_yaw_deg`.
- Survey parameter `marker_size_sd` (0.005, relative, common to all
  markers): the size's own uncertainty, in the frame's 1-sigma in every
  mode (a size off by 0.5 % moves a ceiling marker 3 m up by about 1.4 cm
  in height, and stretches a long site). Not in the relative 1-sigma, so
  the verdicts are unchanged. Stored in `survey_data.npz`;
  `marker_survey_solve --marker-size-sd` sets it again, and
  `marker_survey_solve --marker-size` solves a recording again with the
  black square measured again.
- On a robot project's simulated recording (ceiling markers, a 0.7 % size
  error, a track residual of 6.7 cm RMS) the real error was 3 times the
  frame's 1-sigma horizontally (median; up to 98 times) and 4 times in
  height (up to 25); now 1.4 (up to 2.0) and 1.5 (up to 2.1).
- `core/simulate.py`: `Noise.map_corr_m`, a localizer's error that holds
  over that many metres (the default 0 keeps it independent per frame).
- The log line of the origin marker's facing compares the frame's
  1-sigma with the size taken as exact.

### Localizer

- Parameter and launch argument `gate_sigmas` (3.0): the gate widens to
  that many sd of the prior's own covariance where that is larger than
  `gate_distance` or `gate_angle_deg`, so a state estimator that does not
  know where the robot is (variance 100 at start) takes the fixes, while
  a confident one gates as before. 0: the fixed gate always. The gate's
  reason now names the gate it used (`gate 1.00 m`). `rejected_topic`
  is unchanged.

### Launch files

- `localizer.launch.xml`, `survey.launch.xml` and `web.launch.xml` take
  `node_name` (defaults `marker_localizer`, `marker_survey`,
  `marker_survey_web`, as before); the nodes' `~/` topics and services
  follow it. A launch file that includes several of them puts each
  include in its own `<group>`, or the first one's `node_name` carries
  over to the others; `examples/ceiling/launch/web.launch.xml` does.

### Tools

- `marker_map_import --source-name NAME`: the name the map's header gives
  the source (default: the source file's name, without its directory),
  for a conversion from a temporary copy.

## 0.2.0 (2026-10-15)

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
- Localizer node: topic `~/markers` (`diagnostic_msgs/DiagnosticArray`,
  per image with markers, only while subscribed): per marker in view its
  state, reprojection residual with the fix, and its position as the image
  alone sees it from the prior (or the fix) with the offset and angle from
  the map's pose. `~/fixes` carries the same per marker
  (`core/check.py marker_views`). The package now depends on
  `diagnostic_msgs`.
- `localizer.launch.xml`: arguments `prior_tolerance`, `require_prior`,
  `max_distance`, `max_position_sd`, `min_sd`, `min_sd_deg`, and
  `param_file` (a YAML file of node parameters, read after the arguments,
  so its values win; default `config/no_params.yaml`, which sets none).
  `survey.launch.xml` takes `param_file` too. A robot project can include
  the stock launch files instead of starting the nodes itself.
- Survey node: parameter `max_pose_gap` (0.5 s): the odometry is
  interpolated across gaps up to this, so a lost odometry message no
  longer drops the frames in its gap (before, any gap over twice
  `pose_tolerance` did; on a recording that cost a keyframe and once failed
  a CI limit). The save logs how many frames were dropped.
  `marker_survey_synthetic --drop-odom F` leaves out a share of the
  odometry messages; `ci/synthetic_survey.sh wall` drops 3 % and fails on
  any frame dropped.
- The survey, localizer and web nodes spin through `marker_survey.spin`:
  SIGTERM sets a flag and the executor waits at most 0.5 s at a time, so
  a node exits within that on SIGTERM or Ctrl-C whatever wakes or does not
  wake its wait. Before, `rclpy.spin` relied on rclpy's own SIGTERM
  handling, and a survey node once did not exit in CI on Jazzy (with sim
  time and the recording over, none of its timers fired any more). The
  hang did not reproduce locally (37 runs of the CI sequence, 60 of an idle
  node); `ci/lib.sh` keeps its bounded stop.
- `core/mount_calibration.Observations` takes a `MarkerMap` (or
  `{id: MapMarker}`): tilted markers and per-marker sizes go through, each
  corner at its own map height. The older `{id: {x, y, z, yaw}}` with one
  `marker_size` still works.
- Localizer: parameter and launch argument `rejected_topic` (default
  empty: off): fixes that only the gate turned down, far from the prior,
  are published there, for a relocalizer recovering a robot whose estimate
  is lost (kidnapped). `Localizer.gated(result)` tells them apart. Nothing
  changes while it is empty.
- `marker_map_import` rounds the angles it converts from radians within
  the source's own precision, by at most 0.005 deg: 0.5236 rad is
  `facing_deg: 30.0`, no longer 30.00007. Its check against the source
  poses allows that rounding.

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
