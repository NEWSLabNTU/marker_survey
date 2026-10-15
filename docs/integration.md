# Using the packages on a robot

A robot project runs the survey and the localizer from this repository with
nothing but its own launch file, its camera remappings and its state
estimator's configuration. This guide lists what the robot provides, the
frames, every parameter of the two nodes, how to check a setup, and the two
examples, which run on a simulated recording without a robot.

## What a robot project provides

| Item | What | Example |
|---|---|---|
| Camera topics | an image (`sensor_msgs/Image`, or `CompressedImage` with `compressed:=true`) and its `camera_info`; rectified, or raw with distortion coefficients in `camera_info` | `/camera/image_rect`, `/camera/camera_info` |
| TF, camera mounting | `base_frame -> camera optical frame`, usually static from the URDF; the image's `header.frame_id` is the optical frame | `base_link -> camera_optical_frame` |
| A pose source, for the survey | an odometry topic (`nav_msgs/Odometry`) or a TF frame that holds the camera's pose | visual-inertial odometry `/camera/odom`; wheel odometry `odom -> base_link` |
| A reference trajectory, for a survey in an existing map | TF `map -> base_frame` from the robot's localizer (`align_to_track`, `pose_frame`) | a LiDAR localizer's `map -> odom` |
| Map file location | one `marker_map.yaml` per site, in the robot project | `sites/<site>/marker_map.yaml` |
| Estimator wiring | the localizer's output fused by the state estimator with the covariance it publishes; the estimator's pose back to the localizer for its gate | robot_localization: `pose0: /marker_localizer/pose`, `prior_topic:=/odometry/filtered` |

A robot project typically holds:

```
my_robot/
  launch/markers_survey.launch.xml      # includes this repository's launch file, with the robot's topics
  launch/markers_localizer.launch.xml
  config/ekf.yaml                        # the estimator, fusing /marker_localizer/pose
  sites/<site>/marker_map.yaml           # written by the survey, kept in git
  third_party/marker_survey/             # this repository, as a git submodule pinned to a tag
```

Its launch files include `$(find-pkg-share marker_survey)/launch/survey.launch.xml`
and `$(find-pkg-share marker_localizer)/launch/localizer.launch.xml`, as the
examples below do, or start the nodes `marker_survey` and `marker_localizer`
with a parameter file. Nothing in this repository needs to change for a
robot.

## Frames

| Frame | Convention |
|---|---|
| camera optical frame | OpenCV's and ROS's optical convention: x right, y down, z out of the lens. The image's `header.frame_id`. |
| `base_frame` (`base_link`) | REP 103: x forward, y left, z up. The floor is the plane below it: its normal is the mean z axis of the robot's poses during the survey, its height `floor_offset` below `base_frame` (0 for a robot whose `base_link` is on the floor). |
| a marker | its centre; x to the right of the printed image, y towards its top, z out of the marker (OpenCV ArUco). |
| the map's `frame_id` | the frame every marker in `marker_map.yaml` is given in, and the frame of the localizer's output. It must be the frame the state estimator works in (`map` for robot_localization's map-frame instance). |
| `origin_marker` frame | written by a survey with `frame.mode: origin_marker`: the origin on the floor below the chosen marker, x the way the marker faces (for a marker lying flat, its image x axis), z up. Named `frame.frame_id` (default `map`) in the file. |

The rotation of each marker in the file is given relative to its mount
(`wall`, `ceiling`, `floor`); the README's "The marker map file" section and
`marker_survey/marker_survey/core/marker_map.py` give the angles.

Covariances follow ROS: a 6 x 6 matrix over x, y, z and the rotations about
x, y and z of the map frame, row-major.

## The survey node

```bash
ros2 launch marker_survey survey.launch.xml output_dir:=$PWD/sites/my_site \
    dictionary:=tag36h11 marker_size:=0.45 frame_mode:=align_to_track ...
```

| Topic, service | Type | |
|---|---|---|
| `image_topic` (in) | `sensor_msgs/Image`, or `CompressedImage` on `image_topic/compressed` | best effort; reliable with `process_all` |
| `camera_info_topic` (in) | `sensor_msgs/CameraInfo` | the first one is used |
| `odom_topic` (in, `pose_source: odometry`) | `nav_msgs/Odometry` | best effort, which also matches a reliable publisher |
| TF (in) | image frame -> `base_frame`; `pose_source: tf`: `pose_frame` -> image frame at the image's stamp; `align_to_track`: `frame.reference_frame` -> `base_frame` at the image's stamp; odometry: its `child_frame_id` -> image frame | |
| `~/markers` (out) | `visualization_msgs/MarkerArray` | the markers so far, for RViz |
| `~/debug/image` (out) | `sensor_msgs/Image` | detections drawn: green known, yellow new, red not used; only while subscribed |
| `~/status` (out) | `std_msgs/String` | JSON twice a second, only while subscribed: state, keyframes, every marker with its live position, a light (`good`, `pass again`, after a save the report's verdict) and a hint, the last frame's detections, the path; for the web interface |
| `~/save` (service) | `std_srvs/Trigger` | solve and write now; Ctrl-C does the same (`save_on_exit`) |
| `~/start`, `~/pause` (services) | `std_srvs/Trigger` | record, or stop recording and keep what was recorded; after a save, new keyframes add to the same recording ("pass again") |
| `~/reset` (service) | `std_srvs/Trigger` | drop the recording and start afresh; the files stay |

It writes `marker_map.yaml`, `survey_report.yaml` and `survey_data.npz` to
`output_dir`; with `keep_history` the previous three are first copied to
`output_dir/history/<date-time>/`, so a survey saved again (after another
pass) keeps the earlier solve and its recording. `frame.mode` and
`frame.origin_marker` can be set at run time (`ros2 param set`), before a
save; the web interface sets them when a survey starts. A reference frame
(`frame.reference_frame`) is recorded whenever it is in TF, so that the mode
can change, and anchors the uncertainty only in `align_to_track`.

Parameters (`marker_survey/marker_survey/params.py`; the launch file passes
the ones marked with an argument of the same name, `frame.mode` as
`frame_mode`, `frame.origin_marker` as `origin_marker`, `frame.frame_id` as
`frame_id`, `frame.reference_frame` as `reference_frame`):

| Parameter | Default | Launch arg | Meaning |
|---|---|---|---|
| `output_dir` | (required) | yes | where the three files are written |
| `dictionary` | `DICT_5X5_1000` | yes | the printed family: an OpenCV `DICT_*` name or `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `aruco_mip_36h12`, `aruco_original` |
| `marker_size` | 0.38 | yes | [m] edge of the black square, not the sheet; a wrong size scales every distance |
| `marker_sizes` | `['']` | | per-marker sizes, `["12:0.30", "13:0.30"]` |
| `marker_size_sd` | 0.005 | | relative 1-sigma of the size, common to all markers (a printer's scale, the tape): 0.005 is 0.5 %, 2 mm on 0.45 m; in the report's `frame_sd_*` only; 0: exact. `marker_survey_solve --marker-size-sd` sets it again |
| `image_topic` | `/camera/image` | yes | |
| `camera_info_topic` | `/camera/camera_info` | yes | |
| `compressed` | false | yes | subscribe to `image_topic/compressed` |
| `pose_source` | `odometry` | yes | `odometry` (the `odom_topic`) or `tf` (`pose_frame`) |
| `odom_topic` | `/odom` | yes | |
| `pose_frame` | `''` | yes | `pose_source: tf`: the frame of the camera poses (`odom`, `map`, ...) |
| `base_frame` | `base_link` | yes | the robot frame; the floor is below it |
| `frame.mode` | `origin_marker` | `frame_mode` | `origin_marker`, `pose_frame` or `align_to_track` (README, "Frame modes") |
| `frame.origin_marker` | -1 | `origin_marker` | `origin_marker`: the marker that defines the frame |
| `frame.frame_id` | `map` | `frame_id` | `origin_marker`: the `frame_id` written to the file |
| `frame.reference_frame` | `''` | `reference_frame` | `align_to_track`: TF frame of the reference trajectory; also the result's `frame_id` |
| `floor_offset` | 0.0 | yes | [m] how far the floor lies below `base_frame` |
| `rate` | 5.0 | | [Hz] how often queued frames are processed |
| `process_all` | false | yes | every frame in order over a reliable subscription (recordings); live, only the newest frame |
| `pose_tolerance` | 0.1 | | [s] an image before the first or after the last odometry message takes the nearest one within this |
| `max_pose_gap` | 0.5 | | [s] the odometry is interpolated at an image's stamp across gaps up to this (lost messages); a frame whose odometry has not arrived yet waits for it |
| `save_on_exit` | true | | solve and write on Ctrl-C if `~/save` was not called |
| `keyframe_distance` | 0.15 | | [m] a new keyframe after this much motion ... |
| `keyframe_angle_deg` | 5.0 | | ... or this much rotation, or a new marker |
| `odom_drift` | 0.01 | | odometry translation error per metre, the weight of the motion between keyframes |
| `pose_sd` | 0.05 | yes | `pose_frame`: [m] noise of the pose source per axis |
| `pose_sd_deg` | 1.0 | | `pose_frame`: its angle noise |
| `pose_frame_motion` | false | | `pose_frame`: also use the pose source's motion between keyframes (a smooth source) |
| `min_marker_px` | 20.0 | | markers smaller than this [px] are not used |
| `border_margin_px` | 5.0 | | markers this close to the image border are not used |
| `debug_width` | 960 | | [px] width of `~/debug/image` |
| `start_paused` | false | yes | record only after `~/start` (the web interface starts it); the frame may then be chosen later |
| `keep_history` | true | | copy the previous result to `output_dir/history/` before a save overwrites it |

## The localizer node

```bash
ros2 launch marker_localizer localizer.launch.xml marker_map:=$PWD/sites/my_site/marker_map.yaml \
    mode:=planar prior_topic:=/odometry/filtered prior_type:=odometry
```

| Topic | Type | |
|---|---|---|
| `image_topic`, `camera_info_topic` (in) | as the survey | |
| TF (in) | `base_frame` -> image frame (the mounting, looked up once); `roll_pitch: tf`: `roll_pitch_frame` -> `base_frame` at the image's stamp | |
| `prior_topic` (in, optional) | `geometry_msgs/PoseWithCovarianceStamped`, or `nav_msgs/Odometry` with `prior_type: odometry` | the estimator's pose in the map's `frame_id`, for the gate; best effort, reliable with `process_all` |
| `imu_topic` (in, `roll_pitch: imu`) | `sensor_msgs/Imu` | its orientation gives roll and pitch |
| `pose_topic` (out, default `~/pose`, so `/marker_localizer/pose`) | `geometry_msgs/PoseWithCovarianceStamped` | `base_frame` in the map's `frame_id`, stamped with the image's capture time |
| `~/debug/image` (out) | `sensor_msgs/Image` | markers used (green, map corners projected with the fix as dots, residual written), left out (orange), unusable (red), not in the map (grey), and the fix; only while subscribed |
| `~/fixes` (out) | `std_msgs/String` | JSON per image with markers, only while subscribed: the fix (or why none), the prior at its stamp, the distance between them, and per marker in view what became of it and its check (below) (`marker_localizer/core/check.py`) |
| `~/markers` (out) | `diagnostic_msgs/DiagnosticArray` | per image with markers, only while subscribed: one status per marker in view, `marker_localizer: marker <id>`, level WARN when left out as an outlier, else OK; values below |
| `rejected_topic` (out, only if set) | `geometry_msgs/PoseWithCovarianceStamped` | the fixes the gate turned down, for relocalization; never fused by the estimator |
| `~/status` (out) | `std_msgs/String` | JSON every second, only while subscribed: the map file in use, its markers, the counts |
| `~/reload` (service) | `std_srvs/Trigger` | read the map file again; a file that does not load is refused and the old map stays |

Per-marker checks, in `~/fixes` (per marker) and `~/markers` (as key-value
pairs): `state` (`used`, `outlier`, `unused`, `unusable`, `unknown`);
`residual_px`, the RMS distance of the marker's map corners projected with
the fix from its detected corners, also for a marker left out of the fix;
`seen` (`seen_x`, `seen_y`, `seen_z`), the marker's centre in the map frame
as this image alone sees it, placed by the prior (`seen_from: prior`) or,
without a prior, by the fix (`seen_from: fix`); `offset_m` and `angle_deg`,
its distance and rotation from the map's pose. A robot project checks its
camera and each marker with them: a marker whose `offset_m` stays well
above the prior's own noise has moved or was surveyed badly; a camera whose
markers all show the same offset has a wrong mounting TF.

`marker_map` can also be set at run time: the new file is read at once, and
the change is refused if it does not load. A map file that does not exist
when the node starts is waited for (a new site): the node publishes nothing
until `~/reload` or a new `marker_map` finds one.

The launch file's `node_name` argument (default `marker_localizer`) names
the node, and its `~/` topics and services follow: `node_name:=markers_front`
publishes `/markers_front/pose`; `survey.launch.xml` (default
`marker_survey`) and `web.launch.xml` (default `marker_survey_web`) take
the same argument, and the web node's `survey_node` and `localizer_node`
then name the renamed nodes.

Parameters (`marker_localizer/marker_localizer/params.py`; the launch file
passes the ones marked with an argument of the same name, and any of them
from a YAML file, `param_file:=my_localizer.yaml`, read after the arguments
so its values win: `/**: {ros__parameters: {max_distance: 6.0, ...}}`. A
robot project includes the stock launch file with its own arguments or
file instead of starting the node itself; `survey.launch.xml` takes
`param_file` the same way):

| Parameter | Default | Launch arg | Meaning |
|---|---|---|---|
| `marker_map` | (required) | yes | the site's `marker_map.yaml`; its `dictionary`, sizes and `frame_id` are used; settable at run time |
| `mode` | `planar` | yes | `planar` (x, y, yaw on a flat floor) or `3d` (6-DoF) |
| `base_frame` | `base_link` | yes | the frame whose pose is published |
| `image_topic` | `/camera/image` | yes | |
| `camera_info_topic` | `/camera/camera_info` | yes | |
| `compressed` | false | yes | subscribe to `image_topic/compressed` |
| `pose_topic` | `~/pose` | yes | the output |
| `prior_topic` | `''` | yes | the estimator's pose; empty: no gate |
| `rejected_topic` | `''` | yes | if set, fixes that only the gate turned down (farther from the prior than the gate, `gate_distance` and `gate_angle_deg` or `gate_sigmas` of the prior's sd) are published here, `PoseWithCovarianceStamped` as `pose_topic`: for a relocalizer recovering a robot whose estimate is lost; empty: not published |
| `prior_type` | `pose` | yes | `pose` (PoseWithCovarianceStamped) or `odometry` |
| `prior_tolerance` | 0.5 | yes | [s] the prior is interpolated at the image's stamp, or the nearest within this |
| `require_prior` | false | yes | publish no fix while no prior is near the image's stamp |
| `gate_distance` | 1.0 | yes | [m] fixes farther than this from the prior are dropped (a misread ID, a moved marker) |
| `gate_angle_deg` | 15.0 | yes | the same for yaw |
| `gate_sigmas` | 3.0 | yes | the gate widens to this many sd of the prior's covariance where that is larger than `gate_distance` or `gate_angle_deg`: position, the largest sd in x, y (`planar`) or x, y, z (`3d`); angle, yaw (`planar`) or the largest of the rotation (`3d`). A confident prior (sd under a third of the gate) gates as before; an estimator just started (variance 100) lets the fixes through, so a robot without a relocalizer of its own is placed by the markers. A zero covariance (unknown) keeps the fixed gate; 0: the fixed gate always |
| `enabled` | true | | can be set at run time: `ros2 param set /marker_localizer enabled false` |
| `process_all` | false | yes | every frame in order, each once its prior has arrived (recordings) |
| `roll_pitch` | `zero` | yes | `planar`: the robot's roll and pitch from `zero`, `tf` or `imu` |
| `roll_pitch_frame` | `''` | yes | `roll_pitch: tf`: a level frame, such as `odom` |
| `imu_topic` | `/imu` | yes | |
| `min_marker_px` | 20.0 | | as the survey |
| `border_margin_px` | 5.0 | | as the survey |
| `detect_scale` | 1.0 | yes | under 1: find markers in a downscaled image, refine on the full one (0.5 for 1920 x 1200) |
| `quad_decimate` | -1.0 | | AprilTag quad search decimation; -1: 2 from OpenCV 4.6 on, else none |
| `min_contrast` | 30 | | AprilTag: smallest white-black difference |
| `max_distance` | 0.0 | yes | [m] ignore markers farther from the camera; 0: no limit |
| `max_position_sd` | 0.25 | yes | [m] a fix with a larger position sd is not published; 0: keep all |
| `pixel_sd` | 0.5 | | [px] corner noise the covariance assumes (at least the fit's RMS is used) |
| `robust_px` | 1.0 | | [px] where the robust loss turns linear |
| `max_rms_px` | 2.0 | | a fix whose corners fit worse is dropped |
| `outlier_px` | 3.0 | | a marker fitting worse is left out and the rest refit |
| `min_sd` | 0.01 | yes | [m] added to the position variance (a floor) |
| `min_sd_deg` | 0.3 | yes | added to the angle variances |
| `tilt_sd_deg` | 0.2 | | `planar`: the robot's roll and pitch uncertainty, added to x and y at the markers' distance |
| `base_height` | 0.0 | | `planar`: [m] `base_frame`'s height above the map's z = 0, the published z |
| `debug_width` | 960 | | [px] width of `~/debug/image` |

## Wiring the localizer into the state estimator

The localizer publishes one fix per image with markers, stamped with the
image's capture time, in the map's `frame_id`. Its covariance comes from the
fit: it grows with the distance to the markers and shrinks with the number
of markers and corners, plus `min_sd` and `min_sd_deg`. Fixes with a
position sd over `max_position_sd` and fixes the gate rejects are not
published.

| Mode | Measured | Covariance of the rest |
|---|---|---|
| `planar` | x, y, yaw (rows and columns 0, 1, 5, with their correlations) | z, roll, pitch: variance 1000 (not measured); z is `base_height`, roll and pitch come from `roll_pitch` |
| `3d` | all six | from the fit |

With robot_localization's map-frame EKF (`world_frame: map`, the same as
the marker map's `frame_id`), fuse the fix as a pose and give the
estimator's output back as the prior:

```yaml
ekf_filter_node_map:
  ros__parameters:
    world_frame: map
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    two_d_mode: true                 # planar; false for a 3d vehicle
    # odom0: wheel or visual-inertial odometry, as the robot already fuses it
    pose0: /marker_localizer/pose
    pose0_config: [true,  true,  false,     # x, y, z
                   false, false, true,      # roll, pitch, yaw
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_rejection_threshold: 5.0   # optional: also gate by Mahalanobis distance
    smooth_lagged_data: true         # fixes arrive after their stamp (detection time)
    history_length: 1.0
```

For `3d`, set the first six entries of `pose0_config` to true and
`two_d_mode: false`. Then:

```bash
ros2 launch marker_localizer localizer.launch.xml marker_map:=... \
    prior_topic:=/odometry/filtered prior_type:=odometry
```

Another estimator takes the same message; it should use the covariance
as published and ignore the entries with variance 1000 in planar mode.

## Checking a setup

1. **The map file.** `ros2 run marker_survey marker_map_check sites/my_site/marker_map.yaml --png top.png`
   names the line and the field of every error and draws the markers
   from above.
2. **The camera.** `ros2 topic echo --once /camera/camera_info`: the
   `header.frame_id` is the optical frame, `k` is not zero, `d` is zero for
   a rectified image or holds the distortion of a raw one.
3. **The mounting.** `ros2 run tf2_ros tf2_echo base_link camera_optical_frame`:
   the translation is where the lens is, in metres; the optical z axis
   points forward for a forward camera, up for an upward one.
4. **The survey's inputs.** The node logs every new marker and a status
   line every 5 s (frames, keyframes, views per marker). "no camera pose
   for this frame yet" names the missing odometry topic or TF; a best-effort
   odometry publisher and a reliable subscriber never match, which is why
   the node subscribes best effort.
5. **The detections.** `ros2 run rqt_image_view rqt_image_view /marker_survey/debug/image`
   (or `/marker_localizer/debug/image`): every marker on the sheet is
   outlined; red means too small or cut by the border.
6. **The survey's result.** `survey_report.yaml`: per marker the views,
   reprojection RMS, 1-sigma position, height and angle (relative to the
   other markers), and a verdict; drive past the markers marked
   `pass again` once more, closer, and save again. `frame_sd_xy_cm` and
   `frame_sd_z_cm` are the 1-sigma in the result frame; in `origin_marker`
   mode it grows with the distance from the origin marker, and passing the
   origin marker from more places, or choosing one in the middle of the
   site, shrinks it. It holds the marker size's uncertainty
   (`marker_size_sd`): heights all off by the same centimetre or two mean a
   size measured wrong, and measuring the black square again and solving
   the recording again with it (`marker_survey_solve survey_data.npz
   --marker-size M --output-dir ...`) fixes them. `marker_survey_compare` compares the map with a tape survey of a
   few markers; in `align_to_track` the report's `frame.fit_rms_m` should be
   about the localizer's own noise, and `frame.fit_sd_xy_cm` is how well the
   fit placed the survey on it (a localizer whose error holds over metres
   places it less well than its RMS over the number of keyframes).
7. **The localizer.** A status line every 10 s: images, fixes, rejected,
   priors, and the detection and fit time per frame. Drive a loop with
   the debug image open and the estimator's pose in RViz: fixes should
   arrive wherever markers are in view, the projected map corners (dots)
   should sit on the detected ones, few markers should be orange, and the
   estimator should not jump. A marker that is always orange has moved or
   was surveyed badly; "not published" lines in the log give the reason
   (gate, position sd, fit). The check screen of the web interface (below)
   shows the same from a browser, with a table of how often each marker
   agreed with the estimator's pose.
8. **Offline.** Record the camera, `camera_info`, TF and the odometry with
   `ros2 bag record`, then play the recording into the same launch files
   with `use_sim_time:=true process_all:=true` and `ros2 bag play BAG --clock`,
   as the examples below do.

## The web interface

`marker_survey_web_node` serves one page (port 8090) to any browser on the
robot's network, a phone included: printing and identifying markers, and on
the robot the survey, the check and the use of the map. It is plain HTML,
CSS and JavaScript shipped with the package, served by Python's standard
HTTP server: no build step, no internet. It has no login: run it on a
trusted network, or behind the robot project's own access control; writing
the map needs a confirmation on the page.

```bash
ros2 launch marker_survey_web web.launch.xml map_path:=$PWD/sites/my_site/marker_map.yaml
```

next to the survey node, started with `start_paused:=true` (the page starts
the survey and chooses its frame), and the localizer node.
`examples/ceiling/launch/web.launch.xml site:=$PWD/sites/my_site` starts all
three for the ceiling example. On a laptop without ROS, `marker_survey_web`
serves the printing and identifying pages only.

| Parameter | Default | Meaning |
|---|---|---|
| `host`, `port` | `0.0.0.0`, 8090 | where the page is served; `127.0.0.1`: this computer only |
| `survey_node`, `localizer_node` | `/marker_survey`, `/marker_localizer` | the nodes it talks to: their `node_name`, with a namespace if any |
| `map_path` | `''` | the map the localizer reads, which "Use the map" writes; empty: the localizer's `marker_map` when it is first heard from |
| `camera_rate`, `camera_width` | 5.0, 640 | [Hz], [px] camera frames sent as JPEG, only while the page shows a camera |
| `check.agree_distance`, `check.agree_angle_deg` | 0.15, 5.0 | [m], [deg] a fix this close to the estimator's pose agrees |
| `check.min_samples`, `check.max_disagree` | 5, 0.2 | a marker is flagged when it disagrees in more than this share of at least this many fixes |

What it uses: the survey node's `~/status`, `~/start`, `~/pause`, `~/reset`,
`~/save`, `~/debug/image` and its `frame.*` parameters; the localizer's
`~/status`, `~/fixes`, `~/reload`, `~/debug/image` and its `marker_map`
parameter. The screens:

- **Survey.** Start: "Use the robot's map" (`align_to_track` when the
  survey has a `frame.reference_frame`, else `pose_frame`) or "Start from a
  marker" (`origin_marker`). While driving: the camera with the detections
  drawn, the map from above, the markers with a light each and hints. Save:
  the solve runs on the robot and the report opens; "Drive again" adds to
  the same recording, and the next save keeps the earlier one in
  `history/`.
- **Check.** The localizer on the map just surveyed (a trial: its
  `marker_map` is pointed at the survey's file, nothing is written) or on
  the map in use; each fix against the estimator's pose (the prior at the
  image's stamp), the distance, and per marker how often it agreed. A marker
  that disagrees is flagged: it moved, or it was surveyed badly.
- **Use the map.** What changes (markers added, removed, moved), then, after
  a confirmation, the survey's map is written to `map_path`; the file it
  replaces is kept in `marker_map_versions/` beside it, and the localizer
  reads the new one at once. Undo puts the newest kept version back.

The JSON API (`GET /api/state`, `POST /api/survey`, `GET /api/report`,
`GET|POST /api/map`, `GET|POST /api/check`, WebSocket `/api/live`) is
listed in `marker_survey_web/marker_survey_web/server.py`; a robot project
can embed the page in its own interface with a link or an iframe, and open a
step directly (`/#survey`, `/#check`, `/#use`).

## The examples

| Example | Setup | Survey | Localizer |
|---|---|---|---|
| `examples/wall_course/` | wall boards (`DICT_5X5_1000`, 0.38 m) around a 10 x 4 m course; a vehicle's forward camera with visual-inertial odometry | `origin_marker` (board 10), odometry pose source | `3d`, gated against a `PoseWithCovarianceStamped` |
| `examples/ceiling/` | ceiling markers (tag36h11, 0.45 m at 3 m) in an office; an indoor robot's upward webcam, wheel odometry, a LiDAR localizer | `align_to_track` against the LiDAR localizer's `map -> base_link`, TF pose source | `planar`, gated against robot_localization's `Odometry` |

Each holds `launch/survey.launch.xml`, `launch/localizer.launch.xml` (the
ceiling example also `launch/web.launch.xml`, which starts both with the web interface) and
`sites/<site>/marker_map.yaml` with a `route.csv` (the path of the simulated
drive), laid out as a robot project would. The launch files' defaults are a
real robot's topics; for the simulated recording the commands below pass
the recording's topics instead, the way a robot project passes its own.

The simulated recording (`marker_survey_synthetic`) renders what the camera
sees while the robot drives the route, and writes a rosbag2 with
`/camera/image`, `/camera/camera_info`, `/odom` (drifting 1 % per metre),
`/tf` (`odom -> base_link`, and `map -> odom` so that `map -> base_link` is
a localizer's pose with 5 cm and 1 degree of noise), `/tf_static`
(`base_link -> camera_link -> camera_optical_frame`) and `/prior_pose` (the
same noisy pose, as an estimator's), and beside it the truth.

### From a clean checkout

ROS 2 Humble or Jazzy:

```bash
mkdir -p ~/marker_ws/src && cd ~/marker_ws/src
git clone <this repository> marker_survey
cd ~/marker_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
cd src/marker_survey
```

### Wall boards

Render the recording (426 frames, a minute), then survey it. Start the
survey in one terminal:

```bash
out=/tmp/wall_course
ros2 run marker_survey marker_survey_synthetic $out/sim \
    --map examples/wall_course/sites/test_course/marker_map.yaml \
    --route examples/wall_course/sites/test_course/route.csv \
    --camera forward --origin 10 --laps 1 --sweeps 4 --rate 3 --map-noise 0.05 --map-noise-yaw-deg 1.0

ros2 launch examples/wall_course/launch/survey.launch.xml output_dir:=$out/survey origin_marker:=10 \
    image_topic:=/camera/image odom_topic:=/odom use_sim_time:=true process_all:=true
```

and play the recording in a second one (`out=/tmp/wall_course` there too),
then save and compare with the truth in the frame of board 10:

```bash
ros2 bag play $out/sim/bag --clock --delay 3
ros2 service call /marker_survey/save std_srvs/srv/Trigger
ros2 run marker_survey marker_survey_compare $out/survey/marker_map.yaml $out/sim/truth_origin.yaml
```

The median error is under 1 cm. Stop the survey (Ctrl-C), then localize on
the course's map: start the localizer in the first terminal,

```bash
ros2 launch examples/wall_course/launch/localizer.launch.xml \
    marker_map:=$PWD/examples/wall_course/sites/test_course/marker_map.yaml \
    image_topic:=/camera/image prior_topic:=/prior_pose detect_scale:=1.0 \
    use_sim_time:=true process_all:=true
```

record its fixes in a third (`ros2 bag record -o /tmp/wall_course/poses
/marker_localizer/pose`), play the recording again in the second, stop the
recorder, and score the fixes against the true poses:

```bash
ros2 bag play $out/sim/bag --clock --delay 3
ros2 run marker_localizer marker_localizer_score $out/poses $out/sim/truth_poses.csv --mode 3d
```

`detect_scale:=1.0` because the simulated camera is 960 x 600; the default
0.5 is for a 1920 x 1200 one. The survey's frame is board 10's, the
recording's truth is in the course map's frame, so the localizer uses the
course map here; on a real course it uses the surveyed one.

### Ceiling markers

The same steps (920 frames, two laps of the office at 0.5 m/s):

```bash
out=/tmp/ceiling
ros2 run marker_survey marker_survey_synthetic $out/sim \
    --map examples/ceiling/sites/office/marker_map.yaml \
    --route examples/ceiling/sites/office/route.csv \
    --camera upward --laps 2 --rate 5 --speed 0.5 --map-noise 0.05 --map-noise-yaw-deg 1.0

ros2 launch examples/ceiling/launch/survey.launch.xml output_dir:=$out/survey \
    image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
    use_sim_time:=true process_all:=true
```

```bash
ros2 bag play $out/sim/bag --clock --delay 3
ros2 service call /marker_survey/save std_srvs/srv/Trigger
ros2 run marker_survey marker_survey_compare $out/survey/marker_map.yaml $out/sim/truth_map.yaml
```

`align_to_track` writes the map in the LiDAR map's frame, so the robot
localizes on the map it has just surveyed:

```bash
ros2 launch examples/ceiling/launch/localizer.launch.xml marker_map:=$out/survey/marker_map.yaml \
    image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
    prior_topic:=/prior_pose prior_type:=pose use_sim_time:=true process_all:=true
```

```bash
ros2 bag record -o $out/poses /marker_localizer/pose      # third terminal; Ctrl-C after the play
ros2 bag play $out/sim/bag --clock --delay 3
ros2 run marker_localizer marker_localizer_score $out/poses $out/sim/truth_poses.csv --mode planar
```

The recording's prior is a `PoseWithCovarianceStamped`, hence
`prior_type:=pose`; on the robot it is robot_localization's `Odometry`
(the launch file's default).

### In one go

`ci/example.sh wall_course` and `ci/example.sh ceiling` run exactly these
steps and fail when the survey or the localizer misses its limits; CI runs
both on Humble and Jazzy after installing the dependencies with rosdep, as
above. Results are in
[phase-4-adoption.md](roadmap/phase-4-adoption.md#result).
