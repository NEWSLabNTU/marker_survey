# marker_survey

Map fiducial markers (ArUco, AprilTag) in place with a moving robot's camera,
then localize the robot from them. ROS 2 Humble and Jazzy, Python.

Two kinds of setups are served by the same code:

- **Wall boards, 3D pose.** Marker boards stand around a course or hall, and
  a vehicle's forward camera sees them from the side. The localizer gives
  the full 6-DoF camera pose.
- **Ceiling markers, planar pose.** Markers hang on the ceiling, and an
  indoor robot's upward camera sees them from below. The robot drives on a
  flat floor, so the localizer fits only x, y and yaw; roll and pitch come
  from the robot. With two markers in view a full 6-DoF solve was 36 cm off
  where the planar fit was 0.5 cm.

Both share the detection, the survey (a bundle adjustment of every marker
and camera pose), the marker map file and the localizer core. A robot adds
only its launch file and the remapping into its own state estimator.

Status: phases 0 to 3 of [ROADMAP.md](ROADMAP.md) are done: the survey for
wall boards and ceiling markers, in three frame modes, the marker map file,
the localizer in both modes, the tools and CI. Of phase 4, the integration
guide, [docs/integration.md](docs/integration.md), and the two examples
that run in CI are done; the 0.1.0 tag and the move of the source projects
onto the packages are open. Printing and identifying markers from a browser
are in. Timing on a
Jetson-class board is open; the rest of the web interface comes next.
Changes are in [CHANGELOG.md](CHANGELOG.md).

## Packages

| Package | Contents |
|---|---|
| `marker_survey` | `marker_survey.core` (detection, SE(3), the survey and its bundle adjustment, frames, the map file, the report, simulation and rendering; plain Python, no ROS), the survey node, command-line tools |
| `marker_localizer` | `marker_localizer.core.localize` (the 3d and planar solvers, covariance, gate; no ROS), the localizer node, its evaluation and scoring tools |
| `marker_survey_web` | the browser interface: printing marker sheets and identifying markers from a photo (the rest of phase 5 to come) |

## Install

In a ROS 2 Humble or Jazzy workspace, with this repository under `src/`:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

Dependencies: numpy, scipy, OpenCV (`python3-opencv`, 4.5 to 4.10), PyYAML,
rclpy; `rosbag2_py` for the synthetic recording. The core modules import no
ROS package: with `src/marker_survey/marker_survey` on `PYTHONPATH` they run
in tests, notebooks and offline tools.

## Quick start without a robot

Render a recording of the example wall course, play it through the survey
node, and compare the result with the truth:

```bash
ros2 run marker_survey marker_survey_synthetic /tmp/demo \
    --map examples/wall_course/sites/test_course/marker_map.yaml \
    --route examples/wall_course/sites/test_course/route.csv --camera forward --origin 10 --sweeps 4

ros2 launch marker_survey survey.launch.xml output_dir:=/tmp/demo/result \
    dictionary:=DICT_5X5_1000 marker_size:=0.38 origin_marker:=10 use_sim_time:=true &
ros2 bag play /tmp/demo/bag --clock
ros2 service call /marker_survey/save std_srvs/srv/Trigger

ros2 run marker_survey marker_survey_compare /tmp/demo/result/marker_map.yaml /tmp/demo/truth_origin.yaml
```

`ci/synthetic_survey.sh wall` does all of this in one go, and
`ci/synthetic_survey.sh ceiling_align` (or `ceiling_pose_frame`,
`ceiling_uneven`) the same for nine ceiling markers seen by an upward
webcam, with a noisy localizer pose as the reference.
[docs/integration.md](docs/integration.md) runs the two examples, a wall
course and a ceiling, through their own launch files the same way.

## Surveying a site

1. Print the markers; measure the edge of the black square (`marker_size`),
   not the sheet. A wrong size scales every distance.
2. Start the robot's camera driver, its pose source and its TF, then the
   survey (`examples/wall_course/` and `examples/ceiling/` have a wall-board
   and a ceiling setup; [docs/integration.md](docs/integration.md) lists
   what the robot provides):

   ```bash
   ros2 launch marker_survey survey.launch.xml output_dir:=$PWD/my_site \
       dictionary:=DICT_5X5_1000 marker_size:=0.38 origin_marker:=10
   ```

3. Drive slowly past every marker, from two or three places each; the log
   names every new marker, and `~/debug/image` shows the detections (green
   known, yellow new, red not used).
4. Ctrl-C, or `ros2 service call /marker_survey/save std_srvs/srv/Trigger`.
   The node writes to `output_dir`:
   - `marker_map.yaml`: the markers (below);
   - `survey_report.yaml`: per marker the views, reprojection RMS, 1-sigma
     position, height and angle, flags and a verdict (`good`, `pass again`,
     `bad`);
   - `survey_data.npz`: the recording, to solve again without the robot:
     `ros2 run marker_survey marker_survey_solve my_site/survey_data.npz --output-dir my_site`.

The image may be rectified, or raw with distortion coefficients in its
camera_info (the corners are then undistorted).

### Frame modes

| `frame.mode` | Result frame | For |
|---|---|---|
| `origin_marker` | origin on the floor below a chosen marker (`frame.origin_marker`), x the way it faces, z up | a site with no other map (wall courses) |
| `pose_frame` | the pose source's own frame, such as `map` while a LiDAR localizer runs; every keyframe's pose is a prior weighted by `pose_sd` [m] and `pose_sd_deg` | markers that must agree with an existing map (places, zones, docks are in it) |
| `align_to_track` | survey in the odometry frame, then a rigid fit (Umeyama, no scale) of the solved trajectory to a reference trajectory at the same stamps: TF `frame.reference_frame -> base_frame`, the localizer's poses | the same, without the localizer's jumps inside the solve |

The floor normal (z up) comes from the `base_frame` poses in every mode, so
an upward camera needs nothing special; `floor_offset` moves the floor below
`base_frame` (a vehicle on a stand). In `align_to_track` the report gives
the fit's residual (`frame.fit_rms_m`, `fit_max_m`): about the localizer's
own noise when all is well.

### Pose sources

- `pose_source: odometry`: a `nav_msgs/Odometry` topic (`odom_topic`):
  visual-inertial, wheel or IMU odometry; its motion between keyframes is a
  factor weighted by `odom_drift` (translation error per metre).
- `pose_source: tf`: TF `pose_frame -> camera optical frame`, any frame,
  any camera direction.

### Detection

One detector for ArUco and AprilTag families on OpenCV 4.5 to 4.10.
AprilTag families use `CORNER_REFINE_APRILTAG` (OpenCV 4.6 needs it to
decode tag36h11); every family then gets `cornerSubPix` with a window of half
a cell and a line fit to each edge of the black square, which removes the
0.15 to 0.2 px inward bias of the corners. Markers touching the image border
(`border_margin_px`) or under `min_marker_px` are not used.

## The marker map file

One file per site, `marker_map.yaml`, written by the survey, editable by
hand, read by robots as it is (no conversion step):

```yaml
# Ceiling markers of the warehouse, surveyed 2026-11-03.
version: 1
frame_id: map                 # the frame the robot's map uses
dictionary: tag36h11          # printed family
marker_size: 0.45             # [m] edge of the black square, measured
mount: ceiling                # default for every marker below
markers:
  - {id: 0,  x: -4.000, y:  1.500, z: 3.012, rotation_deg:   0.0}
  - {id: 7,  x:  3.000, y: -2.000, z: 3.204, rotation_deg:  91.7, tilt_x_deg: 1.2}   # under the duct
  - {id: 12, x:  0.300, y:  3.000, z: 3.005, rotation_deg:  17.2, size: 0.30}
```

Metres and degrees. A marker's orientation is given relative to its
mounting, so a well-mounted marker has small numbers; missing angles are
zero:

| `mount` | Reference pose (all angles zero) | Angles |
|---|---|---|
| `wall` | upright, facing +x, top up | `facing_deg` (about z), `lean_deg` (top leaning back), `turn_deg` (in its own plane, counter-clockwise from the front) |
| `ceiling` | facing down, the image's x axis along +x | `rotation_deg` (about z, counter-clockwise from above), `tilt_x_deg`, `tilt_y_deg` |
| `floor` | facing up, the image's x axis along +x | `rotation_deg`, `tilt_x_deg`, `tilt_y_deg` |

- `mount` at the top is the default; a marker may carry its own.
- `size` on a marker overrides `marker_size` (sites mix sheet sizes).
- Uncertainties and view counts are in `survey_report.yaml`, not here.
- Comments survive a survey: a new survey keeps the header and the comments
  of markers it finds again.
- `dictionary`: an OpenCV name (`DICT_5X5_1000`, ...) or `tag16h5`,
  `tag25h9`, `tag36h10`, `tag36h11`, `aruco_mip_36h12`, `aruco_original`.

`marker_survey/marker_survey/core/marker_map.py` defines the rotation
matrices; `marker_survey/schema/marker_map.schema.json` is a JSON Schema
for editors. Errors name the line and the field:

```
$ ros2 run marker_survey marker_map_check site/marker_map.yaml
ERROR: site/marker_map.yaml:9: marker 1: facing_deg is an angle of mount wall, not ceiling (use rotation_deg, tilt_x_deg, tilt_y_deg)
```

A valid map is summarised and drawn from above (`--png top.png` for an
image). Existing sites in the two older formats (a course file of wall
boards with yaw, pitch, roll in radians; a plain list of ceiling markers
with x, y, z, yaw) convert once with
`ros2 run marker_survey marker_map_import course ...` or `... ceiling-list ...`.

## Localizing on the markers

The localizer reads the site's `marker_map.yaml` and the camera, and
publishes `base_frame`'s pose in the map's `frame_id` as
`geometry_msgs/PoseWithCovarianceStamped` (`~/pose`), stamped with the
image's capture time, for the robot's state estimator to fuse:

```bash
ros2 launch marker_localizer localizer.launch.xml marker_map:=$PWD/my_site/marker_map.yaml \
    mode:=planar prior_topic:=/odometry/filtered prior_type:=odometry
```

| `mode` | Fits | For |
|---|---|---|
| `planar` | x, y, yaw; z, roll and pitch known (`roll_pitch`: `zero`, `tf` from a level frame such as `odom`, or `imu`) | a robot on a flat floor: ceiling markers, also wall boards |
| `3d` | the full 6-DoF pose | wall boards; a vehicle whose roll and pitch matter |

- Planar cuts each corner's ray with its own marker's plane from the map
  (uneven ceilings, tilted boards) and fits x, y and yaw to all corners;
  3d starts from each marker's IPPE solutions and refines every corner of
  every marker together. Both use a robust loss and leave out a marker that
  disagrees with the others.
- Covariance from the fit: it grows with distance and shrinks with the
  markers in view, with floors (`min_sd`, `min_sd_deg`); a fix with a
  position sd over `max_position_sd` (0.25 m) is not published.
- `prior_topic` (the estimator's pose, `PoseWithCovarianceStamped` or, with
  `prior_type: odometry`, `Odometry`): fixes farther than `gate_distance`
  (1 m) or `gate_angle_deg` (15) from it are dropped (a misread id, a moved
  marker); in 3d it also picks between the two solutions of a lone marker.
- `enabled` can be switched at run time (`ros2 param set /marker_localizer
  enabled false`); `~/debug/image` shows the markers used (green, the map's
  corners projected with the fix as dots, the residual written), left out
  (orange: outlier, far) or unusable (red), and the fix.
- `detect_scale: 0.5` finds markers in a half-size image and refines their
  corners on the full one (a 1920 x 1200 camera); the node logs its
  detection and fit times every 10 s.

`examples/ceiling/` has a ceiling robot (planar, an `Odometry` prior) and
`examples/wall_course/` a wall-board vehicle (3d); wiring the fixes into
robot_localization is in [docs/integration.md](docs/integration.md). Without a robot,
`ci/synthetic_localize.sh ceiling` (or `wall`, `ceiling_gate`) renders a
recording, plays it through the node and scores the fixes against the truth;
`marker_localizer_evaluate` runs both modes and the source implementations'
methods on the same frames without ROS.

## Printing markers

From a browser, on a laptop or the robot (no ROS needed for this part):

```bash
ros2 run marker_survey_web marker_survey_web --open          # http://127.0.0.1:8090
ros2 run marker_survey_web marker_survey_web --host 0.0.0.0  # also from a phone on the network
```

Give the marker size (the edge of the black square) and how many markers;
the page shows the paper, the IDs and a preview of every page, and
downloads the PDF. "Don't know the size?" works it out from the ceiling
height and the camera (`marker_plan`). The "Identify a marker" tab reads the
IDs from a phone photo of a sheet or a ceiling.

The same from a terminal:

```bash
marker_sheets --size 45cm --count 24 -o markers.pdf                    # smallest of A4 to A0 that fits
marker_sheets --size 45cm --count 24 --paper A1 -o markers.pdf
marker_sheets --size 45cm --count 24 --paper exact --border 6cm -o markers.pdf
marker_sheets --size 45cm --count 24 --split -o markers/               # one PDF per marker
marker_sheets --size 12cm --ids 0-9 --paper A4 -o small.pdf             # these IDs, four per page
marker_sheets --size 45cm --count 6 --site site/marker_map.yaml -o more.pdf   # skip IDs in use
marker_identify photo.jpg                                              # which markers are these?
```

| Choice | Default | Options |
|---|---|---|
| marker size | required, with a unit | `450mm`, `45cm`, `0.45m`, `17.7in` |
| how many | required unless IDs are given | |
| dictionary | `tag36h11` | any OpenCV ArUco or AprilTag dictionary |
| IDs | chosen for you | `--ids 0-9` or `3,7,12`; `--site`/`--exclude` keeps IDs in use out |
| paper | the smallest of A4, A3, A2, A1, A0 that fits | A0 to A6, B1 to B5, Letter, Legal, Tabloid, `600x850mm`, or `exact` (the page is the marker and its white border) |
| white border | one cell of the marker | `--border 6cm` |
| output | one PDF, a page per marker | `--split`: a PDF per marker, named `tag36h11_45cm_id017.pdf` |
| labels | ID on each sheet | `--no-label`, `--back-labels` (a page with the ID in large print after each, for duplex) |

**The IDs chosen for you** are the same every time for the same dictionary,
count and exclusions: starting from the lowest free ID, each next one is the
ID whose bits differ most from all chosen so far and from the ones already
in use, in any rotation. Markers that differ in many bits are the hardest to
mistake for one another (24 tag36h11 markers come out at least 13 bits
apart).

**Every marker says its ID**, because a sheet without a label is easy to
lose track of: the PDF's bookmark and page label ("id 17"), the file name
with `--split`, the ID in large print outside the white border (with an
arrow for the top), small and grey in the border's bottom-right corner on
an exact-size page (which also marks the marker's bottom edge), an optional
back page, and a contact sheet at the end of the PDF plus a CSV list
(`markers.csv`, or `<name>.csv`), with a "placed where" column to fill in on
site. Once a marker is up, a photo tells its ID (`marker_identify`, or the
web page's "Identify a marker").

Pages are vector graphics, one rectangle per run of black cells, so the
printer gets exact millimetres. Each page has a 100 mm scale bar: print at
actual size, check it with a ruler, then measure the black square of a
printed sheet; that length is the `marker_size` of the survey. The tests
render every kind of page with pdftoppm and read the IDs and the size back
with the detector.

## Tools

Each runs as `ros2 run marker_survey <command>` (the last three as
`ros2 run marker_localizer <command>`); `--help` lists the options.

| Command | Does |
|---|---|
| `marker_survey` (node) | the survey |
| `marker_survey_solve` | solve a recording again, other frame or settings |
| `marker_survey_synthetic` | render a recording from a marker map and a route CSV |
| `marker_survey_compare` | a surveyed map against the truth or a tape survey |
| `marker_map_check` | validate a map, draw it from above |
| `marker_map_import` | convert an older site file into `marker_map.yaml` |
| `marker_plan` | ceiling marker size and spacing from the ceiling height and camera |
| `marker_localizer` (node) | the localizer |
| `marker_localizer_evaluate` | both modes and the source methods on a rendered or corner-level scene, no ROS |
| `marker_localizer_score` | a recording of the localizer's fixes against the truth |
| `marker_sheets` | print-ready PDF sheets of markers at an exact size (below) |
| `marker_identify` | the marker IDs in a photo |

`ros2 run marker_survey_web marker_survey_web` serves the browser interface.

## Accuracy

Rendered recordings played through the survey node (`ci/synthetic_survey.sh`,
marker position error in 3D; the same numbers on Humble and Jazzy):

| Scene | Frame mode | Target | Result |
|---|---|---|---|
| 11 wall boards of 0.38 m, 10 x 4 m oval, forward camera 960 x 600 at 102 deg, one lap at 3 Hz, odometry drift 1 % | `origin_marker` | median 1 cm, max 4 cm | median 0.9 cm, max 1.5 cm |
| 9 ceiling tags of 0.45 m at 3 m, office routes (2 laps, 92 m), upward camera 640 x 480 at 70 deg, map pose noise 6 cm per axis and 1 deg | `pose_frame` | median under 4 cm | median 0.7 cm, max 1.0 cm |
| the same | `align_to_track` | median under 2 cm | median 0.6 cm, max 1.2 cm |
| uneven ceiling (2.8 to 3.4 m), tags tilted 6 and 4 deg | `align_to_track` | heights within 2 cm, tilts within 1 deg | heights within 0.7 cm, tilts within 0.12 deg |

Corner-level simulations of the same scenes run as unit tests
(`test/test_scenes.py`).

The localizer, base position error against the truth, mean / max
(details and the comparisons in
[phase-3-localizer.md](docs/roadmap/phase-3-localizer.md)):

| Scene | Mode | Target | Corner-level, 0.5 px noise | Rendered, through the node |
|---|---|---|---|---|
| the office, 9 ceiling tags, 2 laps | `planar` | mean 1 cm, max 3 cm | 0.38 / 2.44 cm | 0.02 / 0.13 cm |
| the same, two tags in view | `planar` against `3d` | planar within 2 cm, 3d worse | 0.37 cm against 7.3 cm max | 0.03 cm against 0.46 cm max (rendered offline) |
| the test course, 11 wall boards, one lap | `3d` | no worse than the source's method | 5.4 / 38.6 cm (source method 33.5 / 102 cm) | 0.90 / 7.8 cm (source method, same frames offline: 5.5 / 97 cm) |
| the office with marker 5 moved 1.5 m | `planar` | rejected by the gate | every fix from it alone rejected | 92 gated, max 0.13 cm |

Per frame on one arm64 server core (CI): 640 x 480 grey, 1.8 ms detection
and 2.0 ms planar fit (6 % of the core at 15 Hz); 1920 x 1200 colour, 39 ms
detection (8 ms with `detect_scale` 0.5) and 8 ms 3d fit. A Jetson-class
board is not measured yet. Not yet measured on a real site
(phase 6).

## Development

```bash
colcon test && colcon test-result --verbose
ci/synthetic_survey.sh wall          # in a sourced workspace
ci/synthetic_localize.sh ceiling     # the localizer through its node
ci/example.sh ceiling                # an example's launch files, survey then localizer
ci/localizer_comparison.sh wall      # against the source methods, no ROS
ci/timing.sh                         # detection and fit per frame, one core
python3 ci/check_words.py --commits  # neutral wording
```

CI (GitHub Actions) builds and tests on `ros:humble` and `ros:jazzy`, runs
the four rendered surveys and the three rendered localizer recordings
through their nodes, the two examples through their launch files (installed
with rosdep), compares the localizer with the source methods, times
it on an x86 and an arm64 runner, and checks the wording.

## Licence

Apache-2.0 (`LICENSE`).
