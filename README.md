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

Status: phases 0 and 1 of [ROADMAP.md](ROADMAP.md) are done (the survey,
its tools and CI; the marker map file); the ceiling survey (phase 2) is in
progress, the localizer and the web interface come after.

## Packages

| Package | Contents |
|---|---|
| `marker_survey` | `marker_survey.core` (detection, SE(3), the survey and its bundle adjustment, frames, the map file, the report, simulation and rendering; plain Python, no ROS), the survey node, command-line tools |
| `marker_localizer` | placeholder for the localizer (phase 3) |
| `marker_survey_web` | placeholder for the browser interface (phase 5) |

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
    --map examples/sites/test_course/marker_map.yaml \
    --route examples/sites/test_course/route.csv --camera forward --origin 10 --sweeps 4

ros2 launch marker_survey survey.launch.xml output_dir:=/tmp/demo/result \
    dictionary:=DICT_5X5_1000 marker_size:=0.38 origin_marker:=10 use_sim_time:=true &
ros2 bag play /tmp/demo/bag --clock
ros2 service call /marker_survey/save std_srvs/srv/Trigger

ros2 run marker_survey marker_survey_compare /tmp/demo/result/marker_map.yaml /tmp/demo/truth_origin.yaml
```

`ci/synthetic_survey.sh wall` does all of this in one go.

## Surveying a site

1. Print the markers; measure the edge of the black square (`marker_size`),
   not the sheet. A wrong size scales every distance.
2. Start the robot's camera driver, its pose source and its TF, then the
   survey (`examples/launch/` has a wall-board and a ceiling setup):

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

### Frame and pose source

`frame.mode: origin_marker` puts the origin on the floor below a chosen
marker (`frame.origin_marker`), x the way it faces, z up (the floor normal,
from the `base_frame` poses).

- `pose_source: odometry`: a `nav_msgs/Odometry` topic (`odom_topic`):
  visual-inertial, wheel or IMU odometry; `odom_drift` is its translation
  error per metre.
- `pose_source: tf`: TF `pose_frame -> camera optical frame`.

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

## Tools

Each runs as `ros2 run marker_survey <command>`; `--help` lists the options.

| Command | Does |
|---|---|
| `marker_survey` (node) | the survey |
| `marker_survey_solve` | solve a recording again, other frame or settings |
| `marker_survey_synthetic` | render a recording from a marker map and a route CSV |
| `marker_survey_compare` | a surveyed map against the truth or a tape survey |
| `marker_map_check` | validate a map, draw it from above |
| `marker_map_import` | convert an older site file into `marker_map.yaml` |
| `marker_plan` | ceiling marker size and spacing from the ceiling height and camera |

## Accuracy

| Test | Marker position (3D) |
|---|---|
| wall course, rendered recording through the node (11 boards of 0.38 m, forward camera 960 x 600, one lap at 3 Hz, odometry drift 1 %) | median 0.8 cm, max 1.4 cm (Humble and Jazzy) |
| the same course, corner-level simulation, 0.48 m boards, 1920 x 1200, two laps | median 0.4 cm, max 0.5 cm |

## Development

```bash
colcon test --packages-select marker_survey && colcon test-result --verbose
ci/synthetic_survey.sh wall          # in a sourced workspace
python3 ci/check_words.py --commits  # neutral wording
```

CI (GitHub Actions) builds and tests on `ros:humble` and `ros:jazzy`, runs
the rendered survey through the node, and checks the wording.

## Licence

Apache-2.0 (`LICENSE`).
