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

Status: planning. See [ROADMAP.md](ROADMAP.md) and the phase documents in
[docs/roadmap/](docs/roadmap/).

## Licence

Apache-2.0 (`LICENSE`).
