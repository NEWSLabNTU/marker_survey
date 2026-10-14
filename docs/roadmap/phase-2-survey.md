# Phase 2: one survey for walls and ceilings

## Goal

The same survey node and solver map wall boards from a forward camera and
ceiling markers from an upward camera, into whichever frame the robot needs.

## How the survey works (kept from the source)

1. Every frame comes with a camera pose from a pose source.
2. Frames are kept as keyframes after 15 cm or 5 degrees of motion, or when
   a new marker appears; each marker gets a live estimate when first seen.
3. On save, a bundle adjustment solves every keyframe pose and every marker
   pose: corner reprojection (robust, 0.5 px) plus the pose source's motion
   between keyframes, weighted for its drift.
4. The result is expressed in the chosen frame and written to
   `marker_map.yaml` and `survey_report.yaml`; `survey_data.npz` keeps the
   recording so it can be solved again without the robot.

## What changes

### Frames

| `frame.mode` | Result frame | For |
|---|---|---|
| `origin_marker` (source behaviour) | origin on the floor below a chosen marker, x the way it faces, z up from the robot's poses | a site with no other map (wall courses) |
| `pose_frame` | the frame the pose source is in, such as `map` while a LiDAR localizer runs | markers that must agree with an existing LiDAR map (places, zones, docks are in it) |
| `align_to_track` | survey in `odom`, then a rigid fit of the keyframe trajectory to a reference trajectory recorded at the same time (the LiDAR localizer's poses) | the same, without the localizer's jumps inside the solve |

`align_to_track` is expected to beat `pose_frame`: odometry is smooth over
short distances, and the fit uses the whole trajectory once.

### Pose sources

- `odometry`: a `nav_msgs/Odometry` topic (visual-inertial tracking, wheel
  and IMU odometry), with its drift parameters;
- `tf`: `pose_frame -> camera optical frame`, any frame, any camera
  direction.

The floor normal (z up) comes from `base_frame` poses in every mode, so an
upward camera needs nothing special.

### Ceiling specifics

- Marker heights and small tilts come out of the solve; no hand-measured
  ceiling height.
- Markers seen only straight from below have weak height; the report flags a
  marker whose height uncertainty is over 2 cm and suggests passing it at an
  angle.
- `mount: ceiling` is written when the solved normals point down.

### Detection

One `core/detection.py` for ArUco and AprilTag families on OpenCV 4.5 to
4.10, per family settings that work: AprilTag families use
`CORNER_REFINE_APRILTAG` then `cornerSubPix`; markers touching the image
border are dropped; markers under `min_marker_px` are dropped.

### Report

`survey_report.yaml` per marker: views, reprojection RMS, 1-sigma position
and angle, flags (few views, large uncertainty, tilt over 10 degrees, weak
height). A one-line verdict per marker (`good`, `pass again`, `bad`) that
the web interface shows as a colour.

## Tests

| Scene | Camera | Frame mode | Target |
|---|---|---|---|
| 11 wall boards, 10 x 4 m oval, 2 laps | forward, 102 degrees | `origin_marker` | median 1 cm, max 4 cm (source) |
| 9 ceiling markers at 3 m, office routes | upward, 70 degrees, 640 x 480 | `pose_frame` with a noisy map pose (6 cm) | median under 4 cm |
| the same | the same | `align_to_track` | median under 2 cm |
| uneven ceiling (2.8 to 3.4 m), 2 tilted markers | upward | `align_to_track` | heights within 2 cm, tilts within 1 degree |

Each runs as corner-level simulation (fast, in unit tests) and as a rendered
recording played through the node (CI).

## Tasks

- [ ] Frame modes, with `align_to_track` (Umeyama fit, report of the fit
      residual)
- [ ] TF pose source for any frame; odometry source unchanged
- [ ] Ceiling scenes in the simulator and the renderer
- [ ] Detector settings per family; border and size rejection
- [ ] Report verdicts
- [ ] The tests above in CI

## Done when

All four tests pass in CI.
