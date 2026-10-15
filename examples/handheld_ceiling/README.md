# Ceiling markers, handheld: a survey with the camera alone

No robot: a person holds a laptop with a USB webcam pointing up at the
ceiling markers and walks the routes a robot will drive, surveys the
markers with the camera alone, then localizes handheld on the map; the map
then makes a Gazebo world for a simulated robot.
[docs/integration.md](../../docs/integration.md#a-handheld-survey-no-robot)
explains the setup; this directory holds its launch files and a simulated
site.

| Item | Value |
|---|---|
| markers | twenty tag36h11, 0.40 m black square on 0.50 m sheets, a 1.4 m grid (5 x 4) under a flat 2.9 m ceiling; marker 0, near the middle, at the map's origin with rotation 0 |
| camera | 640 x 480, 70 degrees horizontal, held 1.0 m above the floor, looking up, the image's long side along the walk |
| walk | 0.5 m/s along every row, then every column (`sites/office/route.csv`, 60 m, 2 minutes), out past the west end where no marker is in view, back along the middle row; the hand shakes (2 degrees of roll and pitch, 2 cm of height) and, halfway along the middle column, tilts the camera away from the ceiling for 3 s |
| survey | `launch/survey.launch.xml`: `pose_source: markers`, `origin_marker` 0, `vertical: ceiling`, the floor 1.0 m below the camera |
| localizer | `launch/localizer.launch.xml`: `3d`, no prior, `base_frame` empty (the pose is the camera's) |

`marker_plan --ceiling 2.9 --hfov 70 --handheld --size 0.40` gives the
spacing: R = 0.69 m, D <= 1.39 m without odometry, so 1.4 m is at the
limit; holding the image's long side along the walk (R = 1.03 m that way)
keeps two markers of a row in view together over 0.67 m of the way.

## On the simulated recording

From the repository root, in a sourced workspace (or all of it with
`ci/example.sh handheld_ceiling`):

```bash
out=/tmp/handheld
ros2 run marker_survey marker_survey_synthetic $out/sim \
    --map examples/handheld_ceiling/sites/office/marker_map.yaml \
    --route examples/handheld_ceiling/sites/office/route.csv \
    --camera handheld --open --rate 5 --speed 0.5 --origin 0 --tilt-at 67

ros2 launch examples/handheld_ceiling/launch/survey.launch.xml output_dir:=$out/survey \
    image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
    use_sim_time:=true process_all:=true
```

In a second terminal:

```bash
ros2 bag play $out/sim/bag --clock --delay 3
ros2 service call /marker_survey/save std_srvs/srv/Trigger
ros2 run marker_survey marker_survey_compare $out/survey/marker_map.yaml $out/sim/truth_origin.yaml --require-good
```

The survey's frame is marker 0's, which is the site map's, so the localizer
runs on the map just surveyed and is scored against the true camera poses;
`scene.json` gives the stamps of the tilted stretch:

```bash
ros2 launch examples/handheld_ceiling/launch/localizer.launch.xml marker_map:=$out/survey/marker_map.yaml \
    image_topic:=/camera/image camera_info_topic:=/camera/camera_info \
    use_sim_time:=true process_all:=true
ros2 bag record -o $out/poses /marker_localizer/pose      # third terminal; Ctrl-C after the play
ros2 bag play $out/sim/bag --clock --delay 3
ros2 run marker_localizer marker_localizer_score $out/poses $out/sim/truth_poses.csv --mode 3d \
    --gap $(python3 -c "import json; print(*json.load(open('$out/sim/scene.json'))['tilt_window'])")
```

And a Gazebo Harmonic world from the surveyed map:

```bash
ros2 run marker_survey marker_map_to_sdf $out/survey/marker_map.yaml $out/world/office.world --world
gz sim $out/world/office.world
```

## Result

`ci/example.sh handheld_ceiling`, the same on Humble and Jazzy:

| | Limit in CI | Result |
|---|---|---|
| survey: markers mapped, verdicts | all 20, all `good` | all 20 `good` |
| survey: 3D position error | median 1 cm, max 2 cm | median 0.20 cm, max 0.24 cm |
| survey: error over the frame's 1-sigma | (reported) | horizontal max 0.14, height max 0.21: the 1-sigma is cautious, mostly the marker size's 0.5 % (`marker_size_sd`) |
| frames not placed (tilted away, past the west end) | some | 57 of 597 |
| localizer, `3d`, no prior: camera position error | mean 1 cm, max 6 cm | mean 0.37 cm, median 0.25 cm, max 4.6 cm (539 fixes of 597 images) |
| no fix while the camera is tilted away (2.4 s) | none | none |
| the first fix after it | within 2 cm | 0.11 cm, 0.3 s after |

The limits are half the robot ceiling example's survey limit (median 2 cm,
there against a reference track with 5 cm of noise) and its localizer's
mean (1 cm); nothing but the rendered corners limits the camera alone
here. The largest localizer errors come from the frames at the edges of the
tilted stretch, which see markers at a slant. The world from
`marker_map_to_sdf --world` passes `gz sdf -k` (Jazzy, in CI); a camera
rendered by Gazebo Harmonic 1 m below that world's ceiling was placed by
the localizer within 2 mm of where it was (checked once, not in CI).
