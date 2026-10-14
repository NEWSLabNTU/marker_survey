# Phase 6: real sites

## Goal

Measured accuracy on one real site of each kind, recorded in the README,
and the problems that only real sites show fixed.

## Wall-board site

1. Place 10 or more boards around a course; measure each marker's square.
2. Measure the true positions: tape from two fixed points per marker, or a
   total station if one is available (`truth.yaml`, the same format).
3. Survey from the web interface, two laps, `origin_marker`.
4. `marker_survey_compare marker_map.yaml truth.yaml`: position and angle
   errors per marker.
5. Localizer: drive the course with an independent reference if available
   (tape marks at stops), 3d and planar.

## Ceiling site

1. Plan and place markers from the web interface; measure the ceiling height
   at each marker with a laser distance meter.
2. Calibrate the camera's tilt on the robot (the tilt fit from phase 0)
   before the survey: one degree of tilt moves every ceiling point about
   5 cm at 3 m.
3. Survey, `align_to_track` against the LiDAR localizer.
4. Truth: tape from walls to marker centres for a subset; compare.
5. Localizer: stops at taped places, error measured by hand.

## What to watch for

| Effect | Where it shows | Check |
|---|---|---|
| Motion blur | fewer detections while turning | detection rate against speed and turn rate |
| Lighting: glare, flicker, backlight | missed or jumping corners | fixed exposure; 50/60 Hz anti-flicker; matte prints |
| Sagging or bent sheets | reprojection RMS and tilt flags in the report | mounting on rigid boards |
| Rolling shutter | position error that grows with speed | compare stopped and moving fixes |
| Wrong `marker_size` | every distance scaled | measured square in the report header; check against tape |

## Done when

Both sites' accuracy tables are in the README, with the conditions (camera,
marker size, speed, lighting) next to them.
