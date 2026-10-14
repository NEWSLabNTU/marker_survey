# Roadmap

Goal: a person who is not a software engineer can plan, print, place and
survey markers at a new site, check the result, and have the robot localize
on them, all from a browser.

The code starts from two working implementations, one per setup:

| Part | Wall-board implementation | Ceiling implementation |
|---|---|---|
| Detection | OpenCV ArUco (`DICT_5X5_1000`), OpenCV 4.5.4 | tag36h11, OpenCV 4.6: needs `CORNER_REFINE_APRILTAG` and `cornerSubPix`; ignores markers clipped by the image border |
| Survey | keyframes from the camera's visual-inertial tracking, bundle adjustment of every keyframe and marker, 6-DoF markers with uncertainty, report, re-solve, synthetic recording, compare tool; 1 to 3 cm in simulation | averages sightings using the robot's map pose and a hand-measured ceiling height; x, y, yaw only; 6.7 cm in simulation |
| Localizer | full 6-DoF from the markers, gated against the state estimator | corner rays cut with the ceiling, 2D fit of x, y, yaw; 0.5 cm mean in simulation |
| Checks | drive by hand and watch localization; camera check | against the true pose in simulation; kidnap test; camera tilt calibration |

## Phases

| Phase | Document | Done when |
|---|---|---|
| 0 Repository | [phase-0-repository.md](docs/roadmap/phase-0-repository.md) | both distros build and test green; the synthetic survey matches the source's accuracy |
| 1 Marker map file | [phase-1-marker-map.md](docs/roadmap/phase-1-marker-map.md) | both setups' existing maps convert into it and back unchanged |
| 2 Survey for walls and ceilings | [phase-2-survey.md](docs/roadmap/phase-2-survey.md) | synthetic wall and ceiling surveys reach about 2 cm median |
| 3 Localizer, 3D and planar | [phase-3-localizer.md](docs/roadmap/phase-3-localizer.md) | each mode matches or beats its source implementation |
| 4 Adoption | [phase-4-adoption.md](docs/roadmap/phase-4-adoption.md) | a robot project runs on the packages with its own launch file only |
| 5 Web interface | [phase-5-web.md](docs/roadmap/phase-5-web.md) | someone new to the tools surveys a simulated site from the browser alone |
| 6 Real sites | [phase-6-real-sites.md](docs/roadmap/phase-6-real-sites.md) | accuracy measured against tape on one site of each kind |

```
P0 -> P1 -> P2 --+--> P4 -> P6
         \-> P3 -+
             P2 -> P5 (survey screens), P3 -> P5 (check screen)
```

## Decisions

- **Python** for every package. A compiled port only if measurements on the
  target computer show the localizer cannot keep up.
- **One map file**, `marker_map.yaml`, human-readable and editable by hand;
  robots read it directly, with no conversion step (phase 1).
- **Neutral repository.** It names no organisation, product, course or
  person; examples use generic sites.
