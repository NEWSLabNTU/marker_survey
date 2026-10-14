# Phase 4: adoption by robot projects

## Goal

A robot project uses the packages as a git submodule (or from a release)
with nothing but its own launch file, its camera remappings and its state
estimator's configuration. This phase writes the guide and the examples for
that, and moves the two source implementations onto the packages.

## What a robot project provides

| Item | Example |
|---|---|
| Camera topics | rectified image and `camera_info` |
| TF | `base_frame -> camera optical frame` (URDF); a pose source for the survey (odometry topic, or the map localizer's TF) |
| Map file location | `sites/<site>/marker_map.yaml` |
| Estimator wiring | remap the localizer's output into robot_localization or another EKF, with the covariance it publishes |

## What the repository provides

- `examples/wall_course/`: survey and localizer launch for a forward camera
  with visual-inertial tracking, a 6-DoF estimator;
- `examples/ceiling/`: survey (`align_to_track` against a LiDAR localizer)
  and planar localizer launch for an upward camera, robot_localization;
- `docs/integration.md`: the table above, frame conventions, parameters,
  how to check the setup (debug image, report, a drive with the check
  screen).

## Moving the source implementations

Each source project, in its own repository:

1. adds the submodule, pinned to a tag;
2. imports its existing site with the phase 1 importer, once;
3. switches its launch files to `survey_node` and `localizer_node`;
4. runs its own checks (simulation goals, kidnap recovery, course runs);
5. deletes its copies.

## Releases

Semantic version tags; the map file has its own `version` and the loader
keeps reading older versions. A changelog entry for every change of a
parameter or a file format.

## Tasks

- [ ] `docs/integration.md`
- [ ] The two examples, each with a simulated recording that runs in CI
- [ ] Tag 0.1.0 once phases 1 to 3 are done
- [ ] Support both source projects' migration (issues and fixes here)

## Done when

Both source projects run on a tagged release with their own checks green,
and an example runs from a clean checkout following only the guide.
