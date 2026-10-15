# Phase 5: the web interface

## Goal

Someone who has never used ROS sets up markers at a new site from a
browser: plan, print, place, survey, check, use. No terminal, no RViz, no
editing files.

## Shape

- `marker_survey_web`: a ROS 2 node that serves one page and a small JSON
  API on the robot (default port 8090), talking to the survey and localizer
  nodes. Any browser on the same network; a phone works for the survey step.
- Plain HTML, CSS and JavaScript shipped as files: no build step, no npm, no
  internet needed on site.
- Live data over a WebSocket: detections, keyframe count, marker list, the
  top-down map, localization fixes. Camera frames as JPEG at 5 Hz, only
  while the page shows them.
- A robot project embeds it in its own operator interface with a link or an
  iframe; the page works on its own too.
- No login of its own: it runs behind the robot project's access control,
  or on a trusted network. Writing a map needs a confirmation.

## The five steps

### 1. Plan and print

- Inputs: ceiling height (or the distance at which wall boards are seen),
  the camera (preset list or field of view and resolution), the routes
  drawn on the site map if there is one.
- Output: the marker size and spacing (the ceiling planner from phase 0),
  where to put each marker, and print-ready PDF sheets: one marker per
  sheet, ID in small print, cut marks, the measured-square reminder.

### 2. Place

A checklist per marker: flat, matte, no glare, white margin all round,
within the planned spot; a photo of each placed marker can be attached to
the site for later maintenance.

### 3. Survey

- Start: choose the frame mode in plain words ("use the robot's map" or
  "start from marker N").
- While driving: the camera with every detection drawn (green known, yellow
  new, red too small), the top-down map filling in, the marker list with
  one light each (from the report verdicts), and hints ("marker 7: seen
  from one place only, pass it again from another side").
- Save: the solve runs on the robot (seconds); the report opens; markers
  marked "pass again" can be re-driven and the solve repeated with the old
  recording kept.

### 4. Check

Drive the routes by hand. The page shows each marker fix against the robot's
fused pose, the distance between them, and a per-marker table of how often
it agreed. A marker that disagrees is flagged: it moved, or it was surveyed
badly.

### 5. Use

Write the map (with a confirmation) where the robot's localizer reads it,
keep the previous version, and tell the localizer to reload. Undo puts the
previous version back.

## API sketch

| Method and path | Does |
|---|---|
| `GET /api/state` | survey or check running, counts, verdicts |
| `POST /api/survey` | `{"action": "start", "frame": {...}}`, `stop`, `save` |
| `GET /api/report` | the last report |
| `POST /api/map` | `{"action": "use"}`, `undo` |
| `POST /api/plan` | inputs of step 1; returns the plan and a PDF link |
| `WS /api/live` | detections, map, fixes |

## Tests

- API tests against recorded survey data (no camera).
- A browser test (Playwright) that walks the five steps on a simulated
  recording.
- A usability session: one person who has not seen the tools surveys a
  simulated site; every point where they asked for help becomes an issue.

## Tasks

- [x] Node, static files, JSON API, WebSocket
- [x] Plan and print, pulled forward: `marker_sheets` and `marker_identify`
      on the command line, and the "Print markers" and "Identify a marker"
      pages (below)
- [x] Survey screens on top of the phase 2 node
- [x] Check screen on top of the phase 3 localizer
- [x] Map use, versions, undo, localizer reload
- [x] API tests against recorded survey data; browser test
- [ ] Usability session
- [ ] Place (step 2): the checklist per marker and photos of placed markers;
      plan (step 1): routes drawn on a site map
- [x] No robot: plan, a camera-only survey and a localization test with a
      handheld webcam (below)

## Printing and identifying, done ahead of the rest

- **Command line** (`marker_survey`): `marker_sheets` (size and count, or
  IDs; paper or exact size with a white border; one PDF or one per marker;
  labels, back labels, contact sheet, CSV) and `marker_identify` (IDs from a
  photo, any common family). `core/sheets.py` and `core/identify.py`, no
  ROS. The PDF is written directly as vector graphics; no reportlab or
  cairo dependency.
- **Web** (`marker_survey_web`): Python's standard HTTP server, no ROS, a
  static page: size and count first, paper, output, labels, then the
  dictionary, IDs and site exclusions under "More"; a live summary with
  plain-language checks, previews of the pages drawn from the same layout
  as the PDF, downloads (PDF, zip of PDFs, CSV); "Don't know the size?"
  runs the ceiling planner; "Identify a marker" takes a photo, also from a
  phone's camera.
- **Tests**: unit tests of units, papers, ID choice, layouts and the PDF;
  every kind of page rendered with pdftoppm and read back with the
  detector (IDs and the black square's size within a pixel); the web API
  against a running server.

## Done when

The usability session finishes a simulated site with no help on the
software, only on the hardware.

## Result

Everything but the usability session and the place checklist is done: the
node, the survey, check and map screens, the API tests and a browser test
that runs in CI. The usability session needs a person new to the tools and
is open, so the phase is not done yet.

The browser test (`ci/web_browser.sh`, CI job `web-browser`) starts
`examples/ceiling/launch/web.launch.xml` on simulated recordings of the
office (9 ceiling tags of 0.45 m at 3 m, upward webcam 640 x 480, map pose
noise 5 cm and 1 deg) and walks the steps in Chromium, playing a recording
wherever a person would drive. The map in use at the start is an older
survey in which marker 5 lies 30 cm from where it now hangs. The numbers
are identical on Humble and Jazzy, in CI run 38695440466 and locally:

| Step | What the page does | Result |
|---|---|---|
| survey, first 60 s of the drive | start "in the robot's map" (`align_to_track`), camera with detections, map from above, lights; a marker seen twice is "pass again" with a hint; save | 9 markers, all good |
| survey, the rest (124 s) | "Drive again", save | 9 good, 384 keyframes; the first solve kept in `history/` with its recording |
| check, the old map, one lap | fixes against the prior pose | 367 fixes; marker 5 flagged (0 % agreement, 29.9 cm), every other marker 83 to 100 % |
| check, the map just surveyed (trial), one lap | the same | 387 fixes, 382 agree (99 %), median distance 5.9 cm (the prior's own noise), nothing flagged |
| use | the change ("moved more than 2 cm: 5 (30.3 cm)"), write after a confirmation, undo, write again | the file is the survey's; the old one in `marker_map_versions/`; the localizer reloaded each time |

Each `web-browser` job takes about 9 minutes, of which rendering the two
drives is about 2 and playing them in real time about 6. The API tests
(`test_robot_api.py` and the unit tests of the pieces) run in the
`build-and-test` job: 167 tests on each distro, all passing.

### Pieces

- `marker_survey_web_node` (`node.py`, `ros_bridge.py`): rclpy, the HTTP
  server in a thread; the survey node's and localizer's status topics, the
  localizer's fix records, their services and parameters; camera frames as
  JPEG at 5 Hz, subscribed only while a page shows them (the nodes draw the
  debug image only while subscribed). `launch/web.launch.xml`.
- `hub.py`: the screens' logic without ROS (state, survey actions, report,
  map use, check), behind a five-method robot interface; `mapstore.py`: the
  map in use, versions, undo, every file checked with the loader;
  `websocket.py`: RFC 6455 on Python's HTTP server, no dependency.
- The page: `robot.js` and the new panels of `index.html`, plain
  JavaScript; tabs follow the address (`/#survey`, `/#check`, `/#use`), so a
  robot project can link to a step or embed it. Without a robot
  (`marker_survey_web`), the robot tabs stay hidden.
- Survey node: `~/status`, `~/start`, `~/pause`, `~/reset`, `start_paused`,
  `keep_history`, the frame chosen at run time; `core/live.py` the lights
  and hints (before a solve: under 3 keyframes, or seen from places less
  than 0.3 m apart, is "pass again"; after it, the report's verdict until
  the marker gets new views).
- Localizer node: `~/fixes`, `~/status`, `~/reload`, `marker_map` at run
  time, a missing map waited for; `core/check.py` the fix records and the
  per-marker agreement (a fix agrees within 15 cm and 5 deg of the prior; a
  marker left out as an outlier, or in a fix that disagrees, disagrees;
  flagged above 20 % of at least 5).
- Tests: `test_live.py`, `test_check.py` (a marker moved 30 cm is flagged,
  and only it), `test_mapstore.py`, `test_robot_api.py` (a survey recording,
  `survey_data.npz`, played through a stand-in for the two nodes: start in
  both frame modes, save, pass again with the first solve kept, use with
  and without confirmation, undo, the check flagging a moved marker, the
  WebSocket's state and camera frames only when asked); the browser test
  above. `marker_survey_synthetic` gained `--split-at` and `--start-time` for
  it.

### Deviations, and why

- **Check before use is a trial.** The check screen points the localizer
  at the survey's own file (its `marker_map` parameter); nothing is written
  until "Use the map". The phase plan put check before use without saying
  which map the localizer reads meanwhile.
- **The services are `std_srvs/Trigger` and the status is JSON in
  `std_msgs/String`.** Custom messages would need a CMake package; every
  package here is Python.
- **Status timers run on wall time.** With `use_sim_time` the ROS clock
  stands still until a recording plays, and the page would show nothing.
  The survey's processing timer is unchanged.
- **Playwright runs in CI,** installed with pip in the job (Chromium and its
  libraries, about 1 minute); the page itself needs neither.
- **No camera on the Use screen, no map editing.** Not asked for; a
  surveyed marker that is wrong is surveyed again.


## No robot: a handheld webcam

A person holds a laptop with a USB webcam pointing up at the ceiling
markers and walks the routes a robot would drive; the page runs on the same
laptop. `launch/handheld.launch.xml`: `usb_cam` (optional, `launch_camera`,
a params file), the survey with `pose_source: markers` (the camera alone)
and the localizer in 3d with no prior, both with the camera's optical frame
as their base (no TF), and the web node.

- **Plan** (`#plan`, also without ROS; `planner.py`, `POST /api/plan`):
  presets "a robot" and "a person" (camera 1.0 m, 0.5 m/s), the field of
  view typed or read from the live `camera_info` (the web node's
  `camera_info_topic`: 2 atan(w / 2 fx)); every number from
  `core/marker_plan.py`: readable size range, recommended size and sheet,
  zone R, spacing D with no odometry (2R), on a grid and with odometry, the
  count for a route length. A side view (SVG drawn in the page from these
  numbers): floor, camera, field of view, the width seen on the ceiling,
  the zone, markers of size S every D. "Print these markers" opens the
  print tab with that size and the IDs (`#print?size=40&count=23&ids=0-22`).
- **Survey**, camera alone: only "Start from a marker" is offered (the
  marker defines the frame); a banner while `tracking` is `lost`: "No mapped
  marker in view (N s): go back to marker M"; the path walked on the map.
- **Test localization** (`#localize`; `loctest.py`, `GET|POST
  /api/localize`, `GET /api/localize.csv`): the localizer's camera, the
  pose (x, y, yaw, z in 3d), the time since the last fix (the localizer's
  `last_fix_age`, else the time since the web node heard one) with a banner
  past `localize.gap_s`, the map's markers and the trajectory; a kidnap (a
  gap over `localize.gap_s` followed by a jump over `localize.jump_m`) is
  drawn dashed and listed. Test points: the measured x, y typed in, the
  latest pose stored beside it, the error; mean, RMS and largest; CSV.

Tests: `test_handheld_api.py` (the plan against `marker_plan` for 2.9 m and
the handheld preset, the camera-only survey losing track in a step aside,
the kidnap, test points, the CSV; `handheld_fixture.py` plays a simulated
walk under a row of eleven 0.40 m markers 1.2 m apart through the real
localizer); `test_launch_names.py` starts `handheld.launch.xml` without the
camera; `ci/web_handheld.sh` walks plan, survey and test in Chromium on the
same simulated walk (the step `The handheld case from the browser` of the
`web-browser` job).
