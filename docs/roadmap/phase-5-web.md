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

- [ ] Node, static files, JSON API, WebSocket
- [ ] Plan and print (PDF with reportlab or cairo; markers drawn by OpenCV)
- [ ] Survey screens on top of the phase 2 node
- [ ] Check screen on top of the phase 3 localizer
- [ ] Map use, versions, undo, localizer reload
- [ ] Browser test; usability session

## Done when

The usability session finishes a simulated site with no help on the
software, only on the hardware.
