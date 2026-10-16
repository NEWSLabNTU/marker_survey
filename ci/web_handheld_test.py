#!/usr/bin/env python3
"""Walk the web interface's handheld case in Chromium (Playwright): plan, survey, test.

    python3 ci/web_handheld_test.py http://127.0.0.1:8090 OUT [--drive DIR]

Started by ci/web_handheld.sh. No robot: a person with a laptop and an upward
webcam at 1.0 m walks under ceiling markers (2.9 m, tag36h11 of 0.40 m).

  plan      handheld preset, ceiling 2.9 m, 70 deg, 640 x 480, a 30 m route: the
            numbers are marker_plan's (40 cm, R, D = 2R, the count), the side view
            draws the markers; "Print these markers" opens the print tab with
            that size and the IDs
  survey    start from marker 0 (pose_source markers: no robot map offered); the
            walk steps aside where no marker is in view: the banner "no mapped
            marker in view: go back to marker N" appears, then goes; the map
            grows to every marker; save
  test      the localizer on the survey just made: the pose freezes with no
            marker in view ("no marker in view for N s"), then jumps: a kidnap,
            dashed on the map; two test points measured with a tape, their
            errors, the CSV

The walks: with --drive DIR, recordings in DIR (survey_walk, test_walk) are
played with ros2 bag play when the person would walk; without it the server
is the stand-in of marker_survey_web/test/handheld_fixture.py, which walks by
itself when the survey starts and when the localizer is given a map.

Screenshots of each screen go to OUT/shots. Exits non-zero on the first step
that fails.
"""
import argparse
import json
import os
import subprocess
import time
import urllib.request

from playwright.sync_api import expect, sync_playwright

from marker_survey.core import marker_plan as mp

a = argparse.ArgumentParser()
a.add_argument('url')
a.add_argument('out')
a.add_argument('--drive', default='', help='directory with the recorded walks; empty: the stand-in walks')
ARGS = a.parse_args()
URL, OUT = ARGS.url.rstrip('/'), ARGS.out
SHOTS = os.path.join(OUT, 'shots')
LONG = 600_000


def state():
    with urllib.request.urlopen(URL + '/api/state', timeout=10) as r:
        return json.loads(r.read())


def wait_for(cond, what, timeout=600.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if cond():
                return
        except (OSError, KeyError, TypeError, ValueError):
            pass
        time.sleep(0.3)
    raise SystemExit(f'timed out waiting for {what}')


def play(name):
    """Walk: play a recording (--drive), or nothing (the stand-in walks by itself)."""
    if not ARGS.drive:
        return None
    bag = os.path.join(ARGS.drive, name)
    log = open(os.path.join(OUT, f'{name}_play.log'), 'w')
    print(f'-- walking: ros2 bag play {bag}', flush=True)
    return subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock', '--delay', '2'], stdout=log,
                            stderr=subprocess.STDOUT)


def walked(proc, key, what):
    """The walk is over and the node has caught up (a count stops growing for 4 s)."""
    if proc is not None:
        proc.wait()
    last, since = None, time.time()

    def done():
        nonlocal last, since
        v = key(state())
        if v != last:
            last, since = v, time.time()
        return time.time() - since > 4.0
    wait_for(done, what)


def shot(page, name, locator=None):
    path = os.path.join(SHOTS, name)
    (locator or page).screenshot(path=path, **({} if locator else {'full_page': True}))
    print(f'-- screenshot {path}', flush=True)


def main():
    os.makedirs(SHOTS, exist_ok=True)
    errors = []
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        ctx = browser.new_context(viewport={'width': 1280, 'height': 900}, accept_downloads=True)
        page = ctx.new_page()
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.set_default_timeout(30_000)
        try:
            steps(page)
        except BaseException:
            page.screenshot(path=os.path.join(SHOTS, 'failed.png'), full_page=True)
            print(f'-- failed; the page is in {SHOTS}/failed.png; errors on the page: {errors}', flush=True)
            try:
                print(json.dumps(state())[:3000], flush=True)
            except OSError:
                pass
            raise
        browser.close()
    if errors:
        raise SystemExit('errors on the page: ' + '; '.join(errors))
    print('web interface, handheld: every step passed', flush=True)


def steps(page):
    # ------------------------------------------------------------ plan
    print('== plan', flush=True)
    page.goto(URL + '/#plan')
    expect(page.locator('#plan')).to_be_visible()
    page.check('input[name="pl_preset"][value="handheld"]')
    expect(page.locator('#pl_camera')).to_have_value('1')
    page.fill('#pl_ceiling', '2.9')
    page.fill('#pl_hfov', '70')
    page.fill('#pl_w', '640')
    page.fill('#pl_h', '480')
    page.fill('#pl_length', '30')
    p = mp.plan(2.9, 1.0, 70, 640, 480, 0.5, 0.02, 6.0, 5.0, 0.05, 0.05)
    r = mp.recommend(p)
    d = r['route_spacing_no_odom']
    count = mp.route_markers(30, d)
    size = round(r['size'] * 100, 1)
    nums = page.locator('#pl_numbers')
    expect(nums).to_contain_text(f'Markers for the route{count} for 30 m')
    expect(nums).to_contain_text(f'Recommended marker size{size:g} cm black square')
    expect(nums).to_contain_text(f'Zone R{r["zone_radius"]:.2f} m')
    expect(nums).to_contain_text(f'Spacing D, no odometry{d:.2f} m along a route (2R)')
    expect(nums).to_contain_text(f'Readable marker sizes{round(p["s_min"] * 100, 1):g} to '
                                 f'{round(p["s_max"] * 100, 1):g} cm')
    assert size == 40.0, size
    svg = page.locator('#pl_figure svg')
    expect(svg).to_be_visible()
    assert page.locator('#pl_figure rect.marker').count() >= 3
    expect(page.locator('#pl_figure')).to_contain_text(f'D {d:.2f} m')
    expect(page.locator('#pl_print')).to_contain_text(
        f'Print {count} markers of 40 cm (IDs 0 to {count - 1})')
    print(f'-- plan: {size} cm, R {r["zone_radius"]:.2f} m, D {d:.2f} m, {count} markers for 30 m',
          flush=True)
    shot(page, 'plan.png')
    page.click('#pl_print')
    expect(page.locator('#print')).to_be_visible()
    expect(page.locator('#size')).to_have_value('40')
    expect(page.locator('#ids')).to_have_value(f'0-{count - 1}')
    expect(page.locator('#checks')).to_contain_text(f'{count} markers, tag36h11, black square 40')
    shot(page, 'print_from_plan.png')

    # ------------------------------------------------------------ survey
    print('== survey, the camera alone', flush=True)
    page.goto(URL + '/#survey')
    expect(page.locator('#sv_start')).to_be_visible()
    expect(page.locator('input[name="sv_frame"][value="robot_map"]')).to_be_disabled()
    expect(page.locator('#sv_frames')).to_contain_text('the camera alone gives no map')
    page.fill('#sv_origin', '0')
    page.click('#sv_start_btn')
    expect(page.locator('#sv_state')).to_contain_text('Recording')
    walk = play('survey_walk')
    expect(page.locator('#sv_state')).to_contain_text('walk slowly', timeout=120_000)
    page.wait_for_function("document.querySelectorAll('#sv_markers tr').length >= 3", timeout=LONG)
    banner = page.locator('#sv_lost')
    expect(banner).to_be_visible(timeout=LONG)
    expect(banner).to_contain_text('No mapped marker in view')
    expect(banner).to_contain_text('go back to marker')
    page.wait_for_function("document.getElementById('sv_cam').getAttribute('src')", timeout=60_000)
    print(f'-- {banner.inner_text()}', flush=True)
    shot(page, 'survey_lost.png')
    expect(banner).to_be_hidden(timeout=LONG)
    walked(walk, lambda s: (s['survey']['frames'], s['survey']['keyframes']), 'the walk to end')
    n = len(state()['survey']['markers'])
    print(f'-- survey: {n} markers', flush=True)
    shot(page, 'survey_map.png')
    page.click('#sv_save')
    expect(page.locator('#sv_report')).to_be_visible(timeout=LONG)
    expect(page.locator('#rp_summary')).to_contain_text(f'{n} markers')
    assert n >= 10, n

    # ------------------------------------------------------------ the localization test
    print('== localization test', flush=True)
    page.click('.tab[data-tab="localize"]')
    expect(page.locator('#localize')).to_be_visible()
    page.check('input[name="lz_which"][value="survey"]')
    wait_for(lambda: state()['map']['localizer_reads'] == 'survey', 'the localizer to read the survey')
    walk = play('test_walk')
    page.wait_for_function("document.getElementById('lz_x').textContent !== '-'", timeout=LONG)
    nofix = page.locator('#lz_nofix')
    expect(nofix).to_be_visible(timeout=LONG)
    expect(nofix).to_contain_text('No marker in view for')
    expect(page.locator('#lz_z_box')).to_be_visible()
    print(f'-- {nofix.inner_text()}', flush=True)
    shot(page, 'localize_no_fix.png')
    expect(page.locator('#lz_kidnaps')).not_to_have_text('0', timeout=LONG)
    expect(page.locator('#lz_events')).to_contain_text('kidnap: no fix for')
    walked(walk, lambda s: s['localize']['images'], 'the walk to end')
    lz = state()['localize']
    assert len(lz['kidnaps']) >= 1 and any(t[2] == 1 for t in lz['track']), lz['kidnaps']
    for k in lz['kidnaps']:
        print(f'-- kidnap: {k["gap_s"]} s without a fix, then {k["jump_m"]} m', flush=True)
    assert max(k['jump_m'] for k in lz['kidnaps']) > 1.0, lz['kidnaps']

    pose = lz['pose']
    page.fill('#lz_px', f'{pose["x"] + 0.03:.3f}')
    page.fill('#lz_py', f'{pose["y"] - 0.04:.3f}')
    page.click('#lz_record')
    expect(page.locator('#lz_points tr')).to_have_count(1)
    expect(page.locator('#lz_point_msg')).to_contain_text('Point 1: error 5.0 cm')
    page.fill('#lz_px', f'{pose["x"] + 0.30:.3f}')
    page.fill('#lz_py', f'{pose["y"]:.3f}')
    page.click('#lz_record')
    expect(page.locator('#lz_points tr')).to_have_count(2)
    expect(page.locator('#lz_errors')).to_contain_text('2 test points: error mean 17.5 cm')
    with page.expect_download() as dl:
        page.click('#lz_csv')
    path = os.path.join(OUT, 'localization_test.csv')
    dl.value.save_as(path)
    rows = open(path).read().splitlines()
    assert rows[0].startswith('n,x_measured,y_measured,x,y,z,yaw_deg,error_m') and len(rows) == 3, rows
    print(f'-- test points: {page.locator("#lz_errors").inner_text()}; CSV {len(rows) - 1} rows', flush=True)
    shot(page, 'localize.png')


if __name__ == '__main__':
    main()
