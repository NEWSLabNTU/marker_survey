#!/usr/bin/env python3
"""Walk the web interface's survey, check and map steps in Chromium (Playwright).

    python3 ci/web_browser_test.py http://127.0.0.1:8090 OUT

Started by ci/web_browser.sh, which renders the drives into OUT and starts
the nodes; this script clicks through the page as a person would and plays a
recording wherever that person would drive:

  survey   start in the robot's map; drive (survey_drive/bag_1); save; the
           report; "drive again"; drive on (bag_2); save: the first solve is
           kept, every marker is in the result
  check    the map in use (an older survey in which marker 5 has since moved
           by 30 cm) on check_drive/bag_1: marker 5 is flagged; the map just
           surveyed, as a trial, on bag_2: nothing is flagged
  use      the change shows marker 5 moved; write with a confirmation; undo;
           write again: the file is the survey's, the old one kept

Screenshots of each screen go to OUT/shots. Exits non-zero on the first step
that fails.
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.request

from playwright.sync_api import expect, sync_playwright

URL, OUT = sys.argv[1].rstrip('/'), sys.argv[2]
SHOTS = os.path.join(OUT, 'shots')
LONG = 900_000                     # [ms] a solve, a drive


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
        time.sleep(0.5)
    raise SystemExit(f'timed out waiting for {what}')


def play(bag):
    log = open(os.path.join(OUT, os.path.basename(os.path.dirname(bag)) + '_' + os.path.basename(bag)
                            + '_play.log'), 'w')
    print(f'-- driving: ros2 bag play {bag}', flush=True)
    return subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock', '--delay', '2'], stdout=log,
                            stderr=subprocess.STDOUT)


def settled(key, what):
    """Wait until a count stops growing for 4 s (the nodes have caught up with a drive)."""
    last, since = None, time.time()

    def done():
        nonlocal last, since
        v = key(state())
        if v != last:
            last, since = v, time.time()
        return time.time() - since > 4.0
    wait_for(done, what)
    return last


def shot(page, name):
    path = os.path.join(SHOTS, name)
    page.screenshot(path=path, full_page=True)
    print(f'-- screenshot {path}', flush=True)


def main():
    os.makedirs(SHOTS, exist_ok=True)
    errors = []
    with sync_playwright() as pw:
        browser = pw.chromium.launch()
        page = browser.new_page(viewport={'width': 1280, 'height': 900})
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
    print('web interface: every step passed', flush=True)


def steps(page):
    """The steps, as a person would take them."""

    # ------------------------------------------------------------ survey
    print('== survey', flush=True)
    page.goto(URL + '/#survey')
    expect(page.locator('.tab[data-tab="survey"]')).to_be_visible()
    expect(page.locator('#sv_start')).to_be_visible()
    robot_map = page.locator('input[name="sv_frame"][value="robot_map"]')
    expect(robot_map).to_be_checked()
    expect(page.locator('#sv_frames')).to_contain_text("Use the robot's map (map)")
    page.click('#sv_start_btn')
    expect(page.locator('#sv_state')).to_contain_text('Recording')
    drive = play(os.path.join(OUT, 'survey_drive', 'bag_1'))
    page.wait_for_function("document.getElementById('sv_cam').getAttribute('src')", timeout=120_000)
    page.wait_for_function("document.querySelectorAll('#sv_markers tr').length >= 3", timeout=120_000)
    shot(page, '1_survey_running.png')
    drive.wait()
    settled(lambda s: (s['survey']['frames'], s['survey']['queued']), 'the survey to catch up')
    page.click('#sv_save')
    expect(page.locator('#sv_report')).to_be_visible(timeout=LONG)
    expect(page.locator('#rp_summary')).to_contain_text('markers')
    first = page.locator('#rp_markers tr').count()
    print(f'-- first solve: {page.locator("#rp_summary").inner_text()}', flush=True)
    expect(page.locator('#rp_history')).to_contain_text('first solve')

    page.click('#rp_again')
    expect(page.locator('#sv_state')).to_contain_text('Recording')
    drive = play(os.path.join(OUT, 'survey_drive', 'bag_2'))
    drive.wait()
    settled(lambda s: (s['survey']['frames'], s['survey']['queued']), 'the survey to catch up')
    page.click('#sv_save')
    expect(page.locator('#sv_report')).to_be_visible(timeout=LONG)
    expect(page.locator('#rp_summary')).to_contain_text('9 markers')
    expect(page.locator('#rp_history li')).to_have_count(1)
    expect(page.locator('#rp_history')).to_contain_text('survey_data.npz')
    print(f'-- second solve: {page.locator("#rp_summary").inner_text()} (first: {first} markers)',
          flush=True)
    assert first <= 9
    shot(page, '2_survey_report.png')

    # ------------------------------------------------------------ check
    print('== check', flush=True)
    page.click('.tab[data-tab="check"]')
    expect(page.locator('input[name="ck_which"][value="site"]')).to_be_checked()
    page.click('#ck_reset')
    wait_for(lambda: state()['check']['fixes'] == 0, 'the counts to start afresh', 30)
    drive = play(os.path.join(OUT, 'check_drive', 'bag_1'))
    page.wait_for_function("document.getElementById('ck_cam').getAttribute('src')", timeout=120_000)
    drive.wait()
    settled(lambda s: s['check']['images'], 'the localizer to catch up')
    expect(page.locator('#ck_flags')).to_contain_text('Marker 5 disagrees')
    s = state()['check']
    assert s['flagged'] == [5], s['flagged']
    print(f'-- old map: {s["fixes"]} fixes, flagged {s["flagged"]}', flush=True)
    shot(page, '3_check_old_map.png')

    page.check('input[name="ck_which"][value="survey"]')
    wait_for(lambda: state()['map']['localizer_reads'] == 'survey', 'the localizer to read the survey')
    wait_for(lambda: state()['check']['fixes'] == 0, 'the counts to start afresh', 30)
    drive = play(os.path.join(OUT, 'check_drive', 'bag_2'))
    drive.wait()
    settled(lambda s: s['check']['images'], 'the localizer to catch up')
    s = state()['check']
    assert s['fixes'] > 100 and s['flagged'] == [], (s['fixes'], s['flagged'])
    assert s['agreed'] / s['with_prior'] > 0.9, s['agreed'] / s['with_prior']
    expect(page.locator('#ck_flags')).to_be_empty()
    print(f'-- surveyed map: {s["fixes"]} fixes, {s["agreed"]} of {s["with_prior"]} agree, median '
          f'{s["distance_median_m"] * 100:.1f} cm', flush=True)
    shot(page, '4_check_new_map.png')

    # ------------------------------------------------------------ use
    print('== use', flush=True)
    page.click('.tab[data-tab="use"]')
    expect(page.locator('#us_new')).to_contain_text(re.compile(r'moved more than 2 cm: 5 \('))
    expect(page.locator('#us_versions')).to_contain_text('None yet')
    page.click('#us_write')
    expect(page.locator('#us_confirm')).to_be_visible()
    expect(page.locator('#us_confirm_text')).to_contain_text('kept as an earlier version')
    shot(page, '5_use_confirm.png')
    page.click('#us_yes')
    expect(page.locator('#us_result')).to_contain_text('Written')
    expect(page.locator('#us_versions li')).to_have_count(1)
    st = state()
    site_map, survey_map = st['map']['path'], st['map']['survey_map']
    wait_for(lambda: state()['map']['localizer_reads'] == 'site', 'the localizer to read the map in use')
    assert open(site_map).read() == open(survey_map).read()
    old = open(os.path.join(os.path.dirname(site_map), 'marker_map_versions',
                            os.listdir(os.path.join(os.path.dirname(site_map),
                                                    'marker_map_versions'))[0])).read()
    assert 'id: 5, x:  0.300' in old

    page.click('#us_undo')
    page.click('#us_undo_yes')
    expect(page.locator('#us_result')).to_contain_text('Put back')
    assert open(site_map).read() == old
    page.click('#us_write')
    page.click('#us_yes')
    expect(page.locator('#us_result')).to_contain_text('Written')
    assert open(site_map).read() == open(survey_map).read()
    shot(page, '6_use_written.png')


if __name__ == '__main__':
    main()
