"""The nodes exit promptly on SIGTERM and SIGINT, also with sim time and no clock (as
after a recording), when no timer of theirs fires any more."""
import os
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip('rclpy')

NODE = [sys.executable, '-m', 'marker_survey.survey_node', '--ros-args',
        '-p', 'use_sim_time:=true', '-p', 'save_on_exit:=false', '-p', 'output_dir:=/tmp/unused',
        '-p', 'frame.origin_marker:=10', '-r', '__node:=marker_survey_signal_test']


def run_and_signal(sig, tmp_path):
    log = open(tmp_path / f'node_{sig}.log', 'w+')
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '91'))
    p = subprocess.Popen(NODE, stdout=log, stderr=subprocess.STDOUT, env=env)
    try:
        for _ in range(150):                         # up to 15 s to start
            log.seek(0)
            if 'surveying' in log.read():
                break
            assert p.poll() is None, 'the node stopped by itself'
            time.sleep(0.1)
        time.sleep(1.0)
        p.send_signal(sig)
        t0 = time.monotonic()
        rc = p.wait(timeout=10)
        return rc, time.monotonic() - t0
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()


@pytest.mark.parametrize('sig', [signal.SIGTERM, signal.SIGINT])
def test_the_survey_node_exits_on_a_signal(sig, tmp_path):
    for _ in range(3):
        rc, took = run_and_signal(sig, tmp_path)
        assert rc == 0 and took < 3.0, (rc, took)
