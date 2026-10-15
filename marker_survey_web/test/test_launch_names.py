"""The stock launch files name their node from the node_name argument, and the
nodes' ~/ topics follow that name. Each launch file is started with ros2 launch
and the ROS graph is read: the renamed node and its topics are there, nothing
under the default name."""
import os
import shutil
import signal
import socket
import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
ament = pytest.importorskip('ament_index_python.packages')

SITE = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'sites', 'office')


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def cases(tmp):
    return {
        'localizer': ('marker_localizer', 'localizer.launch.xml',
                      [f'marker_map:={os.path.abspath(SITE)}/marker_map.yaml'],
                      'marker_localizer', ['status', 'fixes', 'pose']),
        'survey': ('marker_survey', 'survey.launch.xml',
                   [f'output_dir:={tmp}/survey', 'start_paused:=true'],
                   'marker_survey', ['status']),
        'web': ('marker_survey_web', 'web.launch.xml',
                ['host:=127.0.0.1', f'port:={free_port()}'],
                'marker_survey_web', []),
    }


def graph(node, timeout, want):
    """The node names and topic names once `want` is satisfied, or after timeout."""
    t_end = time.monotonic() + timeout
    while True:
        names = {('/' + n) if ns == '/' else f'{ns}/{n}'
                 for n, ns in node.get_node_names_and_namespaces()}
        topics = {t for t, _ in node.get_topic_names_and_types()}
        if want(names, topics) or time.monotonic() > t_end:
            return names, topics
        time.sleep(0.2)


@pytest.mark.parametrize('which', ['localizer', 'survey', 'web'])
def test_the_launch_file_names_the_node_from_node_name(which, tmp_path, monkeypatch):
    if shutil.which('ros2') is None:
        pytest.skip('ros2 not on PATH')
    pkg, launch_file, args, default, topics = cases(str(tmp_path))[which]
    try:
        ament.get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001
        pytest.skip(f'{pkg} is not installed')
    name = f'renamed_{which}_node'
    if not os.environ.get('ROS_DOMAIN_ID'):       # away from other ROS work on the machine
        monkeypatch.setenv('ROS_DOMAIN_ID', str(60 + os.getpid() % 30))
    log = open(tmp_path / 'launch.log', 'w+')
    p = subprocess.Popen(['ros2', 'launch', pkg, launch_file, f'node_name:={name}'] + args,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('launch_names_test')
    try:
        want = {f'/{name}/{t}' for t in topics}
        names, seen = graph(node, 30.0, lambda n, t: f'/{name}' in n and want <= t)
        log.seek(0)
        out = log.read()
        assert f'/{name}' in names, (names, out[-2000:])
        assert want <= seen, (sorted(seen), out[-2000:])
        assert f'/{default}' not in names
        if which != 'web':       # the web node subscribes to the others' default topics
            assert not any(t.startswith(f'/{default}/') for t in seen), sorted(seen)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()


def test_one_launch_file_starts_the_three_nodes_under_their_own_names(tmp_path, monkeypatch):
    """examples/ceiling/launch/web.launch.xml includes the three stock files: each keeps
    its own node_name default (the includes are scoped, so one file's arguments do not
    carry over to the next)."""
    if shutil.which('ros2') is None:
        pytest.skip('ros2 not on PATH')
    try:
        for pkg in ('marker_survey', 'marker_localizer', 'marker_survey_web'):
            ament.get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001
        pytest.skip('the packages are not installed')
    if not os.environ.get('ROS_DOMAIN_ID'):
        monkeypatch.setenv('ROS_DOMAIN_ID', str(60 + os.getpid() % 30))
    launch = os.path.join(os.path.dirname(__file__), '..', '..', 'examples', 'ceiling', 'launch',
                          'web.launch.xml')
    log = open(tmp_path / 'launch.log', 'w+')
    p = subprocess.Popen(['ros2', 'launch', os.path.abspath(launch), f'site:={tmp_path}/site',
                          f'port:={free_port()}'],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('launch_names_test')
    try:
        nodes = {'/marker_survey', '/marker_localizer', '/marker_survey_web'}
        topics = {'/marker_survey/status', '/marker_localizer/status', '/marker_localizer/pose'}
        names, seen = graph(node, 30.0, lambda n, t: nodes <= n and topics <= t)
        log.seek(0)
        out = log.read()
        assert nodes <= names, (names, out[-2000:])
        assert topics <= seen, (sorted(seen), out[-2000:])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
