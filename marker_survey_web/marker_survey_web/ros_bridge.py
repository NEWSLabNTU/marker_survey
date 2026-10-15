"""The web interface's link to the survey and localizer nodes (rclpy).

Subscribes to the survey node's ~/status, the localizer's ~/status and
~/fixes, the camera's camera_info (if camera_info_topic is set: the plan screen
reads the field of view from it), and, only while a page shows a camera, to their ~/debug/image, sent
on as JPEG at camera_rate. Calls their services (std_srvs/Trigger) and sets
their parameters for the page's actions. Everything else is in hub.py.
"""

import json
import threading
import time

from rcl_interfaces.srv import SetParameters
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


def image_to_jpeg(msg, width: int, quality: int = 75):
    import cv2
    import numpy as np
    enc = msg.encoding.lower()
    ch = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(enc)
    if ch is None:
        return None
    buf = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 else buf[:, :msg.width]
    if enc == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if width and img.shape[1] > width:
        img = cv2.resize(img, None, fx=width / img.shape[1], fy=width / img.shape[1],
                         interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return jpg.tobytes() if ok else None


class RosRobot:
    def __init__(self, node, hub, survey_node='/marker_survey', localizer_node='/marker_localizer',
                 camera_rate=5.0, camera_width=640, camera_info_topic=''):
        self.node, self.hub = node, hub
        self.survey_ns, self.loc_ns = survey_node.rstrip('/'), localizer_node.rstrip('/')
        self.period = 1.0 / max(camera_rate, 0.1)
        self.width = int(camera_width)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, f'{self.survey_ns}/status', self.on_survey, reliable)
        node.create_subscription(String, f'{self.loc_ns}/status', self.on_loc, reliable)
        node.create_subscription(String, f'{self.loc_ns}/fixes', self.on_fix,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.triggers = {name: node.create_client(Trigger, f'{self.survey_ns}/{name}')
                         for name in ('start', 'pause', 'reset', 'save')}
        self.reload = node.create_client(Trigger, f'{self.loc_ns}/reload')
        self.set_survey = node.create_client(SetParameters, f'{self.survey_ns}/set_parameters')
        self.set_loc = node.create_client(SetParameters, f'{self.loc_ns}/set_parameters')
        if camera_info_topic:
            self.info_topic = camera_info_topic
            node.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info,
                                     qos_profile_sensor_data)
        self.cameras = {}                  # which -> (subscription, last sent)
        self.topics = {'survey': f'{self.survey_ns}/debug/image', 'check': f'{self.loc_ns}/debug/image'}
        node.create_timer(0.5, self.manage_cameras, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ------------------------------------------------------------ in
    def on_survey(self, msg):
        try:
            self.hub.on_survey_status(json.loads(msg.data))
        except ValueError:
            pass

    def on_loc(self, msg):
        try:
            self.hub.on_localizer_status(json.loads(msg.data))
        except ValueError:
            pass

    def on_fix(self, msg):
        try:
            self.hub.on_fix(json.loads(msg.data))
        except ValueError:
            pass

    def on_camera_info(self, msg):
        self.hub.on_camera_info(msg.width, msg.height, msg.k[0], self.info_topic)

    def manage_cameras(self):
        """Subscribe to a debug image only while a page shows it: the nodes draw it only
        while someone subscribes."""
        wanted = self.hub.camera_wanted()
        for which, topic in self.topics.items():
            if which in wanted and which not in self.cameras:
                sub = self.node.create_subscription(
                    Image, topic, lambda m, w=which: self.on_image(w, m), qos_profile_sensor_data)
                self.cameras[which] = [sub, 0.0]
            elif which not in wanted and which in self.cameras:
                self.node.destroy_subscription(self.cameras.pop(which)[0])

    def on_image(self, which, msg):
        entry = self.cameras.get(which)
        now = time.monotonic()
        if entry is None or now - entry[1] < self.period:
            return
        entry[1] = now
        jpg = image_to_jpeg(msg, self.width)
        if jpg:
            self.hub.on_camera(which, jpg)

    # ------------------------------------------------------------ out
    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f'{client.srv_name} is not available'
        done = threading.Event()
        fut = client.call_async(request)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None, f'{client.srv_name} did not answer within {timeout:.0f} s'
        return fut.result(), ''

    def _trigger(self, client, timeout):
        res, err = self._call(client, Trigger.Request(), timeout)
        if res is None:
            return False, err
        return bool(res.success), res.message

    def _set(self, client, params: dict):
        req = SetParameters.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]
        res, err = self._call(client, req, 10.0)
        if res is None:
            return False, err
        bad = [r.reason for r in res.results if not r.successful]
        return (False, '; '.join(bad) or 'refused') if bad else (True, 'set')

    def connected(self) -> dict:
        return {'survey': self.triggers['save'].service_is_ready(),
                'localizer': self.reload.service_is_ready()}

    def survey_call(self, name: str, timeout: float):
        return self._trigger(self.triggers[name], timeout)

    def survey_set(self, params: dict):
        return self._set(self.set_survey, params)

    def localizer_set_map(self, path: str):
        ok, msg = self._set(self.set_loc, {'marker_map': str(path)})
        return ok, (f'the localizer reads {path}' if ok else msg)

    def localizer_reload(self):
        return self._trigger(self.reload, 30.0)
