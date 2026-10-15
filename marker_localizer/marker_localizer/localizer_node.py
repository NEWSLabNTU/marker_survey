"""marker_localizer: the robot's pose from the markers in view, against marker_map.yaml.

  in   image (raw or compressed) and its camera_info (rectified, or with distortion
       coefficients: the corners are then undistorted)
       TF base_frame -> the image's frame: the camera's mounting
       mode planar: the robot's roll and pitch (roll_pitch: zero, tf or imu)
       prior_topic (optional): the state estimator's pose, for the gate and the
       single-marker ambiguity (geometry_msgs/PoseWithCovarianceStamped, or
       nav_msgs/Odometry with prior_type odometry)
  out  pose_topic      geometry_msgs/PoseWithCovarianceStamped of base_frame in the
                       map's frame_id, stamped with the image's capture time
       ~/debug/image   the detections and the fit's residuals (only while something
                       subscribes)
       ~/fixes         std_msgs/String, JSON per image with markers: the fix, the prior
                       at its stamp, the distance, each marker's use and check
                       (core/check.py);
                       only while something subscribes
       ~/markers       diagnostic_msgs/DiagnosticArray per image with markers: per
                       marker its state, reprojection residual with the fix, and its
                       pose as the image sees it against the map's (core/check.py
                       marker_views, diagnostics.py); only while something subscribes
       ~/status        std_msgs/String, JSON every second: the map in use, its
                       markers, the counts; only while something subscribes
  services
       ~/reload        std_srvs/Trigger: read the map file again

The map is read from `marker_map`, which can be set at run time (the new file
is read, and refused if it does not load). A missing file at start is
waited for: the node localizes once ~/reload or a new marker_map finds one.

Modes (core/localize.py): `3d`, a full 6-DoF pose (wall boards, a vehicle whose
roll and pitch matter); `planar`, x, y and yaw on a flat floor (ceiling markers,
or wall boards for a robot on a flat floor). `enabled` can be set at run time:
`ros2 param set /marker_localizer enabled false`.
"""

import collections
import json
import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from marker_survey.core import marker_map as mm
from marker_survey.core.detection import Detector
from marker_survey.core.se3 import interpolate_T, make_T, matrix_to_quat, quat_to_matrix
from marker_survey.survey_node import decode, stamp_sec, tf_to_T

from marker_localizer.core.check import fix_record, marker_views
from marker_localizer.core.localize import Localizer, roll_pitch_of
from marker_localizer.diagnostics import marker_diagnostics
from marker_localizer.params import PARAMETERS, LocalizerParams


def pose_msg_to_T(p) -> np.ndarray:
    q = p.orientation
    return make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.position.x, p.position.y, p.position.z])


class MarkerLocalizerNode(Node):
    def __init__(self):
        super().__init__('marker_localizer')
        for k, v in PARAMETERS.items():
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.p = p = LocalizerParams(gp)
        self.map = self.detector = None
        self.map_path = p.marker_map
        self.map_loaded = None
        self.localizer = None
        if os.path.exists(p.marker_map):
            self.load_map(p.marker_map)
        else:
            self.get_logger().warning(f'no map at {p.marker_map} yet: no fixes until ~/reload '
                                      'or a new marker_map finds one')
        self.enabled = p.enabled
        self.undistort = None
        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.priors = collections.deque(maxlen=400)
        self.imu = None
        self.T_base_imu = None
        self.n = collections.Counter()
        self.t_detect, self.t_fit = [], []

        qos = (QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE) if p.process_all
               else qos_profile_sensor_data)
        img_type = CompressedImage if p.compressed else Image
        topic = p.image_topic + ('/compressed' if p.compressed else '')
        self.create_subscription(img_type, topic, self.on_image, qos)
        self.create_subscription(CameraInfo, p.camera_info_topic, self.on_info, qos)
        if p.prior_topic:
            prior_qos = QoSProfile(depth=50, reliability=(
                ReliabilityPolicy.RELIABLE if p.process_all else ReliabilityPolicy.BEST_EFFORT))
            typ = Odometry if p.prior_type == 'odometry' else PoseWithCovarianceStamped
            self.create_subscription(typ, p.prior_topic, self.on_prior, prior_qos)
        if p.mode == 'planar' and p.roll_pitch == 'imu':
            self.create_subscription(Imu, p.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, p.pose_topic, 10)
        self.pub_dbg = self.create_publisher(Image, '~/debug/image', 1)
        self.pub_fixes = self.create_publisher(String, '~/fixes', 50)
        self.pub_status = self.create_publisher(String, '~/status', 1)
        self.pub_markers = self.create_publisher(DiagnosticArray, '~/markers', 10)
        self.create_service(Trigger, '~/reload', self.on_reload)
        # wall time: with use_sim_time the status must flow before a recording plays
        self.create_timer(1.0, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.on_set)
        self.pending = collections.deque(maxlen=300)
        if p.process_all:
            # a recording: a frame waits until the prior pose at its stamp has arrived
            self.create_timer(0.02, self.drain)
        self.create_timer(10.0, self.on_status)
        self.get_logger().info(
            (f'{len(self.map.markers)} {self.map.dictionary} markers from {p.marker_map} '
             f'(frame {self.map.frame_id})' if self.map is not None else 'no map yet')
            + f'; mode {p.mode}; '
            + (f'gate against {p.prior_topic}: {p.settings.gate_distance} m, '
               f'{p.settings.gate_angle_deg} deg' if p.prior_topic else 'no prior, no gate'))

    # ------------------------------------------------------------ the map
    def load_map(self, path: str):
        """Read a map; raises (OSError, MarkerMapError) and keeps the old one if it fails."""
        new = mm.load(path)
        p = self.p
        if self.detector is None or self.detector.name != new.dictionary:
            self.detector = Detector(new.dictionary, p.min_marker_px, p.border_margin,
                                     detect_scale=p.detect_scale, quad_decimate=p.quad_decimate,
                                     min_contrast=p.min_contrast)
        self.map, self.map_path = new, path
        self.map_loaded = time.time()
        self.localizer = None                       # rebuilt with the new map on the next image
        return new

    def on_reload(self, request, response):
        try:
            m = self.load_map(self.map_path)
        except (OSError, ValueError) as e:
            response.success, response.message = False, f'{self.map_path}: {e}'
            self.get_logger().error(f'reload: {response.message}; the previous map stays in use')
            return response
        response.success = True
        response.message = f'{len(m.markers)} markers from {self.map_path}'
        self.get_logger().info(f'reloaded: {response.message}')
        return response

    # ------------------------------------------------------------ parameters
    def on_set(self, params):
        for prm in params:
            if prm.name == 'enabled':
                self.enabled = bool(prm.value)
                self.get_logger().info('fixes ' + ('on' if self.enabled else 'OFF'))
            elif prm.name == 'marker_map':
                try:
                    m = self.load_map(str(prm.value))
                except (OSError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=f'{prm.value}: {e}')
                self.get_logger().info(f'map: {len(m.markers)} markers from {prm.value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ inputs
    def on_info(self, msg):
        if self.K is not None:
            return
        if any(abs(v) > 1e-9 for v in msg.d):
            K = np.array(msg.k, float).reshape(3, 3)
            self.undistort = (K, np.array(msg.d, float))
        else:
            K = (np.array(msg.p, float).reshape(3, 4)[:, :3] if any(msg.p)
                 else np.array(msg.k, float).reshape(3, 3))
        self.K = K
        self.get_logger().info(f'camera: {msg.width}x{msg.height}, fx {K[0, 0]:.1f} px'
                               + (', corners undistorted' if self.undistort else ''))

    def on_prior(self, msg):
        pose = msg.pose.pose
        t = stamp_sec(msg.header.stamp)
        self.n['priors'] += 1
        if self.priors and t < self.priors[-1][0]:      # late: keep the buffer in order
            items = sorted(list(self.priors) + [(t, pose_msg_to_T(pose))], key=lambda x: x[0])
            self.priors.clear()
            self.priors.extend(items)
        else:
            self.priors.append((t, pose_msg_to_T(pose)))
        if self.pending:
            self.drain()

    def drain(self):
        """process_all: the queued frames in order, each once the prior covers its stamp
        (or 3 s of newer frames have arrived without it)."""
        while self.pending:
            msg = self.pending[0]
            t = stamp_sec(msg.header.stamp)
            if self.p.prior_topic and (not self.priors or self.priors[-1][0] < t - 1e-6):
                if stamp_sec(self.pending[-1].header.stamp) - t < 3.0:
                    return
            self.pending.popleft()
            self.process(msg)

    def on_imu(self, msg):
        q = msg.orientation
        self.imu = (msg.header.frame_id, quat_to_matrix(q.x, q.y, q.z, q.w))

    def lookup(self, parent, child, t):
        try:
            return tf_to_T(self.tf_buffer.lookup_transform(parent, child, Time(seconds=t)))
        except tf2_ros.TransformException:
            return None

    def prior_at(self, t):
        """The prior pose at stamp t: interpolated, or the nearest within prior_tolerance."""
        if not self.priors:
            return None
        pr = list(self.priors)
        ts = np.array([x[0] for x in pr])
        i = int(np.searchsorted(ts, t))
        tol = self.p.prior_tolerance
        if 0 < i < len(pr) and ts[i] - ts[i - 1] <= 2 * tol:
            f = (t - ts[i - 1]) / max(ts[i] - ts[i - 1], 1e-9)
            return interpolate_T(pr[i - 1][1], pr[i][1], f)
        j = min(max(i, 0), len(pr) - 1)
        if i > 0 and (i >= len(pr) or abs(ts[i - 1] - t) < abs(ts[j] - t)):
            j = i - 1
        return pr[j][1] if abs(ts[j] - t) <= tol else None

    def roll_pitch(self, t):
        if self.p.mode != 'planar' or self.p.roll_pitch == 'zero':
            return 0.0, 0.0
        if self.p.roll_pitch == 'tf':
            T = self.lookup(self.p.roll_pitch_frame, self.p.base_frame, t)
            return roll_pitch_of(T[:3, :3]) if T is not None else None
        if self.imu is None:
            return None
        frame, R_w_imu = self.imu
        if self.T_base_imu is None:
            self.T_base_imu = self.lookup(self.p.base_frame, frame or self.p.base_frame, 0.0) \
                if frame and frame != self.p.base_frame else np.eye(4)
            if self.T_base_imu is None:
                return None
        return roll_pitch_of(R_w_imu @ self.T_base_imu[:3, :3].T)

    # ------------------------------------------------------------ work
    def on_image(self, msg):
        self.n['images'] += 1
        if self.p.process_all:
            self.pending.append(msg)
            self.drain()
        else:
            self.process(msg)

    def process(self, msg):
        if not self.enabled or self.K is None or self.map is None:
            return
        if self.localizer is None:
            T = self.lookup(self.p.base_frame, msg.header.frame_id, 0.0)
            if T is None:
                self.get_logger().info(f'waiting for TF {self.p.base_frame} -> {msg.header.frame_id}',
                                       throttle_duration_sec=5.0)
                return
            self.localizer = Localizer(self.map, self.K, T, self.p.settings)
        t = stamp_sec(msg.header.stamp)
        try:
            t0 = time.perf_counter()
            gray = decode(msg, self.p.compressed)
            dets = self.detector.detect_all(gray)
            if self.undistort is not None:
                import cv2
                K, D = self.undistort
                for d in dets:
                    d.corners = cv2.undistortPoints(d.corners.reshape(-1, 1, 2), K, D,
                                                    P=K).reshape(4, 2)
            t1 = time.perf_counter()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'image: {e}', throttle_duration_sec=5.0)
            return
        rp = self.roll_pitch(t)
        if rp is None:
            self.get_logger().info(f'no roll and pitch ({self.p.roll_pitch}) for this frame',
                                   throttle_duration_sec=5.0)
            return
        T_prior = self.prior_at(t) if self.p.prior_topic else None
        if self.p.prior_topic and T_prior is None:
            self.n['no prior'] += 1
            if self.p.require_prior:
                self.get_logger().info(f'no prior pose from {self.p.prior_topic} near this frame: '
                                       'fix not published (require_prior)', throttle_duration_sec=5.0)
        res = self.localizer.locate(dets, T_prior=T_prior, roll=rp[0], pitch=rp[1])
        t2 = time.perf_counter()
        self.t_detect.append(t1 - t0)
        self.t_fit.append(t2 - t1)
        publish = self.localizer.accepted(res) and not (
            self.p.prior_topic and T_prior is None and self.p.require_prior)
        if publish:
            self.publish(msg.header.stamp, res.fix)
            self.n['fixes'] += 1
        elif res.fix is not None:
            self.n['rejected'] += 1
            self.get_logger().info(f'fix from {res.fix.ids} not published: {res.reason}',
                                   throttle_duration_sec=2.0)
        if self.pub_dbg.get_subscription_count() > 0:
            self.publish_debug(msg.header, gray, dets, res, publish)
        want_fixes = self.pub_fixes.get_subscription_count() > 0
        want_markers = self.pub_markers.get_subscription_count() > 0
        if dets and (want_fixes or want_markers):
            rec = fix_record(res, dets, T_prior, publish, t, self.p.mode,
                             marker_views(self.localizer, res, dets, T_prior))
            if want_fixes:
                self.pub_fixes.publish(String(data=json.dumps(rec)))
            if want_markers:
                self.pub_markers.publish(marker_diagnostics(rec, msg.header.stamp))

    def publish(self, stamp, fix):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp                     # the capture time, not now
        out.header.frame_id = self.map.frame_id
        T = fix.T_map_base
        p = out.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = (float(v) for v in matrix_to_quat(T[:3, :3]))
        out.pose.covariance = [float(v) for v in fix.covariance.ravel()]
        self.pub.publish(out)

    def publish_status(self):
        if self.pub_status.get_subscription_count() == 0:
            return
        m = self.map
        st = {'map': self.map_path, 'loaded': self.map_loaded, 'mode': self.p.mode,
              'enabled': self.enabled, 'camera': self.K is not None,
              'prior_topic': self.p.prior_topic, 'counts': dict(self.n),
              'frame_id': None if m is None else m.frame_id,
              'dictionary': None if m is None else m.dictionary,
              'markers': [] if m is None else [
                  {'id': int(k), 'x': round(float(T[0, 3]), 3), 'y': round(float(T[1, 3]), 3),
                   'z': round(float(T[2, 3]), 3)} for k, T in sorted(m.poses().items())]}
        self.pub_status.publish(String(data=json.dumps(st)))

    def on_status(self):
        if not self.t_detect:
            self.get_logger().info(f'{self.n["images"]} images, none processed yet '
                                   f'(camera_info: {"yes" if self.K is not None else "no"})')
            return
        det = 1e3 * np.mean(self.t_detect)
        fit = 1e3 * np.mean(self.t_fit)
        self.get_logger().info(
            f'{self.n["images"]} images, {self.n["fixes"]} fixes, {self.n["rejected"]} rejected'
            + (f', {self.n["priors"]} priors, {self.n["no prior"]} frames without one'
               if self.p.prior_topic else '')
            + f'; detection {det:.1f} ms, fit {fit:.1f} ms per frame (last {len(self.t_detect)})')
        self.t_detect, self.t_fit = [], []

    def publish_debug(self, header, gray, dets, res, published):
        """Green: in the fit (its map corners projected with the fix as dots, the mean
        residual written); orange: left out as an outlier or too far; red: not usable
        (cut by the border, too small); grey: not in the map."""
        import cv2
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        scale = min(1.0, self.p.debug_width / img.shape[1])
        fix = res.fix
        th = max(1, int(round(2 / scale)))
        for d in dets:
            c = d.corners.astype(np.int32).reshape(-1, 1, 2)
            label = f'{d.id}'
            if fix is not None and d.id in fix.ids:
                color = (0, 220, 0)
                r = fix.residuals[d.id]
                label += (f' {np.mean(r) * 100:.1f} cm' if fix.mode == 'planar'
                          else f' {np.mean(r):.1f} px')
                for q in fix.projected[d.id]:
                    cv2.circle(img, (int(round(q[0])), int(round(q[1]))), 2 * th, (255, 0, 255), -1)
            elif d.id in res.dropped:
                color = (0, 0, 255) if d.rejected else (0, 140, 255)
                label += f' {res.dropped[d.id]}'
            else:
                color = (160, 160, 160)
                label += ' not in map'
            cv2.polylines(img, [c], True, color, th)
            org = (int(d.corners[:, 0].min()), max(20, int(d.corners[:, 1].min()) - 8))
            cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7 / scale, color, th, cv2.LINE_AA)
        if scale < 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        if fix is not None:
            sd = math.sqrt(max(fix.covariance[0, 0], fix.covariance[1, 1]))
            status = (f'{self.p.mode}  x {fix.x:.2f} y {fix.y:.2f} yaw {math.degrees(fix.yaw):.1f}  '
                      f'{len(fix.ids)} markers  {fix.rms_px:.2f} px  sd {sd * 100:.1f} cm  '
                      + ('PUBLISHED' if published else res.reason))
        else:
            status = f'{self.p.mode}  no fix: {res.reason}'
        cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(img, status, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding, out.step = 'bgr8', img.shape[1] * 3
        out.data = img.tobytes()
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MarkerLocalizerNode()
    except (ValueError, OSError) as e:
        print(f'marker_localizer: {e}', flush=True)
        rclpy.shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
