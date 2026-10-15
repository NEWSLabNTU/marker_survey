"""marker_survey: map the markers around a robot while it drives, write marker_map.yaml.

Drive the robot slowly past every marker. Every processed frame is matched
to the camera's pose from the pose source; frames that move or show a new
marker become keyframes, and every marker is placed in the map the moment
it is first seen (the log names each new id; ~/markers shows the live map
in RViz). At the end (Ctrl-C, or the ~/save service) the recording is
bundle-adjusted and the result is written to `output_dir` in the frame
`frame.mode` chooses (core/frames.py).

  in   image (raw or compressed) and its camera_info (rectified, or with distortion
       coefficients: the corners are then undistorted)
       the camera's pose:
         pose_source: odometry  nav_msgs/Odometry (visual-inertial, wheel and IMU)
         pose_source: tf        TF pose_frame -> image frame, any frame
       TF image frame -> base_frame: where the floor is
       frame.mode align_to_track: TF frame.reference_frame -> base_frame, the
       reference trajectory (a localizer's poses)
  out  ~/markers       visualization_msgs/MarkerArray, live
       ~/debug/image   the processed frames with the detections drawn (only while
                       something subscribes)
       output_dir/marker_map.yaml, survey_report.yaml, survey_data.npz (solve again
       with marker_survey_solve)
"""

import collections
import os
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from marker_survey.core.detection import Detector
from marker_survey.core.se3 import interpolate_T, make_T, matrix_to_quat, quat_to_matrix
from marker_survey.core.solve import DATA_FILE, finish, save_dump
from marker_survey.core.survey import Survey
from marker_survey.params import PARAMETERS, SurveyParams


def tf_to_T(t) -> np.ndarray:
    q, p = t.transform.rotation, t.transform.translation
    return make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def stamp_sec(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


def decode(msg, compressed: bool):
    import cv2
    if compressed:
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    ch = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(enc)
    if ch is None:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    code = {'bgr8': cv2.COLOR_BGR2GRAY, 'rgb8': cv2.COLOR_RGB2GRAY,
            'bgra8': cv2.COLOR_BGRA2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY}[enc]
    return cv2.cvtColor(img, code)


class MarkerSurveyNode(Node):
    def __init__(self):
        super().__init__('marker_survey')
        for k, v in PARAMETERS.items():
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.p = p = SurveyParams(gp)
        self.process_all = bool(gp('process_all'))
        self.detector = Detector(p.dictionary, p.min_marker_px, p.border_margin)
        self.compressed = bool(gp('compressed'))
        self.odom_topic = gp('odom_topic')
        self.pose_tol = float(gp('pose_tolerance'))
        self.save_on_exit = bool(gp('save_on_exit'))
        self.origin = p.frame.origin_marker

        self.survey = None
        self.lock = threading.Lock()
        self.queue = collections.deque(maxlen=300)
        self.odom = collections.deque(maxlen=2000)
        self.odom_child = None
        self.odom_frame = 'odom'
        self.T_child_cam = None
        self.undistort = None
        self.frames = 0
        self.saved = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        img_type = CompressedImage if self.compressed else Image
        topic = gp('image_topic') + ('/compressed' if self.compressed else '')
        # process_all (a recording): a reliable subscription loses no frame; live, a
        # best-effort one matches any driver
        qos = (QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE) if self.process_all
               else qos_profile_sensor_data)
        self.create_subscription(img_type, topic, self.on_image, qos)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self.on_info, qos)
        if p.pose_source == 'odometry':
            # many drivers publish odometry BEST_EFFORT; a RELIABLE subscription never matches it
            self.create_subscription(
                Odometry, self.odom_topic, self.on_odom,
                QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.pub_dbg = self.create_publisher(Image, '~/debug/image', 1)
        self.debug_width = int(gp('debug_width'))
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_timer(1.0 / float(gp('rate')), self.on_tick)
        self.create_timer(5.0, self.on_status)
        origin = f'origin marker {self.origin}' if self.origin is not None else ''
        self.get_logger().info(
            f'surveying {p.dictionary} markers; marker_size {p.settings.marker_size} m is the edge '
            'of the BLACK SQUARE, not of the sheet: measure it, a wrong size scales every distance. '
            f'Frame mode {p.frame.mode} {origin}; output -> {p.output_dir}. Drive slowly past '
            'every marker; Ctrl-C (or the ~/save service) solves and writes the map.')

    # ------------------------------------------------------------ inputs
    def on_info(self, msg):
        if self.survey is None:
            if any(abs(v) > 1e-9 for v in msg.d):
                # a raw image: detect on it, then undistort the corners onto the pinhole K
                K = np.array(msg.k, float).reshape(3, 3)
                self.undistort = (K, np.array(msg.d, float))
                what = f'distortion {msg.distortion_model}, corners undistorted'
            else:
                K = (np.array(msg.p, float).reshape(3, 4)[:, :3] if any(msg.p)
                     else np.array(msg.k, float).reshape(3, 3))
                what = 'rectified'
            self.survey = Survey(self.p.settings, K)
            self.get_logger().info(f'camera: {msg.width}x{msg.height}, fx {K[0, 0]:.1f} px, {what}')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom.append((stamp_sec(msg.header.stamp),
                          make_T(quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])))
        self.odom_child = msg.child_frame_id
        self.odom_frame = msg.header.frame_id

    def on_image(self, msg):
        if not self.process_all:
            self.queue.clear()          # live: only the newest frame matters
        self.queue.append(msg)

    # ------------------------------------------------------------ poses
    def lookup(self, parent: str, child: str, t: float):
        try:
            return tf_to_T(self.tf_buffer.lookup_transform(parent, child, Time(seconds=t)))
        except tf2_ros.TransformException:
            return None

    def pose_at(self, t: float, cam_frame: str):
        if self.p.pose_source == 'tf':
            return self.lookup(self.p.pose_frame, cam_frame, t)
        if len(self.odom) < 2 or self.odom_child is None:
            return None
        if self.T_child_cam is None:
            self.T_child_cam = self.lookup(self.odom_child, cam_frame, 0.0)
            if self.T_child_cam is None:
                self.get_logger().info(f'waiting for TF {self.odom_child} -> {cam_frame}',
                                       throttle_duration_sec=5.0)
                return None
        od = list(self.odom)
        ts = np.array([o[0] for o in od])
        i = int(np.searchsorted(ts, t))
        if i == 0 or i >= len(od):
            j = 0 if i == 0 else len(od) - 1
            if abs(ts[j] - t) > self.pose_tol:
                return None
            T = od[j][1]
        else:
            t0, t1 = ts[i - 1], ts[i]
            if t1 - t0 > 2 * self.pose_tol:
                return None
            T = interpolate_T(od[i - 1][1], od[i][1], (t - t0) / max(t1 - t0, 1e-9))
        return T @ self.T_child_cam

    # ------------------------------------------------------------ work
    def on_tick(self):
        """Process the queued frame (live: the newest one; process_all: every frame in
        order). A frame whose pose has not arrived yet waits, up to 2 s of stamps."""
        while self.queue and self.survey is not None:
            msg = self.queue[0]
            if not self.process(msg):
                newest = stamp_sec(self.queue[-1].header.stamp)
                if newest - stamp_sec(msg.header.stamp) < 2.0:
                    return
            if self.queue and self.queue[0] is msg:
                self.queue.popleft()
            if not self.process_all:
                return

    def process(self, msg) -> bool:
        """Survey one frame; False if its pose is not available (yet)."""
        cam_frame = msg.header.frame_id
        if self.survey.T_cam_base is None:
            T = self.lookup(cam_frame, self.p.base_frame, 0.0)
            if T is None:
                self.get_logger().info(f'waiting for TF {cam_frame} -> {self.p.base_frame}',
                                       throttle_duration_sec=5.0)
                return False
            self.survey.T_cam_base = T
        t = stamp_sec(msg.header.stamp)
        T = self.pose_at(t, cam_frame)
        if T is None:
            # keep the frame: its pose may arrive a moment later
            src = (f'the odometry topic {self.odom_topic} (a QoS mismatch gives exactly this)'
                   if self.p.pose_source == 'odometry' else f'TF {self.p.pose_frame} -> {cam_frame}')
            self.get_logger().info(f'no camera pose for this frame yet: is {src} arriving?',
                                   throttle_duration_sec=5.0)
            return False
        T_ref = None
        if self.p.frame.mode == 'align_to_track':
            T_ref = self.lookup(self.p.reference_frame, self.p.base_frame, t)
        try:
            gray = decode(msg, self.compressed)
            dets = self.detector.detect_all(gray)
            if self.undistort is not None:
                import cv2
                K, D = self.undistort
                for d in dets:
                    d.corners = cv2.undistortPoints(d.corners.reshape(-1, 1, 2), K, D,
                                                    P=K).reshape(4, 2)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'image: {e}', throttle_duration_sec=5.0)
            return True
        self.frames += 1
        with self.lock:
            known = set(self.survey.markers)
            n_kf = len(self.survey.keyframes)
            new = self.survey.add(t, T, dets, T_ref)
            keyframe = len(self.survey.keyframes) > n_kf
        if self.pub_dbg.get_subscription_count() > 0:
            self.publish_debug(msg.header, gray, dets, known, keyframe)
        for mid in new:
            m = self.survey.markers[mid]
            dist = np.linalg.norm(m.T_world_marker[:3, 3] - T[:3, 3])
            tag = ' (the origin)' if mid == self.origin else ''
            self.get_logger().info(f'new marker {mid}{tag}, {dist:.1f} m away; '
                                   f'{len(self.survey.markers)} so far')
        if new:
            self.publish_markers()
        return True

    def on_status(self):
        if self.survey is None:
            self.get_logger().info('waiting for camera_info', throttle_duration_sec=10.0)
            return
        views = ', '.join(f'{k}:{v.views}' for k, v in sorted(self.survey.markers.items()))
        origin = '' if self.origin is None or self.origin in self.survey.markers else \
            f'; origin marker {self.origin} NOT SEEN YET'
        self.get_logger().info(f'{self.frames} frames, {len(self.survey.keyframes)} keyframes; '
                               f'markers (views) {views or "none"}{origin}')
        self.publish_markers()

    def publish_debug(self, header, gray, dets, known, keyframe):
        """The frame with every detection drawn: green a known marker, yellow one seen
        for the first time, red one not used (too small, or cut by the border); the
        origin is outlined thicker."""
        import cv2
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        scale = min(1.0, self.debug_width / img.shape[1])
        for d in dets:
            c = d.corners.astype(np.int32).reshape(-1, 1, 2)
            usable = self.survey.usable(d)
            color = (0, 0, 255) if not usable else (0, 220, 0) if d.id in known else (0, 220, 255)
            thick = max(2, int(round(3 / scale))) * (2 if d.id == self.origin else 1)
            cv2.polylines(img, [c], True, color, thick)
            cv2.circle(img, tuple(int(v) for v in d.corners[0]), thick * 2, color, -1)   # top-left
            label = f'{d.id}' + (' ORIGIN' if d.id == self.origin else '')
            if d.id in self.survey.markers:
                label += f' ({self.survey.markers[d.id].views})'
            if d.rejected:
                label += f' {d.rejected}'
            org = (int(d.corners[:, 0].min()), max(20, int(d.corners[:, 1].min()) - 10))
            cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9 / scale, color,
                        max(1, int(round(2 / scale))), cv2.LINE_AA)
        if scale < 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        status = (f'markers {len(self.survey.markers)}  keyframes {len(self.survey.keyframes)}'
                  f'{"  [KEYFRAME]" if keyframe else ""}')
        if self.origin is not None:
            status += '  origin seen' if self.origin in self.survey.markers else \
                f'  origin {self.origin} NOT SEEN'
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(img, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding, out.step = 'bgr8', img.shape[1] * 3
        out.data = img.tobytes()
        self.pub_dbg.publish(out)

    def publish_markers(self):
        arr = MarkerArray()
        frame = self.p.pose_frame if self.p.pose_source == 'tf' else self.odom_frame
        for k, (mid, m) in enumerate(sorted(self.survey.markers.items())):
            T = m.T_world_marker
            q = matrix_to_quat(T[:3, :3])
            box = Marker(type=Marker.CUBE, action=Marker.ADD, id=2 * k, ns='marker')
            box.header.frame_id = frame
            box.pose.position.x, box.pose.position.y, box.pose.position.z = (float(v) for v in T[:3, 3])
            (box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z,
             box.pose.orientation.w) = (float(v) for v in q)
            s = self.p.settings.size_of(mid)
            box.scale.x, box.scale.y, box.scale.z = s, s, 0.01
            box.color.g = 1.0 if mid != self.origin else 0.3
            box.color.r = 0.0 if mid != self.origin else 1.0
            box.color.a = 0.7
            txt = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=2 * k + 1, ns='id')
            txt.header.frame_id = frame
            pos = T[:3, 3] + T[:3, 1] * s
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = (float(v) for v in pos)
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{mid} ({m.views})'
            arr.markers += [box, txt]
        self.pub.publish(arr)

    # ------------------------------------------------------------ the result
    def save(self):
        with self.lock:
            if self.survey is None or not self.survey.keyframes:
                return False, 'nothing recorded'
            os.makedirs(self.p.output_dir, exist_ok=True)
            dump = os.path.join(self.p.output_dir, DATA_FILE)
            save_dump(dump, self.survey, self.p.dictionary, self.p.frame)
            log = self.get_logger().info
            log(f'wrote {dump}')
            try:
                res = finish(self.survey, self.p.frame, self.p.dictionary, self.p.output_dir,
                             log=lambda s: [log(line) for line in str(s).splitlines()])
            except ValueError as e:
                self.get_logger().error(f'not solved: {e}. The recording is in {dump}: '
                                        'marker_survey_solve can try again.')
                return False, str(e)
        self.saved = True
        flagged = [m.id for m in res['markers'] if m.verdict != 'good']
        return True, (f'{len(res["markers"])} markers written to {self.p.output_dir}'
                      + (f'; check {flagged}' if flagged else ''))

    def on_save(self, request, response):
        response.success, response.message = self.save()
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MarkerSurveyNode()
    except ValueError as e:
        print(f'marker_survey: {e}', flush=True)
        rclpy.shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.save_on_exit and not node.saved:
            ok, msg = node.save()
            print(f'survey: {msg}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
