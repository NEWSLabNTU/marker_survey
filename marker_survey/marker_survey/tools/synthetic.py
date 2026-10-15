"""marker_survey_synthetic: a survey recording without a robot, for testing.

Renders what a camera sees while a robot drives a route among the markers of
a marker map (real marker images warped onto their corners), and writes a
rosbag2 with the topics the survey node reads:

  /camera/image          mono8, rectified
  /camera/camera_info
  /odom                  nav_msgs/Odometry of base_link in odom, drifting
  /tf                    odom -> base_link (the same odometry) and map -> odom
                         (so that map -> base_link is a localizer's pose: the
                         truth plus --map-noise)
  /tf_static             base_link -> camera_link -> camera_optical_frame
  /prior_pose            geometry_msgs/PoseWithCovarianceStamped, the same
                         localizer pose at every image's stamp (a state
                         estimator's pose, for the localizer's gate)

With --split-at S, also bag_1 (the first S seconds) and bag_2 (the rest),
each with /tf_static: a drive that stops and goes on, as a survey that is
saved and then continued ("pass again") sees it.

And beside it the truth: truth_map.yaml (the markers in the map frame, what
pose_frame and align_to_track surveys should give), with --origin
truth_origin.yaml (in the frame of that marker, what origin_marker gives), and
truth_poses.csv (stamp and true base_link pose in the map of every image, to
score a localizer).

    marker_survey_synthetic OUT --map examples/wall_course/sites/test_course/marker_map.yaml \\
        --route examples/wall_course/sites/test_course/route.csv --camera forward --origin 10
"""

import argparse
import json
import math
import os

import numpy as np

from marker_survey.core import frames as fr
from marker_survey.core import marker_map as mm
from marker_survey.core import simulate as sim
from marker_survey.core.detection import opencv_dictionary
from marker_survey.core.path_csv import load_path_csv
from marker_survey.core.render import Renderer
from marker_survey.core.se3 import interpolate_T, inv_T, make_T, matrix_to_quat

CAMERAS = {
    # name: (width, height, hfov_deg, factory)
    'forward': (960, 600, 102.0, sim.forward_camera),
    'upward': (640, 480, 70.0, sim.upward_camera),
}
TF_STATIC_QOS = ('- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n'
                 '  deadline: {sec: 2147483647, nsec: 4294967295}\n'
                 '  lifespan: {sec: 2147483647, nsec: 4294967295}\n  liveliness: 1\n'
                 '  liveliness_lease_duration: {sec: 2147483647, nsec: 4294967295}\n'
                 '  avoid_ros_namespace_conventions: false\n')


def topic_metadata(rosbag2_py, n, name, typ, qos_yaml):
    """TopicMetadata on Humble (QoS as YAML) and on Jazzy and newer (an id, QoS objects)."""
    try:
        return rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr',
                                        offered_qos_profiles=qos_yaml)
    except TypeError:
        from rosbag2_py._storage import to_rclcpp_qos_vector
        qos = to_rclcpp_qos_vector(qos_yaml, 9) if qos_yaml else []
        return rosbag2_py.TopicMetadata(id=n, name=name, type=typ, serialization_format='cdr',
                                        offered_qos_profiles=qos)


def truth_map(site: mm.MarkerMap, T_target_map: np.ndarray, header: str) -> mm.MarkerMap:
    markers = []
    for m in site.markers:
        mk = mm.MapMarker.from_pose(m.id, T_target_map @ m.pose(), mount=m.mount, size=m.size)
        markers.append(mk)
    return mm.MarkerMap(dictionary=site.dictionary, marker_size=site.marker_size, markers=markers,
                        frame_id=site.frame_id, mount=site.mount, header=[header])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out', help='output directory (bag/, truth_*.yaml, scene.json)')
    ap.add_argument('--map', required=True, help='marker_map.yaml of the site')
    ap.add_argument('--route', required=True, help='route CSV: x, y waypoints [m]')
    ap.add_argument('--camera', choices=sorted(CAMERAS), default='forward')
    ap.add_argument('--width', type=int, help='image width (default by camera)')
    ap.add_argument('--height', type=int)
    ap.add_argument('--hfov', type=float, help='horizontal field of view [deg]')
    ap.add_argument('--origin', type=int, help='also write the truth in this marker\'s frame')
    ap.add_argument('--marker-size', type=float, help='override the map\'s marker_size [m]')
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--open', action='store_true', help='the route is not a loop')
    ap.add_argument('--sweeps', type=int, default=0, help='turns on the spot per lap')
    ap.add_argument('--sweep-deg', type=float, default=70.0)
    ap.add_argument('--speed', type=float, default=0.3, help='[m/s]')
    ap.add_argument('--rate', type=float, default=3.0, help='camera frames per second')
    ap.add_argument('--drift', type=float, default=0.01, help='odometry sd per metre')
    ap.add_argument('--yaw-drift-deg-per-m', type=float, default=0.3)
    ap.add_argument('--map-noise', type=float, default=0.0,
                    help='[m] sd per horizontal axis of the map -> base_link pose')
    ap.add_argument('--map-noise-yaw-deg', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--drop-odom', type=float, default=0.0,
                    help='share of the /odom messages left out (lost messages), seeded')
    ap.add_argument('--start-time', type=float, default=1_000_000_000.0,
                    help='[s] the stamp the recording starts at; a later one for a second drive '
                         'played after the first')
    ap.add_argument('--split-at', type=float,
                    help='[s] also write bag_1 (up to this time into the drive) and bag_2 (the rest)')
    return ap.parse_args(argv)


def scene(args):
    """(site map, camera, frames) of a synthetic recording; no ROS needed."""
    site = mm.load(args.map)
    if args.marker_size:
        site.marker_size = args.marker_size
    w, h, hfov, factory = CAMERAS[args.camera]
    camera = factory(width=args.width or w, height=args.height or h, hfov_deg=args.hfov or hfov)
    step = args.speed / args.rate
    poses = sim.route(load_path_csv(args.route), laps=args.laps, closed=not args.open, step=step,
                      sweeps=args.sweeps, sweep_deg=args.sweep_deg)
    noise = sim.Noise(drift=args.drift, yaw_drift_deg_per_m=args.yaw_drift_deg_per_m,
                      map_sd=args.map_noise, map_sd_yaw_deg=args.map_noise_yaw_deg, seed=args.seed)
    return site, camera, sim.make_frames(poses, noise, rate=args.rate)


def main(argv=None):
    import rclpy.serialization as ser
    import rosbag2_py
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    args = parse_args(argv)
    site, camera, frames = scene(args)
    if args.origin is not None and args.origin not in site.ids():
        raise SystemExit(f'origin {args.origin} is not a marker of {args.map}')
    markers, sizes = site.poses(), site.sizes()
    renderer = Renderer(opencv_dictionary(site.dictionary), camera.K, camera.width, camera.height)
    K = camera.K
    T_base_link = make_T(np.eye(3), camera.T_base_cam[:3, 3])       # base_link -> camera_link
    T_link_opt = inv_T(T_base_link) @ camera.T_base_cam              # camera_link -> optical

    os.makedirs(args.out, exist_ok=True)
    bag = os.path.join(args.out, 'bag')
    names = [bag] + ([os.path.join(args.out, 'bag_1'), os.path.join(args.out, 'bag_2')]
                     if args.split_at is not None else [])
    writers = []
    for b in names:
        if os.path.exists(b):
            import shutil
            shutil.rmtree(b)
        wr = rosbag2_py.SequentialWriter()
        wr.open(rosbag2_py.StorageOptions(uri=b, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
        writers.append(wr)
    topics = {
        '/camera/image': 'sensor_msgs/msg/Image',
        '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/odom': 'nav_msgs/msg/Odometry',
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
        '/prior_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    }
    for wr in writers:
        for n, (name, typ) in enumerate(topics.items()):
            wr.create_topic(topic_metadata(rosbag2_py, n, name, typ,
                                           TF_STATIC_QOS if name == '/tf_static' else ''))
    t0 = float(args.start_time)
    split_ns = None if args.split_at is None else int(round((t0 + 1.0 + args.split_at) * 1e9))

    def put(topic, data, t_ns):
        """Into the whole recording, and into the part the time falls in."""
        writers[0].write(topic, data, t_ns)
        if split_ns is not None:
            writers[1 if t_ns < split_ns else 2].write(topic, data, t_ns)

    def stamp(header, t):
        header.stamp.sec = int(t)
        header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000

    def tfs(parent, child, T, t):
        m = TransformStamped()
        stamp(m.header, t)
        m.header.frame_id, m.child_frame_id = parent, child
        (m.transform.translation.x, m.transform.translation.y,
         m.transform.translation.z) = (float(v) for v in T[:3, 3])
        (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
         m.transform.rotation.w) = (float(v) for v in matrix_to_quat(T[:3, :3]))
        return m

    def ns(t):
        return int(round(t * 1e9))

    def msg_stamp(t):
        """The stamp as the messages carry it: sec.nanosec, written exactly."""
        sec = int(t)
        return f'{sec}.{int(round((t - sec) * 1e9)) % 1_000_000_000:09d}'

    static = TFMessage(transforms=[tfs('base_link', 'camera_link', T_base_link, t0),
                                   tfs('camera_link', 'camera_optical_frame', T_link_opt, t0)])
    for wr in writers[:2]:
        wr.write('/tf_static', ser.serialize_message(static), ns(t0))
    if split_ns is not None:            # the second part starts where it starts: its own static TF
        writers[2].write('/tf_static', ser.serialize_message(static), split_ns)

    odo_div = 3                        # odometry 3x the camera rate
    drop_rng = np.random.default_rng(args.seed + 3000)
    dt = 1.0 / args.rate
    truth_rows = []
    prior_var = [max(args.map_noise, 0.01) ** 2, max(math.radians(args.map_noise_yaw_deg), 0.005) ** 2]
    for k, f in enumerate(frames):
        t = t0 + 1.0 + k * dt
        prev = frames[k - 1].T_odom_base if k > 0 else f.T_odom_base
        steps = range(1, odo_div + 1) if k > 0 else [odo_div]
        for j in steps:                # odometry since the last frame, interpolated
            tj = t - dt + j * dt / odo_div
            Tj = interpolate_T(prev, f.T_odom_base, j / odo_div)
            o = Odometry()
            stamp(o.header, tj)
            o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
            (o.pose.pose.position.x, o.pose.pose.position.y,
             o.pose.pose.position.z) = (float(v) for v in Tj[:3, 3])
            (o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z,
             o.pose.pose.orientation.w) = (float(v) for v in matrix_to_quat(Tj[:3, :3]))
            if not (args.drop_odom and k > 0 and drop_rng.random() < args.drop_odom):
                put('/odom', ser.serialize_message(o), ns(tj))
            tf = [tfs('odom', 'base_link', Tj, tj)]
            if j == odo_div:
                T_map_odom = f.T_mapmeas_base @ inv_T(f.T_odom_base)
                tf.append(tfs('map', 'odom', T_map_odom, tj))
            put('/tf', ser.serialize_message(TFMessage(transforms=tf)), ns(tj))
        pr = PoseWithCovarianceStamped()
        stamp(pr.header, t)
        pr.header.frame_id = 'map'
        P = f.T_mapmeas_base
        pr.pose.pose.position.x, pr.pose.pose.position.y, pr.pose.pose.position.z = \
            (float(v) for v in P[:3, 3])
        (pr.pose.pose.orientation.x, pr.pose.pose.orientation.y, pr.pose.pose.orientation.z,
         pr.pose.pose.orientation.w) = (float(v) for v in matrix_to_quat(P[:3, :3]))
        cov = np.diag([prior_var[0], prior_var[0], 1e-4, 1e-4, 1e-4, prior_var[1]])
        pr.pose.covariance = [float(v) for v in cov.ravel()]
        put('/prior_pose', ser.serialize_message(pr), ns(t) - 1)
        Tt = f.T_map_base
        truth_rows.append((msg_stamp(t), *Tt[:3, 3], *matrix_to_quat(Tt[:3, :3])))
        img = renderer.render(markers, sizes, f.T_map_base @ camera.T_base_cam)
        msg = Image()
        stamp(msg.header, t)
        msg.header.frame_id = 'camera_optical_frame'
        msg.height, msg.width, msg.encoding, msg.step = camera.height, camera.width, 'mono8', camera.width
        msg.data = img.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = camera.width, camera.height
        info.k = [float(v) for v in K.ravel()]
        info.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, 0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        info.distortion_model = 'plumb_bob'
        put('/camera/image', ser.serialize_message(msg), ns(t))
        put('/camera/camera_info', ser.serialize_message(info), ns(t))
    writers.clear()                    # closes the bags

    mm.write(truth_map(site, np.eye(4), f'Truth of the synthetic recording, frame {site.frame_id}.'),
             os.path.join(args.out, 'truth_map.yaml'))
    if args.origin is not None:
        T_om = inv_T(fr.origin_marker_frame(markers[args.origin], np.zeros(3), np.array([0, 0, 1.0])))
        mm.write(truth_map(site, T_om, f'Truth of the synthetic recording, frame of marker '
                                       f'{args.origin} (origin_marker mode).'),
                 os.path.join(args.out, 'truth_origin.yaml'))
    with open(os.path.join(args.out, 'truth_poses.csv'), 'w') as fh:
        fh.write('# stamp [s], true base_link pose in the map: x, y, z [m], qx, qy, qz, qw\n')
        for r in truth_rows:
            fh.write(r[0] + ',' + ','.join(f'{v:.9f}' for v in r[1:]) + '\n')
    with open(os.path.join(args.out, 'scene.json'), 'w') as fh:
        json.dump({k: v for k, v in vars(args).items()}, fh, indent=1)
    length = sum(np.linalg.norm(frames[i + 1].T_map_base[:3, 3] - frames[i].T_map_base[:3, 3])
                 for i in range(len(frames) - 1))
    print(f'{len(frames)} frames, {len(frames) * dt:.0f} s, {length:.0f} m driven; bag {bag}; '
          f'truth in {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
