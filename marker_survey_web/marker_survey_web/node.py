"""marker_survey_web_node: the web interface on the robot, as a ROS 2 node.

    ros2 run marker_survey_web marker_survey_web_node --ros-args \
        -p map_path:=$PWD/sites/my_site/marker_map.yaml
    ros2 launch marker_survey_web web.launch.xml map_path:=...

Serves the same page as marker_survey_web, plus the survey, check and map
screens, which drive the survey node (start it with start_paused:=true) and
the localizer node. Parameters:

  host, port            where the page is served (default 0.0.0.0:8090: any device on
                        the robot's network)
  survey_node           the survey node's name (/marker_survey)
  localizer_node        the localizer node's name (/marker_localizer)
  map_path              the map the robot's localizer reads, written by "Use the map";
                        empty: the localizer's marker_map when it is first heard from
  camera_rate           [Hz] camera frames sent to a page that shows them
  camera_width          [px] their width
  check.agree_distance  [m] and check.agree_angle_deg: a fix this close to the fused
                        pose agrees; check.min_samples, check.max_disagree: when a
                        marker is flagged
"""

import threading

PARAMETERS = {
    'host': '0.0.0.0',
    'port': 8090,
    'survey_node': '/marker_survey',
    'localizer_node': '/marker_localizer',
    'map_path': '',
    'camera_rate': 5.0,
    'camera_width': 640,
    'check.agree_distance': 0.15,
    'check.agree_angle_deg': 5.0,
    'check.min_samples': 5,
    'check.max_disagree': 0.2,
}


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    from marker_localizer.core.check import CheckSettings
    from marker_survey_web.hub import Hub
    from marker_survey_web.ros_bridge import RosRobot
    from marker_survey_web.server import make_server

    rclpy.init(args=args)
    node = rclpy.create_node('marker_survey_web')
    for k, v in PARAMETERS.items():
        node.declare_parameter(k, v)
    gp = lambda k: node.get_parameter(k).value  # noqa: E731
    hub = Hub(None, str(gp('map_path')), CheckSettings(
        agree_distance=float(gp('check.agree_distance')),
        agree_angle_deg=float(gp('check.agree_angle_deg')),
        min_samples=int(gp('check.min_samples')), max_disagree=float(gp('check.max_disagree'))))
    hub.robot = RosRobot(node, hub, str(gp('survey_node')), str(gp('localizer_node')),
                         float(gp('camera_rate')), int(gp('camera_width')))
    try:
        server = make_server(str(gp('host')), int(gp('port')), hub)
    except OSError as e:
        node.get_logger().error(f'cannot serve on {gp("host")}:{gp("port")}: {e}')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host = gp('host')
    shown = "this robot's address" if host == '0.0.0.0' else host
    node.get_logger().info(f'open http://{shown}:{server.server_address[1]}/ in a browser; '
                           f'survey node {gp("survey_node")}, localizer {gp("localizer_node")}, '
                           f'map in use {gp("map_path") or "(from the localizer)"}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        hub.stop()
        stopper = threading.Thread(target=server.shutdown, daemon=True)
        stopper.start()
        stopper.join(2.0)
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
