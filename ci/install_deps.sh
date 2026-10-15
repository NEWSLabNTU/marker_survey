#!/bin/bash
# Packages the build, the tests and the synthetic survey need, in a ros:<distro> image.
set -e
distro=${1:?distro}
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-pytest python3-jsonschema poppler-utils \
  ros-$distro-rosbag2-py ros-$distro-rosbag2-storage-default-plugins ros-$distro-ros2bag \
  ros-$distro-tf2-ros-py ros-$distro-tf2-msgs ros-$distro-nav-msgs ros-$distro-sensor-msgs \
  ros-$distro-geometry-msgs ros-$distro-visualization-msgs ros-$distro-std-srvs
