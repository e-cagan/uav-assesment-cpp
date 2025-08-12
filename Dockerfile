# UAV assessment CI/runtime image (C++)
# - ROS 2 Humble (desktop)
# - PX4 v1.12.3 (SITL + Gazebo)
# - colcon + ament + gtest (C++)
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# Sistem paketleri
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git build-essential cmake ninja-build wget zip \
    python3-pip python3-colcon-common-extensions \
    gazebo libgazebo-dev libeigen3-dev libopencv-dev \
    libprotobuf-dev protobuf-compiler \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    ros-humble-launch-testing ros-humble-launch-testing-ros \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    iproute2 net-tools \
 && rm -rf /var/lib/apt/lists/*

# PX4 v1.12.3 kurulumu
WORKDIR /opt
RUN git clone --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
WORKDIR /opt/PX4-Autopilot

# PX4 build bağımlılıkları
RUN pip3 install --no-cache-dir pyros-genmsg empy jinja2

# GCC 11 uyumluluk düzeltmesi (PX4 v1.12)
RUN sed -i \
  's/math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize))/math::max((size_t)PTHREAD_STACK_MIN, (size_t)PX4_STACK_ADJUSTED(wq->stacksize))/g' \
  platforms/common/px4_work_queue/WorkQueueManager.cpp

# Gazebo pluginleriyle birlikte SITL derle
RUN make px4_sitl gazebo

# GeographicLib datasetlerini kur
RUN bash -lc 'source /opt/ros/humble/setup.bash && ros2 run mavros install_geographiclib_datasets.sh'

# ROS entrypoint
RUN printf '%s\n' \
  '#!/usr/bin/env bash' \
  'set -e' \
  'source /opt/ros/humble/setup.bash' \
  'exec "$@"' > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Çalışma alanı
WORKDIR /ws
