# ROS Noetic + Gazebo 11 + RViz with a virtual display, for building the package and recording the demo video.
FROM osrf/ros:noetic-desktop-full
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      python3-catkin-tools libeigen3-dev xvfb ffmpeg xdotool x11-utils \
      ros-noetic-topic-tools ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller \
    && rm -rf /var/lib/apt/lists/*
# the house world uses three models from the Gazebo model database; bake them in so it loads offline
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates && rm -rf /var/lib/apt/lists/*     && git clone --depth 1 --filter=blob:none --no-checkout https://github.com/osrf/gazebo_models /tmp/gm     && cd /tmp/gm && git checkout HEAD -- cafe_table first_2015_trash_can mailbox     && mkdir -p /root/.gazebo/models && cp -r cafe_table first_2015_trash_can mailbox /root/.gazebo/models/     && rm -rf /tmp/gm
COPY ros_ws/src /ros_ws/src
RUN . /opt/ros/noetic/setup.sh && cd /ros_ws && catkin build --no-status
COPY docker/record_demo.sh /record_demo.sh
ENV LIBGL_ALWAYS_SOFTWARE=1
CMD ["bash", "/record_demo.sh"]
