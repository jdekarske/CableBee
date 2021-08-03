# change the tag here when we decide the ur5e setup is finished
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# For gui stuff (I think)
RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Setup environment
WORKDIR /catkin_ws/

RUN source /opt/ros/$ROS_DISTRO/setup.bash

##########################################

# We'll mount this separately
# COPY src/ src/

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

# For rosbridge
EXPOSE 9090

CMD ["/bin/bash"]