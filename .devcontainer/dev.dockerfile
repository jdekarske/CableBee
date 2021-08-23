# change the tag here when we decide the ur5e setup is finished
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# For gui stuff (I think)
RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri \
  ros-foxy-usb-cam && ros-foxy-navigation2 ros-foxy-image-transport-plugins && \
  rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src

# Setup environment
WORKDIR /catkin_ws/src

RUN source /opt/ros/$ROS_DISTRO/setup.bash

##########################################

RUN git clone -b ros2 https://github.com/ros-perception/image_pipeline.git \
  && mv image_pipeline image_pipeline_ \
  && mv image_pipeline_/* . \
  && rm -rf image_pipeline_

# RUN git clone -b ros2 https://github.com/agutenkunst/fiducials.git

# We'll mount this separately
# COPY src/ src/

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
  && apt-get update -qq \
  && rosdep update \
  && rosdep install --from-path . --ignore-src -y \
  && cd .. \
  && colcon build --cmake-args -DBUILD_TESTING=OFF

# For rosbridge
EXPOSE 9090

CMD ["/bin/bash"]