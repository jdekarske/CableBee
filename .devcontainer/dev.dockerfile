# change the tag here when we decide the ur5e setup is finished
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

RUN \
  apt-get update -qq && \
  # ros-foxy-usb-cam ros-foxy-navigation2 && \
  rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src

# Setup environment
WORKDIR /catkin_ws/src

##########################################

RUN git clone -b ros2 https://github.com/ros-perception/image_pipeline.git \
  && mv image_pipeline image_pipeline_ \
  && mv image_pipeline_/* . \
  && rm -rf image_pipeline_

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
  && apt-get update -qq \
  && rosdep update \
  && rosdep install --from-path . --ignore-src -y \
  && cd .. \
  && colcon build --cmake-args -DBUILD_TESTING=OFF

# need to add a non-root user so bind mount permissions work correctly
ARG USERNAME=cablebee
RUN useradd -m $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  usermod --shell /bin/bash $USERNAME && \
  usermod -aG sudo $USERNAME && \
  echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  # Replace 1000 with your user/group id
  usermod  --uid 1000 $USERNAME && \
  groupmod --gid 1000 $USERNAME

USER $USERNAME
RUN mkdir -p /home/$USERNAME/catkin_ws/src
WORKDIR /home/$USERNAME/catkin_ws/src

RUN git clone -b launch-cleanup https://github.com/jdekarske/fiducials.git

CMD ["/bin/bash"]