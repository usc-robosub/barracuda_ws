FROM ros:humble-ros-base-jammy

# specified in docker-compose.yaml - either jetson
ARG PLATFORM

RUN apt-get update && apt-get install -y \
    vim \
    git \
    software-properties-common \
    && add-apt-repository ppa:maveonair/helix-editor \
    && apt-get update && apt-get install -y helix

COPY src /root/ros2_ws/src
COPY entrypoint.sh /root/entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    && cd ~/ros2_ws \
    && rosdep update && rosdep install --from-paths src -y --ignore-src \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install

RUN echo ${PLATFORM}

WORKDIR /root/ros2_ws/

CMD [ "/bin/bash", "/root/entrypoint.sh" ]
