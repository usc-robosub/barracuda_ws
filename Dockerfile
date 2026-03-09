FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    vim \
    git \
    software-properties-common \
    && add-apt-repository ppa:maveonair/helix-editor \
    && apt-get update && apt-get install -y helix

COPY src /root/ros2_ws/src
COPY entrypoint.sh /root/entrypoint.sh

ARG PLATFORM
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    && cd ~/ros2_ws \
    && if [ "${PLATFORM}" = "pi" ]; \
    then rosdep update && rosdep install --from-paths src/barracuda_thrusters -y --ignore-src \
    && colcon build --symlink-install --packages-select barracuda_thrusters barracuda_onboard; \
    else rosdep update && rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install; \
    fi \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /root/ros2_ws/

CMD [ "/bin/bash", "/root/entrypoint.sh" ]
