FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y software-properties-common && add-apt-repository ppa:maveonair/helix-editor && apt-get update && apt-get install -y \
    vim \
    git \
    python3-pip \
    python3-pylsp \
    clangd \
    software-properties-common \
    helix
        

COPY src /root/ros2_ws/src

ARG PKG_SEL
ENV PKG_SEL=$PKG_SEL
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    # && rosdep update \
    && cd ~/ros2_ws \
    && if [ -n "$PKG_SEL" ]; \
    then touch ~/pkg_sel.txt && cd ~/ros2_ws/src && rosdep install --from-paths barracuda_onboard $PKG_SEL -y --ignore-src && cd ~/ros2_ws && colcon build --symlink-install --packages-select barracuda_onboard $PKG_SEL; \
    else touch ~/no_pkg_sel.txt && cd ~/ros2_ws/src && rosdep install --from-paths . -y --ignore-src && cd ~/ros2_ws && colcon build --symlink-install; \
    fi \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /root/ros2_ws/

COPY entrypoint.sh /root/entrypoint.sh
CMD [ "/bin/bash", "/root/entrypoint.sh" ]
