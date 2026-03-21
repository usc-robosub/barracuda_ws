FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    vim \
    git \
    python3-pip \
    python3-pylsp \
    clangd \
    software-properties-common \ 
    && cd && curl -L -O https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-$(uname -m)-linux.tar.xz \
    && tar xf helix-25.07.1-$(uname -m)-linux.tar.xz && mv helix-25.07.1-$(uname -m)-linux/hx /usr/local/bin \
    && mkdir -p ~/.config/helix && mv helix-25.07.1-$(uname -m)-linux/runtime ~/.config/helix \
    && rm -rf helix-25.07.1-$(uname -m)-linux
            

COPY src /root/barracuda_ws/src

ARG PKG_SEL
ENV PKG_SEL=$PKG_SEL
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/barracuda_ws/install/setup.bash ] && source ~/barracuda_ws/install/setup.bash" >> ~/.bashrc \
    && . /opt/ros/humble/setup.sh \
    # && apt-get update \
    # && rosdep update \
    && cd ~/barracuda_ws \
    && if [ -n "$PKG_SEL" ]; \
    then cd ~/barracuda_ws/src && rosdep install --from-paths barracuda_onboard $PKG_SEL -y --ignore-src && cd ~/barracuda_ws && colcon build --symlink-install --packages-select barracuda_onboard $PKG_SEL; \
    else cd ~/barracuda_ws/src && rosdep install --from-paths . -y --ignore-src && cd ~/barracuda_ws && colcon build --symlink-install; \
    fi \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /root/barracuda_ws/

COPY entrypoint.sh /root/entrypoint.sh
CMD [ "/bin/bash", "/root/entrypoint.sh" ]
